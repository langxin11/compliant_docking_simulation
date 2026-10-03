"""Shared model, contact telemetry and seating gates for the imported interface."""
import hashlib
import json
from dataclasses import replace
from types import SimpleNamespace

import mujoco
import numpy as np
import pinocchio as pin
import pytest

from compliant_docking.contact_diagnostics import ContactDiagnostics
from compliant_docking.docking_task import build_docking_trajectory
from compliant_docking.models import load_assembled_pin_model
from compliant_docking.petal_geometry import evaluate_petal_seating
from compliant_docking.scene import load_scene
from compliant_docking.simulation.mujoco_env import MujRobot


@pytest.fixture(scope="module")
def scene():
    return load_scene("scenes/iiwa14_petal_insertion.yaml")


def test_shared_tool_model_has_matching_frames_mass_matrix_and_joint_limits(scene):
    model, robot = scene.build_mjmodel(), load_assembled_pin_model(scene)
    data, pin_data = mujoco.MjData(model), robot.createData()
    frame = robot.getFrameId(scene.robot.ee_frame)
    assert model.nq == robot.nq == 7
    np.testing.assert_allclose(model.jnt_range[:, 0], robot.lowerPositionLimit, atol=1e-12)
    np.testing.assert_allclose(model.jnt_range[:, 1], robot.upperPositionLimit, atol=1e-12)
    for q in [scene.task.ik_guess, np.zeros(7), np.array([.2, .5, -.2, -1., .3, 1.5, .1])]:
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        pin.forwardKinematics(robot, pin_data, q)
        pin.updateFramePlacements(robot, pin_data)
        body = data.body(scene.eef_body)
        np.testing.assert_allclose(pin_data.oMf[frame].translation, body.xpos, atol=1e-12)
        np.testing.assert_allclose(pin_data.oMf[frame].rotation, body.xmat.reshape(3, 3), atol=1e-12)
        np.testing.assert_allclose(body.xpos, data.site(scene.robot.ee_site).xpos, atol=1e-12)
        mass = np.zeros((7, 7))
        mujoco.mj_fullM(model, data, mass)
        np.testing.assert_allclose(pin.crba(robot, pin_data, q), mass, atol=1e-12)
    info = json.loads((scene.tool.mjcf.parent / "model_info.json").read_text())
    body_id = model.body(scene.eef_body).id
    assert model.body_mass[body_id] == pytest.approx(info["body_mass_kg"], rel=1e-6)
    principal_rotation = pin.Quaternion(*model.body_iquat[body_id]).matrix()
    tensor = principal_rotation @ np.diag(model.body_inertia[body_id]) @ principal_rotation.T
    np.testing.assert_allclose(tensor, info["body_inertia_about_com_kg_m2"], atol=1e-9)
    assert abs(tensor[0, 2]) > 1e-7  # importing only the diagonal would lose this term
    assert model.neq == 0  # physical contact only, no imported weld lock


def test_simulation_wrapper_preserves_declared_solver_settings(scene):
    robot = MujRobot(scene.build_mjmodel(), render=False, record=False,
                     dt=scene.physics.timestep, eef_body=scene.eef_body)
    assert robot.model.opt.tolerance == pytest.approx(1e-10)
    assert robot.model.opt.iterations == 60
    assert robot.model.opt.timestep == pytest.approx(.0005)


def test_mating_phase_is_known_geometry_and_truth_is_not_planner_input(scene):
    plan = build_docking_trajectory(scene.task, scene.docking, scene.trajectory)
    reference = pin.Quaternion(0., .9238795325112867, .3826834323650898, 0.).matrix()
    expected = pin.exp3(np.array([0., 0., np.deg2rad(5.)])) @ reference
    np.testing.assert_allclose(plan.poses[-1].rotation, expected, atol=1e-12)
    np.testing.assert_allclose(plan.poses[-1].translation, [.002, .498, .39625], atol=1e-12)
    altered = replace(scene, target=replace(scene.target, pos=scene.target.pos+[.01, 0, 0]))
    unchanged = build_docking_trajectory(altered.task, altered.docking, altered.trajectory)
    for t in np.linspace(0., plan.total_duration, 20):
        np.testing.assert_array_equal(plan.get_pose(t).homogeneous, unchanged.get_pose(t).homogeneous)


def test_compact_contact_loads_match_event_mode_and_analytical_static_weight():
    model = mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 -9.81"/>
      <worldbody><geom name="target_stop_floor" type="plane" size="1 1 .1"/>
        <body name="carrier" pos="0 0 .101"><joint type="slide" axis="0 0 1"/>
          <body name="tool_root"><inertial mass="1" pos="0 0 0" diaginertia=".01 .01 .01"/>
            <site name="sensor"/><geom name="tool_stop_box" type="box" size=".1 .1 .1"/>
          </body></body></worldbody>
      <sensor><force name="force_sensor" site="sensor"/>
        <torque name="torque_sensor" site="sensor"/></sensor></mujoco>''')
    data = mujoco.MjData(model)
    descriptor = SimpleNamespace(eef_body="tool_root", sensor_site="sensor",
                                 target=SimpleNamespace(prefix="target_"))
    full, compact = ContactDiagnostics(model, descriptor), ContactDiagnostics(model, descriptor, store_events=False)
    for _ in range(300):
        t, q, v = data.time, data.qpos.copy(), data.qvel.copy()
        mujoco.mj_step(model, data)
        a, rows = full.capture(data, t, q, v)
        b, empty = compact.capture(data, t, q, v)
        np.testing.assert_allclose(a["interface_world"], b["interface_world"], atol=1e-12)
        np.testing.assert_allclose(b["balance_residual_world"], 0., atol=1e-9)
        assert a["interface_count"] == b["interface_count"]
        assert a["stop_contact_count"] == b["stop_contact_count"]
        assert empty == []
    assert rows and b["stop_contact_count"] > 0
    assert b["interface_world"][2] == pytest.approx(9.81, abs=.01)


def test_nominal_pose_alone_cannot_pass_without_loaded_stop_contact(scene):
    reference = pin.Quaternion(0., .9238795325112867, .3826834323650898, 0.).matrix()
    samples = [dict(t=float(t), position=scene.target.pos+[0, 0, .0464],
                    rotation=reference, stop_contact_count=4) for t in np.arange(0., 2., .01)]
    assert evaluate_petal_seating(samples, scene, complete=True)["status"] == "SEATED_CANDIDATE"
    assert evaluate_petal_seating(samples, scene, complete=False)["status"] == "INCOMPLETE"
    for sample in samples:
        sample["stop_contact_count"] = 0
    gate = evaluate_petal_seating(samples, scene, complete=True)
    assert gate["status"] == "NOT_SEATED"
    assert "stop contact not maintained" in gate["reasons"]


def test_imported_asset_hashes_match_new_manifest(scene):
    root = scene.tool.mjcf.parent
    manifest = json.loads((root / "manifest.json").read_text())
    assert manifest["bundled_manifest_used"] is False
    for name, digest in manifest["imported_files"].items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == digest, name
