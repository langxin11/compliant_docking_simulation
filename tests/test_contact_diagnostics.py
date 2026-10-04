from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from compliant_docking.cli import _load_run_docking
from compliant_docking.contact_diagnostics import (
    ContactDiagnostics,
    contact_wrench,
    evaluate_contact_load,
)
from compliant_docking.scene import load_scene


def test_contact_action_reaction_and_moment_arm():
    contact = SimpleNamespace(frame=np.array([0., 1., 0., -1., 0., 0., 0., 0., 1.]),
                              pos=np.array([.1, 0., 0.]))
    wrench = np.array([10., 2., 0., 0., 0., 1.])
    result = contact_wrench(contact, wrench, on_geom2=True, origin=np.zeros(3))
    np.testing.assert_allclose(result, [-2., 10., 0., 0., 0., 2.])
    np.testing.assert_allclose(contact_wrench(contact, wrench, on_geom2=False, origin=np.zeros(3)),
                               -result)


@pytest.mark.parametrize("contact", [False, True])
def test_sensor_contact_inertial_balance(contact):
    # The parent accelerates during free fall; the fixed tool includes an offset
    # COM and a rotated/translated sensor, exercising the reference-point shift.
    floor = '<geom name="target_floor" type="plane" size="1 1 .1"/>' if contact else ''
    model = mujoco.MjModel.from_xml_string(f'''<mujoco>
      <option timestep=".001" gravity="0 0 -9.81"/>
      <worldbody>{floor}<body name="carrier" pos="0 0 .11">
        <joint type="slide" axis="0 0 1"/>
        <body name="tool_root"><site name="sensor" pos=".03 .02 .01" euler="20 30 40"/>
          <inertial mass="1" pos=".02 0 0" diaginertia=".1 .1 .1"/>
          <geom name="tool_sphere" type="sphere" size=".1"/>
        </body></body></worldbody>
      <sensor><force name="force_sensor" site="sensor"/>
        <torque name="torque_sensor" site="sensor"/></sensor></mujoco>''')
    data = mujoco.MjData(model)
    diagnostic = ContactDiagnostics(model, SimpleNamespace(eef_body="tool_root", sensor_site="sensor",
                                                            target=SimpleNamespace(prefix="target_")))
    for _ in range(250):
        t, q, v = data.time, data.qpos.copy(), data.qvel.copy()
        mujoco.mj_step(model, data)
        sample, rows = diagnostic.capture(data, t, q, v)
        np.testing.assert_allclose(sample["balance_residual_world"], 0., atol=1e-9)
        assert sample["state_t"]-sample["t"] == pytest.approx(.001)
    if contact:
        assert rows
        assert sample["interface_world"][2] == pytest.approx(9.81, abs=.01)
        assert sample["inertial_world"][2] == pytest.approx(9.81, abs=.01)
        # Gravity and ground support cancel on the child; its movable parent
        # carries no load. A raw F/T reading is therefore not the contact force.
        assert sample["sensor_world"][2] == pytest.approx(0., abs=.01)
    else:
        assert not rows
        np.testing.assert_allclose(sample["sensor_world"], 0., atol=1e-10)


def test_rotating_tool_with_applied_load_has_balanced_wrench():
    model = mujoco.MjModel.from_xml_string('''<mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody><body name="carrier"><joint axis="0 1 0"/>
        <body name="tool_root" pos=".2 0 .1">
          <site name="sensor" pos=".01 .02 .03" euler="10 20 30"/>
          <inertial mass="2" pos=".05 .02 .01" diaginertia=".1 .2 .3"/>
        </body></body></worldbody>
      <sensor><force name="force_sensor" site="sensor"/>
        <torque name="torque_sensor" site="sensor"/></sensor></mujoco>''')
    data = mujoco.MjData(model)
    data.qpos[0], data.qvel[0] = .3, 1.7
    body = model.body("tool_root").id
    data.xfrc_applied[body] = [1., 2., 3., .4, .5, .6]
    diagnostic = ContactDiagnostics(model, SimpleNamespace(eef_body="tool_root", sensor_site="sensor",
                                                            target=SimpleNamespace(prefix="target_")))
    for _ in range(20):
        t, q, v = data.time, data.qpos.copy(), data.qvel.copy()
        mujoco.mj_step(model, data)
        sample, _ = diagnostic.capture(data, t, q, v)
        np.testing.assert_allclose(sample["balance_residual_world"], 0., atol=1e-9)


def test_diagnostics_preserve_controls_and_record_preintegration_pose():
    scene = load_scene('scenes/iiwa14_compliant_insertion.yaml')
    run = _load_run_docking().main
    plain = run(scene=scene, duration=.04, render=False, record=False, plot=False)
    diagnostic = run(scene=scene, duration=.04, render=False, record=False, plot=False, diagnostics=True)
    np.testing.assert_array_equal(plain.joint_angles, diagnostic.joint_angles)
    np.testing.assert_array_equal(plain.tau_hist, diagnostic.tau_hist)
    model = scene.build_mjmodel()
    data = mujoco.MjData(model)
    for sample in diagnostic.contact_diagnostics:
        data.qpos[:] = sample["q"]
        data.qvel[:] = sample["v"]
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.body(scene.eef_body).xpos, sample["position"], atol=1e-12)
        np.testing.assert_allclose(data.body(scene.eef_body).xmat.reshape(3, 3), sample["rotation"], atol=1e-12)


def test_actual_contact_load_gate_rejects_inertially_cancelled_ft_load():
    scene = load_scene('scenes/iiwa14_compliant_insertion.yaml')
    # Contact and inertial terms can cancel in F/T. The true contact channel must
    # still reject these loads against the unchanged declared 40 N / 2 Nm limits.
    wrench = np.array([[45., 0., 0., 0., 0., 2.5]])
    result = evaluate_contact_load(wrench, scene.docking, np.eye(3), complete=True)
    assert result["status"] == "FAIL"
    assert result["reasons"] == ["contact force limit", "contact axial moment limit"]
    assert evaluate_contact_load(np.zeros((2, 6)), scene.docking, np.eye(3), complete=False)["status"] == "INCOMPLETE"
