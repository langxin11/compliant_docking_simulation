"""Unit scale, CAD inertia and side-port mating checks for the default module."""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
import assembly_sequence as seq
import crown_assembly as engine
import hexframe_integration as hf


@pytest.fixture(scope="module")
def pair(tmp_path_factory):
    with hf.configure(engine) as hooks:
        engine.OUT = seq.OUT = tmp_path_factory.mktemp("hexframe")
        model = hooks.build_model()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        yield model, data


def test_metric_scale_and_cad_inertia(pair):
    model, data = pair
    body = model.body("module1")
    assert body.mass[0] == pytest.approx(3.5082656535035275)
    R = Rotation.from_quat(body.iquat, scalar_first=True).as_matrix()
    actual = R@np.diag(body.inertia)@R.T
    np.testing.assert_allclose(actual, hf.INFO["body_inertia_kg_m2"], atol=1e-12)
    center = data.body("module1").xpos
    assert np.linalg.norm(data.body("module1_port_1").xpos-center) == pytest.approx(hf.A)


def test_six_side_ports_and_mating_clocking(pair):
    _, data = pair
    for module in ["module1", "module2"]:
        root = data.body(module)
        for port in range(1, 7):
            local = root.xmat.reshape(3, 3).T@(data.body(f"{module}_port_{port}").xpos-root.xpos)
            assert abs(local[2]) < 1e-10
        np.testing.assert_allclose(data.body(f"{module}_port_1").xmat.reshape(3, 3)[:, 2], [0, 0, 1], atol=1e-10)
        np.testing.assert_allclose(data.body(f"{module}_port_4").xmat.reshape(3, 3)[:, 2], [0, 0, -1], atol=1e-10)
    lower = data.body("module2_port_1").xmat.reshape(3, 3)
    upper = data.body("module1_port_4").xmat.reshape(3, 3)
    expected = Rotation.from_euler("z", 45, degrees=True).as_matrix()@np.diag([1., -1., -1.])
    np.testing.assert_allclose(lower.T@upper, expected, atol=1e-10)


def test_nominal_seat_and_overtravel(pair):
    model, _ = pair
    data = mujoco.MjData(model)
    data.eq_active[:] = False
    data.qpos[7:10] = seq.INSTALLED
    mujoco.mj_forward(model, data)
    assert max((-float(c.dist) for c in data.contact), default=0) < .00002
    data.qpos[9] -= .0001
    mujoco.mj_forward(model, data)
    matched = [c for c in data.contact if "_stop_" in (model.geom(c.geom1).name or "")
               and "_stop_" in (model.geom(c.geom2).name or "")]
    assert matched
    assert .00005 < max(-float(c.dist) for c in matched) < .0002


def test_default_trajectory_is_reachable(pair):
    model, _ = pair
    with hf.configure(engine) as hooks:
        _, _, planning = seq.plan(model, initial_q=hf.vertical.INITIAL_Q, phase_list=hooks.phases())
        assert planning["fk_max_error_m"] < 1e-6


def test_prelocked_base_interface_matches_module2_side_port(pair):
    model, data = pair
    checked = hf.check_base_dock(model, data)
    assert checked["prelocked"] and checked["mating_position_error_m"] < 1e-10
    np.testing.assert_allclose(data.body("base_dock_port").xmat.reshape(3, 3)[:, 2], [0, 0, 1], atol=1e-10)
    assert data.body("base_dock_port").xpos[2] > .002
    # Use the actual standard interface meshes, rather than a decorative pad.
    for part in ["head_plate", "stop_land", "petal_0", "petal_1", "petal_2", "petal_3", "adapter"]:
        assert model.geom(f"base_dock_port_{part}").dataid[0] == model.geom(f"gripper_petal_head_{part}").dataid[0]
    assert not any((model.geom(i).name or "").startswith("seed_station")
                   and model.geom(i).contype[0] for i in range(model.ngeom))


def test_scenario_restores_state_after_error(pair):
    before_pick, before_out, before_builder = seq.PICK, engine.OUT, engine.build_model
    with pytest.raises(RuntimeError, match="test error"):
        with hf.configure(engine):
            assert engine.build_model is before_builder
            raise RuntimeError("test error")
    assert seq.PICK is before_pick
    assert engine.OUT == before_out
    assert engine.build_model is before_builder


def test_storage_and_spare_socket_geometry_and_safe_release(pair):
    model, _ = pair
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert hf.check_storage_docks(model, data)["spare_empty"]
    assert data.eq_active.tolist() == [True, False, False]
    with pytest.raises(RuntimeError, match="receiving latch unconfirmed"):
        seq.handover(model, data, "rack_off")
    assert data.eq_active.tolist() == [True, False, False]
    data.qpos[:7] = engine.reference_ik(model, mujoco.MjData(model), hf.vertical.INITIAL_Q.copy(),
                                      seq.PICK_TIP, hf.vertical.TIP_ROTATION)
    mujoco.mj_forward(model, data)
    seq.handover(model, data, "grip_on")
    assert data.eq_active.tolist() == [True, True, False]
    seq.handover(model, data, "rack_off")
    assert data.eq_active.tolist() == [False, True, False]
    force, depth, _ = hf.storage_wrench(model, data)
    assert depth < .00002 and np.linalg.norm(force) < 10
    data.eq_active[:] = False
    data.qpos[9] += .04
    mujoco.mj_forward(model, data)
    assert hf.storage_wrench(model, data)[2] == 0
    assert np.linalg.norm(data.site("module1_port_4_mating").xpos-data.site("storage_dock_mating").xpos) == pytest.approx(.04)
    # A controlled overtravel must load the real stop cells; storage load must
    # not be counted as module-to-module assembly force or an unexpected hit.
    data.qpos[7:10] = seq.PICK-[0, 0, .0001]
    mujoco.mj_forward(model, data)
    force, depth, contacts = hf.storage_wrench(model, data)
    assert contacts > 0 and .00005 < depth < .0002
    assert np.linalg.norm(force) > 0
    assembly_force, _, count, unexpected, _ = hf.wrench(model, data)
    np.testing.assert_allclose(assembly_force, 0, atol=1e-10)
    assert count == 0 and not unexpected
    for port in ["storage_dock_port", "spare_dock_port"]:
        assert model.geom(f"{port}_petal_0").dataid[0] == model.geom("base_dock_port_petal_0").dataid[0]
