"""Formal scene entry, independent state, same-source dynamics and lock safety."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest
import yaml

from compliant_docking import cli
from compliant_docking.assembly.config import AssemblyScene, load_assembly_scene
from compliant_docking.assembly.consistency import prepare_models
from compliant_docking.assembly.runner import run
from compliant_docking.assembly.runtime import AssemblyRuntime
from compliant_docking.scene import DEFAULT_SCENE_PATH, load_scene

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes/hexframe_assembly.yaml"


@pytest.fixture(scope="module")
def formal(tmp_path_factory):
    r = AssemblyRuntime(load_scene(SCENE), tmp_path_factory.mktemp("formal_hexframe"))
    model = r.geometry.build_model()
    checks = prepare_models(r)
    return r, model, checks


def test_formal_scene_and_old_default_are_distinct():
    assert isinstance(load_scene(SCENE), AssemblyScene)
    assert not isinstance(load_scene(DEFAULT_SCENE_PATH), AssemblyScene)
    assert load_scene(SCENE).resource.mapping == (3, 4, 5, 0, 1, 2)


def test_full_inertia_and_every_port_from_resource(formal):
    r, model, _ = formal
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for name in ["module1", "module2"]:
        module = data.body(name)
        for number in range(1, 7):
            port = data.body(f"{name}_port_{number}")
            expected = r.resource.port(number)
            np.testing.assert_allclose(module.xmat.reshape(3, 3).T@(port.xpos-module.xpos), expected["position_m"], atol=1e-12)
            np.testing.assert_allclose(module.xmat.reshape(3, 3).T@port.xmat.reshape(3, 3), expected["rotation_matrix"], atol=1e-12)
    assert model.body("module1").mass[0] == pytest.approx(r.resource.info["body_mass_kg"], abs=1e-12)
    assert r.resource.gap == pytest.approx(.0464)
    assert r.geometry.check_base_dock(model, data)["prelocked"]
    assert r.geometry.check_storage_docks(model, data)["spare_empty"]


def test_same_source_arm_and_payload_dynamics_and_frames(formal):
    r, _, checks = formal
    for name in ["arm_tool", "carried"]:
        assert checks[name]["status"] == "PASS"
        assert max(checks[name]["maximum_absolute_errors"].values()) < 1e-8
        assert checks[name]["gripper_mass_kg"] == pytest.approx(.25)
    assert checks["carried"]["moving_mass_kg"] - checks["arm_tool"]["moving_mass_kg"] == pytest.approx(r.resource.info["body_mass_kg"], abs=1e-12)
    assert "module1_port_4_frame" in checks["carried"]["control_frames"]


def test_full_path_is_reachable_with_real_joint_limits(formal):
    r, model, _ = formal
    _, phases, planning = r.plan(model)
    assert sum(p.seconds for p in phases) == 53
    assert planning["ik_samples"] >= 1500
    assert planning["fk_max_error_m"] < 1e-6
    assert planning["spline_joint_limit_margin_rad"] > 0
    np.testing.assert_allclose(r.pin_model.lowerPositionLimit, r.joint_limits[0], atol=1e-12)
    np.testing.assert_allclose(r.pin_model.upperPositionLimit, r.joint_limits[1], atol=1e-12)


def test_handover_requires_receiving_lock_and_real_seating(formal):
    r, model, _ = formal
    assert r.verify_interlocks(model, r.initial_q) == ["grip_on", "rack_off", "grip_off"]
    data = mujoco.MjData(model)
    data.qpos[7:10] = r.installed
    mujoco.mj_forward(model, data)
    with pytest.raises(RuntimeError, match="physical seating"):
        r.handover(model, data, "assembly_on")
    assert data.eq_active.tolist() == [True, False, False]


def test_two_runtimes_do_not_pollute_layout_or_legacy_state(formal, tmp_path):
    import sys
    r, _, _ = formal
    legacy = sys.modules.get("assembly_sequence")
    saved = (legacy.PICK, legacy.OUT, legacy.phases) if legacy else None
    before = r.pick.copy()
    other = AssemblyRuntime(replace(r.scene, pick=(.32, .35, .195)), tmp_path)
    model = other.geometry.build_model()
    np.testing.assert_allclose(model.body("module1").pos, [.32, .35, .195])
    np.testing.assert_array_equal(r.pick, before)
    assert other.output != r.output
    if legacy:
        assert legacy.PICK is saved[0] and legacy.OUT == saved[1] and legacy.phases is saved[2]


def test_new_run_refuses_to_overwrite(formal):
    r, _, _ = formal
    with pytest.raises(ValueError, match="nonempty"):
        run(r.scene, output=r.output)


@pytest.mark.parametrize("field,value", [("contact_force_n", .1), ("contact_duration", -1), ("lift_height", float("nan"))])
def test_invalid_assembly_configuration_rejected(field, value):
    raw = yaml.safe_load(SCENE.read_text())
    raw["assembly"]["layout"][field] = value
    with pytest.raises(ValueError):
        load_assembly_scene(SCENE, raw)


def test_cli_routes_formal_scene_and_propagates_failure(monkeypatch, tmp_path):
    from compliant_docking.assembly import runner
    calls = []
    def fake_run(scene, **kwargs):
        calls.append((scene, kwargs))
        return 2
    monkeypatch.setattr(runner, "run", fake_run)
    monkeypatch.setattr(cli, "_load_run_docking", lambda: pytest.fail("Wrong runner"))
    assert cli.main(["--scene", str(SCENE), "--out", str(tmp_path), "--record"]) == 2
    assert isinstance(calls[0][0], AssemblyScene)
    assert calls[0][1]["record"]


def test_cli_does_not_silently_change_assembly_controller():
    with pytest.raises(ValueError, match="validated"):
        cli.main(["--scene", str(SCENE), "--controller", "hqp"])


def test_old_cli_still_routes_to_original_runner(monkeypatch):
    from compliant_docking.telemetry import Log
    log = Log()
    log.reset_logs()
    monkeypatch.setattr(cli, "_load_run_docking", lambda: SimpleNamespace(main=lambda **_: log))
    assert cli.main(["--quick"]) == 0
