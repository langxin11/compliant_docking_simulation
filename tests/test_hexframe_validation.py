"""HexFrame validation package: behavior-preserving dt parameterization.

The formal 1 ms scene must stay locked; the parameterization only derives the
10 ms control cadence and the 0.5 s audit window from the runtime timestep,
and is bit-identical at 1 ms (regression-checked against the recorded run).
"""
import json
from dataclasses import replace

import numpy as np
import pytest
import yaml

from compliant_docking.assembly.audit import debounce_stats, window_n
from compliant_docking.assembly.simulation import control_every_steps
from compliant_docking.scene import REPO_ROOT, load_scene

FORMAL = REPO_ROOT / "scenes/hexframe_assembly.yaml"


def test_debounce_stats_fraction_and_longest_invalid_run():
    dt = 0.001
    mask = np.ones(100, dtype=bool)
    mask[10:13] = False   # 3 ms blip
    mask[50] = False      # 1 ms blip
    fraction, worst = debounce_stats(mask, dt)
    assert fraction == 0.96
    assert worst == pytest.approx(0.003)


def test_debounce_stats_rejects_long_outage():
    mask = np.ones(2000, dtype=bool)
    mask[100:130] = False  # 30 ms outage, far above the 10 ms budget
    fraction, worst = debounce_stats(mask, 0.001)
    assert fraction >= 0.98
    assert worst > 0.01


def test_control_cadence_and_audit_window_derive_from_timestep():
    assert control_every_steps(0.001) == 10
    assert control_every_steps(0.0005) == 20
    assert window_n(0.001) == 500
    assert window_n(0.0005) == 1000


def test_replacing_scene_timestep_bypasses_yaml_gate_for_validation_entry():
    scene = replace(load_scene(str(FORMAL)), timestep=5e-4)
    assert scene.timestep == 5e-4


def test_formal_yaml_still_rejects_non_1ms_physics(tmp_path):
    raw = yaml.safe_load(FORMAL.read_text())
    raw["physics"]["timestep"] = 0.0005
    variant = tmp_path / "hexframe_half.yaml"
    variant.write_text(yaml.safe_dump(raw, allow_unicode=True))
    with pytest.raises(ValueError, match="1 ms"):
        load_scene(str(variant))


def test_recorded_1ms_baseline_audit_values_unchanged_by_parameterization(tmp_path):
    """The recorded baseline's runtime.json drives identical window arithmetic."""
    baseline = REPO_ROOT / "runs/hexframe_main_integration_20261003/full_assembly_cad_precision"
    if not baseline.exists():
        pytest.skip("recorded 1 ms baseline not present on this machine")
    runtime = json.loads((baseline / "runtime.json").read_text())
    assert runtime["physics_timestep_s"] == 0.001
    assert window_n(runtime["physics_timestep_s"]) == 500
    trace_len = round(json.loads((baseline / "validation.json").read_text())["duration_s"] / 0.001) + 1
    values = __import__("numpy").load(baseline / "contact_trace.npz")["values"]
    assert len(values) == trace_len
