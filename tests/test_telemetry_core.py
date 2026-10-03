"""save_rollout telemetry modes: core omits per-step diagnostic channels on disk."""
import importlib
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))
suite = importlib.import_module("insertion_suite")


@dataclass
class _Scene:
    name: str = "stub"
    physics: dict = field(default_factory=lambda: {"timestep": 0.001})


def _log():
    samples = [{"t": 0.0, "phase": "approach"}, {"t": 0.001, "phase": "insert"}]
    trajectory = SimpleNamespace(names=["start", "entry"], times=np.array([0.0, 1.0]), poses=[])
    return SimpleNamespace(
        docking_samples=samples,
        joint_angles=np.zeros((2, 7)), joint_velocities=np.zeros((2, 7)),
        pos_desired=np.zeros((2, 3)), pos_actual=np.zeros((2, 3)),
        tau_hist=np.zeros((2, 7)), torque_saturated=np.zeros(2, dtype=bool),
        se3_diagnostics=None, docking_gate={"status": "PASS"},
        docking_trajectory=trajectory,
        contact_diagnostics=[{"state_t": 0.0, "interface_world": np.zeros(6)},
                             {"state_t": 0.001, "interface_world": np.zeros(6)}],
        contact_events=[{"t": 0.5, "force": np.zeros(3)}],
        contact_summary={"peak_balance_force_residual_N": 0.0},
        geometry_evaluation={"status": "SEATED_CANDIDATE", "reasons": []},
    )


def test_full_telemetry_writes_diagnostic_channels_and_events(tmp_path):
    out = tmp_path / "full"
    out.mkdir()
    arrays = suite.save_rollout(out, "case", _Scene(), _log(), telemetry="full")
    with np.load(out / "case.npz") as archive:
        assert "diagnostic_interface_world" in archive.files
        assert "q" in archive.files and "t" in archive.files
    assert (out / "case.contacts.npz").exists()
    metadata = json.loads((out / "case.json").read_text())
    assert metadata["telemetry"] == "full"
    assert "diagnostic_interface_world" in arrays


def test_core_telemetry_omits_diagnostic_channels_but_returns_full_dict(tmp_path):
    out = tmp_path / "core"
    out.mkdir()
    arrays = suite.save_rollout(out, "case", _Scene(), _log(), telemetry="core")
    with np.load(out / "case.npz") as archive:
        assert not [key for key in archive.files if key.startswith("diagnostic_")]
        assert {"t", "phase", "q", "qd", "position", "desired_position",
                "torque", "torque_saturated"} <= set(archive.files)
    assert not (out / "case.contacts.npz").exists()
    metadata = json.loads((out / "case.json").read_text())
    assert metadata["telemetry"] == "core"
    assert metadata["contact_diagnostics"] == {"peak_balance_force_residual_N": 0.0}
    assert metadata["geometry_evaluation"]["status"] == "SEATED_CANDIDATE"
    # In-memory assessment still sees the full channel set.
    assert "diagnostic_interface_world" in arrays


def test_save_rollout_rejects_unknown_telemetry_mode(tmp_path):
    with pytest.raises(ValueError, match="telemetry"):
        suite.save_rollout(tmp_path, "case", _Scene(), _log(), telemetry="compact")
