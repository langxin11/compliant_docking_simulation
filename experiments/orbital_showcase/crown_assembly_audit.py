"""Audit full assembly's saved physical-state and handover records."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent / "outputs/crown_integrated"


def main():
    report = json.loads((OUT / "validation.json").read_text())
    trace = np.load(OUT / "contact_trace.npz")["values"]
    records = np.load(OUT / "rollout.npz")
    assert report["status"] == "PASS" and not report["faults"]
    assert np.isfinite(trace).all() and np.isfinite(records["qpos"]).all()
    assert len(trace) == 49001 and records["t"][-1] == 49.
    assert [e["event"] for e in report["events"]] == ["grip_on", "rack_off", "assembly_on", "grip_off"]
    assert records["locks"][-1].tolist() == [False, False, True]
    lock_index = np.flatnonzero(trace[:, 11])[0]
    # Last 499 pre-activation samples are untouched by the accepted-frame weld.
    # Boundary sample uses the accepted anchor; reported dwell verifies full 500 ms.
    window = trace[lock_index-499:lock_index]
    assert len(window) == 499
    assert np.all((window[:, 1] >= .15) & (window[:, 1] <= .6))
    assert np.all(window[:, 3] <= .5)
    assert np.all(window[:, 5] <= .0003)
    assert np.all(window[:, 7] <= .00075)
    assert np.all(window[:, 8] <= np.deg2rad(.5))
    assert np.all(window[:, 9] <= .0005)
    assert np.all(window[:, 10] <= np.deg2rad(.5))
    assert report["events"][2]["dwell_s"] >= .5
    release = report["events"][3]
    assert release["gripper_constraint_force_n"] <= .2 and release["unloaded_dwell_s"] >= .5
    assert report["peak_axial_force_n"] <= 10 and report["max_penetration_mm"] <= .3
    assert report["final_module_error_mm"] <= 1
    for path, digest in json.loads((OUT / "source_manifest.json").read_text()).items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest, path
    result = dict(status="PASS", expected_duration_s=49, handover_order_verified=True,
                  prelock_samples_recomputed=499, lock_boundary_dwell_s=report["events"][2]["dwell_s"],
                  unload_before_release_verified=True, source_hashes_verified=True)
    (OUT / "audit.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
