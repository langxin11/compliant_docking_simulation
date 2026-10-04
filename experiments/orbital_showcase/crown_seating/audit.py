"""Audit saved trajectories independently of the simulation's status strings."""
import hashlib
import json
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent / "outputs"


def main():
    summary = json.loads((OUT / "summary.json").read_text())
    checks = {}
    for item in summary:
        name = item["name"]
        trace = np.load(OUT / name / "trace.npz")
        a = trace["values"]
        assert np.isfinite(a).all() and np.isfinite(trace["qpos"]).all(), name
        assert item["warnings"] == 0, name
        assert len(a) == round(12 / item["timestep"]), name
        active = np.flatnonzero(a[:, 12])
        if len(active):
            first = active[0]
            window = a[first-round(.5/item["timestep"]):first]
            assert len(window)*item["timestep"] >= .5-1e-12, name
            # Recompute physical conditions, rather than trusting eligible flag.
            assert np.all(window[:, 4] <= .0005), name
            assert np.all(abs(window[:, 5]) <= .00075), name
            assert np.all(window[:, 6] <= .5), name
            assert np.all(window[:, 7] <= .0005), name
            assert np.all(window[:, 8] <= np.deg2rad(.5)), name
            assert np.all((window[:, 9] >= .1) & (window[:, 9] <= 10)), name
            assert np.all(window[:, 10] <= .0003), name
            assert np.all(a[first:, 12] == 1), name
            assert abs(a[first, 0]-item["lock_time_s"]) < 1e-8, name
            assert item["latch_position_drift_m"] <= 1e-5, name
        assert bool(len(active)) == (not item["stiff_yaw"]), name
        checks[name] = "PASS" if len(active) else "UNSEATED_LOCK_CORRECTLY_REJECTED"
    comparisons = {}
    by_name = {item["name"]: item for item in summary}
    for stem in ["xy1mm_yaw2deg", "xy1mm_yaw2deg_stiff"]:
        base, half = by_name[stem], by_name[stem+"_halfstep"]
        differences = {key: abs(base[key]-half[key]) for key in
                       ["peak_force_n", "final_lateral_mm", "final_abs_gap_mm", "final_angle_deg"]}
        assert differences["peak_force_n"] <= max(.1, .1*base["peak_force_n"])
        assert all(differences[key] <= .1 for key in differences if key != "peak_force_n")
        assert base["status"] == half["status"]
        comparisons[stem] = differences
    for path, digest in json.loads((OUT / "source_manifest.json").read_text()).items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest, path
    report = dict(status="PASS", trajectory_checks=checks, timestep_differences=comparisons,
                  scope="two physical timesteps at fixed 1 kHz control; not convergence proof")
    (OUT / "audit.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
