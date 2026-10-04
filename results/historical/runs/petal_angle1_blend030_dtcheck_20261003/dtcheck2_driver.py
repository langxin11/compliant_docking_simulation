"""Supplementary rollouts: combo points at 0.0625 ms via half-timestep base.

The pre-registered dtcheck verdict (KEEP_CANDIDATE_NOT_PROMOTED, based on the
0.25-vs-0.125 ms rule) stands regardless of this run. This supplement only
attributes the SENSITIVE cause: if 0.125-vs-0.0625 ms is stable at both combo
points, the peak-force sensitivity reflects the coarseness of the archived
0.25 ms records, not divergence of the candidate at certified step sizes.

Files are written into supplement_0p0625/ because the dt_quarter label is
shared with the 0.125 ms records in the parent directory.
"""
import hashlib
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
os.chdir(REPO)
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MPLBACKEND", "Agg")
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
sys.path.insert(0, str(REPO / "experiments"))

OUT = Path(__file__).resolve().parent
SUP = OUT / "supplement_0p0625"
HALF_YAML = OUT / "iiwa14_petal_insertion_dt00025.yaml"


def job(case):
    import petal_guidance_study as study
    import petal_selected_study as selected
    selected.configure()
    from compliant_docking.scene import load_scene
    base = load_scene(str(HALF_YAML))
    # base 0.25 ms / 4 = 0.0625 ms; label stays dt_quarter, stage records intent
    return study.run_one(SUP, base, "selected", case, "dt_quarter", "dtcheck_0p0625ms")


def main():
    start = time.monotonic()
    assert HALF_YAML.exists(), "run dtcheck_driver.py first"
    SUP.mkdir(exist_ok=True)
    tracked = ["experiments/petal_selected_study.py", "experiments/petal_guidance_study.py",
               "experiments/petal_insertion_suite.py", "experiments/petal_capture_grid.py",
               "experiments/insertion_suite.py", "experiments/run_docking.py",
               "scenes/iiwa14_petal_insertion.yaml"]
    (SUP / "dtcheck2_manifest.json").write_text(json.dumps(dict(
        purpose="attribute combo-point peak-force SENSITIVE: 0.125 vs 0.0625 ms",
        preresistered_verdict_unchanged="KEEP_CANDIDATE_NOT_PROMOTED",
        base_scene="../iiwa14_petal_insertion_dt00025.yaml, dt_quarter setting => 0.0625 ms",
        sources={p: hashlib.sha256((REPO / p).read_bytes()).hexdigest() for p in tracked},
    ), indent=1) + "\n")

    with ProcessPoolExecutor(max_workers=2) as executor:
        futures = [executor.submit(job, case) for case in ("combo_ny6_p15", "combo_py6_n15")]
        for future in futures:
            future.result()

    import petal_capture_grid as grid
    load = lambda root, name: json.loads((root / f"{name}.json").read_text())
    pairs = {}
    for case in ("combo_ny6_p15", "combo_py6_n15"):
        name = f"g_selected_{case}_lateral_released_dt_quarter"
        pairs[f"S_{case}_0p125_vs_0p0625"] = grid.sensitivity(
            load(OUT, name), load(SUP, name))
    statuses = {k: v["status"] for k, v in pairs.items()}
    converged = all(s == "STABLE_IN_TWO_STEPS" for s in statuses.values())
    summary = dict(pairs=pairs, statuses=statuses,
                   attribution="archived 0.25 ms coarseness" if converged
                               else "combo peak forces not converged",
                   preresistered_verdict_unchanged=True,
                   wall_seconds=time.monotonic() - start)
    (SUP / "dtcheck2_summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print("SUPPLEMENT", json.dumps(statuses), flush=True)


if __name__ == "__main__":
    main()
