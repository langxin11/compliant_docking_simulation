"""dtcheck driver: three rollouts to resolve the nx6 timestep SENSITIVE status.

Group A: combo_ny6_p15 / combo_py6_n15 at dt_quarter (0.125 ms), paired against
         the archived selected dt_half (0.25 ms) records  -> is the sensitivity
         point-specific?
Group B: nx6 with a half-timestep base scene (0.25 ms) run through the
         dt_quarter setting (=> 0.0625 ms), paired against the archived
         selected nx6 dt_quarter (0.125 ms)                -> does the marginal
         0.25-vs-0.125 discrepancy converge under refinement?

All gates, gains, geometry and acceptance thresholds are the reviewed ones;
this driver only orchestrates existing entry points and records provenance.
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
ARCHIVE = REPO / "runs/petal_angle1_blend030_20261003"


def job_a(case):
    import petal_selected_study as selected
    return selected.run_one(OUT, case, "dt_quarter")


def job_b(_=None):
    import petal_guidance_study as study
    import petal_selected_study as selected
    selected.configure()
    base = load_scene_b()
    return study.run_one(OUT, base, "selected", "nx6", "dt_quarter", "dtcheck_0p0625ms")


def load_scene_b():
    from compliant_docking.scene import load_scene
    return load_scene(str(OUT / "iiwa14_petal_insertion_dt00025.yaml"))


def main():
    start = time.monotonic()
    scene_src = REPO / "scenes/iiwa14_petal_insertion.yaml"
    yaml_text = scene_src.read_text()
    assert yaml_text.count("timestep: 0.0005") == 1, "unexpected scene layout"
    (OUT / "iiwa14_petal_insertion_dt00025.yaml").write_text(
        yaml_text.replace("timestep: 0.0005", "timestep: 0.00025"))

    tracked = ["experiments/petal_selected_study.py", "experiments/petal_guidance_study.py",
               "experiments/petal_insertion_suite.py", "experiments/petal_capture_grid.py",
               "experiments/insertion_suite.py", "experiments/run_docking.py",
               "scenes/iiwa14_petal_insertion.yaml"]
    manifest = dict(
        purpose="resolve closeout open item 1 (nx6 timestep sensitivity) for angle1_blend030",
        archived_evidence=str(ARCHIVE.relative_to(REPO)),
        plan={
            "A_combo_ny6_p15": "dt_quarter 0.125 ms vs archived dt_half 0.25 ms",
            "A_combo_py6_n15": "dt_quarter 0.125 ms vs archived dt_half 0.25 ms",
            "B_nx6": "base scene timestep 0.25 ms via dt_quarter => 0.0625 ms "
                     "vs archived dt_quarter 0.125 ms",
        },
        controls_and_gates_unchanged=True,
        sources={p: hashlib.sha256((REPO / p).read_bytes()).hexdigest() for p in tracked},
        group_b_scene="iiwa14_petal_insertion_dt00025.yaml (copy with timestep 0.00025)",
    )
    (OUT / "dtcheck_manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")

    with ProcessPoolExecutor(max_workers=3) as executor:
        futures = [executor.submit(job_a, "combo_ny6_p15"),
                   executor.submit(job_a, "combo_py6_n15"),
                   executor.submit(job_b)]
        for future in futures:
            future.result()

    import petal_capture_grid as grid
    arch = lambda name: json.loads((ARCHIVE / f"{name}.json").read_text())
    new = lambda name: json.loads((OUT / f"{name}.json").read_text())
    pairs = {
        "A_combo_ny6_p15_0p25_vs_0p125": grid.sensitivity(
            arch("g_selected_combo_ny6_p15_lateral_released_dt_half"),
            new("g_selected_combo_ny6_p15_lateral_released_dt_quarter")),
        "A_combo_py6_n15_0p25_vs_0p125": grid.sensitivity(
            arch("g_selected_combo_py6_n15_lateral_released_dt_half"),
            new("g_selected_combo_py6_n15_lateral_released_dt_quarter")),
        "B_nx6_0p125_vs_0p0625": grid.sensitivity(
            arch("g_selected_nx6_lateral_released_dt_quarter"),
            new("g_selected_nx6_lateral_released_dt_quarter")),
    }
    statuses = {k: v["status"] for k, v in pairs.items()}
    for name in ("g_selected_combo_ny6_p15_lateral_released_dt_quarter",
                 "g_selected_combo_py6_n15_lateral_released_dt_quarter",
                 "g_selected_nx6_lateral_released_dt_quarter"):
        record = new(name)
        print("ROLLOUT", name, record["assessment"]["status"],
              "dt=", record["scene"]["physics"]["timestep"], flush=True)
    promote = all(s == "STABLE_IN_TWO_STEPS" for s in statuses.values())
    summary = dict(pairs=pairs, statuses=statuses,
                   decision="PROMOTE_CANDIDATE" if promote else "KEEP_CANDIDATE_NOT_PROMOTED",
                   decision_rule="promote iff all three pairs STABLE_IN_TWO_STEPS "
                                 "(sensitivity localised as a 0.25 ms marginal effect)",
                   wall_seconds=time.monotonic() - start)
    (OUT / "dtcheck_summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print("DECISION", summary["decision"], json.dumps(statuses), flush=True)


if __name__ == "__main__":
    main()
