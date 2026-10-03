"""HexFrame P1: grasp/fit error grid at the validated 1 ms controller.

Each job perturbs one scene-level layout field (the commanded pose, not the
true module pose), which triggers full replanning, then runs the complete
assembly with unchanged gates and the independent audit:

- pick_x/pick_y ±10 mm: commanded grasp offset vs the true stored module
  (capture and handover robustness).
- seed_x/seed_y ±2, ±4 mm: commanded install offset vs the true receiving
  port (whether admittance + guide stops seat within the contact window).

The nominal point is the recorded 1 ms baseline, not rerun. Yaw/tilt errors
need model-level injection and are out of scope for this first grid.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MPLBACKEND", "Agg")

import mujoco
import numpy as np

from compliant_docking.assembly.audit import audit
from compliant_docking.assembly.consistency import prepare_models
from compliant_docking.assembly.runner import provenance, write_json
from compliant_docking.assembly.runtime import AssemblyRuntime
from compliant_docking.assembly.simulation import simulate
from compliant_docking.scene import REPO_ROOT, load_scene

BASELINE = REPO_ROOT / "runs/hexframe_main_integration_20261003/full_assembly_cad_precision"
DEFAULT_OUT = REPO_ROOT / "runs/hexframe_grid_20261003"

# (label, field, delta) in metres; nominal baseline covers delta 0.
GRID = [("pick_x_m10", "pick", (-0.010, 0., 0.)), ("pick_x_p10", "pick", (0.010, 0., 0.)),
        ("pick_y_m10", "pick", (0., -0.010, 0.)), ("pick_y_p10", "pick", (0., 0.010, 0.)),
        ("seed_x_m2", "seed", (-0.002, 0., 0.)), ("seed_x_p2", "seed", (0.002, 0., 0.)),
        ("seed_x_m4", "seed", (-0.004, 0., 0.)), ("seed_x_p4", "seed", (0.004, 0., 0.)),
        ("seed_y_m2", "seed", (0., -0.002, 0.)), ("seed_y_p2", "seed", (0., 0.002, 0.)),
        ("seed_y_m4", "seed", (0., -0.004, 0.)), ("seed_y_p4", "seed", (0., 0.004, 0.))]


def run_one(job, out_root):
    label, field, delta = job
    out = out_root / label
    start = time.monotonic()
    if out.exists() and any(out.iterdir()):
        raise ValueError(f"Refusing to overwrite a nonempty run directory: {out}")
    base = load_scene("scenes/hexframe_assembly.yaml")
    layout = dict(pick=tuple(np.asarray(base.pick)+np.array(delta)), seed=tuple(np.asarray(base.seed)+np.array(delta)))
    scene = replace(base, **{field: layout[field]})
    out.mkdir(parents=True, exist_ok=True)
    r = AssemblyRuntime(scene, out)
    model = r.geometry.build_model()
    prepare_models(r)
    spline, phases, planning = r.plan(model)
    if planning["spline_joint_limit_margin_rad"] <= 0:
        return dict(label=label, status="FAIL", reasons=["planned spline exceeds joint limits"],
                    wall_seconds=time.monotonic()-start)
    planning["rejected_unsafe_events"] = r.verify_interlocks(model, spline(0))
    write_json(out/"planning.json", planning)
    write_json(out/"phases.json", [dict(key=p.key, title=p.title, seconds=p.seconds,
                                        end=p.end.tolist(), event=p.event) for p in phases])
    write_json(out/"runtime.json", dict(python=platform.python_version(), platform=platform.platform(),
                mujoco=mujoco.__version__, numpy=np.__version__, physics_timestep_s=r.dt,
                state_record_hz=100, contact_record_hz=round(1/r.dt), video_fps=24,
                controller="MuJoCo bias-compensated joint servo with contact admittance",
                validation_context=f"P1 error grid {label}: {field} offset {delta} m"))
    try:
        _, report = simulate(r, model, spline, phases, planning)
        report["planning"] = planning
        write_json(out/"validation.json", report)
    finally:
        provenance(r)
    audit_result = audit(out) if report["status"] == "PASS" else None
    return dict(label=label, field=field, delta_m=list(delta), status=report["status"],
                audit=audit_result and audit_result["status"], faults=report["faults"],
                lock_time_s=report["lock_time_s"], peak_axial_force_n=report["peak_axial_force_n"],
                max_penetration_mm=report["max_penetration_mm"],
                final_module_error_mm=report["final_module_error_mm"],
                events={e["event"]: e["t"] for e in report["events"]},
                wall_seconds=time.monotonic()-start)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--jobs", type=int, choices=range(1, 7), default=3)
    parser.add_argument("--only", nargs="*", help="restrict to these labels")
    args = parser.parse_args()
    out_root = args.out.resolve()
    out_root.mkdir(parents=True, exist_ok=True)
    jobs = [job for job in GRID if not args.only or job[0] in args.only]
    (out_root/"grid_plan.json").write_text(json.dumps(dict(
        jobs=[dict(label=job[0], field=job[1], delta_m=list(job[2])) for job in jobs],
        baseline=str(BASELINE.relative_to(REPO_ROOT)), gates_unchanged=True,
        nominal_rerun=False, timestep_s=1e-3), indent=1)+"\n")
    results = []
    with ProcessPoolExecutor(max_workers=args.jobs) as executor:
        futures = {executor.submit(run_one, job, out_root): job[0] for job in jobs}
        for future in as_completed(futures):
            record = future.result()
            results.append(record)
            print("GRID", record["label"], record["status"],
                  "audit:", record.get("audit"), flush=True)
    baseline = json.loads((BASELINE/"validation.json").read_text())
    nominal = dict(status=baseline["status"], lock_time_s=baseline["lock_time_s"],
                   peak_axial_force_n=baseline["peak_axial_force_n"],
                   max_penetration_mm=baseline["max_penetration_mm"],
                   final_module_error_mm=baseline["final_module_error_mm"])
    order = {label: i for i, (label, _, _) in enumerate(GRID)}
    results.sort(key=lambda r: order[r["label"]])
    summary = dict(nominal_baseline=nominal, results=results,
                   passes=sum(1 for r in results if r["status"] == "PASS"),
                   total=len(results), continuous_capture_verified=False)
    write_json(out_root/"grid_summary.json", summary)
    print(f"GRID DONE {summary['passes']}/{summary['total']} PASS", flush=True)


if __name__ == "__main__":
    main()
