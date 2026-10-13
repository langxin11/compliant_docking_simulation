"""HexFrame P0: full 53 s assembly at 0.5 ms physics, independent of the formal scene.

The formal CLI keeps its validated 1 ms controller locked (config and cli
rejections unchanged). This entry builds the same runtime via
dataclasses.replace(scene, timestep=5e-4), runs simulate() + audit() with the
unchanged gates, and reports drift of every acceptance-relevant quantity
against the recorded 1 ms baseline.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import time
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
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
DEFAULT_OUT = REPO_ROOT / "runs/hexframe_halfstep_20261003"
HALF_DT = 5e-4


def run_halfstep(out: Path, dt: float = HALF_DT):
    start = time.monotonic()
    if out.exists() and any(out.iterdir()):
        raise ValueError(f"Refusing to overwrite a nonempty run directory: {out}")
    scene = replace(load_scene("scenes/hexframe_assembly.yaml"), timestep=dt)
    out.mkdir(parents=True, exist_ok=True)
    r = AssemblyRuntime(scene, out)
    model = r.geometry.build_model()
    prepare_models(r)
    spline, phases, planning = r.plan(model)
    if planning["spline_joint_limit_margin_rad"] <= 0:
        raise RuntimeError("Planned spline exceeds joint limits")
    planning["rejected_unsafe_events"] = r.verify_interlocks(model, spline(0))
    write_json(out/"planning.json", planning)
    write_json(out/"phases.json", [dict(key=p.key, title=p.title, seconds=p.seconds,
                                        end=p.end.tolist(), event=p.event) for p in phases])
    write_json(out/"runtime.json", dict(python=platform.python_version(), platform=platform.platform(),
                mujoco=mujoco.__version__, numpy=np.__version__, physics_timestep_s=r.dt,
                state_record_hz=100, contact_record_hz=round(1/r.dt), video_fps=24,
                controller="MuJoCo bias-compensated joint servo with contact admittance",
                pinocchio_role="same-source IK and nominal rigid-body consistency checks",
                validation_context="P0 half-timestep validation entry; formal scene unchanged"))
    try:
        _, report = simulate(r, model, spline, phases, planning)
        report["planning"] = planning
        write_json(out/"validation.json", report)
    finally:
        provenance(r)
    audit_result = audit(out) if report["status"] == "PASS" else None
    comparison = compare(out, BASELINE, dt)
    write_json(out/"halfstep_comparison.json", comparison)
    write_json(out/"halfstep_result.json", dict(
        status=report["status"], audit=audit_result, physics_timestep_s=dt,
        baseline=str(BASELINE.relative_to(REPO_ROOT)), comparison=comparison,
        wall_seconds=time.monotonic()-start))
    print(json.dumps(dict(status=report["status"], audit=audit_result and audit_result["status"],
                          drift=comparison["drift"]), ensure_ascii=False), flush=True)
    return report, audit_result, comparison


def compare(out: Path, baseline: Path, dt: float):
    new = json.loads((out/"validation.json").read_text())
    old = json.loads((baseline/"validation.json").read_text())
    fields = ["first_contact_s", "lock_time_s", "peak_axial_force_n",
              "max_penetration_mm", "final_module_error_mm"]
    drift = {k: (None if new[k] is None or old[k] is None else round(new[k]-old[k], 6))
             for k in fields}
    events_new = {e["event"]: e["t"] for e in new["events"]}
    events_old = {e["event"]: e["t"] for e in old["events"]}
    event_drift = {k: round(events_new[k]-events_old[k], 6) for k in events_old}
    return dict(baseline_timestep_s=1e-3, validation_timestep_s=dt,
                baseline_status=old["status"], validation_status=new["status"],
                drift=drift, event_time_drift_s=event_drift,
                gates_unchanged=True,
                scope="fixed-base zero-gravity; same geometry, gains, gates and phase durations")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--dt", type=float, default=HALF_DT)
    args = parser.parse_args()
    run_halfstep(args.out.resolve(), args.dt)


if __name__ == "__main__":
    main()
