"""iiwa14 PetalDock replacement: matched yaw gains and timestep sensitivity.

Uses the existing robot/control/planner loop. Compact passive diagnostics retain
every solve-time wrench and stop contact without storing millions of events.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MPLBACKEND", "Agg")

from compliant_docking.plotting import COLORS
from compliant_docking.research.petal_trials import (
    feedback_audit as feedback_audit,
)
from compliant_docking.research.petal_trials import (
    preflight as preflight,
)
from compliant_docking.research.petal_trials import (
    run_case as run_case,
)
from compliant_docking.research.petal_trials import (
    summarize as summarize,
)
from compliant_docking.research.petal_trials import (
    variant as variant,
)
from compliant_docking.research.rollout import CASES
from compliant_docking.scene import REPO_ROOT

DEFAULT_OUT = REPO_ROOT / "runs/petal_contact_control_current"
PROFILES = {"stiff": 25., "compliant": .5, "released": .5,
            "lateral_released": .5, "lateral_soft": .5, "lateral_released_slow": .5}
RELEASE_PROFILES = {"released", "lateral_released", "lateral_soft", "lateral_released_slow"}
PROFILE_NAMES = {"stiff": "固定高刚度", "compliant": "固定低刚度", "released": "绕轴释放",
                 "lateral_released": "绕轴与横向释放", "lateral_soft": "横向降至20 N/m",
                 "lateral_released_slow": "横向释放与减速"}
PROFILE_COLORS = {"stiff": COLORS["stiff"], "compliant": COLORS["compliant"], "released": "#009E73",
                  "lateral_released": "#A56CB1", "lateral_soft": "#D55E00",
                  "lateral_released_slow": "#3975A4"}












def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--profile", nargs="+", choices=PROFILES, default=["stiff", "compliant", "released"])
    parser.add_argument("--setting", nargs="+", choices=["baseline", "dt_half", "dt_quarter"], default=["baseline"])
    parser.add_argument("--telemetry", choices=["auto", "full", "core"], default="auto",
                        help="auto: baseline 存完整遥测，dt_half/dt_quarter 只存核心通道（省约一半空间）")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--jobs", type=int, choices=[1, 2, 3], default=1)
    parser.add_argument("--grid", action="store_true", help="Scan released-policy XY/yaw errors")
    parser.add_argument("--lateral-study", action="store_true", help="Matched contact lateral-release validation")
    parser.add_argument("--geometry-study", action="store_true", help="Paired Petal guide geometry validation")
    parser.add_argument("--xy-mm", nargs="+", type=float, default=[-6., 0., 6.])
    parser.add_argument("--yaw-deg", nargs="+", type=float, default=[-15., 0., 15.])
    parser.add_argument("--refine", type=int, default=4, help="Maximum adjacent pass/fail midpoints")
    parser.add_argument("--boundary-checks", type=int, default=4)
    parser.add_argument("--reuse-from", type=Path,
                        help="Reuse identical released-policy records after provenance checks")
    args = parser.parse_args()
    if sum((args.grid,args.lateral_study,args.geometry_study)) > 1:
        parser.error("Choose grid, lateral study or geometry study")
    if args.geometry_study:
        from experiments.models_interfaces.petal_guidance import run_study
        if args.out == DEFAULT_OUT:
            args.out = REPO_ROOT / "runs/petal_guidance_geometry_20261003"
        run_study(args)
        return
    if args.lateral_study:
        from experiments.control.rq2_lateral import run_study
        if args.out == DEFAULT_OUT:
            args.out = REPO_ROOT / "runs/petal_lateral_control_20261003"
        run_study(args)
        return
    if args.grid:
        from experiments.control.capture_range import run_grid
        if args.out == DEFAULT_OUT:
            args.out = REPO_ROOT / "runs/petal_capture_grid_current"
        run_grid(args)
        return
    from experiments.control.rq1_yaw import run_matrix
    run_matrix(args, legacy=True)


if __name__ == "__main__":
    main()
