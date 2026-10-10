"""绕轴释放实验：固定原版 Petal 接口上的绕轴刚度策略配对实验。

主要变量为 stiff/compliant/released；误差来自 CASES，单位 (m, m, deg)。
运行目录保存计划、来源快照、单次试验与汇总；成功退出不表示所有工况通过。
物理步长复核仅在显式选择 setting 时执行，控制周期保持场景声明值。
"""
from __future__ import annotations

import argparse
import json
import shutil
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from compliant_docking.research.petal_trials import preflight, run_case, summarize
from compliant_docking.research.protocols import source_manifest, write_json
from compliant_docking.research.rollout import CASES
from compliant_docking.scene import REPO_ROOT, load_scene

DEFAULT_OUT = REPO_ROOT / "runs/yaw_release_current"
PROFILES = ("stiff", "compliant", "released")

def build_parser():
    """建立固定绕轴释放实验参数接口；默认只运行 baseline 物理步长。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--profile", nargs="+", choices=PROFILES, default=list(PROFILES))
    parser.add_argument("--setting", nargs="+", choices=("baseline", "dt_half", "dt_quarter"), default=["baseline"])
    parser.add_argument("--telemetry", choices=("auto", "full", "core"), default="auto")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--jobs", type=int, choices=(1, 2, 3), default=1)
    return parser

def run_matrix(args, *, legacy=False):
    """校验来源和计划，写入快照并串行或并行执行矩阵；协议不匹配时拒绝续跑。"""
    base = load_scene("scenes/iiwa14_petal_original_insertion.yaml")
    manifest = source_manifest(base)
    plan = dict(cases=args.case, profiles=args.profile, settings=args.setting,
                model="original PetalDock100 V2", primary_factor="yaw stiffness policy",
                fixed_control_period_s=base.se3_impedance.control_period)
    if legacy:
        plan = dict(protocol="legacy-default matrix", model="original PetalDock100 V2",
                    profile_selection="legacy CLI retains historical profiles and incremental runs")
    args.out.mkdir(parents=True, exist_ok=True)
    for filename, value in (("source_manifest.json", manifest), ("study_plan.json", plan)):
        path = args.out / filename
        if path.exists() and json.loads(path.read_text()) != value:
            raise ValueError("Sources or yaw-release plan changed; select a new output directory")
        write_json(path, value)
    for name in manifest["sources"]:
        snapshot = args.out / "source_snapshot" / name
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPO_ROOT / name, snapshot)
    write_json(args.out / "preflight.json", preflight(base))
    pending = []
    for case in args.case:
        for profile in args.profile:
            for setting in args.setting:
                name = f"{case}_{profile}" + ("" if setting == "baseline" else "_" + setting)
                if args.resume and (args.out / f"{name}.json").exists():
                    continue
                pending.append((case, profile, setting))
    # Keyword telemetry preserves the run_case error slot, including workers.
    if args.jobs == 1:
        for case, profile, setting in pending:
            run_case(args.out, base, case, profile, setting,
                     preview=args.preview, telemetry=args.telemetry)
    else:
        with ProcessPoolExecutor(max_workers=args.jobs) as executor:
            futures = [executor.submit(run_case, args.out, base, case, profile, setting,
                       preview=args.preview, telemetry=args.telemetry) for case, profile, setting in pending]
            for future in futures:
                future.result()
    summarize(args.out)

def main(argv=None):
    """解析参数并执行绕轴释放实验；各工况结论读取输出 assessment。"""
    run_matrix(build_parser().parse_args(argv))

if __name__ == "__main__":
    main()
