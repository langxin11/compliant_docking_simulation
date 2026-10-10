"""横向释放实验：固定绕轴释放后的 XY 保持/释放同点对照。

POINTS 使用 (mm, mm, deg)，run_one 转为单次试验的 (m, m, deg)。
默认 paired 阶段仅运行九点配对；numerics 和 speed 为显式补充阶段。
旧入口 legacy-full 保留历史追加规则。所有阶段写入计划、来源和评价，不修改冻结报告。
"""
from __future__ import annotations

import hashlib
import json
import shutil
from concurrent.futures import ProcessPoolExecutor, as_completed
from importlib.metadata import version
from pathlib import Path

import numpy as np

from compliant_docking.research.cases import POINTS
from compliant_docking.scene import REPO_ROOT, load_scene

PILOT = ("nx6", "combo_ny6_p15", "yaw_n15")
NUMERICAL = ("nx6", "combo_ny6_p15", "yaw_n15")
PROFILES = ("released", "lateral_released")


def record_name(case, profile, setting="baseline"):
    """按工况、策略和步长设置生成记录名。"""
    return f"{case}_{profile}" + ("" if setting == "baseline" else "_"+setting)


def read_records(out):
    """读取目录中的试验 JSON，按 (case, profile, numerics) 索引；跳过非试验元数据。"""
    result = {}
    for path in out.glob("*.json"):
        r = json.loads(path.read_text())
        if "study_error" in r:
            result[(r["case"], r["profile"], r["numerics"])] = r
    return result


def run_one(out, base, case, profile, setting, stage):
    """将 POINTS 的 XY 从 mm 转为 m，执行试验并保存工况与阶段标签。"""
    from compliant_docking.research import protocols as grid
    from compliant_docking.research.petal_trials import run_case
    x, y, yaw = POINTS[case]
    r = run_case(out, base, case, profile, setting, error=(x/1000., y/1000., yaw))
    r.update(study_error=dict(x_mm=x, y_mm=y, yaw_deg=yaw), study_stage=stage)
    grid.write_json(out / (record_name(case, profile, setting)+".json"), r)
    print("LATERAL", case, profile, setting, r["assessment"], flush=True)
    return r


def execute(out, base, jobs, stage, workers):
    """并行运行缺少 JSON 记录的工况；调用方须先校验目录来源与计划。"""
    existing = read_records(out)
    pending = [job for job in jobs if job not in existing]
    if not pending:
        return
    with ProcessPoolExecutor(max_workers=workers) as executor:
        futures = [executor.submit(run_one, out, base, *job, stage) for job in pending]
        for future in as_completed(futures):
            future.result()


def compare_pairs(out):
    """核对同点时间、参考和触发前关节状态完全相同，写配对检查；不一致则报错。"""
    from compliant_docking.research import protocols as grid
    records = read_records(out)
    checks = {}
    for case in POINTS:
        left, right = ((case, p, "baseline") for p in PROFILES)
        if left not in records or right not in records:
            continue
        ra, rb = records[left], records[right]
        trigger = ra["feedback_audit"]["yaw_trigger_s"]
        assert trigger is not None and trigger == rb["feedback_audit"]["yaw_trigger_s"], case
        arrays = []
        for key in (left, right):
            with np.load(out / (record_name(*key)+".npz")) as archive:
                arrays.append({k: archive[k] for k in ("t", "q", "desired_position")})
        a, b = arrays
        np.testing.assert_array_equal(a["t"], b["t"])
        np.testing.assert_array_equal(a["desired_position"], b["desired_position"])
        np.testing.assert_array_equal(a["q"][a["t"] <= trigger], b["q"][b["t"] <= trigger])
        checks[case] = dict(status="MATCHED", before_trigger_max_joint_difference=0.,
                            reference_positions_identical=True, trigger_s=trigger)
        del arrays, a, b
    grid.write_json(out / "matched_pair_checks.json", checks)


def summarize(out):
    """汇总各策略的离散通过数和已有步长配对，写 JSON 并返回数值比较结果。"""
    from compliant_docking.research import protocols as grid
    records = read_records(out)
    checks = {}
    for (case, profile, setting), r in records.items():
        if setting == "baseline" and (case, profile, "dt_half") in records:
            checks[record_name(case, profile)] = grid.sensitivity(r, records[(case, profile, "dt_half")])
        if setting == "dt_half" and (case, profile, "dt_quarter") in records:
            checks[record_name(case, profile, "dt_half")] = grid.sensitivity(r, records[(case, profile, "dt_quarter")])
    grid.write_json(out / "numerical_comparison.json", checks)
    stats = dict(records=len(records), matched_pairs=sum(
        (case, "released", "baseline") in records and (case, "lateral_released", "baseline") in records
        for case in POINTS),
        baseline_passes={profile: sum(r["assessment"]["status"] == "CANDIDATE_PASS"
            for (case, p, setting), r in records.items() if p == profile and setting == "baseline")
            for profile in PROFILES},
        numerical_checks=checks, continuous_capture_region_verified=False)
    grid.write_json(out / "summary.json", stats)
    return checks


def run_study(args):
    """按 paired/numerics/legacy-full 调度并持久化固定协议；新入口默认止于 paired。"""
    from compliant_docking.research import protocols as grid
    from compliant_docking.research.petal_trials import preflight
    stage = getattr(args, "stage", "legacy-full")
    if stage == "speed":
        return run_speed(args)
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    base = load_scene("scenes/iiwa14_petal_original_insertion.yaml")
    plan = dict(points=POINTS, pilot=PILOT, profiles=PROFILES, numerical_cases=NUMERICAL,
                physics_dt_s=base.physics.timestep, control_period_s=.0005, feedback_delay_s=.0005,
                lateral_before_N_m=80., lateral_after_N_m=0., release_s=.25,
                quarter_budget=3, slower_point_budget=1, slower_settings=["dt_half", "dt_quarter"],
                claim="matched discrete trials; no truth in controller; no continuous capture claim")
    manifest = grid.source_manifest(base)
    manifest["sources"][str(Path(__file__).relative_to(REPO_ROOT))] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    for filename, value in (("source_manifest.json", manifest), ("study_plan.json", plan)):
        path = out / filename
        normalized = json.loads(json.dumps(value))
        if path.exists() and json.loads(path.read_text()) != normalized:
            raise ValueError("Study sources or plan changed; use a new output directory")
        grid.write_json(path, value)
    for name in manifest["sources"]:
        snapshot = out / "source_snapshot" / name
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPO_ROOT / name, snapshot)
    for name in ["tests/test_contact_yaw.py", "tests/test_petal_grid.py", "tests/test_petal_integration.py",
                 "pyproject.toml", "uv.lock"]:
        snapshot = out / "source_snapshot" / name
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPO_ROOT / name, snapshot)
    grid.write_json(out / "environment.json", {p: version(p) for p in ("mujoco", "pin", "numpy", "scipy", "matplotlib")})
    grid.write_json(out / "preflight.json", preflight(base))
    if stage == "numerics" and any((case, profile, "baseline") not in read_records(out)
                                  for case in POINTS for profile in PROFILES):
        raise ValueError("Run the paired stage in this directory before numerical checks")
    pilot_jobs = [(case, profile, "baseline") for case in PILOT for profile in PROFILES]
    execute(out, base, pilot_jobs, "pilot", args.jobs)
    compare_pairs(out)
    summarize(out)
    extension = [(case, profile, "baseline") for case in POINTS if case not in PILOT for profile in PROFILES]
    execute(out, base, extension, "extension", args.jobs)
    compare_pairs(out)
    if stage == "paired":
        summarize(out)
        return
    execute(out, base, [(c, "lateral_released", "dt_half") for c in NUMERICAL], "half_check", args.jobs)
    checks = summarize(out)
    quarter = [case for case in NUMERICAL if checks[record_name(case, "lateral_released")]["status"] == "SENSITIVE"]
    path = out / "quarter_plan.json"
    if path.exists():
        assert json.loads(path.read_text()) == quarter
    grid.write_json(path, quarter)
    execute(out, base, [(c, "lateral_released", "dt_quarter") for c in quarter], "quarter_check", args.jobs)
    checks = summarize(out)
    records = read_records(out)
    if stage == "numerics":
        return
    slow = [case for case in quarter if checks[record_name(case, "lateral_released", "dt_half")]["status"] == "SENSITIVE"
            and records[(case, "lateral_released", "dt_quarter")]["assessment"]["status"] == "CANDIDATE_PASS"]
    # The slower trial isolates one trajectory factor at one already seated
    # point. It never turns an unseated point into an accepted numeric result.
    slow = sorted(slow, key=lambda case: (case != "yaw_n15", case))[:1]
    path = out / "slow_plan.json"
    if path.exists():
        assert json.loads(path.read_text()) == slow
    grid.write_json(path, slow)
    execute(out, base, [(c, "lateral_released_slow", s) for c in slow for s in ("dt_half", "dt_quarter")],
            "slow_factor", args.jobs)
    summarize(out)


def run_speed(args):
    """Separate paired insertion-speed factor; never part of lateral-release primary pairs."""
    from compliant_docking.research import protocols as grid
    from compliant_docking.research.petal_trials import preflight
    base = load_scene("scenes/iiwa14_petal_original_insertion.yaml")
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    plan = dict(primary_factor="insertion_speed", case=args.speed_case,
                profiles=["lateral_released", "lateral_released_slow"], settings=args.setting,
                speeds_m_s=[base.docking.insertion_speed, base.docking.insertion_speed / 2],
                control_period_s=base.se3_impedance.control_period)
    manifest = grid.source_manifest(base)
    for filename, value in (("source_manifest.json", manifest), ("study_plan.json", plan)):
        path = out / filename
        if path.exists() and json.loads(path.read_text()) != value:
            raise ValueError("Speed-factor sources or plan changed; use a separate new directory")
        grid.write_json(path, value)
    for name in manifest["sources"]:
        snapshot = out / "source_snapshot" / name
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPO_ROOT / name, snapshot)
    grid.write_json(out / "preflight.json", preflight(base))
    execute(out, base, [(args.speed_case, profile, setting)
                      for profile in plan["profiles"] for setting in args.setting], "speed_factor", args.jobs)
    summarize(out)

def build_parser():
    """建立配对及显式补充阶段参数；speed 使用独立目录与因素。"""
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("paired", "numerics", "speed"), default="paired")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--jobs", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--speed-case", choices=POINTS, default="yaw_n15")
    parser.add_argument("--setting", nargs="+", choices=("baseline", "dt_half", "dt_quarter"), default=["baseline"])
    return parser

def main(argv=None):
    """校验参数适用阶段后运行；跨阶段使用 speed 参数时拒绝执行。"""
    args = build_parser().parse_args(argv)
    if args.stage != "speed" and (args.speed_case != "yaw_n15" or args.setting != ["baseline"]):
        raise ValueError("--speed-case/--setting apply only to the independent speed factor")
    run_study(args)

if __name__ == "__main__":
    main()
