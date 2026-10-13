"""Discrete uncertainty maps; all rollouts reuse the Petal insertion suite.

The grid is experimental setup, never controller knowledge. Results describe
sampled points; no interpolation is used to claim a continuous capture region.
"""
from __future__ import annotations

import hashlib
import itertools
import json
import shutil
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
from importlib.metadata import version
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from compliant_docking.plotting import apply_style
from compliant_docking.research.protocols import (
    adjacent_pairs,
    axes_checked,
    boundary_points,
    error_tuple,
    midpoint_candidates,
    passed,
    point_key,
    record_name,
    reusable_sources,
    sensitivity,
    source_manifest,
    write_json,
)
from compliant_docking.research.rollout import _json_default
from compliant_docking.scene import REPO_ROOT, load_scene

DEFAULT_OUT = REPO_ROOT / "runs/petal_capture_grid_20261003"
























def run_point(out, base, point, setting, stage):
    from compliant_docking.research.petal_trials import run_case, variant
    name = record_name(point, setting)
    try:
        record = run_case(out, base, point_key(point), "released", setting,
                          error=error_tuple(point))
    except ValueError as error:
        if "unreachable" not in str(error).lower():
            raise
        record = dict(scene=asdict(variant(base, point_key(point), "released", error_tuple(point))),
                      case=point_key(point), profile="released", numerics=setting,
                      assessment=dict(status="FAIL", reasons=["planning: "+str(error)], locking_verified=False),
                      geometry_evaluation=None, contact_load_gate=None, simulation_warnings={},
                      execution="INFEASIBLE_PLAN")
    record.update(grid_error=dict(x_mm=point[0], y_mm=point[1], yaw_deg=point[2]), grid_stage=stage)
    write_json(out/f"{name}.json", record)
    return point, record


def read_records(out, setting="baseline"):
    records = {}
    for path in out.glob("grid_*.json"):
        record = json.loads(path.read_text())
        if record.get("numerics") == setting and "grid_error" in record:
            e = record["grid_error"]
            records[(e["x_mm"], e["y_mm"], e["yaw_deg"])] = record
    return records


def execute(out, base, points, setting, stage, jobs):
    pending = [p for p in points if p not in read_records(out, setting)]
    if not pending:
        return
    with ProcessPoolExecutor(max_workers=jobs) as executor:
        futures = [executor.submit(run_point, out, base, p, setting, stage) for p in pending]
        for future in as_completed(futures):
            point, record = future.result()
            print(f"GRID {stage} {point} {setting}: {record['assessment']}", flush=True)




def reuse(out, directory, base, current, coarse):
    from compliant_docking.research.petal_trials import variant
    previous = json.loads((directory/"source_manifest.json").read_text())
    if not reusable_sources(previous, current, directory):
        raise ValueError("Reuse rejected: physics, controller or assessment changed")
    for path in directory.glob("*.json"):
        record = json.loads(path.read_text())
        if "assessment" not in record or record.get("profile") != "released":
            continue
        old_scene = record["scene"]
        dx = 1000*(old_scene["docking"]["estimate_pos"][0]-old_scene["target"]["pos"][0])
        dy = 1000*(old_scene["docking"]["estimate_pos"][1]-old_scene["target"]["pos"][1])
        point = (round(dx, 10), round(dy, 10), old_scene["docking"]["estimate_yaw_deg"])
        if point not in coarse and record["numerics"] != "baseline":
            continue
        setting = record["numerics"]
        expected = variant(base, point_key(point), "released", error_tuple(point))
        if setting == "dt_half":
            from dataclasses import replace
            expected = replace(expected, physics=replace(expected.physics, timestep=expected.physics.timestep/2))
        expected = json.loads(json.dumps(asdict(expected), default=_json_default))
        comparable = dict(old_scene)
        comparable["name"] = expected["name"]
        if comparable != expected:
            raise ValueError("Reuse rejected: effective scene differs")
        name = record_name(point, setting)
        if (out/f"{name}.json").exists():
            continue
        hashes = {}
        for extension in (".npz", ".contacts.npz", ".log", ".json"):
            source = path.with_name(path.stem+extension)
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            hashes[source.name] = digest
            if extension != ".json":
                shutil.copy2(source, out/f"{name}{extension}")
        record.update(scene=expected, case=point_key(point),
                      grid_error=dict(x_mm=point[0], y_mm=point[1], yaw_deg=point[2]),
                      grid_stage="coarse" if point in coarse else "anchor",
                      provenance=dict(status="REUSED_IDENTICAL_EFFECTIVE_SCENE",
                          source_directory=str(directory), original_files_sha256=hashes,
                          source_manifest_sha256=hashlib.sha256((directory/"source_manifest.json").read_bytes()).hexdigest(),
                          verification="Controller/physics/loop/assets and assessment body identical; scene identical except experiment label"))
        write_json(out/f"{name}.json", record)
        print("REUSED", point, setting, flush=True)




def summarize(out, plan, coarse, pairs):
    records, halves = read_records(out), read_records(out, "dt_half")
    checks = {record_name(p): sensitivity(records[p], halves[p]) for p in halves if p in records}
    write_json(out/"boundary_numerics.json", checks)
    apply_style("report", cjk_first=True)
    xy, yaw = plan["xy_mm"], plan["yaw_deg"]
    fig, axes = plt.subplots(1, len(yaw), figsize=(4.4*len(yaw), 4.8), squeeze=False, constrained_layout=True)
    from matplotlib.colors import ListedColormap
    for ax, angle in zip(axes.flat, yaw, strict=True):
        cells = np.full((len(xy), len(xy)), np.nan)
        for iy, y in enumerate(xy):
            for ix, x in enumerate(xy):
                r = records.get((x, y, angle))
                if r is not None:
                    cells[iy, ix] = int(passed(r))
        ax.imshow(cells, origin="lower", cmap=ListedColormap(["#DA795F", "#71BBA5"]), vmin=0, vmax=1,
                  extent=(-.5, len(xy)-.5, -.5, len(xy)-.5), interpolation="nearest")
        for iy, y in enumerate(xy):
            for ix, x in enumerate(xy):
                r = records.get((x, y, angle))
                text = "未运行" if r is None else "通过" if passed(r) else "未通过"
                ax.text(ix, iy, text, ha="center", va="center", fontsize=10)
        ax.set(xticks=range(len(xy)), yticks=range(len(xy)), xticklabels=xy, yticklabels=xy,
               xlabel="X 估计误差 [mm]", ylabel="Y 估计误差 [mm]", title=f"偏航误差 {angle:g}°")
    fig.suptitle("离散采样结果：绿色通过，红色未通过（色块不代表连续捕获范围）", fontsize=12)
    for extension in ("png", "pdf"):
        fig.savefig(out/f"capture_grid.{extension}", dpi=170)
    plt.close(fig)
    transitions = [(a, b) for a, b in pairs if a in records and b in records and passed(records[a]) != passed(records[b])]
    stats = dict(coarse_points=len(coarse), coarse_completed=sum(p in records for p in coarse),
                 coarse_passes=sum(p in records and passed(records[p]) for p in coarse),
                 total_baseline_points=len(records), half_dt_points=len(halves),
                 adjacent_status_transitions=[dict(a=a, b=b) for a, b in transitions],
                 boundary_checks=checks, continuous_capture_region_verified=False)
    write_json(out/"summary.json", stats)
    lines = ["# PetalDock100 XY／偏航误差离散扫描", "",
             f"粗网格 {stats['coarse_completed']}/{len(coarse)} 个采样点完成，{stats['coarse_passes']} 个达到落座候选要求。",
             "控制周期/反馈延迟 0.5 ms，物理步长 0.5 ms；复核为 0.25 ms。零重力，同一起点和接触释放策略，验收门槛保持原样。",
             "每个色块只表示一个实际运行点；不据此证明格子内部或连续区域。anchor 为已有验证点，outer 为全通过时的更大误差探针，refinement 为已运行通过/失败点连线的中点。", "",
             "![离散网格](capture_grid.png)", "",
             "| X mm | Y mm | yaw ° | 阶段 | 综合结论 | 横向 mm | 相位 ° | 轴向间隙 mm | 止挡占比 | 力 N | 轴矩 N·m | 原因 |",
             "|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---:|---|"]
    for p, r in sorted(records.items(), key=lambda item: (item[0][2], item[0][1], item[0][0])):
        g, c = r.get("geometry_evaluation"), r.get("contact_load_gate")
        numeric = (f"{g['last_second_max_lateral_mm']:.4f} | {g['last_second_max_phase_error_deg']:.4f} | "
                   f"{g['last_second_max_abs_axial_gap_mm']:.4f} | {g['last_second_stop_contact_fraction']:.0%} | "
                   f"{c['peak_contact_force_N']:.3f} | {c['peak_contact_axial_moment_Nm']:.4f}") if g and c else "— | — | — | — | — | —"
        lines.append(f"| {p[0]:g} | {p[1]:g} | {p[2]:g} | {r['grid_stage']} | {r['assessment']['status']} | {numeric} | {'; '.join(r['assessment']['reasons']) or '—'} |")
    lines += ["", "## 边界时间步复核", "",
              "沿用原比较阈值：状态相同，横向/轴向差 ≤0.1 mm、相位差 ≤0.1°；峰值力差 ≤max(0.5 N, 10%)，峰值轴矩差 ≤max(0.05 N·m, 10%)。"]
    for name, check in checks.items():
        lines.append(f"- {name}: {check['status']}；{check['differences']}")
    if not transitions:
        lines.append("扫描点之间未发现通过/失败转换，不能据此确定真实捕获边界。")
    lines += ["", "JSON/NPZ 保留逐步数据和独立验收；provenance 记录复用样本的有效配置/源码及原始数据哈希。",
              "未锁紧；重力、起点、倾斜、控制频率或接触参数改变后须重新验证。"]
    (out/"report.md").write_text("\n".join(lines)+"\n")


def run_grid(args):
    from compliant_docking.research.petal_trials import preflight
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    xy, yaw = axes_checked(args.xy_mm), axes_checked(args.yaw_deg)
    if min(args.refine, args.boundary_checks) < 0:
        raise ValueError("Refinement and boundary budgets must be nonnegative")
    base = load_scene("scenes/iiwa14_petal_original_insertion.yaml")
    plan = dict(xy_mm=xy, yaw_deg=yaw, refine=args.refine, boundary_checks=args.boundary_checks,
                profile="released", control_period_s=.0005, physics_dt_s=base.physics.timestep,
                all_pass_outer_probe_budget=4,
                claim="discrete points only; no symmetry/interpolation assumptions")
    manifest = source_manifest(base)
    for filename, value in (("source_manifest.json", manifest), ("grid_plan.json", plan)):
        path = out/filename
        if path.exists() and json.loads(path.read_text()) != value:
            raise ValueError("Sources or grid plan changed; use a new output directory")
        write_json(path, value)
    for name in manifest["sources"]:
        target = out/"source_snapshot"/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPO_ROOT/name, target)
    write_json(out/"preflight.json", preflight(base))
    write_json(out/"environment.json", {p: version(p) for p in ("mujoco", "pin", "numpy", "scipy", "matplotlib")})
    coarse = [tuple(p) for p in itertools.product(xy, xy, yaw)]
    if args.reuse_from:
        reuse(out, args.reuse_from.resolve(), base, manifest, set(coarse))
    pairs = adjacent_pairs([xy, xy, yaw])
    execute(out, base, coarse, "baseline", "coarse", args.jobs)
    records = read_records(out)
    outer_path = out/"outer_plan.json"
    if outer_path.exists():
        outer_pairs = [(tuple(v["inner"]), tuple(v["outer"])) for v in json.loads(outer_path.read_text())]
    else:
        outer_pairs = []
        if all(passed(records[p]) for p in coarse) and 0. in xy and 0. in yaw:
            seeds = [(xy[0], xy[0], 0.), (xy[-1], xy[-1], 0.),
                     (0., 0., yaw[0]), (0., 0., yaw[-1])]
            outer_pairs = [(p, tuple(2*v for v in p)) for p in seeds
                           if any(v != 0. for v in p)]
        write_json(outer_path, [dict(inner=a, outer=b) for a, b in outer_pairs])
    execute(out, base, [b for _, b in outer_pairs], "baseline", "outer", args.jobs)
    pairs += outer_pairs
    records = read_records(out)
    refinement_path = out/"refinement_plan.json"
    if refinement_path.exists():
        selected = [(tuple(v["point"]), tuple(v["a"]), tuple(v["b"]))
                    for v in json.loads(refinement_path.read_text())]
    else:
        selected = midpoint_candidates(pairs, records, args.refine)
        write_json(refinement_path, [dict(point=p, a=a, b=b) for p, a, b in selected])
    execute(out, base, [p for p, _, _ in selected], "baseline", "refinement", args.jobs)
    records = read_records(out)
    boundary_path = out/"boundary_plan.json"
    if boundary_path.exists():
        checks = [tuple(p) for p in json.loads(boundary_path.read_text())]
    else:
        checks = boundary_points(records, pairs, args.boundary_checks)
        write_json(boundary_path, checks)
    execute(out, base, checks, "dt_half", "boundary_check", args.jobs)
    summarize(out, plan, coarse, pairs)


def build_parser():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "runs/petal_capture_grid_current")
    parser.add_argument("--jobs", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--xy-mm", nargs="+", type=float, default=[-6., 0., 6.])
    parser.add_argument("--yaw-deg", nargs="+", type=float, default=[-15., 0., 15.])
    parser.add_argument("--refine", type=int, default=4)
    parser.add_argument("--boundary-checks", type=int, default=4)
    parser.add_argument("--reuse-from", type=Path)
    parser.add_argument("--resume", action="store_true", help="Continue only under identical sources and plan")
    return parser

def main(argv=None):
    run_grid(build_parser().parse_args(argv))

if __name__ == "__main__":
    main()
