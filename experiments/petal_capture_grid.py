"""Discrete uncertainty maps; all rollouts reuse the Petal insertion suite.

The grid is experimental setup, never controller knowledge. Results describe
sampled points; no interpolation is used to claim a continuous capture region.
"""
from __future__ import annotations

import ast
import hashlib
import itertools
import json
import shutil
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
from importlib.metadata import version
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pinocchio as pin
from insertion_suite import _json_default

from compliant_docking.plotting import apply_style
from compliant_docking.scene import REPO_ROOT, load_scene

DEFAULT_OUT = REPO_ROOT / "runs/petal_capture_grid_20261003"


def point_key(point):
    def token(value):
        return f"{0. if value == 0 else value:+g}".replace("+", "p").replace("-", "m").replace(".", "d")
    return "grid_x"+token(point[0])+"_y"+token(point[1])+"_yaw"+token(point[2])


def record_name(point, setting="baseline"):
    return point_key(point)+"_released"+("_dt_half" if setting == "dt_half" else "")


def error_tuple(point):
    return point[0]/1000., point[1]/1000., point[2]


def axes_checked(values):
    if not values or not np.isfinite(values).all():
        raise ValueError("Grid axes must contain finite values")
    return sorted(set(float(v) for v in values))


def passed(record):
    return record["assessment"]["status"] == "CANDIDATE_PASS"


def adjacent_pairs(axes):
    """Only actual neighboring grid nodes, never diagonal interpolation."""
    result = []
    for point in itertools.product(*axes):
        for axis in range(3):
            index = axes[axis].index(point[axis])
            if index+1 < len(axes[axis]):
                other = list(point)
                other[axis] = axes[axis][index+1]
                result.append((tuple(point), tuple(other)))
    return result


def midpoint_candidates(pairs, records, limit):
    candidates = []
    for a, b in pairs:
        if a not in records or b not in records or passed(records[a]) == passed(records[b]):
            continue
        point = tuple(round((x+y)/2, 10) for x, y in zip(a, b, strict=True))
        if point not in records:
            # Prefer the nearest observed transition, spread across different
            # failure endpoints, and do not infer sign symmetry.
            failure = b if passed(records[a]) else a
            candidates.append((sum(abs(v) for v in point), point, failure, a, b))
    selected, used_failures = [], set()
    for _, point, failure, a, b in sorted(candidates):
        if failure not in used_failures and point not in [s[0] for s in selected]:
            selected.append((point, a, b))
            used_failures.add(failure)
        if len(selected) >= limit:
            return selected[:limit]
    for _, point, _, a, b in sorted(candidates):
        if point not in [s[0] for s in selected]:
            selected.append((point, a, b))
        if len(selected) >= limit:
            break
    return selected[:limit]


def margin(record):
    g, c = record.get("geometry_evaluation"), record.get("contact_load_gate")
    if g is None or c is None:
        return float("inf")
    return max(g["last_second_max_lateral_mm"]/.5,
               g["last_second_max_phase_error_deg"]/2.,
               g["last_second_max_abs_axial_gap_mm"]/.75,
               g["last_second_max_tilt_deg"]/.5,
               c["peak_contact_force_N"]/40., c["peak_contact_axial_moment_Nm"]/2.,
               1.01 if g["last_second_stop_contact_fraction"] < .95 else 0.)


def boundary_points(records, pairs, limit):
    pool = {p for a, b in pairs if a in records and b in records
            and passed(records[a]) != passed(records[b]) for p in (a, b)}
    pool |= {p for p, r in records.items() if r.get("grid_stage") == "refinement"}
    if not pool:
        pool = set(records)
    good = sorted((p for p in pool if passed(records[p])), key=lambda p: (-margin(records[p]), p))
    bad = sorted((p for p in pool if not passed(records[p]) and records[p].get("geometry_evaluation")),
                 key=lambda p: (margin(records[p]), p))
    selected = []
    for index in range(max(len(good), len(bad))):
        for group in (good, bad):
            if index < len(group) and group[index] not in selected:
                selected.append(group[index])
            if len(selected) >= limit:
                return selected[:limit]
    return selected[:limit]


def source_manifest(base):
    files = sorted((REPO_ROOT/"src").rglob("*.py")) + [
        REPO_ROOT/"experiments/petal_insertion_suite.py", Path(__file__),
        REPO_ROOT/"experiments/run_docking.py", REPO_ROOT/"experiments/insertion_suite.py", base.path]
    return dict(mujoco_version=mujoco.__version__, pinocchio_version=pin.__version__,
                sources={str(p.relative_to(REPO_ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
                assets=json.loads((base.tool.mjcf.parent/"manifest.json").read_text()))


def write_json(path, value):
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, default=_json_default)+"\n")
    temporary.replace(path)


def run_point(out, base, point, setting, stage):
    from petal_insertion_suite import run_case, variant
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


def reusable_sources(previous, current, directory):
    """Keep physics/control and the entire assessment body unchanged."""
    if previous["assets"] != current["assets"] or any(previous[k] != current[k] for k in
                                                       ("mujoco_version", "pinocchio_version")):
        return False
    for name, digest in previous["sources"].items():
        if name != "experiments/petal_insertion_suite.py" and current["sources"].get(name) != digest:
            return False
    old_tree = ast.parse((directory/"source_snapshot/experiments/petal_insertion_suite.py").read_text())
    new_tree = ast.parse((REPO_ROOT/"experiments/petal_insertion_suite.py").read_text())
    old = next(f for f in old_tree.body if isinstance(f, ast.FunctionDef) and f.name == "run_case")
    new = next(f for f in new_tree.body if isinstance(f, ast.FunctionDef) and f.name == "run_case")
    return ast.dump(ast.Module(body=old.body[1:], type_ignores=[])) == ast.dump(
        ast.Module(body=new.body[1:], type_ignores=[]))


def reuse(out, directory, base, current, coarse):
    from petal_insertion_suite import variant
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


def sensitivity(a, b):
    if not a.get("geometry_evaluation") or not b.get("geometry_evaluation"):
        return dict(status="NOT_COMPARABLE", differences={})
    g1, g2 = a["geometry_evaluation"], b["geometry_evaluation"]
    c1, c2 = a["contact_load_gate"], b["contact_load_gate"]
    differences = dict(lateral_mm=abs(g1["last_second_max_lateral_mm"]-g2["last_second_max_lateral_mm"]),
        axial_gap_mm=abs(g1["last_second_max_abs_axial_gap_mm"]-g2["last_second_max_abs_axial_gap_mm"]),
        phase_deg=abs(g1["last_second_max_phase_error_deg"]-g2["last_second_max_phase_error_deg"]),
        peak_force_N=abs(c1["peak_contact_force_N"]-c2["peak_contact_force_N"]),
        peak_axial_moment_Nm=abs(c1["peak_contact_axial_moment_Nm"]-c2["peak_contact_axial_moment_Nm"]))
    stable = (a["assessment"]["status"] == b["assessment"]["status"] and differences["lateral_mm"] <= .1
              and differences["axial_gap_mm"] <= .1 and differences["phase_deg"] <= .1
              and differences["peak_force_N"] <= max(.5, .1*c1["peak_contact_force_N"])
              and differences["peak_axial_moment_Nm"] <= max(.05, .1*c1["peak_contact_axial_moment_Nm"]))
    return dict(status="STABLE_IN_TWO_STEPS" if stable else "SENSITIVE", differences=differences,
                baseline=a["assessment"], half_dt=b["assessment"])


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
    from petal_insertion_suite import preflight
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    xy, yaw = axes_checked(args.xy_mm), axes_checked(args.yaw_deg)
    if min(args.refine, args.boundary_checks) < 0:
        raise ValueError("Refinement and boundary budgets must be nonnegative")
    base = load_scene("scenes/iiwa14_petal_insertion.yaml")
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
