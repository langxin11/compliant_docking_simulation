"""One scene / one matrix for approach, XY uncertainty and axial compliance.

uv run python experiments/insertion_suite.py
uv run python experiments/insertion_suite.py --case combined --profile compliant
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from contextlib import redirect_stdout
from dataclasses import asdict, replace
from importlib.metadata import version
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pinocchio as pin
from matplotlib.colors import to_rgba

from compliant_docking.cli import _load_run_docking
from compliant_docking.collision_geometry import CONVEX_DIRECTORY, with_convex_interface
from compliant_docking.contact_diagnostics import evaluate_contact_load
from compliant_docking.docking_task import (
    build_docking_trajectory,
    estimated_target_pose,
    target_rotation,
)
from compliant_docking.interface_geometry import InterfaceGeometry, asset_fingerprints
from compliant_docking.plotting import COLORS, apply_style
from compliant_docking.rendering import CAMERAS, apply_render_theme, scene_options
from compliant_docking.scene import REPO_ROOT, load_scene
from compliant_docking.simulation.mujoco_env import MujRobot

CASES = {"nominal": (0.0, 0.0, 0.0), "xy": (0.002, -0.002, 0.0),
         "combined": (0.002, -0.002, 5.0)}
PROFILES = {"stiff": 25.0, "compliant": 0.5}


def variant(base, case, profile):
    """Only axial stiffness changes across a matched control pair.

Truth is used here solely to CREATE an uncertainty experiment. The planner gets
the resulting estimate and never receives the physical target pose.
"""
    dx, dy, yaw = CASES[case]
    truth_R = target_rotation(base.target)
    truth_yaw = np.rad2deg(np.arctan2(truth_R[1, 0], truth_R[0, 0]))
    spec = replace(base.docking, estimate_pos=tuple(base.target.pos + [dx, dy, 0]),
                   estimate_yaw_deg=float(truth_yaw + yaw))
    gains = list(base.se3_impedance.k_diag)
    gains[5] = PROFILES[profile]
    return replace(base, name=f"{base.name}_{case}_{profile}", docking=spec,
                   se3_impedance=replace(base.se3_impedance, k_diag=gains))


def _json_default(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(type(value).__name__)


def save_rollout(out, name, scene, log, telemetry="full"):
    """Persist one rollout. telemetry="core" omits per-step diagnostic_* channels
    (and the raw contacts.npz events) from the file while still returning the full
    array dict, so in-memory gate assessment keeps working; intended for physical
    timestep refinement reruns whose conclusions live in the JSON gates/peaks."""
    if telemetry not in ("full", "core"):
        raise ValueError(f"unknown telemetry mode: {telemetry}")
    samples = log.docking_samples
    arrays = {key: np.asarray([s[key] for s in samples]) for key in samples[0]}
    arrays.update(q=np.asarray(log.joint_angles), qd=np.asarray(log.joint_velocities),
                  desired_position=np.asarray(log.pos_desired), position=np.asarray(log.pos_actual),
                  torque=np.asarray(log.tau_hist), torque_saturated=np.asarray(log.torque_saturated))
    metadata = dict(scene=asdict(scene), gate=log.docking_gate,
                    waypoint_names=log.docking_trajectory.names,
                    waypoint_times=log.docking_trajectory.times.tolist(),
                    waypoints=[pose.homogeneous.tolist() for pose in log.docking_trajectory.poses])
    if log.se3_diagnostics:
        arrays.update({"control_"+key: np.asarray([sample[key] for sample in log.se3_diagnostics])
                       for key in log.se3_diagnostics[0]})
    if hasattr(log, "feedback_convention"):
        metadata["feedback_convention"] = log.feedback_convention
    if log.contact_diagnostics:
        arrays.update({"diagnostic_"+key: np.asarray([sample[key] for sample in log.contact_diagnostics])
                       for key in log.contact_diagnostics[0]})
        metadata["contact_diagnostics"] = log.contact_summary
        metadata["geometry_evaluation"] = log.geometry_evaluation
        if telemetry == "full":
            # Re-save with synchronized poses, solve-time F/T, inertia and contact sum.
            events = {key: np.asarray([row[key] for row in log.contact_events])
                      for key in log.contact_events[0]} if log.contact_events else {"t": np.array([])}
            np.savez_compressed(out / f"{name}.contacts.npz", **events)
    metadata["telemetry"] = telemetry
    written = arrays if telemetry == "full" else {
        key: value for key, value in arrays.items() if not key.startswith("diagnostic_")}
    np.savez_compressed(out / f"{name}.npz", **written)
    (out / f"{name}.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False, default=_json_default)+"\n")
    return arrays


def numerical_variant(base, case, profile, setting):
    scene = variant(base, case, profile)
    if setting == "dt_half":
        scene = replace(scene, physics=replace(scene.physics, timestep=scene.physics.timestep/2))
    elif setting == "sdf_refined":
        scene = replace(scene, physics=replace(scene.physics, sdf_iterations=2*scene.physics.sdf_iterations))
    return scene


def numerical_settings(base, case, sensitivity):
    if not sensitivity or case != "nominal":
        return ["baseline"]
    if base.tool.mjcf.parent == CONVEX_DIRECTORY:
        return ["baseline", "dt_half"]
    return ["baseline", "dt_half", "sdf_refined"]


def assess_record(record):
    scene, data = record["scene"], record["data"]
    duration = float(data["diagnostic_state_t"][-1])
    trajectory_end = build_docking_trajectory(scene.task, scene.docking, scene.trajectory).total_duration
    complete = duration+scene.physics.timestep >= trajectory_end+scene.docking.hold_s
    load = evaluate_contact_load(data["diagnostic_interface_world"], scene.docking,
                                 target_rotation(scene.target), complete=complete)
    reasons = ["benchmark: "+r for r in record["gate"]["reasons"]]
    if record["geometry"]["status"] != "INCOMPLETE":
        reasons += ["geometry: "+r for r in record["geometry"]["reasons"]]
    reasons += load["reasons"]
    balance = record["diagnostics"]
    if (balance["peak_balance_force_residual_N"] > 1e-6
            or balance["peak_balance_moment_residual_Nm"] > 1e-6):
        reasons.append("wrench balance verification")
    candidate = (complete and record["gate"]["status"] == "PASS"
                 and record["geometry"]["status"] == "SEATED_CANDIDATE" and load["status"] == "PASS")
    record["contact_load_gate"] = load
    record["assessment"] = dict(status="CANDIDATE_PASS" if candidate and not reasons else
                                "FAIL" if complete or reasons else "INCOMPLETE",
                                reasons=reasons, locking_verified=False)


def reanalyze_saved(out, base, cases, profiles, sensitivity):
    """Rebuild derived gates/figures only; preserve raw rollouts and run manifest."""
    geometry = InterfaceGeometry(base)
    reference = json.loads((out / "geometry_reference.json").read_text())
    if reference["assets"] != asset_fingerprints(base):
        raise ValueError("Interface assets changed; saved geometry reference cannot be reused")
    geometry.calibration = reference
    records = []
    for case in cases:
        for profile in profiles:
            settings = numerical_settings(base, case, sensitivity)
            for setting in settings:
                name = f"{case}_{profile}" + (f"_{setting}" if setting != "baseline" else "")
                path = out / f"{name}.json"
                metadata = json.loads(path.read_text())
                scene = numerical_variant(base, case, profile, setting)
                saved_scene = json.loads(json.dumps(asdict(scene), default=_json_default))
                if saved_scene != metadata["scene"]:
                    raise ValueError(f"{name}: scene differs from recorded configuration; use the original --scene")
                with np.load(out / f"{name}.npz") as archive:
                    data = {key: archive[key] for key in archive.files}
                if "diagnostic_t" not in data:
                    raise ValueError(f"{name}: no synchronized diagnostics in saved rollout")
                trajectory_end = build_docking_trajectory(scene.task, scene.docking, scene.trajectory).total_duration
                complete = data["diagnostic_state_t"][-1]+scene.physics.timestep >= trajectory_end+scene.docking.hold_s
                if complete:
                    times = data["diagnostic_t"]
                    indices = np.unique([0, *(int(np.argmin(abs(times-t)))
                        for t in np.linspace(times[-1]-1, times[-1], 5))])
                    samples = [dict(t=times[i], position=data["diagnostic_position"][i],
                                    rotation=data["diagnostic_rotation"][i]) for i in indices]
                    metadata["geometry_evaluation"] = geometry.evaluate(samples, scene.target.pos, target_rotation(scene.target))
                else:
                    metadata["geometry_evaluation"] = dict(status="INCOMPLETE", reasons=["incomplete trajectory"])
                record = dict(name=name, case=case, profile=profile, numerics=setting, scene=scene,
                              data=data, gate=metadata["gate"], geometry=metadata["geometry_evaluation"],
                              diagnostics=metadata["contact_diagnostics"])
                assess_record(record)
                metadata.update(contact_load_gate=record["contact_load_gate"], assessment=record["assessment"])
                path.write_text(json.dumps(metadata, indent=2, default=_json_default)+"\n")
                records.append(record)
    return records, reference


def plot_suite(out, records):
    apply_style("report", cjk_first=True)
    cases = list(dict.fromkeys(r["case"] for r in records))
    fig, axes = plt.subplots(len(cases), 4, figsize=(15, 3.2*len(cases)), squeeze=False,
                             constrained_layout=True)
    signals = [("depth_mm", "入口进给 [mm]"), ("yaw_deg", "实际绕轴角 [°]"),
               ("axial_force_N", "轴向力 [N]"), ("axial_moment_Nm", "轴向力矩 [N·m]")]
    case_names = {"nominal": "零新增误差", "xy": "XY 偏差", "combined": "XY + yaw 偏差"}
    profile_names = {"stiff": "高绕轴刚度", "compliant": "低绕轴刚度"}
    for record in records:
        row = cases.index(record["case"])
        data = record["data"]
        for col, (key, label) in enumerate(signals):
            axis = axes[row, col]
            # Never decimate force/moment: single-step peaks determine the gate.
            stride = 1 if col >= 2 else 10
            axis.plot(data["t"][::stride], data[key][::stride],
                      label=f"{profile_names[record['profile']]} · {record['gate']['status']}",
                      color=COLORS[record["profile"]], linewidth=1.2)
            axis.set(xlabel="时间 [s]", ylabel=label, title=case_names[record["case"]])
            axis.grid(alpha=0.7)
            axis.legend(loc="upper left", frameon=False)
    fig.savefig(out / "comparison.png", dpi=200)
    fig.savefig(out / "comparison.pdf")
    plt.close(fig)


def preview_rollout(out, scene, log):
    """Three phases from the actual recorded joint states, with waypoint frames."""
    apply_style("report", cjk_first=True)
    robot = MujRobot(scene.build_mjmodel(), render=False, record=False, eef_body=scene.eef_body)
    apply_render_theme(robot.model, tool_prefix=scene.tool.prefix, target_prefix=scene.target.prefix)
    trajectory = log.docking_trajectory
    path = np.asarray([trajectory.get_state(t)[0] for t in np.linspace(0, trajectory.total_duration, 100)])
    frames = dict(zip(trajectory.names, trajectory.poses, strict=True))
    frames["truth"] = pin.SE3(target_rotation(scene.target), scene.target.pos)
    frames["estimate"] = estimated_target_pose(scene.docking)
    robot.coordinate_frame_labels = False
    times = np.asarray([s["t"] for s in log.docking_samples])
    above_t = trajectory.times[trajectory.names.index("approach")]
    indices = [0, int(np.argmin(abs(times-above_t))), len(times)-1]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.1))
    fig.subplots_adjust(left=.01, right=.99, bottom=.16, top=.87, wspace=.035)
    labels = ["(a) 侧方起点 · 全景", "(b) 估计目标上方", "(c) 插入后的保持状态"]
    with mujoco.Renderer(robot.model, height=600, width=800) as renderer:
        for panel, (axis, index, label, camera_name) in enumerate(
                zip(axes, indices, labels, ["overview", "approach", "contact"], strict=True)):
            robot.data.qpos[:] = log.joint_angles[index]
            mujoco.mj_forward(robot.model, robot.data)
            body = robot.data.body(scene.eef_body)
            actual = pin.SE3(body.xmat.reshape(3, 3), body.xpos)
            robot.set_coordinate_frames(frames if panel == 0 else
                                        {"actual": actual, "truth": frames["truth"]})
            camera = CAMERAS[camera_name].camera()
            # Presets follow target translation; all panels retain one azimuth.
            camera.lookat[:] += scene.target.pos - np.array([0, .5, .35])
            option = scene_options(robot.model, interface_only=panel > 0,
                                   prefixes=(scene.tool.prefix, scene.target.prefix))
            renderer.update_scene(robot.data, camera=camera, scene_option=option)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            robot._add_coordinate_frames(renderer.scene)
            if panel == 0:
                for point in path:
                    robot._append_marker(renderer.scene, point, to_rgba(COLORS["planned"]), .002)
            axis.imshow(renderer.render())
            axis.set_title(f"{label}\nt = {times[index]:.1f} s", fontsize=12, linespacing=1.5)
            axis.axis("off")
    fig.text(.015, .075, "青色：规划路径    坐标轴：X 红 / Y 绿 / Z 蓝", fontsize=10)
    fig.text(.505, .09, "局部图：金色公头 / 蓝色母头（隐藏机械臂）", fontsize=10)
    fig.text(.505, .045, "坐标系：实际末端与真实目标", fontsize=10)
    for suffix in ("png", "pdf"):
        fig.savefig(out / f"scene_preview.{suffix}", dpi=200)
    plt.close(fig)


def write_report(out, records):
    rows = ["# 组合对接实验：接近、定位偏差与绕轴柔顺", "",
            "公头原有 40° 安装相位保留。nominal 表示零新增估计误差，不代表接口天然完美配合。",
            "每个工况只改变绕轴刚度 25 → 0.5 N·m/rad；其他惯量、阻尼、平动/倾斜刚度完全相同。",
            "PASS 仅表示场景声明的插入深度、保持与载荷门禁通过，不等于 CAD 完全就位或锁紧。", "",
            "| 工况 | 绕轴 | 状态 | 入口进给 mm | 横偏 mm | 接触后转角 ° | 峰值力 N | 峰值轴矩 Nm | 原因 |",
            "|---|---|---|---:|---:|---:|---:|---:|---|"]
    for record in records:
        if record.get("numerics", "baseline") != "baseline":
            continue
        g = record["gate"]
        rows.append(f"| {record['case']} | {record['profile']} | {g['status']} | "
                    f"{g['final_depth_mm']:.2f} | {g['final_lateral_mm']:.2f} | "
                    f"{g['contact_yaw_change_deg']:.2f} | {g['peak_force_N']:.2f} | "
                    f"{g['peak_axial_moment_Nm']:.3f} | {', '.join(g['reasons']) or '—'} |")
    rows.extend(["", "![综合曲线](comparison.png)", "",
                 "逐步数据在同名 NPZ；完整场景、路点时间和验收结果在 JSON；运行输出在 log。"])
    diagnostic = [r for r in records if "diagnostics" in r]
    if diagnostic:
        rows.extend(["", "## 几何与接触诊断", "",
                     "几何状态按独立三角网格的同轴紧凑参考评估；SEATED_CANDIDATE 只表示采样位姿符合研究容差，未验证锁紧。",
                     "原进给/载荷门禁保持不变。数值变体单独列出；dt_half 只减半步长，sdf_refined 只加倍 SDF 迭代次数，保持初始点数。",
                     "F/T 与逐接触载荷均记录于积分前求解时刻 t；诊断姿态与其同步，另存积分后 state_t。",
                     "轴矩统一取目标 +Z，参考点为公头根；传感器 = 接触载荷 - 有效惯性载荷（包含重力项）。", "",
                     "实际接触载荷使用相同的 40 N / 2 N·m 限值；F/T 通过而接触载荷超限时，综合评估仍失败。", "",
                     "| 工况 | 绕轴 | 数值设置 | 原门禁 | 几何状态 | 接触门禁 | 综合评估 | 传感器峰矩 Nm | 接触峰矩 Nm | 同步残差 Nm |",
                     "|---|---|---|---|---|---|---|---:|---:|---:|"])
        for r in diagnostic:
            d = r["diagnostics"]
            rows.append(f"| {r['case']} | {r['profile']} | {r.get('numerics', 'baseline')} | "
                        f"{r['gate']['status']} | {r['geometry']['status']} | "
                        f"{r['contact_load_gate']['status']} | {r['assessment']['status']} | "
                        f"{d['peak_sensor_axial_moment_Nm']:.4f} | {d['peak_contact_axial_moment_Nm']:.4f} | "
                        f"{d['peak_balance_moment_residual_Nm']:.2e} |")
        rows.extend(["", "![几何参考](geometry_reference.png)", "",
                     "![载荷分解](load_diagnostics.png)", "",
                     "![力矩尖峰接触位置](contact_peaks.png)", "",
                     "每组 contacts.npz 保存逐接触位置、法向、距离和对公头的载荷；JSON 保存峰值时刻的接触明细。",
                     "geometry_reference.json 包含资产指纹、相位参考、加密误差、研究容差和 SDF 首次接触对照。"])
        if any(r.get("numerics", "baseline") != "baseline" for r in diagnostic):
            rows.extend(["", "![数值敏感性](sensitivity.png)"])
    (out / "report.md").write_text("\n".join(rows)+"\n", encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default="scenes/iiwa14_compliant_insertion.yaml")
    parser.add_argument("--collision", choices=["sdf", "convex"], default="sdf",
                        help="Use validated convex crown fragments; CAD visual and inertial properties are preserved")
    parser.add_argument("--case", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--profile", nargs="+", choices=PROFILES, default=list(PROFILES))
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "runs" / "compliant_insertion")
    parser.add_argument("--quick", action="store_true", help="2 s wiring check; never a completed insertion")
    parser.add_argument("--record", action="store_true", help="Also record native MuJoCo MP4s")
    parser.add_argument("--preview", action="store_true", help="Render a three-phase scene overview")
    parser.add_argument("--diagnose", action="store_true", help="Add synchronized contacts, inertial balance and mesh seating reference")
    parser.add_argument("--sensitivity", action="store_true", help="Compare half dt for nominal, plus refined iterations for SDF (implies --diagnose)")
    parser.add_argument("--reanalyze", action="store_true", help="Rebuild diagnostic gates and figures from saved data; never rerun physics")
    args = parser.parse_args(argv)
    args.diagnose |= args.sensitivity or args.reanalyze
    source_files = sorted((REPO_ROOT / "src").rglob("*.py")) + [Path(__file__), REPO_ROOT / "experiments/run_docking.py"]
    sources_at_start = {str(p.relative_to(REPO_ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in source_files}
    base = load_scene(args.scene)
    if args.collision == "convex":
        base = with_convex_interface(base)
    if base.docking is None or base.se3_impedance is None:
        parser.error("The suite requires a waypoint docking scene with explicit SE(3) gains")
    args.out.mkdir(parents=True, exist_ok=True)
    if args.reanalyze:
        records, reference = reanalyze_saved(args.out, base, args.case, args.profile, args.sensitivity)
        from compliant_docking.diagnostic_plots import plot_diagnostic_suite
        plot_suite(args.out, [r for r in records if r["numerics"] == "baseline"])
        plot_diagnostic_suite(args.out, records, reference)
        write_report(args.out, records)
        analysis_manifest = dict(operation="reanalyze", packages={p: version(p) for p in ["mujoco", "pin", "numpy"]},
                                 sources={str(p.relative_to(REPO_ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                          for p in sorted((REPO_ROOT / "src").rglob("*.py"))+[Path(__file__)]})
        (args.out / "analysis_manifest.json").write_text(json.dumps(analysis_manifest, indent=2)+"\n")
        print(f"Reanalyzed {len(records)} saved rollouts: {args.out / 'report.md'}")
        return 2 if any(r["assessment"]["status"] == "FAIL" for r in records) else 0
    run = _load_run_docking().main
    records = []
    geometry = InterfaceGeometry(base) if args.diagnose else None
    if geometry:
        print("Calibrating independent mesh reference...", flush=True)
        reference = geometry.calibrate()
        reference["collision_onset_comparison"] = geometry.compare_collision_onsets()
        (args.out / "geometry_reference.json").write_text(json.dumps(reference, indent=2)+"\n")
        print(f"Mesh reference phases: {reference['candidates']}", flush=True)
    for case in args.case:
        for profile in args.profile:
            numerics = numerical_settings(base, case, args.sensitivity)
            for setting in numerics:
                scene = numerical_variant(base, case, profile, setting)
                name = f"{case}_{profile}" + (f"_{setting}" if setting != "baseline" else "")
                print(f"Running {name}...", flush=True)
                with (args.out / f"{name}.log").open("w") as stream, redirect_stdout(stream):
                    log = run(scene=scene, dt=scene.physics.timestep, duration=2.0 if args.quick else None,
                              render=False, record=args.record, plot=False, diagnostics=args.diagnose)
                if geometry:
                    log.geometry_evaluation = (geometry.evaluate(log.contact_diagnostics, scene.target.pos, target_rotation(scene.target))
                                               if not args.quick else dict(status="INCOMPLETE", reasons=["quick run"]))
                data = save_rollout(args.out, name, scene, log)
                record = dict(case=case, profile=profile, gate=log.docking_gate, data=data,
                              numerics=setting, name=name, scene=scene)
                if geometry:
                    record.update(diagnostics=log.contact_summary, geometry=log.geometry_evaluation)
                    assess_record(record)
                    path = args.out / f"{name}.json"
                    metadata = json.loads(path.read_text())
                    metadata.update(contact_load_gate=record["contact_load_gate"], assessment=record["assessment"])
                    path.write_text(json.dumps(metadata, indent=2, default=_json_default)+"\n")
                records.append(record)
                print(f"{name}: {log.docking_gate['status']}; "
                      f"depth={log.docking_gate['final_depth_mm']:.2f} mm; "
                      f"contact twist={log.docking_gate['contact_yaw_change_deg']:.2f} deg", flush=True)
                if args.preview and case == args.case[-1] and profile == args.profile[-1] and setting == "baseline":
                    preview_rollout(args.out, scene, log)
    plot_suite(args.out, [r for r in records if r["numerics"] == "baseline"])
    if geometry:
        from compliant_docking.diagnostic_plots import plot_diagnostic_suite
        plot_diagnostic_suite(args.out, records, geometry.calibration)
    write_report(args.out, records)
    sources_at_end = {str(p.relative_to(REPO_ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in source_files}
    manifest = dict(packages={package: version(package) for package in ["mujoco", "pin", "numpy"]},
                    sources=sources_at_start, source_capture="start of run", quick=args.quick, collision=args.collision,
                    sources_changed_during_run={path: dict(before=digest, after=sources_at_end[path])
                                                for path, digest in sources_at_start.items()
                                                if digest != sources_at_end[path]})
    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print(f"Report: {args.out / 'report.md'}")
    return 2 if any(r.get("assessment", r["gate"])["status"] == "FAIL" for r in records) else 0


if __name__ == "__main__":
    raise SystemExit(main())
