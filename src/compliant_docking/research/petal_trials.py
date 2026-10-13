"""固定场景的 Petal 单次试验、评价与反馈审计，共享给各实验入口。

误差 error=(dx_m, dy_m, yaw_deg) 仅注入目标估计；目标真值用于评分与显示。
run_case 生成配置、执行完整流程并写入运行目录；物理步长复核固定控制周期和反馈延迟。
CANDIDATE_PASS 表示声明条件下的落座候选，不表示真实锁紧或连续捕获范围。
"""
from __future__ import annotations

import json
import os
import time
from contextlib import redirect_stdout
from dataclasses import replace

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pinocchio as pin

from compliant_docking.contact_diagnostics import evaluate_contact_load
from compliant_docking.docking_task import target_rotation
from compliant_docking.models import load_assembled_pin_model
from compliant_docking.petal_geometry import evaluate_petal_seating
from compliant_docking.plotting import COLORS, apply_style

from .rollout import CASES, preview_rollout, save_rollout


def _load_run_docking():
    """Lazy library import after headless environment setup."""
    from compliant_docking.orchestration import run_docking
    return run_docking


PROFILES = {"stiff": 25., "compliant": .5, "released": .5,
            "lateral_released": .5, "lateral_soft": .5, "lateral_released_slow": .5}

RELEASE_PROFILES = {"released", "lateral_released", "lateral_soft", "lateral_released_slow"}

PROFILE_NAMES = {"stiff": "固定高刚度", "compliant": "固定低刚度", "released": "绕轴释放",
                 "lateral_released": "绕轴与横向释放", "lateral_soft": "横向降至20 N/m",
                 "lateral_released_slow": "横向释放与减速"}

PROFILE_COLORS = {"stiff": COLORS["stiff"], "compliant": COLORS["compliant"], "released": "#009E73",
                  "lateral_released": "#A56CB1", "lateral_soft": "#D55E00",
                  "lateral_released_slow": "#3975A4"}

def variant(base, case, profile, error=None):
    """Create uncertainty here; the planner still sees only the estimate.

    error is (dx_m, dy_m, yaw_deg); preset cases keep their historical values.
    """
    dx, dy, yaw = CASES[case] if error is None else error
    if not np.isfinite([dx, dy, yaw]).all():
        raise ValueError("Estimation errors must be finite")
    truth_R = target_rotation(base.target)
    truth_yaw = np.rad2deg(np.arctan2(truth_R[1, 0], truth_R[0, 0]))
    docking = replace(base.docking, estimate_pos=tuple(base.target.pos+[dx, dy, 0.]),
                      estimate_yaw_deg=float(truth_yaw+yaw))
    gains = list(base.se3_impedance.k_diag)
    gains[5] = PROFILES[profile]
    if profile in RELEASE_PROFILES and base.se3_impedance.contact_yaw is None:
        raise ValueError("released profile requires a declared contact_yaw policy")
    policy = base.se3_impedance.contact_yaw if profile in RELEASE_PROFILES else None
    if profile == "released":
        policy = replace(policy, lateral_stiffness_after=None)
    if profile.startswith("lateral_"):
        policy = replace(policy, lateral_stiffness_after=20. if profile == "lateral_soft" else 0.)
    if profile == "lateral_released_slow":
        docking = replace(docking, insertion_speed=base.docking.insertion_speed/2)
    return replace(base, name=f"{base.name}_{case}_{profile}", docking=docking,
                   se3_impedance=replace(base.se3_impedance, k_diag=gains,
                       contact_yaw=policy))

def feedback_audit(log):
    """Check saved feedback against the originating sensor solve, independently."""
    values = log.se3_diagnostics
    updates = [s for s in values if s["control_update"] and s["control_t"] > 0.]
    period = log.feedback_convention["control_period_s"]
    age_error = max(abs(s["feedback_age_s"]-period) for s in updates)
    transform_error = 0.
    for s in updates:
        f, n = s["feedback_world_at_origin"][:3], s["feedback_world_at_origin"][3:]
        R, p = s["control_rotation"], s["control_position"]
        expected = np.r_[R.T @ f, R.T @ (n+np.cross(s["feedback_origin"]-p, f))]
        transform_error = max(transform_error, float(np.max(abs(expected-s["feedback_body"]))))
    pose_error = max(float(np.max(abs(p-s["position"])))
                     for p, s in zip(log.pos_actual, log.contact_diagnostics, strict=True))
    first_stop = next((s["t"] for s in log.contact_diagnostics if s["stop_contact_count"]), None)
    last = values[-1]
    return dict(status="PASS" if max(age_error, transform_error, pose_error) < 1e-10 else "FAIL",
                control_updates=len(updates)+1, max_feedback_age_error_s=float(age_error),
                max_wrench_transform_error=float(transform_error), max_solve_pose_error_m=pose_error,
                control_period_s=period, final_yaw_stiffness_Nm_rad=last["yaw_stiffness"],
                yaw_trigger_s=None if last["yaw_trigger_t"] < 0 else last["yaw_trigger_t"],
                final_lateral_stiffness_N_m=last["lateral_stiffness"].tolist(),
                first_loaded_stop_s=first_stop)

# 检查初值及固定随机位形的运动学/惯量一致性；返回数据，不执行对接。
def preflight(scene):
    model, robot = scene.build_mjmodel(), load_assembled_pin_model(scene)
    data, pin_data = mujoco.MjData(model), robot.createData()
    fid = robot.getFrameId(scene.robot.ee_frame)
    rng = np.random.default_rng(15)
    errors = []
    for q in [scene.task.ik_guess, *rng.uniform(-1., 1., size=(10, 7))]:
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        pin.forwardKinematics(robot, pin_data, q)
        pin.updateFramePlacements(robot, pin_data)
        mass = np.zeros((7, 7))
        mujoco.mj_fullM(model, data, mass)
        errors.append([
            np.max(abs(pin_data.oMf[fid].translation-data.body(scene.eef_body).xpos)),
            np.max(abs(pin_data.oMf[fid].rotation-data.body(scene.eef_body).xmat.reshape(3, 3))),
            np.max(abs(np.array(pin.crba(robot, pin_data, q))-mass)),
        ])
    data.qpos[:] = scene.task.ik_guess
    mujoco.mj_forward(model, data)
    maximum = np.max(errors, axis=0)
    assert np.all(maximum < 1e-10), maximum
    assert data.ncon == 0, "Initial pose has contact"
    assert np.all(scene.task.ik_guess > robot.lowerPositionLimit+.05)
    assert np.all(scene.task.ik_guess < robot.upperPositionLimit-.05)
    return dict(status="PASS", configurations=len(errors), initial_contacts=data.ncon,
                max_position_error_m=float(maximum[0]), max_rotation_matrix_error=float(maximum[1]),
                max_mass_matrix_error=float(maximum[2]),
                tool_mass_kg=float(model.body_mass[model.body(scene.eef_body).id]),
                tolerance=float(model.opt.tolerance), iterations=int(model.opt.iterations),
                lower_position_limits=robot.lowerPositionLimit.tolist(),
                upper_position_limits=robot.upperPositionLimit.tolist(),
                full_tool_inertia_preserved=True, default_scene_replaced=False)

# 执行完整试验并持久化；error 单位 (m, m, deg)，setting 只选择声明的时间方案。
def run_case(out, base, case, profile, setting, preview=False, error=None, telemetry="auto"):
    # dt refinement reruns re-verify peaks/gates only; conclusions live in the JSON
    # records, so per-step diagnostic channels are not persisted for them by default.
    if telemetry not in ("auto", "full", "core"):
        raise ValueError(f"unknown telemetry mode: {telemetry}")
    effective = telemetry if telemetry != "auto" else ("core" if setting != "baseline" else "full")
    scene = variant(base, case, profile, error)
    if setting not in ("baseline", "dt_half", "dt_quarter"):
        raise ValueError("Unknown physical timestep setting")
    divisor = {"baseline": 1, "dt_half": 2, "dt_quarter": 4}[setting]
    scene = replace(scene, physics=replace(scene.physics, timestep=scene.physics.timestep/divisor))
    name = f"{case}_{profile}" + ("" if setting == "baseline" else "_"+setting)
    start = time.monotonic()
    print(f"START {name}", flush=True)
    with (out / f"{name}.log").open("w") as stream, redirect_stdout(stream):
        log = _load_run_docking().main(scene=scene, dt=scene.physics.timestep,
                                     render=False, record=False, plot=False, diagnostics="summary")
    complete = (log.docking_samples[-1]["t"]+scene.physics.timestep >=
                log.docking_trajectory.total_duration+scene.docking.hold_s)
    log.geometry_evaluation = evaluate_petal_seating(log.contact_diagnostics, scene, complete=complete)
    contact = evaluate_contact_load([s["interface_world"] for s in log.contact_diagnostics],
                                   scene.docking, target_rotation(scene.target), complete=complete)
    reasons = ["benchmark: "+r for r in log.docking_gate["reasons"]]
    reasons += ["geometry: "+r for r in log.geometry_evaluation["reasons"]] if complete else []
    reasons += contact["reasons"]
    if max(log.contact_summary["peak_balance_force_residual_N"],
           log.contact_summary["peak_balance_moment_residual_Nm"]) > 1e-6:
        reasons.append("wrench balance verification")
    if log.simulation_warnings:
        reasons.append("MuJoCo warnings")
    feedback = feedback_audit(log)
    if feedback["status"] != "PASS":
        reasons.append("feedback timing/frame verification")
    if profile in RELEASE_PROFILES and (feedback["yaw_trigger_s"] is None
                                  or feedback["final_yaw_stiffness_Nm_rad"] != 0.):
        reasons.append("yaw release not completed")
    policy = scene.se3_impedance.contact_yaw
    if (policy is not None and policy.lateral_stiffness_after is not None
            and not np.allclose(feedback["final_lateral_stiffness_N_m"],
                                policy.lateral_stiffness_after, rtol=0., atol=1e-12)):
        reasons.append("lateral release not completed")
    assessment = dict(status="FAIL" if reasons else "CANDIDATE_PASS" if complete else "INCOMPLETE",
                      reasons=reasons, locking_verified=False)
    save_rollout(out, name, scene, log, telemetry=effective)
    path = out / f"{name}.json"
    metadata = json.loads(path.read_text())
    metadata.update(contact_load_gate=contact, assessment=assessment,
                    simulation_warnings=log.simulation_warnings, wall_seconds=time.monotonic()-start,
                    case=case, profile=profile, numerics=setting, feedback_audit=feedback,
                    telemetry=effective,
                    diagnostic_mode="every-step contact sums and stop counts; no individual events")
    path.write_text(json.dumps(metadata, indent=2)+"\n")
    print(f"DONE {name}: {assessment}; wall {metadata['wall_seconds']:.1f} s", flush=True)
    if preview and case == "combined" and profile == "released" and setting == "baseline":
        preview_rollout(out, scene, log)
    return metadata

# 从已有 JSON 汇总评价并绘图；历史 assessment 按原门禁读取。
def summarize(out):
    records = []
    for path in sorted(out.glob("*.json")):
        metadata = json.loads(path.read_text())
        if "assessment" in metadata:
            records.append((path.stem, metadata))
    apply_style("report", cjk_first=True)
    fig, axes = plt.subplots(2, 3, figsize=(13, 7), constrained_layout=True)
    signals = [("lateral_mm", "横向残差 [mm]"), ("yaw_deg", "实际绕轴角 [°]"),
               ("depth_mm", "入口进给 [mm]"), ("diagnostic_interface_world", "净接触合力 [N]"),
               ("axial_contact", "绕轴接触力矩 [N·m]"), ("diagnostic_stop_contact_count", "承载止挡接触数")]
    for name, record in records:
        if record["case"] != "combined":
            continue
        with np.load(out / f"{name}.npz") as a:
            t = a["diagnostic_t"]
            for ax, (key, label) in zip(axes.flat, signals, strict=True):
                if key == "axial_contact":
                    value = a["diagnostic_interface_world"][:, 5]
                elif key == "diagnostic_interface_world":
                    value = np.linalg.norm(a[key][:, :3], axis=1)
                else:
                    value = a[key]
                ax.plot(t if key.startswith("diagnostic_") or key == "axial_contact" else a["t"],
                        value, color=PROFILE_COLORS[record["profile"]],
                        linestyle="--" if record["numerics"] == "dt_half" else "-", lw=1.,
                        label=PROFILE_NAMES[record["profile"]]
                        + (" · 半步长" if record["numerics"] == "dt_half" else ""))
                ax.set(xlabel="求解时间 [s]", ylabel=label)
                ax.grid(alpha=.6)
    axes[0, 0].legend(fontsize=8)
    for suffix in ["png", "pdf"]:
        fig.savefig(out / f"combined_comparison.{suffix}", dpi=170)
    plt.close(fig)
    lines = ["# PetalDock100 接触后的绕轴释放与反馈同步验证", "",
             "同一机械臂、接口、初始姿态、轨迹和控制采样协议；固定高/低绕轴刚度为 25 / 0.5 N·m/rad。",
             "released 从同一低刚度开始，仅在插入/保持阶段由 F/T 轴向载荷触发，平滑释放到零；保留原惯量和阻尼。",
             "nominal 无估计误差；xy 为 (2, −2) mm；combined 同时增加 5° 偏航误差。",
             "控制周期固定 0.5 ms；物理步长 0.5 ms，dt_half 为 0.25 ms。F/T 延迟固定一个控制周期，位姿/接触/传感器遥测统一到求解时刻。",
             "载荷逐物理步记录，无 weld 锁定。检测不使用目标真值或仿真接触标记。",
             "几何门禁使用附件名义法兰间距 46.4 mm、45° 配合相位和承载的止挡接触。",
             "研究容差：横向 0.5 mm、倾斜 0.5°、相位 2°、轴向 0.75 mm；最后一秒止挡接触占比 ≥95%。",
             "载荷门禁：净接触合力 ≤40 N、绕轴接触力矩 ≤2 N·m；同时检查传感器载荷、关节范围、力矩饱和与静止。", "",
             "| 工况 | 绕轴 | 数值设置 | 综合结论 | 横向 mm | 相位 ° | 轴向间隙 mm | 止挡占比 | 接触力 N | 接触轴矩 N·m | 原因 |",
             "|---|---|---|---|---:|---:|---:|---:|---:|---:|---|"]
    for _name, r in records:
        g, c, a = r["geometry_evaluation"], r["contact_load_gate"], r["assessment"]
        lines.append(f"| {r['case']} | {r['profile']} | {r['numerics']} | {a['status']} | "
                     f"{g['last_second_max_lateral_mm']:.4f} | {g['last_second_max_phase_error_deg']:.4f} | "
                     f"{g['last_second_max_abs_axial_gap_mm']:.4f} | {g['last_second_stop_contact_fraction']:.1%} | "
                     f"{c['peak_contact_force_N']:.3f} | {c['peak_contact_axial_moment_Nm']:.4f} | "
                     f"{'; '.join(a['reasons']) or '—'} |")
    lines += ["", "![组合工况曲线](combined_comparison.png)", "",
              "每组 JSON 保存完整场景、规划路点、传感器门禁、接触门禁与几何门禁；NPZ 保存每个仿真步的同步接触合力、力矩和止挡接触数。",
              "本轮验证零重力、单一起点和声明的三个误差条件，不等于捕获范围、硬件载荷或锁紧认证。"]
    sensitivity = {}
    indexed = dict(records)
    for profile in PROFILES:
        name = f"combined_{profile}"
        if name not in indexed or name+"_dt_half" not in indexed:
            continue
        a, b = indexed[name], indexed[name+"_dt_half"]
        ga, gb = a["geometry_evaluation"], b["geometry_evaluation"]
        ca, cb = a["contact_load_gate"], b["contact_load_gate"]
        differences = dict(
            lateral_mm=abs(ga["last_second_max_lateral_mm"]-gb["last_second_max_lateral_mm"]),
            axial_gap_mm=abs(ga["last_second_max_abs_axial_gap_mm"]-gb["last_second_max_abs_axial_gap_mm"]),
            phase_deg=abs(ga["last_second_max_phase_error_deg"]-gb["last_second_max_phase_error_deg"]),
            peak_force_N=abs(ca["peak_contact_force_N"]-cb["peak_contact_force_N"]),
            peak_axial_moment_Nm=abs(ca["peak_contact_axial_moment_Nm"]-cb["peak_contact_axial_moment_Nm"]))
        stable = (a["assessment"]["status"] == b["assessment"]["status"]
                  and differences["lateral_mm"] <= .1 and differences["axial_gap_mm"] <= .1
                  and differences["phase_deg"] <= .1
                  and differences["peak_force_N"] <= max(.5, .1*ca["peak_contact_force_N"])
                  and differences["peak_axial_moment_Nm"] <= max(.05, .1*ca["peak_contact_axial_moment_Nm"]))
        sensitivity[profile] = dict(status="STABLE_IN_TWO_STEPS" if stable else "SENSITIVE",
                                    baseline_assessment=a["assessment"]["status"],
                                    half_dt_assessment=b["assessment"]["status"], differences=differences)
    if sensitivity:
        (out / "numerical_comparison.json").write_text(json.dumps(sensitivity, indent=2)+"\n")
        lines += ["", "时间步检查：固定控制周期/反馈延迟，只比较物理步长 0.5 / 0.25 ms，不声明数学收敛。判据：综合状态相同；横向/轴向差 ≤0.1 mm、相位差 ≤0.1°；峰值力差 ≤max(0.5 N, 10%)、峰值轴矩差 ≤max(0.05 N·m, 10%)。"]
        for profile, result in sensitivity.items():
            lines.append(f"- {profile}: {result['status']}；差异 {result['differences']}")
    (out / "report.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    (out / "summary.json").write_text(json.dumps({n: r for n, r in records}, indent=2)+"\n")
