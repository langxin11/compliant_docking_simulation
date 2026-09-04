"""对接性能指标套件（对标 Ren & Shan 2026, Acta Astronautica, Table 10）。

三层指标：
1. 接触安全：峰值轴向力、峰值广义力范数、稳态轴向力、稳态广义力范数；
2. 内部安全：最大关节角度/速度占模型限值百分比、最小可操作度；
3. 跟踪精度：位置跟踪 RMS、姿态跟踪 RMS、末端稳态横向误差、稳态姿态误差。

指标定义（与论文表 10 的对应关系）：
- 峰值轴向力：全程 |f_ext · axis| 的最大值，f_ext 为世界系外力，axis 为对接轴
  单位向量（世界系，取轨迹推进方向）；峰值取绝对值，接触反力沿轴反向时同样捕捉；
- 峰值/稳态广义力范数：每步 6 维广义力 [f; τ] 的 2-范数的最大值/稳态均值。
  注意：广义力范数包含力矩分量 τ，数值不等于纯接触力大小；
- 稳态窗口：t ≥ t_end - steady_window 的采样段；稳态轴向力、稳态广义力范数、
  稳态横向误差、稳态姿态误差均为该窗口内的时间均值；
- 关节角度占比：|q - 限位区间中点| / 半量程 × 100%（到达任一限位时为 100%），
  使用 pin_model.lowerPositionLimit/upperPositionLimit；
- 关节速度占比：|v| / velocityLimit × 100%；某轴 velocityLimit ≤ 0（或非有限）
  时该轴跳过，全部无效则该项为 None；均报告全程最大百分比与对应关节编号；
- 最小可操作度：min sqrt(det(J Jᵀ))，J 为末端 frame 的世界系雅可比
  （pin.computeFrameJacobian(..., pin.ReferenceFrame.WORLD)，后处理逐帧计算）；
- 位置跟踪 RMS：sqrt(mean(error²))，error 为每步末端位置误差范数（log.error）；
- 姿态跟踪 RMS / 稳态姿态误差：基于世界系姿态误差向量 log(R_d Rᵀ) 的范数；
  仅当 Log.orientation_errors 非空且长度与时间序列一致时计算，否则为 None。

所有数值字段均为 float | None，None 表示数据不足无法计算。
"""
from __future__ import annotations

import unicodedata
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
import pinocchio as pin

from .telemetry import Log


@dataclass(frozen=True)
class DockingMetrics:
    """对接性能指标集合（None = 数据不足无法计算）。"""

    # ---- 1) 接触安全 ----
    peak_axial_force_N: float | None = None
    peak_wrench_norm_N: float | None = None
    steady_axial_force_N: float | None = None
    steady_wrench_norm_N: float | None = None

    # ---- 2) 内部安全 ----
    max_joint_pos_pct: float | None = None
    max_joint_vel_pct: float | None = None
    min_manipulability: float | None = None

    # ---- 3) 跟踪精度 ----
    pos_tracking_rms_m: float | None = None
    ori_tracking_rms_rad: float | None = None
    final_lateral_error_m: float | None = None
    final_orientation_error_rad: float | None = None

    # 达到最大占比的关节编号（0 起索引）
    max_joint_pos_joint: int | None = None
    max_joint_vel_joint: int | None = None


def _stack(rows: list, cols: int) -> np.ndarray | None:
    """把逐 step 的等长向量列表堆叠为 (n, cols) 矩阵；为空或形状不符时返回 None。"""
    if len(rows) == 0:
        return None
    arr = np.asarray(rows, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != cols:
        return None
    return arr


def _joint_pct_max(values: np.ndarray, limit_lo: np.ndarray | None,
                   limit_up: np.ndarray | None) -> tuple[float | None, int | None]:
    """关节角度占比：|q - 区间中点| / 半量程 × 100%，返回 (最大百分比, 关节编号)。

    限位无效（非有限或半量程 ≤ 0）的轴跳过；全部无效时返回 (None, None)。
    """
    if limit_lo is None or limit_up is None or values.shape[1] != len(limit_lo):
        return None, None
    center = 0.5 * (limit_lo + limit_up)
    half = 0.5 * (limit_up - limit_lo)
    valid = np.isfinite(center) & np.isfinite(half) & (half > 0.0)
    if not np.any(valid):
        return None, None
    pct = np.abs(values[:, valid] - center[valid]) / half[valid] * 100.0
    flat = int(np.argmax(pct))
    return float(pct.reshape(-1)[flat]), int(flat % pct.shape[1])


def _vel_pct_max(values: np.ndarray, limit: np.ndarray | None) -> tuple[float | None, int | None]:
    """关节速度占比：|v| / velocityLimit × 100%，返回 (最大百分比, 关节编号)。

    velocityLimit ≤ 0（或非有限）的轴跳过；全部无效时返回 (None, None)。
    """
    if limit is None or values.shape[1] != len(limit):
        return None, None
    valid = np.isfinite(limit) & (limit > 0.0)
    if not np.any(valid):
        return None, None
    pct = np.abs(values[:, valid]) / limit[valid] * 100.0
    flat = int(np.argmax(pct))
    return float(pct.reshape(-1)[flat]), int(flat % pct.shape[1])


def compute_metrics(log: Log, pin_model: pin.Model, *, axis: np.ndarray,
                    ee_frame: str, steady_window: float = 2.0) -> DockingMetrics:
    """从仿真 Log 后处理计算对接性能指标（不进热循环）。

    参数 / Args:
        log: 仿真日志（store_data 产出；可选含 orientation_errors 姿态误差序列）
        pin_model: Pinocchio 模型（提供关节限值/速度上限/雅可比）
        axis: 对接轴方向（世界系；内部归一化，非单位向量也可）
        ee_frame: 末端 frame 名（可操作度雅可比取自该 frame）
        steady_window: 稳态窗口长度 [s]（取 t ≥ t_end - steady_window）
    """
    axis = np.asarray(axis, dtype=float).reshape(3)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm == 0.0:
        raise ValueError("axis 不能为零向量")
    axis = axis / axis_norm

    n_steps = len(log.t_list)
    if n_steps == 0:
        return DockingMetrics()

    t_arr = np.asarray(log.t_list, dtype=float)
    steady_mask = t_arr >= t_arr[-1] - steady_window

    # ---- 1) 接触安全 ----
    forces = _stack(log.force_externals, 3)  # 世界系外力（run 循环里 current_ori @ sensor）
    torques = _stack(log.torque_externals, 3)  # 世界系外力矩
    peak_axial = steady_axial = peak_wrench = steady_wrench = None
    if forces is not None:
        axial = forces @ axis
        peak_axial = float(np.max(np.abs(axial)))
        steady_axial = float(np.mean(axial[steady_mask]))
    if forces is not None and torques is not None:
        wrench_norm = np.linalg.norm(np.hstack([forces, torques]), axis=1)
        peak_wrench = float(np.max(wrench_norm))
        steady_wrench = float(np.mean(wrench_norm[steady_mask]))

    # ---- 2) 内部安全 ----
    q_arr = _stack(log.joint_angles, pin_model.nq)
    v_arr = _stack(log.joint_velocities, pin_model.nq)
    max_pos_pct, max_pos_joint = (None, None)
    max_vel_pct, max_vel_joint = (None, None)
    min_manip = None
    if q_arr is not None:
        max_pos_pct, max_pos_joint = _joint_pct_max(
            q_arr, np.asarray(pin_model.lowerPositionLimit, dtype=float),
            np.asarray(pin_model.upperPositionLimit, dtype=float))
        if pin_model.getFrameId(ee_frame) < len(pin_model.frames):
            frame_id = pin_model.getFrameId(ee_frame)
            pin_data = pin_model.createData()
            w = np.empty(q_arr.shape[0])
            for k, q_k in enumerate(q_arr):
                J = pin.computeFrameJacobian(pin_model, pin_data, q_k, frame_id,
                                             pin.ReferenceFrame.WORLD)
                w[k] = np.sqrt(max(float(np.linalg.det(J @ J.T)), 0.0))
            min_manip = float(np.min(w))
    if v_arr is not None:
        max_vel_pct, max_vel_joint = _vel_pct_max(
            v_arr, np.asarray(pin_model.velocityLimit, dtype=float))

    # ---- 3) 跟踪精度 ----
    pos_rms = None
    if len(log.error) == n_steps:
        pos_rms = float(np.sqrt(np.mean(np.asarray(log.error, dtype=float) ** 2)))

    ori_rms = final_ori_err = None
    if len(log.orientation_errors) == n_steps:
        ori_arr = np.asarray(log.orientation_errors, dtype=float).reshape(n_steps, 3)
        ori_norms = np.linalg.norm(ori_arr, axis=1)
        ori_rms = float(np.sqrt(np.mean(ori_norms**2)))
        final_ori_err = float(np.mean(ori_norms[steady_mask]))

    final_lateral = None
    pos_act = _stack(log.pos_actual, 3)
    pos_des = _stack(log.pos_desired, 3)
    if pos_act is not None and pos_des is not None:
        e_vec = pos_act - pos_des
        e_perp = e_vec - (e_vec @ axis)[:, None] * axis
        final_lateral = float(np.mean(np.linalg.norm(e_perp, axis=1)[steady_mask]))

    return DockingMetrics(
        peak_axial_force_N=peak_axial,
        peak_wrench_norm_N=peak_wrench,
        steady_axial_force_N=steady_axial,
        steady_wrench_norm_N=steady_wrench,
        max_joint_pos_pct=max_pos_pct,
        max_joint_vel_pct=max_vel_pct,
        min_manipulability=min_manip,
        pos_tracking_rms_m=pos_rms,
        ori_tracking_rms_rad=ori_rms,
        final_lateral_error_m=final_lateral,
        final_orientation_error_rad=final_ori_err,
        max_joint_pos_joint=max_pos_joint,
        max_joint_vel_joint=max_vel_joint,
    )


def _label_pad(label: str, width: int = 20) -> str:
    """按显示宽度（CJK 记 2 列）给标签补空格，使表格对齐。"""
    display = sum(2 if unicodedata.east_asian_width(ch) in {"F", "W"} else 1 for ch in label)
    return label + " " * max(width - display, 1)


def format_metrics(m: DockingMetrics) -> str:
    """把指标渲染为对齐的中文表格文本（三段，与 Table 10 分层一致；None 显示 n/a）。"""

    def num(value: float | None, unit: str = "", spec: str = ".4f") -> str:
        if value is None:
            return "n/a"
        text = f"{value:{spec}}"
        return f"{text} {unit}" if unit else text

    def pct_line(pct: float | None, joint: int | None) -> str:
        if pct is None:
            return "n/a"
        where = f"（关节 {joint + 1}）" if joint is not None else ""
        return f"{pct:.2f}%{where}"

    def row(label: str, value: str) -> str:
        return f"  {_label_pad(label)}: {value}"

    lines = [
        "=" * 64,
        "对接性能指标（Ren & Shan 2026, Acta Astronautica, Table 10）",
        "=" * 64,
        "[接触安全]",
        row("峰值轴向力", num(m.peak_axial_force_N, "N")),
        row("峰值广义力范数", num(m.peak_wrench_norm_N, "N")),
        row("稳态轴向力", num(m.steady_axial_force_N, "N")),
        row("稳态广义力范数", num(m.steady_wrench_norm_N, "N")),
        "[内部安全]",
        row("最大关节角度占比", pct_line(m.max_joint_pos_pct, m.max_joint_pos_joint)),
        row("最大关节速度占比", pct_line(m.max_joint_vel_pct, m.max_joint_vel_joint)),
        row("最小可操作度", num(m.min_manipulability, spec=".6g")),
        "[跟踪精度]",
        row("位置跟踪 RMS", num(m.pos_tracking_rms_m, "m", ".6f")),
        row("姿态跟踪 RMS", num(m.ori_tracking_rms_rad, "rad", ".6f")),
        row("稳态横向误差", num(m.final_lateral_error_m, "m", ".6f")),
        row("稳态姿态误差", num(m.final_orientation_error_rad, "rad", ".6f")),
    ]
    return "\n".join(lines)


def tracking_summary(log: Log, segments: Sequence[tuple[str, float, float]]) -> str:
    """圆+8字跟踪测试的分段误差统计（位置误差 RMS/峰值，单位 mm）。

    按 segments 给出的时间窗 [t0, t1) 切片 log.error（每步末端位置误差范数，m），
    计算每段 RMS 与峰值并换算为 mm；再加总全时程（全部采样点，含段外保持段）的
    RMS/峰值。输出多行中文文本，打印风格与 format_metrics 对齐。

    参数 / Args:
        log: 仿真日志（t_list 与 error 逐 step 对齐）
        segments: [(名称, t_start, t_end), ...]，与
            CircleFigure8Trajectory.segments 同构
    """
    t_arr = np.asarray(log.t_list, dtype=float)
    err = np.asarray(log.error, dtype=float)
    if t_arr.size != err.size:
        err = err[:0]  # 时间与误差不对齐时不做统计（全部 n/a）

    def stats_mm(values: np.ndarray) -> tuple[float, float] | None:
        """(RMS, 峰值)，单位 mm；空切片返回 None。"""
        if values.size == 0:
            return None
        rms = float(np.sqrt(np.mean(values**2)) * 1e3)
        peak = float(np.max(values) * 1e3)
        return rms, peak

    lines = [
        "=" * 64,
        "轨迹跟踪统计（圆+8字，按段位置误差）",
        "=" * 64,
        "[分段统计]",
    ]
    for name, t0, t1 in segments:
        seg = stats_mm(err[(t_arr >= t0) & (t_arr < t1)]) if err.size else None
        if seg is None:
            lines.append(f"  {_label_pad(name)}: n/a")
        else:
            lines.append(f"  {_label_pad(name)}: RMS {seg[0]:.4f} mm, 峰值 {seg[1]:.4f} mm")

    lines.append("[全时程]")
    total = stats_mm(err)
    if total is None:
        lines.append(f"  {_label_pad('位置跟踪 RMS')}: n/a")
        lines.append(f"  {_label_pad('峰值误差')}: n/a")
    else:
        lines.append(f"  {_label_pad('位置跟踪 RMS')}: {total[0]:.4f} mm")
        lines.append(f"  {_label_pad('峰值误差')}: {total[1]:.4f} mm")
    return "\n".join(lines)
