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


@dataclass(frozen=True)
class TrackingErrorMetrics:
    """一个跟踪时间窗的位置和姿态误差统计（None 表示没有可用样本）。"""

    position_rms_m: float | None = None
    position_peak_m: float | None = None
    orientation_rms_rad: float | None = None
    orientation_peak_rad: float | None = None


@dataclass(frozen=True)
class TrackingMetrics:
    """自由空间圆形/8 字跟踪门禁所需的完整测量结果。"""

    segments: tuple[tuple[str, TrackingErrorMetrics], ...] = ()
    overall: TrackingErrorMetrics = TrackingErrorMetrics()
    peak_joint_torque_Nm: float | None = None
    torque_saturation_ratio: float | None = None
    max_contacts: int | None = None

    def segment(self, name: str) -> TrackingErrorMetrics | None:
        """按轨迹段名称查询指标，未知名称返回 None。"""
        return dict(self.segments).get(name)


@dataclass(frozen=True)
class TrackingThresholds:
    """自由空间跟踪通过柔顺对接前的默认门槛。"""

    circle_position_rms_m: float = 0.005
    figure8_position_rms_m: float = 0.005
    position_peak_m: float = 0.015
    orientation_rms_rad: float = float(np.deg2rad(0.5))
    torque_saturation_ratio: float = 0.01
    max_contacts: int = 0


@dataclass(frozen=True)
class TrackingGateResult:
    """门禁判定；未跑完轨迹时 ``status`` 为 ``INCOMPLETE``。"""

    metrics: TrackingMetrics
    thresholds: TrackingThresholds
    status: str
    failures: tuple[str, ...] = ()

    @property
    def passed(self) -> bool:
        return self.status == "PASS"


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

    Args:
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

    Args:
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


def _tracking_error_metrics(position_errors: np.ndarray,
                            orientation_errors: np.ndarray | None,
                            mask: np.ndarray) -> TrackingErrorMetrics:
    """从一个逐步掩码计算位置/姿态 RMS 与峰值。"""
    pos = position_errors[mask]
    ori = orientation_errors[mask] if orientation_errors is not None else np.empty(0)
    return TrackingErrorMetrics(
        position_rms_m=float(np.sqrt(np.mean(pos**2))) if pos.size else None,
        position_peak_m=float(np.max(pos)) if pos.size else None,
        orientation_rms_rad=float(np.sqrt(np.mean(ori**2))) if ori.size else None,
        orientation_peak_rad=float(np.max(ori)) if ori.size else None,
    )


def compute_tracking_metrics(
        log: Log, segments: Sequence[tuple[str, float, float]]) -> TrackingMetrics:
    """从 ``Log`` 计算圆形/8 字跟踪门禁指标。

    ``segments`` 使用轨迹规划器公开的 ``(name, start, end)`` 结构。姿态误差、
    力矩限幅标记和接触数均是可选的向后兼容遥测字段；长度不匹配时相应指标为
    ``None``，由门禁作为数据不足处理。
    """
    n_steps = len(log.t_list)
    t_arr = np.asarray(log.t_list, dtype=float)
    position_errors = np.asarray(log.error, dtype=float)
    if t_arr.size != n_steps or position_errors.size != n_steps:
        position_errors = np.empty(0)
        t_arr = np.empty(0)

    # 门禁的总指标只描述实际测试窗口，而非轨迹结束后的保持段。分段仍保持
    # [start, end) 语义；总窗口按调用方给定的最早开始与最晚结束确定。
    window_mask: np.ndarray | None = None
    if position_errors.size and segments:
        bounds = np.asarray([(start, end) for _, start, end in segments], dtype=float)
        if bounds.shape == (len(segments), 2) and np.all(np.isfinite(bounds)) \
                and np.all(bounds[:, 1] > bounds[:, 0]):
            window_start = float(np.min(bounds[:, 0]))
            window_end = float(np.max(bounds[:, 1]))
            window_mask = (t_arr >= window_start) & (t_arr < window_end)

    orientation_norms = None
    if len(log.orientation_errors) == n_steps:
        orientation_array = np.asarray(log.orientation_errors, dtype=float)
        if orientation_array.shape == (n_steps, 3):
            orientation_norms = np.linalg.norm(orientation_array, axis=1)

    segment_metrics = tuple(
        (name, _tracking_error_metrics(
            position_errors, orientation_norms,
            (t_arr >= start) & (t_arr < end),
        ))
        for name, start, end in segments
    ) if position_errors.size else tuple(
        (name, TrackingErrorMetrics()) for name, _, _ in segments
    )
    overall = _tracking_error_metrics(
        position_errors, orientation_norms, window_mask,
    ) if window_mask is not None else TrackingErrorMetrics()

    tau = np.asarray(log.tau_hist, dtype=float)
    peak_torque = None
    if window_mask is not None and tau.ndim == 2 and tau.shape[0] == n_steps and tau.size:
        tau_window = tau[window_mask]
        if tau_window.size:
            peak_torque = float(np.max(np.abs(tau_window)))

    saturation = getattr(log, "torque_saturated", [])
    saturation_ratio = None
    if window_mask is not None and len(saturation) == n_steps and np.any(window_mask):
        saturation_ratio = float(np.mean(np.asarray(saturation, dtype=bool)[window_mask]))

    contacts = getattr(log, "contact_counts", [])
    max_contacts = None
    if window_mask is not None and len(contacts) == n_steps and np.any(window_mask):
        max_contacts = int(np.max(np.asarray(contacts, dtype=int)[window_mask]))

    return TrackingMetrics(
        segments=segment_metrics,
        overall=overall,
        peak_joint_torque_Nm=peak_torque,
        torque_saturation_ratio=saturation_ratio,
        max_contacts=max_contacts,
    )


def evaluate_tracking_gate(metrics: TrackingMetrics, thresholds: TrackingThresholds,
                           *, complete: bool) -> TrackingGateResult:
    """按门槛判定跟踪测试；未完整覆盖轨迹只报告 ``INCOMPLETE``。"""
    if not complete:
        return TrackingGateResult(metrics, thresholds, "INCOMPLETE")

    failures: list[str] = []

    def is_finite_scalar(value: object) -> bool:
        """门禁只接受有限标量；NaN/inf/非标量都必须 fail closed。"""
        try:
            return bool(np.isscalar(value) and np.isfinite(value))
        except TypeError:
            return False

    def nonfinite(label: str, value: object) -> None:
        if value is not None and not is_finite_scalar(value):
            failures.append(f"{label}={value!r}（非有限或非标量）")

    # 即使某项当前没有阈值，也不能让非有限遥测借由未参与比较而通过门禁。
    for segment_name, stats in metrics.segments:
        nonfinite(f"{segment_name}位置 RMS[m]", stats.position_rms_m)
        nonfinite(f"{segment_name}位置峰值[m]", stats.position_peak_m)
        nonfinite(f"{segment_name}姿态 RMS[rad]", stats.orientation_rms_rad)
        nonfinite(f"{segment_name}姿态峰值[rad]", stats.orientation_peak_rad)
    nonfinite("全程位置 RMS[m]", metrics.overall.position_rms_m)
    nonfinite("全程位置峰值[m]", metrics.overall.position_peak_m)
    nonfinite("全程姿态 RMS[rad]", metrics.overall.orientation_rms_rad)
    nonfinite("全程姿态峰值[rad]", metrics.overall.orientation_peak_rad)
    nonfinite("峰值关节力矩[Nm]", metrics.peak_joint_torque_Nm)
    nonfinite("力矩限幅比例", metrics.torque_saturation_ratio)
    nonfinite("最大接触数", metrics.max_contacts)

    def upper_bound(label: str, value: float | int | None, limit: float | int) -> None:
        if value is None:
            failures.append(f"{label}=n/a（缺少数据）")
        elif not is_finite_scalar(value):
            return
        elif not is_finite_scalar(limit):
            failures.append(f"{label} 阈值={limit!r}（非有限或非标量）")
        elif value > limit:
            failures.append(f"{label}={value:.6g} > {limit:.6g}")

    circle = metrics.segment("圆周")
    figure8 = metrics.segment("8字")
    upper_bound("圆周位置 RMS[m]", None if circle is None else circle.position_rms_m,
                thresholds.circle_position_rms_m)
    upper_bound("8字位置 RMS[m]", None if figure8 is None else figure8.position_rms_m,
                thresholds.figure8_position_rms_m)
    upper_bound("全程位置峰值[m]", metrics.overall.position_peak_m,
                thresholds.position_peak_m)
    upper_bound("全程姿态 RMS[rad]", metrics.overall.orientation_rms_rad,
                thresholds.orientation_rms_rad)
    upper_bound("力矩限幅比例", metrics.torque_saturation_ratio,
                thresholds.torque_saturation_ratio)
    upper_bound("最大接触数", metrics.max_contacts, thresholds.max_contacts)
    return TrackingGateResult(
        metrics, thresholds, "PASS" if not failures else "FAIL", tuple(failures))


def format_tracking_gate(result: TrackingGateResult) -> str:
    """以可审计的 PASS/FAIL/INCOMPLETE 文本格式输出跟踪门禁。"""
    def metric(value: float | None, scale: float = 1.0, unit: str = "") -> str:
        if value is None:
            return "n/a"
        return f"{value * scale:.4f}{unit}"

    lines = ["=" * 64, f"自由空间跟踪门禁: {result.status}", "=" * 64]
    for name, stats in result.metrics.segments:
        lines.append(
            f"  {name}: pos RMS {metric(stats.position_rms_m, 1e3, ' mm')}, "
            f"peak {metric(stats.position_peak_m, 1e3, ' mm')}; "
            f"ori RMS {metric(stats.orientation_rms_rad, 180.0 / np.pi, ' deg')}, "
            f"peak {metric(stats.orientation_peak_rad, 180.0 / np.pi, ' deg')}")
    overall = result.metrics.overall
    lines.extend([
        f"  全程: pos RMS {metric(overall.position_rms_m, 1e3, ' mm')}, "
        f"peak {metric(overall.position_peak_m, 1e3, ' mm')}; "
        f"ori RMS {metric(overall.orientation_rms_rad, 180.0 / np.pi, ' deg')}, "
        f"peak {metric(overall.orientation_peak_rad, 180.0 / np.pi, ' deg')}",
        f"  峰值关节力矩: {metric(result.metrics.peak_joint_torque_Nm, 1.0, ' N m')}",
        f"  力矩限幅比例: {metric(result.metrics.torque_saturation_ratio, 100.0, '%')}",
        f"  最大接触数: {result.metrics.max_contacts if result.metrics.max_contacts is not None else 'n/a'}",
    ])
    if result.status == "INCOMPLETE":
        lines.append("  结果: INCOMPLETE（仿真时长不足，未作为门禁失败）")
    elif result.failures:
        lines.append("  失败项: " + "; ".join(result.failures))
    else:
        lines.append("  结果: PASS（可进入柔顺力控对接测试）")
    return "\n".join(lines)
