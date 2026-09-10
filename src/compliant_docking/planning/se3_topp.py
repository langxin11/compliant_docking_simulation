"""
se3_topp.py - SE(3) 分段测地线 + 时间最优参数化轨迹 / SE(3) segmented-geodesic
TOPP trajectory (Ren & Shan 2026 §3.1, Theorem 1 + Algorithm 1)

路径：路点间为常螺旋测地线 T_i(s) = T_i·Exp(ξ̂·s)，s∈[0,1]；姿态与位置
一同插值（纯平移段退化为直线）。

时间参数化（Theorem 1）：体坐标系下 ν̇_b = ξ·s̈ 与路径导数严格线性，
逐分量限幅 −V_max ⪯ ξ·ṡ ⪯ V_max、−A_max ⪯ ξ·s̈ ⪯ A_max 等价于标量界
ṡ ≤ min_j V_j/|ξ_j|、s̈ ≤ min_j A_j/|ξ_j|——每段 rest-to-rest 的一维
时间最优剖面为解析梯形/三角形（先以 A_max 加速到 ṡ_max，巡航，再减速）。

与 TwoPhaseDockingTrajectory 的关系：同一路点结构（起始→预对接→终点），
差异在 (1) SE(3) 测地线含姿态插值，(2) 时间剖面在相同限速下时间最优
（路点处同样 rest-to-rest，与论文 Algorithm 1 一致）。

说明：get_state 返回世界系位置/线速度/线加速度前馈，与既有规划器接口
一致；当前控制栈姿态参考固定（r_des 常量），姿态变化的段需按步姿态
参考后才能真正跟踪（本类的姿态路径与角速度限幅已正确实现并测试）。
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin


def _topp_duration(v_max: float, a_max: float) -> float:
    """rest-to-rest 一维时间最优时长（s∈[0,1]，界 ṡ≤v_max、|s̈|≤a_max）。

    三角形（无巡航）：v_peak = sqrt(a_max) ≤ v_max 时 T = 2/sqrt(a_max)；
    否则梯形 T = 1/v_max + v_max/a_max。
    """
    assert v_max > 0 and a_max > 0
    v_peak = np.sqrt(a_max)
    if v_peak <= v_max:
        return 2.0 * v_peak / a_max
    return 1.0 / v_max + v_max / a_max


def _topp_profile_exact(t: float, v_max: float, a_max: float,
                        T: float) -> tuple[float, float, float]:
    """精确 piecewise 剖面（避免镜像式书写错误的直算版本）。"""
    t = float(np.clip(t, 0.0, T))
    v_peak = min(v_max, np.sqrt(a_max))
    t_ramp = v_peak / a_max
    s_ramp = 0.5 * a_max * t_ramp**2
    if t < t_ramp:  # 加速
        return 0.5 * a_max * t**2, a_max * t, a_max
    if t <= T - t_ramp:  # 巡航
        return s_ramp + v_peak * (t - t_ramp), v_peak, 0.0
    # 减速：s(τ) = 1 - ½a(T-τ)²，以 τ = T - t 回代
    tau = T - t
    return 1.0 - 0.5 * a_max * tau * tau, a_max * tau, -a_max


class SE3ToppTrajectory:
    """两段 SE(3) 测地线 + 每段时间最优参数化的对接轨迹。

    路点：T1（起始位姿）→ T2（预对接：终点沿接近轴后撤 standoff，姿态同
    终点）→ T3（终点位姿）。接近段用宽松限速、对接段用严格限速（论文
    Table 8：接近 0.10 m/s·0.20 rad/s，对接 0.02 m/s·0.05 rad/s）。
    """

    def __init__(self, start_pos: np.ndarray, start_ori: np.ndarray,
                 final_pos: np.ndarray, final_ori: np.ndarray, *,
                 standoff: float,
                 v_max_approach: float, a_max_approach: float,
                 omega_max_approach: float, alpha_max_approach: float,
                 v_max_docking: float, a_max_docking: float,
                 omega_max_docking: float, alpha_max_docking: float):
        assert start_pos.shape == (3,) and final_pos.shape == (3,)
        assert start_ori.shape == (3, 3) and final_ori.shape == (3, 3)
        assert standoff > 0, "Stand-off must be positive"

        stroke = final_pos - start_pos
        norm = float(np.linalg.norm(stroke))
        assert norm > 0, "start 与 final 位置不可重合"
        self.axis = -stroke / norm  # 接近轴（从终点指向起始）

        self._p0 = start_pos.copy()
        self._R0 = np.array(start_ori, dtype=float)
        self._pf = final_pos.copy()
        self._Rf = np.array(final_ori, dtype=float)

        # 路点：T2 = 终点后撤（姿态同终点），T3 = 终点
        p2 = self._pf + standoff * self.axis
        T1 = pin.SE3(self._R0, self._p0)
        T2 = pin.SE3(self._Rf, p2)
        T3 = pin.SE3(self._Rf, self._pf)

        segs = [
            self._make_segment(T1, T2,
                               np.array([v_max_approach] * 3 + [omega_max_approach] * 3),
                               np.array([a_max_approach] * 3 + [alpha_max_approach] * 3)),
            self._make_segment(T2, T3,
                               np.array([v_max_docking] * 3 + [omega_max_docking] * 3),
                               np.array([a_max_docking] * 3 + [alpha_max_docking] * 3)),
        ]
        self._segments = segs
        # 元组布局：(T_a, R_a, ξ, ṡ_max, s̈_max, T)——时长在索引 5
        self._t1 = segs[0][5]
        self._t2 = segs[1][5]

    @staticmethod
    def _make_segment(T_a: pin.SE3, T_b: pin.SE3,
                      V_max6: np.ndarray, A_max6: np.ndarray) -> tuple:
        """段数据：(T_a, R_a, ξ, ṡ_max, s̈_max, T)。Theorem 1 的线性映射。"""
        T_rel = T_a.inverse() * T_b
        xi = pin.log6(T_rel).vector  # 体坐标系常螺旋 [v; ω]
        denom = np.abs(xi)
        mask = denom > 1e-12
        assert np.any(mask), "段螺旋为零（重合路点）"
        s_dot_max = float(np.min(V_max6[mask] / denom[mask]))
        s_ddot_max = float(np.min(A_max6[mask] / denom[mask]))
        T = _topp_duration(s_dot_max, s_ddot_max)
        return (T_a, np.array(T_a.rotation), xi, s_dot_max, s_ddot_max, T)

    # ------------------------------------------------------------------
    @property
    def durations(self) -> tuple[float, float]:
        """(接近段, 对接段) 时长 [s]。"""
        return self._t1, self._t2

    @property
    def total_duration(self) -> float:
        return self._t1 + self._t2

    def get_state(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """世界系 (pos, vel, acc) 线量前馈（t≥总时长停在终点）。"""
        t = float(t)
        if t < 0.0:
            return self._p0.copy(), np.zeros(3), np.zeros(3)
        if t < self._t1:
            return self._segment_state(0, t)
        return self._segment_state(1, t - self._t1)

    def get_pose(self, t: float) -> pin.SE3:
        """t 时刻的完整位姿 T_r(t)（供按步姿态参考与测试使用）。"""
        t = float(t)
        if t < 0.0:
            return pin.SE3(self._R0, self._p0)
        if t < self._t1:
            T_a, _, xi, _, _, _ = self._segments[0]
            s, _, _ = _topp_profile_exact(t, self._segments[0][3], self._segments[0][4], self._segments[0][5])
            return T_a * pin.exp6(pin.Motion(xi * s))
        T_a, _, xi, s_dot_max, s_ddot_max, T = self._segments[1]
        s, _, _ = _topp_profile_exact(t - self._t1, s_dot_max, s_ddot_max, T)
        return T_a * pin.exp6(pin.Motion(xi * s))

    def _segment_state(self, seg_idx: int, t_local: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        T_a, R_a, xi, s_dot_max, s_ddot_max, T = self._segments[seg_idx]
        s, s_dot, s_ddot = _topp_profile_exact(t_local, s_dot_max, s_ddot_max, T)

        v_b = xi[:3] * s_dot
        w_b = xi[3:] * s_dot
        M_s = T_a * pin.exp6(pin.Motion(xi * s))
        R_s = np.array(M_s.rotation)
        # 体坐标系螺旋映射到世界系：ṗ = R·v_b；
        # p̈ = R·(ω_b×v_b)·ṡ² + R·v_b·s̈（后者在纯平移段即 R·v_b·s̈）
        acc = R_s @ (np.cross(w_b, v_b) * s_dot**2 + xi[:3] * s_ddot)
        return np.array(M_s.translation), R_s @ v_b, acc
