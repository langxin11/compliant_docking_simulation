"""
trajectory.py - Robot Trajectory Planning Library

This module implements trajectory generation for robotic manipulators.
It provides a class for quintic polynomial trajectory planning in task space.

Key components:
- DecoupledQuinticTrajectory: Generates smooth trajectories with zero velocity/acceleration at endpoints
- TwoPhaseDockingTrajectory: Two-phase docking trajectory (loose approach + strict docking phase)
  with a stand-off waypoint, in the style of Ren & Shan 2026 (Acta Astronautica)

Author: langxin11
Date: 2025
"""


import numpy as np

# rest-to-rest 五次标量曲线的解析峰值系数：
#   峰值速度 = QUINTIC_VEL_PEAK * L / T（= 30τ²(1-τ)² 在 τ=0.5 处的最大值 30/16）
#   峰值加速度 = QUINTIC_ACC_PEAK * L / T²（= |60τ(1-τ)(1-2τ)| 的最大值 10/√3）
QUINTIC_VEL_PEAK = 1.875
QUINTIC_ACC_PEAK = 5.7735  # ≈ 10/√3


def quintic_rest_to_rest_duration(length: float, v_max: float, a_max: float) -> float:
    """
    由速度/加速度上限反推 rest-to-rest 五次标量曲线的时长 / Derive the duration of a
    rest-to-rest quintic scalar profile from velocity and acceleration limits

    对位移长度 L 的 rest-to-rest 五次曲线，解析峰值为
    v_peak = 1.875·L/T、a_peak = 5.7735·L/T²，故同时满足两约束的最短时长为
    T = max(1.875·L/v_max, sqrt(5.7735·L/a_max))。

    参数 / Parameters:
        length: 该段位移的 3D 范数 / 3D norm of the segment displacement
        v_max: 线速度上限 / linear velocity limit
        a_max: 线加速度上限 / linear acceleration limit

    返回 / Returns:
        该段最短时长（秒）/ shortest feasible duration (s)
    """
    assert length > 0, "Segment length must be positive"
    assert v_max > 0 and a_max > 0, "Velocity/acceleration limits must be positive"
    return max(QUINTIC_VEL_PEAK * length / v_max, np.sqrt(QUINTIC_ACC_PEAK * length / a_max))


class DecoupledQuinticTrajectory:
    """
    三轴解耦的五次多项式轨迹规划器（x/y/z 分别独立）
    Decoupled quintic polynomial trajectory planner for x, y, z axes
    """
    def __init__(self, start_pos: np.ndarray, target_pos: np.ndarray, duration: float):
        """
        Initialize the trajectory planner with decoupled planning for each axis

        Args:
            start_pos: 初始位置 (x0, y0, z0) / Initial position
            target_pos: 目标位置 (xf, yf, zf) / Target position
            duration: 轨迹持续时间（秒） / Trajectory duration (s)

        Note:
            三轴各自满足端点速度/加速度为零，生成 C2 连续的平滑轨迹 /
            Each axis satisfies zero vel/acc at endpoints (C2 continuity)
        """
        assert start_pos.shape == (3,), "Start position must be 3D vector"
        assert target_pos.shape == (3,), "Target position must be 3D vector"
        assert duration > 0, "Duration must be positive"

        self.p0 = start_pos
        self.pf = target_pos
        self.T = duration

        self.ax = self._solve_quintic_coefficients(start_pos[0], target_pos[0])
        self.ay = self._solve_quintic_coefficients(start_pos[1], target_pos[1])
        self.az = self._solve_quintic_coefficients(start_pos[2], target_pos[2])

        self.coefficients = np.vstack([self.ax, self.ay, self.az])

    def _solve_quintic_coefficients(self, p0: float, pf: float) -> np.ndarray:
        """
        单轴五次多项式系数求解 / Solve coefficients of 1D quintic polynomial:
        p(t) = a0 t^5 + a1 t^4 + a2 t^3 + a3 t^2 + a4 t + a5
        约束 / Constraints：p(0)=p0, p(T)=pf, p'(0)=p'(T)=0, p''(0)=p''(T)=0
        parameters / 参数:
            p0: 初始位置 / Initial position
            pf: 目标位置 / Target position
        """
        A = np.array([
            [0, 0, 0, 0, 0, 1],
            [self.T**5, self.T**4, self.T**3, self.T**2, self.T, 1],
            [0, 0, 0, 0, 1, 0],
            [5*self.T**4, 4*self.T**3, 3*self.T**2, 2*self.T, 1, 0],
            [0, 0, 0, 2, 0, 0],
            [20*self.T**3, 12*self.T**2, 6*self.T, 2, 0, 0]
        ])

        b = np.array([p0, pf, 0, 0, 0, 0])

        return np.linalg.solve(A, b)

    def get_state(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        获取时刻 t 的位置/速度/加速度；三轴独立计算 /
        Get position, velocity and acceleration at time t; axes computed independently

        Args:
            t: 当前时间（秒） / Current time (s)

        Returns:
            (pos, vel, acc) 三个 3D 向量 / 3D numpy arrays
        """
        t = np.clip(t, 0, self.T)

        pos = np.zeros(3)
        vel = np.zeros(3)
        acc = np.zeros(3)

        t_pos = np.array([t**5, t**4, t**3, t**2, t, 1])
        t_vel = np.array([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0])
        t_acc = np.array([20*t**3, 12*t**2, 6*t, 2, 0, 0])

        for i in range(3):
            pos[i] = self.coefficients[i] @ t_pos
            vel[i] = self.coefficients[i] @ t_vel
            acc[i] = self.coefficients[i] @ t_acc

        return pos, vel, acc

    def verify_boundary_conditions(self, tol: float = 1e-10) -> bool:
        """
        验证边界条件（起止位置、速度=0、加速度=0）是否满足 /
        Verify that endpoint position/velocity/acceleration constraints hold

        Args:
            tol: 浮点比较容差 / Tolerance for comparisons

        Returns:
            是否全部满足 / True if all constraints satisfied
        """
        pos_start, vel_start, acc_start = self.get_state(0)
        pos_end, vel_end, acc_end = self.get_state(self.T)

        conditions = [
            np.allclose(pos_start, self.p0, atol=tol),
            np.allclose(pos_end, self.pf, atol=tol),
            np.allclose(vel_start, np.zeros(3), atol=tol),
            np.allclose(vel_end, np.zeros(3), atol=tol),
            np.allclose(acc_start, np.zeros(3), atol=tol),
            np.allclose(acc_end, np.zeros(3), atol=tol)
        ]

        return all(conditions)


class TwoPhaseDockingTrajectory:
    """
    两段式对接轨迹：接近段（approach，宽松限速）+ 对接段（docking，严格限速）
    Two-phase docking trajectory: approach phase (loose limits) + docking phase (strict limits)

    任务结构取自 Ren & Shan 2026 (Acta Astronautica) 的两段式对接：末端先以宽松限速
    从 start_pos 接近到预对接点（pre-dock，= final_pos 沿接近轴后撤 standoff），再以
    严格限速（0.02 m/s 量级）完成最后一段进给。峰值接触力主要由接触前速度决定，
    因此对接段的低限速是压低峰值力的关键。

    与论文 SE(3)-TOPP 的关系与简化点 / Relation to the paper's SE(3)-TOPP and simplifications:
    - 论文对整条 SE(3) 路径做时间最优参数化（TOPP），全程不停顿、速度沿路径连续变化；
    - 本实现是其保守简化版：两段各自独立做 rest-to-rest 五次多项式，途经预对接点处
      瞬时停顿（速度/加速度归零）后再进入对接段——时间上不是最优，更保守；
    - 每段时长由限速解析反推（quintic_rest_to_rest_duration）：三轴同步使用同一段
      时长 T，并用该段位移的 3D 标量长度 L 保守估计合成速度/加速度峰值
      （v_peak = 1.875·L/T、a_peak = 5.7735·L/T²）。对直线段而言三轴共享同一无量纲
      时间形状，该估计恰为精确值。

    位置/速度/加速度全程 C2 连续：接合点（预对接点）两侧均为 rest-to-rest，自然衔接。
    """

    def __init__(self,
                 start_pos: np.ndarray,
                 final_pos: np.ndarray,
                 standoff: float,
                 v_max_approach: float,
                 a_max_approach: float,
                 v_max_docking: float,
                 a_max_docking: float):
        """
        Initialize the two-phase docking trajectory

        Args:
            start_pos: 初始位置 (3D) / Initial position
            final_pos: 最终对接目标位置 (3D) / Final docking target position
            standoff: 预对接点沿接近轴的后撤距离 [m] / stand-off retreat distance [m]
            v_max_approach: 接近段线速度上限 / approach phase limits
            a_max_approach: 接近段线加速度上限 / approach phase limits
            v_max_docking: 对接段线速度上限 / docking phase limits
            a_max_docking: 对接段线加速度上限 / docking phase limits
        """
        assert start_pos.shape == (3,), "Start position must be 3D vector"
        assert final_pos.shape == (3,), "Final position must be 3D vector"
        assert standoff > 0, "Stand-off must be positive"
        assert v_max_approach > 0 and a_max_approach > 0, "Approach limits must be positive"
        assert v_max_docking > 0 and a_max_docking > 0, "Docking limits must be positive"

        self.p0 = start_pos
        self.pf = final_pos
        self.standoff = standoff

        # 接近轴 = 推进方向（start→final）的反向，即从目标指向"上方"
        stroke = final_pos - start_pos
        self.axis = -stroke / np.linalg.norm(stroke)

        # 预对接点 = 最终目标沿接近轴后撤 standoff
        self._pre_dock = final_pos + standoff * self.axis

        # 两段时长分别由各自限速反推（L 用该段位移的 3D 范数）
        length_approach = float(np.linalg.norm(self._pre_dock - start_pos))
        length_docking = float(np.linalg.norm(final_pos - self._pre_dock))
        self._t1 = quintic_rest_to_rest_duration(length_approach, v_max_approach, a_max_approach)
        self._t2 = quintic_rest_to_rest_duration(length_docking, v_max_docking, a_max_docking)

        # 复用解耦五次规划器：两段各自 rest-to-rest，接合点自然 C2
        self._approach = DecoupledQuinticTrajectory(start_pos, self._pre_dock, self._t1)
        self._docking = DecoupledQuinticTrajectory(self._pre_dock, final_pos, self._t2)

    @property
    def durations(self) -> tuple[float, float]:
        """(接近段时长 t1, 对接段时长 t2)，单位秒 / (approach, docking) durations in s"""
        return (self._t1, self._t2)

    @property
    def pre_dock_pos(self) -> np.ndarray:
        """预对接点位置（副本）/ Pre-dock waypoint position (copy)"""
        return self._pre_dock.copy()

    @property
    def total_duration(self) -> float:
        """轨迹总时长 t1 + t2（秒）/ Total trajectory duration t1 + t2 (s)"""
        return self._t1 + self._t2

    def get_state(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        获取时刻 t 的位置/速度/加速度 / Get position, velocity and acceleration at time t

        时间分段：
        - t < 0：按 t = 0 处理（停在起点）；
        - t ∈ [0, t1)：接近段（段内局部时间采样）；
        - t ∈ [t1, t1+t2)：对接段（段内局部时间采样）；
        - t ≥ t1+t2：停在 final_pos，速度/加速度为 0。

        Args:
            t: 当前时间（秒） / Current time (s)

        Returns:
            (pos, vel, acc) 三个 3D 向量 / 3D numpy arrays
        """
        if t < 0.0:
            return self._approach.get_state(0.0)
        if t < self._t1:
            return self._approach.get_state(t)
        # t ≥ t1：第二段按局部时间采样；超过 t2 时内部 clip 停在 final（vel/acc = 0）
        return self._docking.get_state(t - self._t1)


class CircleFigure8Trajectory:
    """
    圆形 + 8 字形轨迹跟踪测试规划器：过渡 → 竖直圆 → 过渡 → 平面 8 字 → 保持
    Circle + figure-8 trajectory planner for controller tracking tests (hold at the end)

    在不考虑对接的条件下测试控制器的轨迹跟踪能力（轨迹形状参考同类任务空间
    跟踪实验，速度/加速度改为对位置公式解析求导，无数值差分）。

    时间结构（四段 + 保持）：
    - 段1 过渡 [0, T_tr)：start_pos → 圆起点 P1（rest-to-rest 五次时间缩放）；
    - 段2 圆周 [T_tr, T_tr+T_c)：竖直圆（y-z 平面），θ = 2π·f_c·(t-t0)；
    - 段3 过渡 [T_tr+T_c, 2·T_tr+T_c)：圆终点 P2 → 8 字中心 C（rest-to-rest）；
    - 段4 8字 [2·T_tr+T_c, total)：水平 8 字（x-y 平面），φ = 2π·f_8·(t-t0)；
    - t ≥ total：保持在 8 字结束点，速度/加速度为零。

    几何 / Geometry:
    - 圆心 C = start_pos + [0, 0, circle_center_offset]（默认在起点正下方 0.06 m）；
    - 圆起点 P1 = C + [0, r, 0]；圆终点 P2 = 段2 结束时刻的圆周位置
      （f_c·T_c 非整圈时 P2 ≠ P1，过渡段3 必须从 P2 出发）；
    - 8 字为 Lissajous 曲线 p = C + [ax·sinφ, ay·sin(2φ), 0]。

    连续性说明 / Continuity note:
    每一段均使用五次时间缩放。圆周和 8 字的几何相位也由该缩放推进，因此在每个
    段边界位置、速度和加速度都连续（C2）；这使测试聚焦于轨迹跟踪，而不是人为的
    速度阶跃。``circle_frequency`` / ``figure8_frequency`` 表示该段总圈数除以该段
    时长的平均频率，故总相位仍为 ``2π·frequency·duration``。
    """

    def __init__(self, start_pos: np.ndarray, *,
                 transition_duration: float = 1.5,
                 circle_duration: float = 5.0,
                 circle_radius: float = 0.10,
                 circle_frequency: float = 0.2,
                 circle_center_offset: float = -0.06,
                 figure8_duration: float = 5.0,
                 figure8_radius_x: float = 0.10,
                 figure8_radius_y: float = 0.07,
                 figure8_frequency: float = 0.2):
        """
        Initialize the circle + figure-8 tracking-test trajectory

        Args:
            start_pos: 轨迹出发点 (3D) / Start position (3D)
            transition_duration: 过渡段时长 T_tr [s] / Transition segment duration
            circle_duration: 圆周段时长 T_c [s] / Circle segment duration
            circle_radius: 圆周半径 r [m] / Circle radius
            circle_frequency: 圆周频率 f_c [Hz] / Circle frequency
            circle_center_offset: 圆心相对 start_pos 的 z 偏移 [m]（可正可负） /
                Circle center offset below/above the start position
            figure8_duration: 8 字段时长 T_8 [s] / Figure-8 segment duration
            figure8_radius_x: 8 字 x 半幅值 [m] / Figure-8 radius
            figure8_radius_y: 8 字 y 半幅值 [m] / Figure-8 radius
            figure8_frequency: 8 字频率 f_8 [Hz] / Figure-8 frequency
        """
        assert start_pos.shape == (3,), "Start position must be 3D vector"
        assert transition_duration > 0, "Transition duration must be positive"
        assert circle_duration > 0, "Circle duration must be positive"
        assert circle_radius > 0, "Circle radius must be positive"
        assert circle_frequency > 0, "Circle frequency must be positive"
        assert np.isfinite(circle_center_offset), "Circle center offset must be finite"
        assert figure8_duration > 0, "Figure-8 duration must be positive"
        assert figure8_radius_x > 0, "Figure-8 radius x must be positive"
        assert figure8_radius_y > 0, "Figure-8 radius y must be positive"
        assert figure8_frequency > 0, "Figure-8 frequency must be positive"

        # 存副本，防外部修改 / Store a copy so external mutation cannot affect us
        self.start_pos = np.array(start_pos, dtype=float)

        self._t_tr = float(transition_duration)
        self._t_c = float(circle_duration)
        self._r_c = float(circle_radius)
        self._f_c = float(circle_frequency)
        self._ax8 = float(figure8_radius_x)
        self._ay8 = float(figure8_radius_y)
        self._f_8 = float(figure8_frequency)

        # 关键几何点 / Key waypoints
        self._center = self.start_pos + np.array([0.0, 0.0, float(circle_center_offset)])
        self._p1 = self._center + np.array([0.0, self._r_c, 0.0])
        # 圆终点 = 段2 结束时刻的圆周位置（f_c·T_c 非整圈时不在 P1）
        theta_end = 2.0 * np.pi * self._f_c * self._t_c
        self._p2 = self._center + self._r_c * np.array(
            [0.0, np.cos(theta_end), np.sin(theta_end)])
        # 8 字结束点（t ≥ total 时保持于此）
        phi_end = 2.0 * np.pi * self._f_8 * float(figure8_duration)
        self._end_pos = self._center + np.array(
            [self._ax8 * np.sin(phi_end), self._ay8 * np.sin(2.0 * phi_end), 0.0])

        # 段边界时间 / Segment boundary times
        self._t_8 = float(figure8_duration)
        self._t1 = self._t_tr
        self._t2 = self._t_tr + self._t_c
        self._t3 = 2.0 * self._t_tr + self._t_c
        self._total = 2.0 * self._t_tr + self._t_c + self._t_8

    # ---- 结构属性（供指标分段统计） / Structural properties (for per-segment metrics) ----

    @property
    def durations(self) -> tuple[float, float, float]:
        """(过渡段 T_tr, 圆周段 T_c, 8 字段 T_8) 时长（秒）/ (transition, circle, figure-8) durations"""
        return (self._t_tr, self._t_c, self._t_8)

    @property
    def total_duration(self) -> float:
        """轨迹总时长 2·T_tr + T_c + T_8（秒）/ Total duration 2·T_tr + T_c + T_8 (s)"""
        return self._total

    @property
    def segments(self) -> tuple[tuple[str, float, float], ...]:
        """四段 (名称, t_start, t_end)：过渡1/圆周/过渡2/8字 /
        The four segments as (name, t_start, t_end): transition/circle/transition/figure-8"""
        return (
            ("过渡1", 0.0, self._t1),
            ("圆周", self._t1, self._t2),
            ("过渡2", self._t2, self._t3),
            ("8字", self._t3, self._total),
        )

    # ---- 采样 / Sampling ----

    @staticmethod
    def _quintic_transition(a: np.ndarray, b: np.ndarray,
                            u: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        rest-to-rest 五次时间缩放的过渡段状态（u = (t-t0)/T 已归一化）/
        Quintic rest-to-rest transition state at normalized time u

        s(u) = 10u³ − 15u⁴ + 6u⁵，p = A + s(u)·(B−A)，
        v = s'(u)/T·(B−A)，a = s''(u)/T²·(B−A)；T 因子已折算进返回值的调用侧 /
        The 1/T and 1/T² factors are applied by the caller
        """
        s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        ds = 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4
        dds = 60.0 * u - 180.0 * u**2 + 120.0 * u**3
        delta = b - a
        return a + s * delta, ds * delta, dds * delta

    @staticmethod
    def _quintic_scale(u: float) -> tuple[float, float, float]:
        """五次时间缩放及其对归一化时间的前两阶导数。"""
        return (
            10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5,
            30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4,
            60.0 * u - 180.0 * u**2 + 120.0 * u**3,
        )

    def get_state(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        获取时刻 t 的位置/速度/加速度 / Get position, velocity and acceleration at time t

        时间分段：
        - t < 0：按 t = 0 处理（停在起点）；
        - t ∈ [0, T_tr)：过渡段1（start_pos → P1）；
        - t ∈ [T_tr, T_tr+T_c)：圆周段（竖直圆，y-z 平面）；
        - t ∈ [T_tr+T_c, 2·T_tr+T_c)：过渡段2（P2 → C）；
        - t ∈ [2·T_tr+T_c, total)：8 字段（水平，x-y 平面）；
        - t ≥ total：保持在 8 字结束点，速度/加速度为 0。

        Args:
            t: 当前时间（秒） / Current time (s)

        Returns:
            (pos, vel, acc) 三个 3D 向量 / 3D numpy arrays
        """
        if t < 0.0:
            t = 0.0
        if t >= self._total:
            return self._end_pos.copy(), np.zeros(3), np.zeros(3)

        if t < self._t1:
            # 段1 过渡：start_pos → P1（1/T、1/T² 因子在此折算）
            pos, vel, acc = self._quintic_transition(self.start_pos, self._p1, t / self._t_tr)
            return pos, vel / self._t_tr, acc / self._t_tr**2
        if t < self._t2:
            # 段2 圆周：相位以五次时间缩放推进，边界速度/加速度均为零。
            u = (t - self._t1) / self._t_c
            s, ds, dds = self._quintic_scale(u)
            theta_total = 2.0 * np.pi * self._f_c * self._t_c
            theta = theta_total * s
            theta_dot = theta_total * ds / self._t_c
            theta_ddot = theta_total * dds / self._t_c**2
            pos = self._center + self._r_c * np.array([0.0, np.cos(theta), np.sin(theta)])
            vel = self._r_c * theta_dot * np.array([0.0, -np.sin(theta), np.cos(theta)])
            acc = self._r_c * (
                theta_ddot * np.array([0.0, -np.sin(theta), np.cos(theta)])
                + theta_dot**2 * np.array([0.0, -np.cos(theta), -np.sin(theta)]))
            return pos, vel, acc
        if t < self._t3:
            # 段3 过渡：P2 → C
            u = (t - self._t2) / self._t_tr
            pos, vel, acc = self._quintic_transition(self._p2, self._center, u)
            return pos, vel / self._t_tr, acc / self._t_tr**2

        # 段4 8 字：相位以五次时间缩放推进，保证与前段和保持段 C2 连续。
        u = (t - self._t3) / self._t_8
        s, ds, dds = self._quintic_scale(u)
        phi_total = 2.0 * np.pi * self._f_8 * self._t_8
        phi = phi_total * s
        phi_dot = phi_total * ds / self._t_8
        phi_ddot = phi_total * dds / self._t_8**2
        pos = self._center + np.array(
            [self._ax8 * np.sin(phi), self._ay8 * np.sin(2.0 * phi), 0.0])
        vel = phi_dot * np.array(
            [self._ax8 * np.cos(phi), 2.0 * self._ay8 * np.cos(2.0 * phi), 0.0])
        acc = phi_ddot * np.array(
            [self._ax8 * np.cos(phi), 2.0 * self._ay8 * np.cos(2.0 * phi), 0.0])
        acc += phi_dot**2 * np.array(
            [-self._ax8 * np.sin(phi), -4.0 * self._ay8 * np.sin(2.0 * phi), 0.0])
        return pos, vel, acc
