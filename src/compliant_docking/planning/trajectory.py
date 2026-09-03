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

        Parameters / 参数:
        - start_pos: 初始位置 (x0, y0, z0) / Initial position
        - target_pos: 目标位置 (xf, yf, zf) / Target position
        - duration: 轨迹持续时间（秒） / Trajectory duration (s)
        说明：三轴各自满足端点速度/加速度为零，生成 C2 连续的平滑轨迹 /
        Note: Each axis satisfies zero vel/acc at endpoints (C2 continuity)
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

        参数 / Parameters:
            t: 当前时间（秒） / Current time (s)

        返回 / Returns:
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

        参数 / Parameters:
            tol: 浮点比较容差 / Tolerance for comparisons

        返回 / Returns:
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

        参数 / Parameters:
        - start_pos: 初始位置 (3D) / Initial position
        - final_pos: 最终对接目标位置 (3D) / Final docking target position
        - standoff: 预对接点沿接近轴的后撤距离 [m] / stand-off retreat distance [m]
        - v_max_approach / a_max_approach: 接近段线速度/加速度上限 / approach phase limits
        - v_max_docking / a_max_docking: 对接段线速度/加速度上限 / docking phase limits
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

        参数 / Parameters:
            t: 当前时间（秒） / Current time (s)

        返回 / Returns:
            (pos, vel, acc) 三个 3D 向量 / 3D numpy arrays
        """
        if t < 0.0:
            return self._approach.get_state(0.0)
        if t < self._t1:
            return self._approach.get_state(t)
        # t ≥ t1：第二段按局部时间采样；超过 t2 时内部 clip 停在 final（vel/acc = 0）
        return self._docking.get_state(t - self._t1)
