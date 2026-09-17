"""
motion_reference.py — 轨迹规划器 → SE(3) body 运动参考适配器

把既有轨迹接口适配为 SE(3) Lie 阻抗控制器（Kim et al. 2025 Eq. 44-64）
需要的 (T_d, V_d, Vdot_d) 期望 body 运动量，隔离在控制器之外：

- ``SE3ToppTrajectory``（有 get_motion_state）：直接透传——段内常螺旋
  严格满足 V_d = ξ·ṡ、Vdot_d = ξ·s̈；
- 既有纯位置轨迹（DecoupledQuintic / TwoPhaseDocking / CircleFigure8，
  仅 get_state）：以固定期望姿态 r_des 构造 T_d = SE3(r_des, pos)，姿态
  恒定时 ω_d = 0，线量转 desired body 系：V_d = [R_dᵀ·vel; 0]、
  Vdot_d = [R_dᵀ·acc; 0]（R_d 常量故 body twist 导数即旋转后的线加速度）。
"""
from __future__ import annotations

import numpy as np
import pinocchio as pin


def get_motion_reference(planner, t: float,
                         r_des: np.ndarray) -> tuple[pin.SE3, np.ndarray, np.ndarray]:
    """采样 t 时刻的 (T_d, V_d, Vdot_d) 期望 body 运动参考。

    Args:
        planner: 轨迹规划器（SE3ToppTrajectory 或任何提供 get_state 的
            纯位置规划器）
        t: 采样时刻 [s]
        r_des: 固定期望姿态（世界系 3×3；仅纯位置轨迹使用）

    Returns:
        T_d: 期望位姿（pin.SE3）
        V_d: 期望 body twist (6,) = [v_d; ω_d]
        Vdot_d: 期望 body twist 的时间导数 (6,)
    """
    if hasattr(planner, "get_motion_state"):
        return planner.get_motion_state(t)

    pos, vel, acc = planner.get_state(t)
    R_d = np.asarray(r_des, dtype=float).reshape(3, 3)
    T_d = pin.SE3(R_d, np.asarray(pos, dtype=float).reshape(3))
    V_d = np.concatenate([R_d.T @ vel, np.zeros(3)])
    Vdot_d = np.concatenate([R_d.T @ acc, np.zeros(3)])
    return T_d, V_d, Vdot_d
