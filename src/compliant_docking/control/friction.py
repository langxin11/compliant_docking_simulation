"""friction.py — 关节摩擦/阻尼前馈的共享实现。

从 TaskSpaceController 已验证逻辑中逐位抽取（tests/test_friction_comp.py
与 slow 回归锚点保证行为不变），供 task_space 与 SE(3) Lie 阻抗控制器
共用，保证 iiwa14 / FR3 上多控制器对比的摩擦补偿公平性。

背景：Pinocchio 的 URDF/MJCF 导入不保留 MuJoCo 的 dof_frictionloss 与
dof_damping（被动广义力 -f·sign(q̇)、-d·q̇），故以前馈补偿：

- ``torque`` 模式（默认）：τ_ff = f·tanh(τ_pre·scale/f)，用补偿前力矩
  方向决定摩擦方向（τ₀=f/scale），零速静摩擦死区一出发即被抬过阈值；
- ``velocity`` 模式：τ_ff = f·tanh(q̇/v₀)，零速时补偿消失，低速易粘滑。
"""
from __future__ import annotations

import numpy as np

FRICTION_MODES = ("velocity", "torque")


def validate_friction_mode(mode: str) -> None:
    if mode not in FRICTION_MODES:
        raise ValueError(f"friction_mode 不支持 {mode!r}，可选 'velocity' 或 'torque'")


def friction_feedforward(tau_pre: np.ndarray, v: np.ndarray,
                         frictionloss: np.ndarray, damping: np.ndarray,
                         mode: str = "torque", tau_scale: float = 2.0,
                         v0: float = 0.01) -> np.ndarray:
    """在逆动力学力矩上叠加摩擦/阻尼前馈，返回补偿后力矩。

    Args:
        tau_pre: 补偿前关节力矩（摩擦力矩模式用其方向）
        v: 关节速度
        frictionloss: 摩擦损耗幅值（零向量 = 无补偿）
        damping: 关节阻尼系数（前馈 +damping·v）
        mode: "torque"（默认）或 "velocity"
        tau_scale: torque 模式的力矩-摩擦换算 scale（τ₀=f/scale）
        v0: velocity 模式的 tanh 平滑速度阈值 [rad/s]
    """
    validate_friction_mode(mode)
    if mode == "torque":
        tau = tau_pre + frictionloss * np.tanh(
            tau_pre * tau_scale / np.maximum(frictionloss, 1e-9))
    else:
        tau = tau_pre + frictionloss * np.tanh(v / v0)
    return tau + damping * v
