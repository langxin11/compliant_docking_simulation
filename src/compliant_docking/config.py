"""仿真与控制参数配置：集中管理，避免魔法数字散落各处。"""
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class ImpedanceConfig:
    """操作空间阻抗参数（平动 m/d/k + 姿态 m_rot/d_rot/k_rot）与零空间阻尼。"""

    m: float = 10.0
    d: float = 50.0
    k: float = 100.0
    m_rot: float = 1.0
    d_rot: float = 10.0
    k_rot: float = 25.0
    null_damping: float = 10.0


@dataclass(frozen=True)
class DockingConfig:
    """对接任务与仿真设置。"""

    dt: float = 0.001
    traj_duration: float = 15.0
    duration: float = 18.0
    max_torque: float = 10.0  # 关节力矩限幅 [N·m]
    impedance: ImpedanceConfig = field(default_factory=ImpedanceConfig)


@dataclass(frozen=True)
class HQPConfig:
    """HQP-AC（分层二次规划自适应控制）参数。

    取值参照 Ren & Shan 2026 (Acta Astronautica) 第 3.2 节与 Table D.12；
    供 control.hqp_ac.HQPAdaptiveController 使用。

    字段 / Fields:
        K0: 初始参考刚度对角向量（6 维：平动 3 + 姿态 3），Eq.(27)
        K_min_ratio: K_min = K_min_ratio·K0（逐元素），Eq.(27) 下界
        k_alpha: 自适应刚度 sigmoid 增益，Eq.(26)
        omega_th: 可操作度奇异性阈值，Eq.(39)
        k_sa: 奇异性规避任务权重，Eq.(42)
        K_ji / D_ji: 关节位姿阻抗刚度/阻尼（7×7），Eq.(41)
        k_ji: 关节位姿阻抗任务权重，Eq.(42)
        dt_p: ZOH 短时域预测步长 [s]，Eq.(34)-(36) 约束预测用
        torque_limit: 关节力矩约束幅值 [N·m]；None 时取 model.effortLimit
        eps_abs: ProxQP 求解绝对精度
    """

    K0: np.ndarray = field(
        default_factory=lambda: np.array([300.0, 300.0, 300.0, 50.0, 50.0, 50.0]))
    K_min_ratio: float = 0.3
    k_alpha: float = 0.5
    omega_th: float = 0.08
    k_sa: float = 500.0
    K_ji: np.ndarray = field(default_factory=lambda: 5.0 * np.eye(7))
    D_ji: np.ndarray = field(default_factory=lambda: 10.0 * np.eye(7))
    k_ji: float = 1.0
    dt_p: float = 0.05
    torque_limit: float | None = None
    eps_abs: float = 1e-5

