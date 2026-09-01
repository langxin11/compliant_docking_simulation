"""仿真与控制参数配置：集中管理，避免魔法数字散落各处。"""
from dataclasses import dataclass, field


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
    stroke: tuple = (0.0, 0.0, -0.18)  # 相对初始位置的对接行程
    max_torque: float = 10.0  # 关节力矩限幅 [N·m]
    impedance: ImpedanceConfig = field(default_factory=ImpedanceConfig)
