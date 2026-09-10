"""
momentum_observer.py - PI 广义动量观测器 / PI generalized momentum observer

Ren & Shan 2026 (Acta Astronautica) Eq.(23)-(25)、De Luca 框架的精确模型实现：

    p = M(q)q̇,   β = g(q) - C(q,q̇)ᵀq̇
    p̃̇ = τ_applied - β + τ̃_ext
    τ̃_ext = K_p·Δp + K_i·∫Δp dt,   Δp = p̃ - p
    F̂_ext = (Jᵀ)# τ̃_ext,   (Jᵀ)# = (J Jᵀ)⁻¹ J   （动力学一致伪逆，6×7）

与期末项目参考实现的区别：β 不用有限差分近似——Pinocchio 可精确计算
g（computeGeneralizedGravity）与 Cᵀq̇（computeCoriolisMatrix 后取
data.Cᵀ·q̇；由恒等式 Ṁ = C + Cᵀ，该形式与动量微分严格一致）。

残差 τ̃_ext 收敛到全部未建模广义力：JᵀF_ext + 关节摩擦/阻尼等。
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from ..control.task_space import _FRAME_REFERENCE


class MomentumObserver:
    """无传感器外力估计：PI 动量观测器（世界系 LWA 口径）。

    用法（每控制步，τ_applied 为实际施加——限幅后的力矩）：
        tau_ext = obs.update(q, v, tau_applied)
        force = obs.force   # 世界系 3 维外力估计（同一步的 J 映射）
    """

    def __init__(self, robot_model: pin.Model, ee_frame: str, dt: float, *,
                 kp: float = 20.0, ki: float = 40.0,
                 integral_limit: float = 20.0,
                 frictionloss: np.ndarray | None = None,
                 damping: np.ndarray | None = None,
                 friction_v0: float = 0.01):
        """
        参数 / Args:
            kp/ki: 观测器比例/积分增益 [s⁻¹]（K_p 量纲 1/s，K_i 1/s²）
            integral_limit: 积分项 ∫Δp 的逐关节限幅 [N·m·s]，防饱和
            frictionloss/damping: 已知关节耗散模型（与仿真同源）。提供后
                ``force_contact`` 从残差中扣除耗散项，得到不含摩擦污染的
                纯接触力估计——控制器的前馈已补偿同一模型，残差中的耗散
                分量对力控而言是可减去的已知项
            friction_v0: tanh 平滑化速度阈值 [rad/s]（与控制器一致）
        """
        self.model = robot_model
        self.data = robot_model.createData()
        self.dt = float(dt)
        self.kp = float(kp)
        self.ki = float(ki)
        self.integral_limit = float(integral_limit)
        self.frame_id = robot_model.getFrameId(ee_frame)
        n = robot_model.nv
        self.frictionloss = (np.zeros(n) if frictionloss is None
                             else np.asarray(frictionloss, dtype=float).reshape(n))
        self.damping = (np.zeros(n) if damping is None
                        else np.asarray(damping, dtype=float).reshape(n))
        self.friction_v0 = float(friction_v0)

        self.momentum_hat = np.zeros(n)
        self.integral_err = np.zeros(n)
        self.tau_ext = np.zeros(n)
        self._wrench = np.zeros(6)
        self._wrench_contact = np.zeros(6)
        self._started = False

    @property
    def wrench(self) -> np.ndarray:
        """最新外力/外力矩估计（世界系 LWA，6 维，含摩擦污染）。"""
        return self._wrench.copy()

    @property
    def force(self) -> np.ndarray:
        """最新外力估计（世界系，3 维，含摩擦污染）。"""
        return self._wrench[:3].copy()

    @property
    def wrench_contact(self) -> np.ndarray:
        """扣除已知耗散模型后的纯接触力/力矩估计（世界系 LWA，6 维）。"""
        return self._wrench_contact.copy()

    @property
    def force_contact(self) -> np.ndarray:
        """扣除已知耗散模型后的纯接触力估计（世界系，3 维）。"""
        return self._wrench_contact[:3].copy()

    def reset(self, q: np.ndarray, v: np.ndarray) -> None:
        """以当前状态初始化动量估计（避免初值阶跃）。"""
        M = pin.crba(self.model, self.data, q)
        self.momentum_hat = np.array(M) @ np.asarray(v, dtype=float).reshape(-1)
        self.integral_err[:] = 0.0
        self.tau_ext[:] = 0.0
        self._started = True

    def update(self, q: np.ndarray, v: np.ndarray, tau_applied: np.ndarray) -> np.ndarray:
        """推进一步观测器，返回关节外力矩估计 τ̃_ext（n 维）。

        参数 / Args:
            q, v: 当前关节状态（步进后）
            tau_applied: 上一控制周期实际施加的关节力矩（限幅后）
        """
        q = np.asarray(q, dtype=float).reshape(-1)
        v = np.asarray(v, dtype=float).reshape(-1)
        tau_applied = np.asarray(tau_applied, dtype=float).reshape(-1)
        if not self._started:
            self.reset(q, v)

        # p = M q̇ 与 β = g - Cᵀq̇（精确模型项；重力置零时 g=0）
        M = np.array(pin.crba(self.model, self.data, q))
        M = np.triu(M) + np.triu(M, 1).T
        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        g = pin.computeGeneralizedGravity(self.model, self.data, q)
        beta = np.asarray(g) - np.array(self.data.C).T @ v
        p = M @ v

        # PI 残差：误差 e = p - p̂（真减估），r = K_p·e + K_i·∫e（积分限幅防饱和）。
        # 注意符号方向：该形式误差系统 ė = τ_e - r 稳定（特征多项式
        # s² + K_p·s + K_i 全负实部）；若按 Δp = p̃ - p 正馈则为鞍点发散
        delta_p = p - self.momentum_hat
        self.integral_err = np.clip(
            self.integral_err + self.dt * delta_p,
            -self.integral_limit, self.integral_limit)
        self.tau_ext = self.kp * delta_p + self.ki * self.integral_err

        # 动量估计递推：p̂̇ = τ_applied - β + r
        self.momentum_hat = self.momentum_hat + self.dt * (
            tau_applied - beta + self.tau_ext)

        # 任务空间映射：F̂ = (J Jᵀ)⁻¹ J τ̃_ext（世界系 LWA）
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.frame_id)
        J = np.array(pin.getFrameJacobian(
            self.model, self.data, self.frame_id, _FRAME_REFERENCE))
        self._wrench = np.linalg.solve(J @ J.T + 1e-9 * np.eye(6), J @ self.tau_ext)
        # 纯接触估计：扣除已知耗散模型（控制器前馈补偿的同一项）
        tau_diss = (self.frictionloss * np.tanh(v / self.friction_v0)
                    + self.damping * v)
        self._wrench_contact = np.linalg.solve(
            J @ J.T + 1e-9 * np.eye(6), J @ (self.tau_ext - tau_diss))
        return self.tau_ext.copy()
