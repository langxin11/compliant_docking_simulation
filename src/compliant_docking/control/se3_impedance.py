"""
se3_impedance.py — SE(3) Lie 群阻抗控制器（Kim et al. 2025, T-RO Vol. 41）

实现论文 "Impedance Control Design Framework Using Commutative Map Between
SE(3) and se(3)" Section III-A（Eq. 44-66）：在 SE(3) 上设计阻抗、在 se(3)
指数坐标空间设计控制的"绕行策略"。**不含** Section III-B 的 NRIC 鲁棒内环
（明确留作后续独立工作）。

约定（与论文、Pinocchio Motion 一致）：

- twist ``V = [v; ω]``（线量在前）、wrench ``F = [f; n]``
- 相对位姿 ``T̃ = T⁻¹·T_d``（Eq. 44，{b} 系表达的位移）
- 相对 twist 取**右平移**版本（Eq. 45）：``Ṽ = Ad_T̃·V_d − V``，
  对应 ``[Ṽ] = Ṫ̃·T̃⁻¹``（有限差分验证见 tests/test_se3_impedance.py）
- ``λ = log(T̃)``、``λ̇ = dexp⁻¹_λ·Ṽ``（Eq. 47-48）。注意：此处 dexp 是
  **右平凡化**版（lie_se3.py 的约定 vee(ṪT⁻¹)=dexp(λ)λ̇），T̃ 的"空间系"
  恰为 {b} 系，故 Ṽ 与 dexp 配对自洽；**不是** dexp_{-λ}
- 外部 wrench（Eq. 50）：``F̃ = Ad_T̃^{-T}·F_d − F``，F 为 F/T 传感器
  body wrench（wrench_to_body 提供，与 body Jacobian 同 frame 同参考点）
- 等效有效 wrench（Eq. 56）：``γ = dexp_λᵀ·F̃``
- Lie 代数阻抗（Eq. 57-58）：A_λ = dexpᵀAdexp，D_λ = dexpᵀ(D·dexp+A·dẋpexp)，
  K_λ = K；D_λ 一般不对称，不强加对称化
- 参考加速度（Eq. 60-61）：λ̈_ref = −K_V λ̇ − K_P λ + K_F γ
- 参考 twist 导数（Eq. 64）：V̇_ref = Ad_T̃·V̇_d − dexp·λ̈_ref + ad_Ṽ·V − d/dt dexp·λ̇
- 7-DoF 适配（Eq. 66 的冗余安全版）：J⁻¹ → 动力学一致广义逆
  J_bar = M⁻¹JᵀΛ，Λ=(JM⁻¹Jᵀ)⁻¹；零空间 N = I − J_bar·J 只加稳定阻尼
- 逆动力学（Eq. 65）：τ = M·q̈_ref + ĥ − Jᵀ·F（ĥ = C v + g，本仓库重力置零）

不继承、不复用 TaskSpaceController 的阻抗方程（其姿态通道是 log3 PD，
非本论文方法）；仅共享 control/friction.py 摩擦前馈。
"""
from __future__ import annotations

import warnings

import numpy as np
import pinocchio as pin

from ..config import SE3ImpedanceConfig
from ..wrench import transform_wrench  # noqa: F401  (re-export convenience)
from .friction import friction_feedforward, validate_friction_mode
from .lie_se3 import ad6, adjoint, adjoint_wrench, dexp_dot_se3, dexp_inv_se3, dexp_se3

# run 循环鸭子类型用的世界轴对齐参考系（仅 get_task_space_state / 遥测使用；
# 控制律本体全部使用 ReferenceFrame.LOCAL 的 body Jacobian）
_FRAME_REFERENCE_LWA = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
_FRAME_REFERENCE_BODY = pin.ReferenceFrame.LOCAL


class SE3LieImpedanceController:
    """SE(3) Lie 群阻抗控制器（Kim et al. 2025 §III-A，Eq. 44-66）。

    与 TaskSpaceController / HQPAdaptiveController 并列的第三种控制器：
    主任务严格使用 body（LOCAL）Jacobian 与 body wrench，渲染论文的
    A/D/K 六维阻抗；冗余自由度仅提供动力学一致的零空间阻尼。
    """

    def __init__(self, robot_model: pin.Model, dt: float,
                 config: SE3ImpedanceConfig | None = None,
                 ee_frame: str = "cylinder_link",
                 frictionloss: np.ndarray | None = None,
                 damping: np.ndarray | None = None,
                 friction_mode: str = "torque",
                 friction_tau_scale: float = 2.0,
                 A: np.ndarray | None = None,
                 D: np.ndarray | None = None,
                 K: np.ndarray | None = None):
        """初始化控制器。

        Args:
            robot_model: Pinocchio 模型（重力置零由 load_pin_model 负责）
            dt: 控制步长 [s]
            config: SE3ImpedanceConfig（None 取默认值）
            ee_frame: 末端 frame 名（与场景 robot.ee_frame 同口径）
            frictionloss / damping / friction_mode / friction_tau_scale:
                摩擦/阻尼前馈（与 TaskSpaceController 同源 helper，
                保证多控制器对比公平）
            A / D / K: 一般 6×6 阻抗矩阵直接注入（默认取 config 对角构造；
                数据结构始终支持非对角一般矩阵）
        """
        self.model = robot_model
        self.data = self.model.createData()
        self.dt = dt
        self.config = config or SE3ImpedanceConfig()
        cfg = self.config

        # 阻抗矩阵：内部恒为一般 6×6（对角配置只是特例）
        self.A = (np.diag(np.asarray(cfg.A_diag, dtype=float)) if A is None
                  else np.asarray(A, dtype=float).reshape(6, 6))
        self.D = (np.diag(np.asarray(cfg.D_diag, dtype=float)) if D is None
                  else np.asarray(D, dtype=float).reshape(6, 6))
        self.K = (np.diag(np.asarray(cfg.K_diag, dtype=float)) if K is None
                  else np.asarray(K, dtype=float).reshape(6, 6))

        self.nv = self.model.nv
        self.end_effector_id = self.model.getFrameId(ee_frame)

        self.frictionloss = (np.zeros(self.nv) if frictionloss is None
                             else np.asarray(frictionloss, dtype=float).reshape(self.nv))
        self.damping = (np.zeros(self.nv) if damping is None
                        else np.asarray(damping, dtype=float).reshape(self.nv))
        self._friction_v0 = 0.01
        validate_friction_mode(friction_mode)
        self.friction_mode = friction_mode
        self.friction_tau_scale = float(friction_tau_scale)

        # Λ 条件数告警只发一次（避免每步刷屏），诊断每步更新
        self._condition_warned = False
        # 每步最新诊断（telemetry / 测试探针；键见 _store_diagnostics）
        self.latest_diagnostics: dict[str, float | np.ndarray] = {}

    # ------------------------------------------------------------------
    # run 循环鸭子类型接口（世界系遥测用；控制律不走此路径）
    # ------------------------------------------------------------------
    def get_task_space_state(self, q: np.ndarray, v: np.ndarray):
        """末端位置、线速度（世界系）与姿态矩阵（与既有控制器同口径）。"""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        H = self.data.oMf[self.end_effector_id]
        J = np.array(pin.computeFrameJacobian(
            self.model, self.data, q, self.end_effector_id, _FRAME_REFERENCE_LWA))
        return H.translation.copy(), J[:3, :] @ v, H.rotation.copy()

    def get_body_state(self, q: np.ndarray, v: np.ndarray):
        """EE body 位姿 T、body Jacobian J（LOCAL）与 V = J·v（Eq. 44 前置）。

        返回的 T 是拷贝（data.oMf 返回内部缓冲引用，连续调用会别名覆盖）。
        """
        pin.forwardKinematics(self.model, self.data, q, v)
        pin.computeJointJacobiansTimeVariation(self.model, self.data, q, v)
        pin.updateFramePlacements(self.model, self.data)
        T = pin.SE3(self.data.oMf[self.end_effector_id])
        J = np.array(pin.getFrameJacobian(
            self.model, self.data, self.end_effector_id, _FRAME_REFERENCE_BODY))
        Jdot = np.array(pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.end_effector_id, _FRAME_REFERENCE_BODY))
        return T, J, Jdot

    # ------------------------------------------------------------------
    # 主控制律
    # ------------------------------------------------------------------
    def compute_control(self, q: np.ndarray, v: np.ndarray,
                        T_d: pin.SE3, V_d: np.ndarray, Vdot_d: np.ndarray,
                        F_body: np.ndarray,
                        F_d: np.ndarray | None = None) -> np.ndarray:
        """SE(3) Lie 群阻抗控制律（论文 Eq. 44-66），返回关节力矩。

        Args:
            q, v: 关节位置/速度（MuJoCo 传感值，控制器仅经传感认识物理世界）
            T_d / V_d / Vdot_d: 期望位姿与 body twist 及其导数
                （motion_reference.get_motion_reference 适配）
            F_body: F/T 传感器 body wrench [f; n]（EE frame 原点参考，
                wrench.wrench_to_body 提供；与 body Jacobian 同 frame）
            F_d: 期望 wrench（默认零；论文 Eq. 50 项，API 预留）

        Returns:
            tau: 关节力矩（nv 维）
        """
        q = np.asarray(q, dtype=float).reshape(self.model.nq)
        v = np.asarray(v, dtype=float).reshape(self.nv)
        V_d = np.asarray(V_d, dtype=float).reshape(6)
        Vdot_d = np.asarray(Vdot_d, dtype=float).reshape(6)
        F_body = np.asarray(F_body, dtype=float).reshape(6)
        F_d = (np.zeros(6) if F_d is None
               else np.asarray(F_d, dtype=float).reshape(6))

        # 1) body 运动学量（LOCAL）：T、J、J̇、V = J v（要求 V ↔ vee(T⁻¹Ṫ)，
        #    有限差分验证见 tests/test_se3_impedance.py::test_frame_consistency）
        T, J, Jdot = self.get_body_state(q, v)
        V = J @ v

        # 2) 相对位姿与指数坐标（Eq. 44-48）
        T_tilde = T.inverse() * T_d
        lam = np.array(pin.log6(T_tilde).vector)          # λ = [η; ξ]
        Ad_Tt = adjoint(T_tilde)
        V_tilde = Ad_Tt @ V_d - V                          # Eq. 45（右平移版本）
        dexp = dexp_se3(lam)
        dexp_inv = dexp_inv_se3(lam)
        lam_dot = dexp_inv @ V_tilde                       # Eq. 48
        dexp_dot = dexp_dot_se3(lam, lam_dot)

        # 3) 外部 wrench 与等效有效 wrench（Eq. 50 / 56）
        F_tilde = adjoint_wrench(T_tilde) @ F_d - F_body   # F_d=0 时 F̃ = -F
        gamma = dexp.T @ F_tilde                           # Eq. 56

        # 4) 参考加速度（Eq. 60-61）；矩阵逆一律 solve，不做隐式 pinv
        #    K_V = dexp⁻¹(A⁻¹ D dexp + d/dt dexp)
        #    K_P = dexp⁻¹ A⁻¹ dexp⁻ᵀ K；K_F = dexp⁻¹ A⁻¹ dexp⁻ᵀ（注意不含 K）
        X = np.linalg.solve(self.A, self.D @ dexp)
        Kv = np.linalg.solve(dexp, X + dexp_dot)
        Y = np.linalg.solve(dexp.T, self.K)
        Kp = np.linalg.solve(dexp, np.linalg.solve(self.A, Y))
        Kf = np.linalg.solve(
            dexp, np.linalg.solve(self.A, np.linalg.solve(dexp.T, np.eye(6))))
        lam_ddot_ref = -Kv @ lam_dot - Kp @ lam + Kf @ gamma

        # 5) 参考 body twist 导数（Eq. 64）
        Vdot_ref = (Ad_Tt @ Vdot_d
                    - dexp @ lam_ddot_ref
                    + ad6(V_tilde) @ V
                    - dexp_dot @ lam_dot)

        # 6) 7-DoF 冗余适配（Eq. 66 的动力学一致广义逆版）
        M = np.array(pin.crba(self.model, self.data, q))
        M = np.triu(M) + np.triu(M, 1).T                    # crba 只保证上三角
        M_inv = np.linalg.solve(M, np.eye(self.nv))
        A_task = J @ M_inv @ J.T
        cond_task = float(np.linalg.cond(A_task))
        if cond_task < self.config.condition_threshold:
            Lambda = np.linalg.solve(A_task, np.eye(6))
        else:
            # 近奇异降级：带阈值的 pinv（诊断记录，不静默掩盖——首达告警）
            if not self._condition_warned:
                warnings.warn(
                    f"SE3 impedance: 任务空间矩阵条件数 {cond_task:.3e} 超过阈值 "
                    f"{self.config.condition_threshold:.1e}，降级为阻尼 pinv",
                    stacklevel=2)
                self._condition_warned = True
            Lambda = np.linalg.pinv(A_task, rcond=1.0 / self.config.condition_threshold)
        J_bar = M_inv @ J.T @ Lambda
        N = np.eye(self.nv) - J_bar @ J
        qdd_null = -self.config.null_damping * v            # 零空间只做稳定阻尼
        qdd_ref = J_bar @ (Vdot_ref - Jdot @ v) + N @ qdd_null

        # 7) 逆动力学（Eq. 65）：ĥ = C(q,v)v + g(q)（g≡0，与模型定义一致）
        h = np.array(pin.rnea(self.model, self.data, q, v, np.zeros(self.nv)))
        tau = M @ qdd_ref + h - J.T @ F_body

        # 8) 摩擦/阻尼前馈（与 TaskSpaceController 共享 helper，公平对比）
        tau = friction_feedforward(tau, v, self.frictionloss, self.damping,
                                   mode=self.friction_mode,
                                   tau_scale=self.friction_tau_scale,
                                   v0=self._friction_v0)

        self._store_diagnostics(lam, lam_dot, lam_ddot_ref, V_tilde, Vdot_ref,
                                qdd_ref, dexp, cond_task, F_body, tau)
        return tau

    def _store_diagnostics(self, lam, lam_dot, lam_ddot_ref, V_tilde, Vdot_ref,
                           qdd_ref, dexp, cond_task, F_body, tau) -> None:
        """保存本步诊断（telemetry 选择性记录；键集合保持稳定）。"""
        self.latest_diagnostics = {
            "lam_translation_norm": float(np.linalg.norm(lam[:3])),
            "lam_rotation_norm": float(np.linalg.norm(lam[3:])),
            "lam_dot_norm": float(np.linalg.norm(lam_dot)),
            "lam": lam.copy(),
            "lam_dot": lam_dot.copy(),
            "lam_ddot_ref": lam_ddot_ref.copy(),
            "V_tilde": V_tilde.copy(),
            "Vdot_ref": Vdot_ref.copy(),
            "qdd_ref": qdd_ref.copy(),
            "cond_dexp": float(np.linalg.cond(dexp)),
            "cond_task": cond_task,
            "F_body_norm": float(np.linalg.norm(F_body)),
            "tau_norm": float(np.linalg.norm(tau)),
        }
