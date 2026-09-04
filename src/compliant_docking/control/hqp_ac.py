"""
hqp_ac.py - HQP-AC 分层二次规划自适应控制器（Ren & Shan 2026, §3.2）

将关节位置/速度/力矩极限作为 QP 硬约束（ZOH 短时域预测），刚度按接触力
自适应，零空间做奇异性规避与关节位姿阻抗。

公式编号对照 Ren & Shan 2026 (Acta Astronautica)：
- Eq.(26) 自适应因子 α = 1/(1+exp(-k_alpha·F))
- Eq.(27) 参考刚度 K_r = clip((1-α)·K0, K_min, K0)
- Eq.(29) 参考阻尼 D_r = Λ^(1/2)K_r^(1/2) + K_r^(1/2)Λ^(1/2)
- Eq.(31) 主任务 QP：min ‖Jq̈ + J̇q̇ - (ν̇_r + Λ⁻¹F_r)‖²
- Eq.(34)-(36) 关节位置/速度/力矩硬约束（ZOH 时域 dt_p 预测）
- Eq.(38)-(40) 可操作度 ω 及其梯度、奇异性规避加速度 q̈_sa
- Eq.(41) 关节位姿阻抗 q̈_ji = -D_ji·v - K_ji·(q - q_col)
- Eq.(42) 零空间 QP 目标 a_null = k_sa·q̈_sa + k_ji·q̈_ji
- Eq.(43) 动力学一致广义逆 J# = M⁻¹JᵀΛ 与零空间投影 N = I - J#J

注意：论文的无传感器动量观测器（力估计）不在本实现范围内——本实现直接使用
仿真提供的末端六维力/力矩传感器输入（与 TaskSpaceController 主控制器同
口径）；无传感器方案留作后续工作。
"""

import time

import numpy as np
import pinocchio as pin
import proxsuite

from ..config import HQPConfig, ImpedanceConfig

# ProxQP 稠密 QP 与求解状态枚举（proxsuite 0.7.x 需经属性访问导入）
_DenseQP = proxsuite.proxqp.dense.QP
_QP_SOLVED = proxsuite.proxqp.QPSolverOutput.PROXQP_SOLVED

# 主任务 H 矩阵数值正则化量（保证严格凸）
_H_REG = 1e-8
_FRAME_REFERENCE = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED


class HQPAdaptiveController:
    """HQP-AC 控制器：约束 QP 主任务 + 自适应刚度 + 零空间奇异性规避/关节位姿阻抗。

    与 TaskSpaceController 鸭子类型兼容：提供同签名的
    ``compute_control_task_space_with_orientation_and_imp`` 与
    ``get_task_space_state``，可在 experiments/run_docking.py 中直接互换。
    """

    def __init__(self, robot_model: pin.Model, dt: float,
                 config: HQPConfig | None = None,
                 ee_frame: str = "cylinder_link",
                 r_des: np.ndarray | None = None,
                 frictionloss: np.ndarray | None = None,
                 damping: np.ndarray | None = None,
                 impedance: ImpedanceConfig | None = None,
                 friction_mode: str = "velocity",
                 friction_tau_scale: float = 2.0):
        """初始化控制器：预解析限位并预建两个 ProxQP 实例（主任务/零空间）。

        参数 / Args:
            robot_model: Pinocchio 模型（重力置零由 load_pin_model 负责）
            dt: 控制步长 [s]
            config: HQPConfig 参数（None 时取默认值）
            ee_frame: 末端 frame 名（与 TaskSpaceController 同口径）
            r_des: 期望姿态（世界系 3×3）；None 时取
                ``[[1,0,0],[0,-1,0],[0,0,-1]]``（与 TaskSpaceController
                的 initial_orientation 同口径）
            frictionloss: 关节摩擦损耗幅值 [N·m]（nv 维；None 时取零向量）。
                Pinocchio 导入器不保留 MJCF frictionloss，需由调用方从组装
                MjModel 的 dof_frictionloss 传入；以前馈 τ_ff = frictionloss·
                tanh(q̇/v₀) 并入 ĥ（同时进入力矩硬约束与输出力矩）

        QP 实例复用策略：proxsuite 支持 ``qp.update(...)`` 原地更新 H/g/C/u，
        两个实例在 __init__ 各建一次，之后每个控制步只 update+solve，
        不再重新构造。
        """
        self.model = robot_model
        self.data = self.model.createData()
        self.dt = dt
        self.config = config or HQPConfig()
        if impedance is not None:
            # 场景阻抗覆盖（ImpedanceOverride 的 k/d/k_rot/d_rot 可选字段 → K0）
            from dataclasses import replace

            cfg = self.config
            K0 = cfg.K0.copy()
            for i, val in enumerate((impedance.k, impedance.k, impedance.k,
                                     impedance.k_rot, impedance.k_rot, impedance.k_rot)):
                if val is not None:
                    K0[i] = float(val)
            self.config = replace(cfg, K0=K0)
        cfg = self.config

        self.frame_id = self.model.getFrameId(ee_frame)
        self.n = self.model.nv

        # 摩擦前馈幅值（零向量 = 无补偿，行为与历史实现一致）
        self.frictionloss = (np.zeros(self.n) if frictionloss is None
                             else np.asarray(frictionloss, dtype=float).reshape(self.n))
        self.damping = (np.zeros(self.n) if damping is None
                        else np.asarray(damping, dtype=float).reshape(self.n))
        self._friction_v0 = 0.01  # tanh 平滑化速度阈值 [rad/s]
        # 摩擦前馈模式："velocity"（τ_ff=f·tanh(q̇/v₀)）或 "torque"
        # （τ_ff=f·tanh(τ_pre/τ₀)，用补偿前力矩方向治零速死区；τ₀=f/scale），
        # 与 TaskSpaceController 同语义
        if friction_mode not in ("velocity", "torque"):
            raise ValueError(f"friction_mode 不支持 {friction_mode!r}，可选 'velocity' 或 'torque'")
        self.friction_mode = friction_mode
        self.friction_tau_scale = float(friction_tau_scale)

        # 期望姿态（世界系 3×3）
        if r_des is None:
            self.r_des = np.array([[1.0, 0.0, 0.0],
                                   [0.0, -1.0, 0.0],
                                   [0.0, 0.0, -1.0]])
        else:
            self.r_des = np.array(r_des, dtype=float)

        # 限位预解析：q_min/q_max、v_max/v_min（≤0 的轴视作无限制）、τ_max/τ_min
        self.q_min = np.array(self.model.lowerPositionLimit, dtype=float).copy()
        self.q_max = np.array(self.model.upperPositionLimit, dtype=float).copy()
        self._v_max = np.array(self.model.velocityLimit, dtype=float).copy()
        self._v_max[self._v_max <= 0.0] = np.inf
        if cfg.torque_limit is None:
            self._tau_max = np.array(self.model.effortLimit, dtype=float).copy()
        else:
            self._tau_max = np.full(self.n, float(cfg.torque_limit))
        self._tau_min = -self._tau_max

        # 自适应刚度上下界（对角向量）
        self._K0 = np.array(cfg.K0, dtype=float).reshape(6)
        self._K_min = cfg.K_min_ratio * self._K0

        # 预建两个 ProxQP 实例：n 变量、0 等式、6n 不等式
        # （速度 2n + 位置 2n + 力矩 2n，见 _constraint_matrices）
        self._n_in = 6 * self.n
        self._qp_main = _DenseQP(self.n, 0, self._n_in)
        self._qp_null = _DenseQP(self.n, 0, self._n_in)
        self._l_inf = np.full(self._n_in, -np.inf)
        for qp in (self._qp_main, self._qp_null):
            qp.settings.eps_abs = cfg.eps_abs
            qp.init(np.eye(self.n), np.zeros(self.n), None, None,
                    np.zeros((self._n_in, self.n)), self._l_inf, np.zeros(self._n_in))

        # 诊断状态
        self.q_col: np.ndarray | None = None  # Eq.(41) 关节位姿阻抗参考（首次调用捕获）
        self.n_solver_failures = 0
        self.last_K_r: np.ndarray | None = None
        self._solve_time_sum = 0.0
        self._n_solves = 0

    # ------------------------------------------------------------------
    # 只读诊断属性
    # ------------------------------------------------------------------
    @property
    def last_solve_time_ms(self) -> float:
        """主+零空间 QP 求解耗时滚动均值 [ms]（累计平均；尚无求解时为 0.0）。"""
        if self._n_solves == 0:
            return 0.0
        return 1e3 * self._solve_time_sum / self._n_solves

    # ------------------------------------------------------------------
    # 状态接口（与 TaskSpaceController 同签名，供 run_simulation 复用）
    # ------------------------------------------------------------------
    def get_task_space_state(self, q: np.ndarray, v: np.ndarray) -> tuple:
        """返回末端位置、线速度（世界系）与旋转矩阵（世界系 3×3）。"""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        H = self.data.oMf[self.frame_id]
        J = np.array(pin.computeFrameJacobian(
            self.model, self.data, q, self.frame_id, _FRAME_REFERENCE))
        return H.translation.copy(), J[:3, :] @ v, H.rotation.copy()

    # ------------------------------------------------------------------
    # 内部计算（论文公式逐条对应）
    # ------------------------------------------------------------------
    def _adaptive_stiffness(self, F: float) -> np.ndarray:
        """Eq.(26)(27)：按接触力幅值自适应的参考刚度（6 维对角向量）。"""
        alpha = 1.0 / (1.0 + np.exp(-self.config.k_alpha * F))
        return np.clip((1.0 - alpha) * self._K0, self._K_min, self._K0)

    @staticmethod
    def _reference_damping(Lambda: np.ndarray, K_r: np.ndarray) -> np.ndarray:
        """Eq.(29)：D_r = Λ^(1/2)K_r^(1/2) + K_r^(1/2)Λ^(1/2)。

        K_r 为对角向量（开方取逐元素 sqrt）；Λ 的对称正定平方根经
        ``eigh`` 实现（特征值截断到非负，数值稳健且恒为实矩阵）。
        """
        w, V = np.linalg.eigh(Lambda)
        sqrt_L = (V * np.sqrt(np.clip(w, 0.0, None))) @ V.T
        sqrt_K = np.diag(np.sqrt(np.clip(K_r, 0.0, None)))
        return sqrt_L @ sqrt_K + sqrt_K @ sqrt_L

    def _manipulability(self, q: np.ndarray) -> float:
        """Eq.(38)：ω = sqrt(det(J Jᵀ))（6 维、世界轴对齐 frame 原点雅可比）。"""
        J = np.array(pin.computeFrameJacobian(
            self.model, self.data, q, self.frame_id, _FRAME_REFERENCE))
        return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))

    def _manipulability_gradient(self, q: np.ndarray, delta: float = 1e-6) -> np.ndarray:
        """Eq.(39)：可操作度梯度的数值差分（写法参照 task_space.manipulability_gradient）。"""
        grad = np.zeros(self.n)
        w0 = self._manipulability(q)
        for i in range(self.n):
            q_delta = q.copy()
            q_delta[i] += delta
            grad[i] = (self._manipulability(q_delta) - w0) / delta
        return grad

    def _constraint_matrices(self, q: np.ndarray, v: np.ndarray,
                             M: np.ndarray, h: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Eq.(34)-(36)：ZOH 时域 dt_p 下的关节位置/速度/力矩硬约束。

        全部堆叠为 ``C_mat q̈ ≤ u``（下界侧乘 -1 并入，行下界 l = -inf）：
        行 0..n-1        速度上限：v + dt_p·q̈ ≤ v_max
        行 n..2n-1       速度下限：-(v + dt_p·q̈) ≤ v_max
        行 2n..3n-1      位置上限：q + v·dt_p + ½dt_p²·q̈ ≤ q_max
        行 3n..4n-1      位置下限：-(q + v·dt_p + ½dt_p²·q̈) ≤ -q_min
        行 4n..5n-1      力矩上限：M·q̈ ≤ τ_max - ĥ
        行 5n..6n-1      力矩下限：-M·q̈ ≤ ĥ - τ_min
        其中 ĥ = C(q,v)·v（科氏/离心广义力）。
        """
        n = self.n
        dt_p = self.config.dt_p
        eye = np.eye(n)
        C_mat = np.zeros((self._n_in, n))
        u = np.zeros(self._n_in)
        C_mat[:n] = dt_p * eye
        u[:n] = self._v_max - v
        C_mat[n:2 * n] = -dt_p * eye
        u[n:2 * n] = self._v_max + v
        half_dt2 = 0.5 * dt_p * dt_p
        C_mat[2 * n:3 * n] = half_dt2 * eye
        u[2 * n:3 * n] = self.q_max - q - v * dt_p
        C_mat[3 * n:4 * n] = -half_dt2 * eye
        u[3 * n:4 * n] = q + v * dt_p - self.q_min
        C_mat[4 * n:5 * n] = M
        u[4 * n:5 * n] = self._tau_max - h
        C_mat[5 * n:6 * n] = -M
        u[5 * n:6 * n] = h - self._tau_min
        return C_mat, u

    def _solve_qp(self, qp, H: np.ndarray, g: np.ndarray,
                  C_mat: np.ndarray, u: np.ndarray) -> np.ndarray | None:
        """更新并求解一个 ProxQP 实例；非 solved 返回 None（调用方回退）。"""
        qp.update(H, g, None, None, C_mat, self._l_inf, u)
        qp.solve()
        if qp.results.info.status == _QP_SOLVED:
            return np.array(qp.results.x)
        return None

    # ------------------------------------------------------------------
    # 主入口：与 TaskSpaceController.compute_control_task_space_with_orientation_and_imp
    # 同名同签名（刻意的鸭子类型约定，便于 run_docking 直接换控制器）
    # ------------------------------------------------------------------
    def compute_control_task_space_with_orientation_and_imp(
            self, q: np.ndarray, v: np.ndarray,
            pos_des: np.ndarray, vel_des: np.ndarray,
            acc_des: np.ndarray, current_pos: np.ndarray,
            current_vel: np.ndarray,
            force_ext: np.ndarray, torque_ext: np.ndarray) -> np.ndarray:
        """HQP-AC 控制律：返回关节力矩 τ = M·q̈_c + ĥ。

        参数 / Args: 与 TaskSpaceController 同名方法完全一致——
            q, v 关节状态；pos/vel/acc_des 期望任务位置/速度/加速度；
            current_pos/vel 实际末端位置/线速度；
            force_ext/torque_ext 世界系末端外力/外力矩（3 维各）。
        返回 / Returns: 关节力矩 τ（n 维）。

        回退语义 / Fallback:
            主 QP 非 solved → 该步主任务分量回退为无约束最小二乘解
            （min‖Jq̈-(target-J̇q̇)‖²），并跳过零空间 QP；
            零空间 QP 非 solved → 零空间分量取 0。
            每次失败 ``n_solver_failures`` 自增 1。

        说明：直接使用 F/T 传感器输入（无传感器动量观测器留作后续）。
        """
        q = np.array(q, dtype=float).reshape(self.n)
        v = np.array(v, dtype=float).reshape(self.n)
        pos_des = np.array(pos_des, dtype=float).reshape(3)
        vel_des = np.array(vel_des, dtype=float).reshape(3)
        acc_des = np.array(acc_des, dtype=float).reshape(3)
        current_pos = np.array(current_pos, dtype=float).reshape(3)
        current_vel = np.array(current_vel, dtype=float).reshape(3)
        force_ext = np.array(force_ext, dtype=float).reshape(3)
        torque_ext = np.array(torque_ext, dtype=float).reshape(3)

        # Eq.(41) 的 q_col：首次调用捕获关节位姿
        if self.q_col is None:
            self.q_col = q.copy()

        # 1) FK + 世界轴对齐 frame 原点雅可比及同参考系时间导数。
        pin.forwardKinematics(self.model, self.data, q, v)
        pin.computeJointJacobiansTimeVariation(self.model, self.data, q, v)
        pin.updateFramePlacements(self.model, self.data)
        R_cur = np.array(self.data.oMf[self.frame_id].rotation)
        J = np.array(pin.getFrameJacobian(
            self.model, self.data, self.frame_id, _FRAME_REFERENCE))
        J_dot = np.array(pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.frame_id, _FRAME_REFERENCE))
        J_rot = J[3:, :]

        # 2) 动力学项：M（对称化）、ĥ = C(q,v)·v
        M = np.array(pin.crba(self.model, self.data, q))
        M = np.triu(M) + np.triu(M, 1).T  # crba 只保证上三角，显式对称化
        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        h = np.array(self.data.C) @ v
        # 关节摩擦前馈（Pinocchio 模型不含 frictionloss，仿真侧有）：
        # 并入 ĥ 使力矩硬约束与输出力矩自动一致；torque 模式用补偿前 ĥ
        # 的方向决定摩擦方向（与 TaskSpaceController 同语义）
        if self.friction_mode == "torque":
            h = h + self.frictionloss * np.tanh(
                h * self.friction_tau_scale / np.maximum(self.frictionloss, 1e-9))
        else:
            h = h + self.frictionloss * np.tanh(v / self._friction_v0)
        h = h + self.damping * v

        # 任务空间惯性 Λ 及其逆（Λ⁻¹ = J M⁻¹ Jᵀ，pinv 稳健化后对称化）
        M_inv = np.linalg.pinv(M)
        A_task = J @ M_inv @ J.T
        Lambda = np.linalg.pinv(A_task)
        Lambda = 0.5 * (Lambda + Lambda.T)

        # 3) 误差与自适应刚度：Eq.(26)(27)
        #    姿态误差取 log(R_cur·R_desᵀ)（"当前相对期望"，与 e_pos=current-desired
        #    同向），保证 ė_p = Δν = [ẋ-ẋ_d; ω] 严格成立，使 Eq.(29)(31) 的统一式
        #    F_r = -D_r·Δν - K_r·e_p 在平动/旋转两通道都是耗散阻抗
        #    （ë_p + Λ⁻¹D_r·ė_p + Λ⁻¹K_r·e_p = 0）。注意：TaskSpaceController 的
        #    ori_err = log(R_des·R_curᵀ) 与 u_rot = +k_rot·ori_err + d_rot·(-ω)
        #    在代数上与本处约定完全等价（相差一个整体负号）。
        e_pos = current_pos - pos_des
        e_ori = pin.log3(R_cur @ self.r_des.T)
        delta_nu = np.concatenate([current_vel - vel_des, J_rot @ v])  # 期望角速度为 0
        e_p = np.concatenate([e_pos, e_ori])
        F_contact = float(np.linalg.norm(np.concatenate([force_ext, torque_ext])))
        K_r = self._adaptive_stiffness(F_contact)
        self.last_K_r = K_r

        # 4) 参考阻尼与广义力：Eq.(29)
        D_r = self._reference_damping(Lambda, K_r)
        F_r = -D_r @ delta_nu - K_r * e_p

        # 5) 主任务 QP（Eq.31）：min ‖Jq̈ + J̇q̇ - target‖²，target = ν̇_r + Λ⁻¹F_r
        nu_dot_r = np.concatenate([acc_des, np.zeros(3)])
        target = nu_dot_r + A_task @ F_r
        H_main = 2.0 * (J.T @ J) + _H_REG * np.eye(self.n)
        g_main = 2.0 * (J.T @ (J_dot @ v - target))

        # 6) 硬约束（Eq.34-36，ZOH 时域 dt_p）
        C_mat, u = self._constraint_matrices(q, v, M, h)

        t_start = time.perf_counter()
        a_m = self._solve_qp(self._qp_main, H_main, g_main, C_mat, u)
        t_main = time.perf_counter() - t_start
        if a_m is not None:
            q_ddot_m = a_m
        else:
            # 回退：无约束最小二乘（主任务分量）
            self.n_solver_failures += 1
            q_ddot_m = np.linalg.lstsq(J, target - J_dot @ v, rcond=None)[0]
            self._solve_time_sum += t_main
            self._n_solves += 1
            return M @ q_ddot_m + h  # 主 QP 失败 → 跳过零空间

        # 7) 零空间 QP（Eq.38-43）：奇异性规避 + 关节位姿阻抗
        omega = self._manipulability(q)
        q_ddot_sa = np.zeros(self.n)
        if omega < self.config.omega_th:
            grad_w = self._manipulability_gradient(q)
            norm_gw = float(np.linalg.norm(grad_w))
            if norm_gw > 1e-12:
                q_ddot_sa = ((self.config.omega_th - omega) / self.config.omega_th) \
                    * grad_w / norm_gw
        q_ddot_ji = -self.config.D_ji @ v - self.config.K_ji @ (q - self.q_col)
        a_null = self.config.k_sa * q_ddot_sa + self.config.k_ji * q_ddot_ji

        J_pinv = M_inv @ J.T @ Lambda  # Eq.(43)：J# = M⁻¹JᵀΛ
        N = np.eye(self.n) - J_pinv @ J
        H_null = (2.0 + _H_REG) * np.eye(self.n)
        g_null = -2.0 * a_null
        u_null = u - C_mat @ q_ddot_m

        t_null_start = time.perf_counter()
        a_n = self._solve_qp(self._qp_null, H_null, g_null, C_mat @ N, u_null)
        self._solve_time_sum += t_main + (time.perf_counter() - t_null_start)
        self._n_solves += 1
        if a_n is not None:
            q_ddot_n = a_n
        else:
            # 回退：零空间分量取 0
            self.n_solver_failures += 1
            q_ddot_n = np.zeros(self.n)

        # 8) 合成：q̈_c = q̈*_m + N·q̈*_null，τ = M·q̈_c + ĥ
        q_ddot_c = q_ddot_m + N @ q_ddot_n
        return M @ q_ddot_c + h
