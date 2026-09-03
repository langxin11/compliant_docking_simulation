"""
task_space.py - Robot Task-Space Control Library

This module implements task-space (operational space) control for robotic
manipulators using the Pinocchio robotics library.

Key components:
- TaskSpaceController: Operational space control with impedance and orientation control

Author: langxin11
Date: 2025
"""


import numpy as np
import pinocchio as pin
from scipy.linalg import pinv

from ..config import ImpedanceConfig


class TaskSpaceController:
    """
    任务空间动力学控制器（平动+姿态阻抗，动力学一致映射，零空间阻尼）/
    Task-space dynamics controller (translation + rotation impedance; dynamics-consistent mapping; null damping)
    """
    def __init__(self, robot_model: pin.Model, dt: float,
                 impedance: ImpedanceConfig | None = None,
                 ee_frame: str = "cylinder_link"):
        """
        初始化控制器：设定 Pinocchio 模型、步长与阻抗参数 /
        Initialize controller: set Pinocchio model, time step and impedance params

        参数 / Args:
            robot_model: Pinocchio 模型 / Pinocchio model
            dt: 控制步长 [s] / control time step
            impedance: 阻抗参数（None 时取 ImpedanceConfig 默认值） / impedance params
            ee_frame: 末端 frame 名（由模型/场景决定；默认值为组合 URDF 的
                ``cylinder_link``，即 iiwa14 + 公头圆柱场景的历史名称） /
                end-effector frame name (decided by model/scene; default is the
                historical name of the iiwa14 combined URDF)

        注意：重力置零由 compliant_docking.models.load_pin_model 负责（加载时统一处理）/
        Note: gravity zeroing is owned by compliant_docking.models.load_pin_model

        注意：末端 frame 由 ee_frame 决定，方法内所有矩阵/向量维数均按
        ``model.nq`` 泛化（iiwa14 nq=7 时与历史实现数值逐位一致）。
        """
        self.model = robot_model
        self.data = self.model.createData()
        self.dt = dt
        self.impedance = impedance or ImpedanceConfig()

        self.Kp = np.diag([0.] * 3)
        self.Kd = np.diag([0.] * 3)

        # 末端 frame id（由 ee_frame 参数决定，方法内统一复用）
        self.end_effector_id = self.model.getFrameId(ee_frame)

        # 以下限位/速度上限数组为 iiwa14 专用经验值（仅 _apply_limits 使用，主回路不调用）
        self.q_min = np.array([-2.96706, -2.0944, -2.96706, -2.0944, -2.96706, -2.0944, -3.05433])
        self.q_max = np.array([2.96706, 2.0944, 2.96706, 2.0944, 2.96706, 2.0944, 3.05433])
        self.v_max = np.array([1.4835, 1.4835, 1.7453, 1.3090, 2.2689, 2.3562, 2.3562])

        # 期望初始姿态（固定朝向），用于 log3 误差
        self.initial_orientation = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]])

    def get_task_space_state(self, q: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        计算当前末端位置与线速度（世界系）/
        Compute current end-effector position and linear velocity (world frame)
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        H = self.data.oMf[self.end_effector_id]
        current_pos = H.translation
        current_ori = H.rotation

        J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id, pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]

        current_vel = J_pos @ v  # 末端线速度（线速度雅可比 J_pos 乘关节速度）

        return current_pos, current_vel, current_ori

    def get_task_space_state_with_orientation(self, q: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        返回末端位置、线/角速度，以及相对于期望姿态的李代数姿态误差 /
        Return EE position, linear/angular velocity, and orientation error (log map)
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        H = self.data.oMf[self.end_effector_id]
        current_pos = H.translation
        current_rot = H.rotation

        # 姿态误差：log(Rd Rc^T)
        orientation_error = pin.log3(self.initial_orientation @ current_rot.T)

        J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id, pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]
        J_rot = J[3:, :]

        current_vel_pos = J_pos @ v  # 末端线速度
        current_vel_rot = J_rot @ v  # 末端角速度

        return current_pos, current_vel_pos, orientation_error, current_vel_rot

    def compute_control_task_space_with_orientation_and_imp(self, q: np.ndarray, v: np.ndarray,
                       pos_des: np.ndarray, vel_des: np.ndarray,
                       acc_des: np.ndarray, current_pos: np.ndarray,
                       current_vel: np.ndarray,
                       force_ext: np.ndarray, torque_ext: np.ndarray) -> np.ndarray:
        """
        任务空间控制（含姿态 + 阻抗 + 外力补偿）：输出关节力矩 /
        Task-space control (orientation + impedance + external force): output joint torques

        参数 / Args: q, v 当前关节状态 / current joints; pos/vel/acc_des 期望项 / desired;
        current_pos/vel 实际项 / current; force/torque_ext 外力 / external
        返回 / Returns: tau 关节力矩 / joint torques
        """
        # 1) 获取任务空间状态：当前位置、姿态误差（log 映射）、线/角速度
        pos_cur, vel_pos_cur, ori_err, vel_rot_cur = self.get_task_space_state_with_orientation(q, v)
        # 姿态的速度误差（希望角速度为0）：当前角速度取负
        vel_rot_err = -vel_rot_cur

        # 2) 计算末端雅可比（世界系），拆分为平动与旋转部分
        J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id, pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]
        J_rot = J[3:, :]

        # 3) 机器人动力学项：广义质量矩阵 M 及其伪逆，权重矩阵 W（此处取单位阵）
        M = pin.crba(self.model, self.data, q)  # 质量矩阵
        M_inv = pinv(M)
        W = np.eye(self.model.nq)

        # 4) 雅可比的时间变化项 J_dot（用于前馈/补偿项）
        J_dot = pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.end_effector_id, pin.ReferenceFrame.WORLD)

        # 5) 科氏/离心项：C(q, v)·v（转为一维向量表示广义力）
        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C
        C = C @ v.reshape(-1, 1)  # 广义科氏/离心项乘以速度，得到广义力形式
        C = C.reshape(self.model.nq)

        # 6) 平动阻抗参数与外力（参数由 ImpedanceConfig 集中管理） /
        # 6) Translational impedance params and external force (owned by ImpedanceConfig)
        force_ext = np.array(force_ext).reshape(3)
        imp = self.impedance

        # 期望外力（此处为0，可根据任务需要设置）
        force_desired = np.array([0, 0, 0])

        # 7) 平动阻抗：Md (xdd - xdd_des) + Dd (xd - xd_des) + Kd (x - x_des) = F_ext - F_des
        #    整理得到期望操作空间加速度/力输入 u_pos
        u_pos = (acc_des + (force_ext - force_desired - imp.d * (current_vel - vel_des)
                            - imp.k * (current_pos - pos_des)) / imp.m)

        # 8) 姿态阻抗：类似 PD，在角速度误差与姿态误差上施加控制 /
        # 8) Rotational impedance: PD-like control on orientation/angular-velocity errors
        u_rot = (imp.k_rot * (ori_err) + imp.d_rot * (vel_rot_err)) / imp.m_rot

        # 防止旋转控制过大（对 z 轴分量做简单限幅示例）
        if np.linalg.norm(u_rot) > 0.1:
            u_rot[2] = 0.001

        # 拼接平动与旋转的任务输入（6维）
        u = np.concatenate([u_pos, u_rot])

        # 10) 组合雅可比并计算动力学一致映射矩阵（加权广义逆）
        J_full = np.vstack([J_pos, J_rot])
        # 动力学一致映射矩阵（操作空间惯性的变体实现），将任务输入映射为关节力矩
        lambda_ = W @ M_inv.T @ J_full.T @ pinv(J_full @ M_inv @ W @ M_inv.T @ J_full.T)

        # 11) 零空间阻尼：抑制未约束自由度的速度振荡（阻尼由 ImpedanceConfig 提供） /
        # 11) Null-space damping (coefficient from ImpedanceConfig)
        D_null = imp.null_damping * np.eye(self.model.nq)
        v_null = v
        N = (np.eye(self.model.nq) - lambda_ @ J_full @ M_inv)
        null_term2 = -N @ D_null @ v_null.reshape(self.model.nq)

        # 12) 合成关节力矩：主任务项（含前馈与科氏/离心补偿）+ 零空间阻尼
        tau = lambda_ @ (u - J_dot @ v + J_full @ M_inv @ (C)) + null_term2

        return tau

    def compute_control_task_space_with_orientation_and_imp2(self, q: np.ndarray, v: np.ndarray,
                       pos_des: np.ndarray, vel_des: np.ndarray,
                       acc_des: np.ndarray, current_pos: np.ndarray,
                       current_vel: np.ndarray,
                       force_ext: np.ndarray, torque_ext: np.ndarray | None = None) -> np.ndarray:
        pos_cur, vel_pos_cur, ori_err, vel_rot_cur = self.get_task_space_state_with_orientation(q, v)

        vel_rot_err = -vel_rot_cur

        J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id, pin.ReferenceFrame.WORLD)

        M = pin.crba(self.model, self.data, q)

        J_dot = pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.end_effector_id, pin.ReferenceFrame.WORLD)

        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C
        C = C @ v.reshape(-1, 1)
        C = C.reshape(self.model.nq)

        force_ext = np.array(force_ext).reshape(3)
        m = 10
        d = 20
        k = 100

        force_desired = np.array([0, 0, 0])

        u_pos = acc_des + (-force_desired - d * (current_vel - vel_des) - k * (current_pos - pos_des)) / m
        u_rot = 1 * (vel_rot_err) + 1 * (ori_err)

        u = np.concatenate([u_pos, u_rot])

        lambda_ = pinv(J)

        tau = M @ (lambda_ @ (u - J_dot @ v)) + C

        return tau

    def compute_control_task_space_with_orientation(self, q: np.ndarray, v: np.ndarray,
                       pos_des: np.ndarray, vel_des: np.ndarray,
                       acc_des: np.ndarray, current_pos: np.ndarray,
                       current_vel: np.ndarray) -> np.ndarray:
        """
        任务空间控制（含姿态 PD，但不显式使用外力）/
        Task-space control (with orientation PD; no explicit external force)
        """
        pos_cur, vel_pos_cur, ori_err, vel_rot_cur = self.get_task_space_state_with_orientation(q, v)

        vel_rot_err = -vel_rot_cur

        J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id, pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]
        J_rot = J[3:, :]

        M = pin.crba(self.model, self.data, q)
        M_inv = pinv(M)

        J_dot = pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.end_effector_id, pin.ReferenceFrame.WORLD)

        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C
        C = C @ v.reshape(-1, 1)
        C = C.reshape(self.model.nq)

        u_pos = acc_des + 20 * (vel_des - current_vel) + 100 * (pos_des - current_pos)
        u_rot = 20 * (vel_rot_err) + 100 * (ori_err)

        u = np.concatenate([u_pos, u_rot])

        J_full = np.vstack([J_pos, J_rot])
        lambda_ = M @ M_inv.T @ J_full.T @ pinv(J_full @ M_inv @ M @ M_inv.T @ J_full.T)

        D_null = 1.2 * np.eye(self.model.nq)
        v_null = v
        N = (np.eye(self.model.nq) - lambda_ @ J_full @ M_inv)
        null_term2 = -N @ D_null @ v_null.reshape(self.model.nq)

        tau = lambda_ @ (u - J_dot @ v + J_full @ M_inv @ (C)) + null_term2

        return tau

    def compute_control_task_space(self, q: np.ndarray, v: np.ndarray,
                       pos_des: np.ndarray, vel_des: np.ndarray,
                       acc_des: np.ndarray, current_pos: np.ndarray,
                       current_vel: np.ndarray) -> np.ndarray:
        """
        仅平动任务空间控制（不包含姿态）/
        Task-space control for translation only (no orientation)
        """
        pos_cur, vel_cur, _ = self.get_task_space_state(q, v)

        J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id, pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]

        M = pin.crba(self.model, self.data, q)
        M_inv = pinv(M)

        lambda_ = M @ M_inv.T @ J_pos.T @ pinv(J_pos @ M_inv @ M @ M_inv.T @ J_pos.T)

        J_dot_full = pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.end_effector_id, pin.ReferenceFrame.WORLD)

        J_dot_full = J_dot_full[:3, :]

        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C
        C = C @ v.reshape(-1, 1)
        C = C.reshape(self.model.nq)

        u = acc_des + 20 * (vel_des - current_vel) + 100 * (pos_des - current_pos)

        # 保留调用以维持对 self.data 的任何副作用（与原实现一致）
        self.manipulability_gradient(q)

        D_null = 1.2 * np.eye(self.model.nq)
        v_null = v
        N = (np.eye(self.model.nq) - lambda_ @ J_pos @ M_inv)
        null_term2 = -N @ D_null @ v_null.reshape(self.model.nq)

        tau = lambda_ @ (u - J_dot_full @ v + J_pos @ M_inv @ (C)) + null_term2

        return tau

    def _apply_limits(self, tau: np.ndarray, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """
        软限位与速度缩放（示例）：约束越界趋势并按速度限制缩放力矩 /
        Soft joint limits and velocity scaling (example implementation)
        """
        k_limit = 100.0
        tau_limit = np.zeros_like(tau)
        for i in range(len(q)):
            if q[i] < self.q_min[i]:
                tau_limit[i] = k_limit * (self.q_min[i] - q[i])
            elif q[i] > self.q_max[i]:
                tau_limit[i] = k_limit * (self.q_max[i] - q[i])

        v_scale = np.minimum(1.0, self.v_max / (np.abs(v) + 1e-6))
        tau = tau * v_scale

        return tau + tau_limit

    def manipulability_gradient(self, q, delta=1e-6):
        """
        计算操控度梯度（ Yoshikawa 指标 det(JJ^T) 的数值梯度 ）/
        Compute manipulability gradient (numerical) for det(JJ^T)
        """
        grad = np.zeros_like(q)
        J_current = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id)[:3, :]
        manipulability_current = np.linalg.det(J_current @ J_current.T)

        for i in range(len(q)):
            q_delta = q.copy()
            q_delta[i] += delta
            J_delta = pin.computeFrameJacobian(self.model, self.data, q_delta, self.end_effector_id)[:3, :]
            manipulability_delta = np.linalg.det(J_delta @ J_delta.T)
            grad[i] = (manipulability_delta - manipulability_current) / delta
        return grad
