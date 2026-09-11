"""
kinematics.py - Robot Inverse Kinematics

This module implements inverse kinematics for robotic manipulators
using the Pinocchio robotics library (damped least squares).

Key components:
- compute_ik: Damped-least-squares IK returning joint configuration and convergence flag

Author: langxin11
Date: 2025
"""

import numpy as np
import pinocchio as pin

# compute_ik 的默认初始关节角（模块级单例，避免在参数默认值中调用函数）
_DEFAULT_INITIAL_Q = np.ones(7) * 0.3


def compute_ik(pin_model, pin_data, target_pose, initial_q=_DEFAULT_INITIAL_Q, max_iters=3000, eps=1e-7,
               *, ee_frame: str = "cylinder_link"):
    """
    使用 Pinocchio 进行逆运动学（阻尼最小二乘）：返回关节角与是否收敛 /
    Compute inverse kinematics (damped least squares) using Pinocchio

    Args:
        pin_model (pin.Model): Pinocchio 模型 / Pinocchio model
        pin_data (pin.Data): Pinocchio 数据 / Pinocchio data
        target_pose (pin.SE3): 目标末端位姿 pin.SE3 / target end-effector pose
        initial_q (np.ndarray | None): 初始关节角，None 则取 neutral / initial joint config
        max_iters (int): 最大迭代步数 / maximum iterations
        eps (float): 收敛阈值 / convergence threshold
        ee_frame: 末端 frame 名（由模型/场景决定；默认值为组合 URDF 的
            ``cylinder_link``，即 iiwa14 + 公头圆柱场景的历史名称） /
            end-effector frame name (decided by model/scene; default is the
            historical name of the iiwa14 combined URDF)

    Returns:
        q (np.ndarray): 关节角解 / joint configuration
        success (bool): 是否收敛 / convergence flag
    """
    # 若未提供初始值，则使用模型的中性位姿作为初值
    if initial_q is None:
        q = pin.neutral(pin_model)
    else:
        q = initial_q.copy()

    # Get end effector frame ID（frame 名由 ee_frame 参数决定，默认为历史名称）
    ee_frame_id = pin_model.getFrameId(ee_frame)

    # Damping factor for numerical stability
    damp = 1e-8  # 阻尼因子，提高最小二乘求解的数值稳定性

    for i in range(max_iters):
        # Update robot kinematics
        pin.forwardKinematics(pin_model, pin_data, q)
        pin.updateFramePlacements(pin_model, pin_data)

        # Get current end-effector pose
        current_pose = pin_data.oMf[ee_frame_id]

        # Compute position error
        error_pos = target_pose.translation - current_pose.translation

        # Compute orientation error using matrix logarithm
        # 姿态误差采用李代数对数映射：log(Rd Rc^T)
        error_rot = pin.log3(target_pose.rotation @ current_pose.rotation.T)

        # Combine errors
        error = np.concatenate([error_pos, error_rot])

        # Check convergence
        if np.linalg.norm(error) < eps:
            print(f"IK converged in {i+1} iterations")
            return q, True

        # Compute task Jacobian
        pin.computeJointJacobians(pin_model, pin_data, q)
        J = pin.getFrameJacobian(pin_model, pin_data, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        # Compute joint update using damped least squares
        Jt = J.T
        JJt = J @ Jt
        lambda_eye = damp * np.eye(6)  # 6 DOF task space

        # Solve using damped least squares
        # 阻尼最小二乘（等价于 J^T (J J^T + λI)^{-1} e）
        v = np.linalg.solve(JJt + lambda_eye, error)
        dq = Jt @ v

        # Update joint positions
        q = pin.integrate(pin_model, q, dq)

        # Joint limits handling (if needed)
        q = np.clip(q, pin_model.lowerPositionLimit, pin_model.upperPositionLimit)

    print("IK failed to converge")
    return q, False
