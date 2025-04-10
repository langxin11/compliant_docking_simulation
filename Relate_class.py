"""
Relate_class.py - Robot Control and Trajectory Planning Library

This module implements trajectory generation and task-space control for robotic manipulators.
It provides classes for quintic polynomial trajectory planning and operational space control
using the Pinocchio robotics library.

Key components:
- DecoupledQuinticTrajectory: Generates smooth trajectories with zero velocity/acceleration at endpoints
- TaskSpaceTrajectory: Task space trajectory planning specific to robot models
- TaskSpaceController: Operational space control with impedance and orientation control
- RobotSimulator: Simple physics simulation of robot dynamics using Runge-Kutta integration

Author: RQM
Date: 2024
"""

import pinocchio as pin
import numpy as np
from scipy.linalg import pinv,inv
import matplotlib.pyplot as plt
from typing import Tuple, List



class DecoupledQuinticTrajectory:
    """Decoupled quintic polynomial trajectory planner for x, y, z directions"""
    def __init__(self, start_pos: np.ndarray, target_pos: np.ndarray, duration: float):
        """
        Initialize the trajectory planner with decoupled planning for each axis
        
        Parameters:
        start_pos: Initial position (x0, y0, z0)
        target_pos: Target position (xf, yf, zf)
        duration: Trajectory duration in seconds
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
        Solve quintic polynomial coefficients for a single direction
        
        The polynomial has the form: p(t) = a0*t^5 + a1*t^4 + a2*t^3 + a3*t^2 + a4*t + a5
        With boundary conditions:
        p(0) = p0,    p(T) = pf
        p'(0) = 0,    p'(T) = 0    (zero initial and final velocity)
        p''(0) = 0,   p''(T) = 0   (zero initial and final acceleration)
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
    
    def get_state(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get position, velocity, and acceleration at time t
        
        Parameters:
            t: Current time (seconds)
            
        Returns:
            Tuple of (position, velocity, acceleration) each as 3D numpy arrays
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
        Verify that the trajectory satisfies all boundary conditions
        
        Parameters:
            tol: Tolerance for floating point comparisons
            
        Returns:
            True if all boundary conditions are satisfied
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
    
def compute_ik(pin_model, pin_data, target_pose, initial_q=np.ones(7)*0.3, max_iters=3000, eps=1e-7):
    """
    Compute inverse kinematics using Pinocchio's functions
    
    Args:
        pin_model: Pinocchio model
        pin_data: Pinocchio data
        target_pose: pin.SE3, target end-effector pose
        initial_q: Initial joint configuration, if None uses neutral configuration
        max_iters: Maximum iterations for the IK solver
        eps: Convergence threshold
        
    Returns:
        q: Solved joint angles
        success: Whether IK converged successfully
    """
    initial_q = np.array([1.07746776 , 0.28602036, -2.47939995 , 1.29702927 , 2.96705973 , 1.6159687,-1.4298983 ])
    if initial_q is None:
        q = pin.neutral(pin_model)
    else:
        q = initial_q.copy()
        
    # Get end effector frame ID
    ee_frame_id = pin_model.getFrameId("cylinder_link")
    
    # Damping factor for numerical stability
    damp = 1e-8
    
    for i in range(max_iters):
        # Update robot kinematics
        pin.forwardKinematics(pin_model, pin_data, q)
        pin.updateFramePlacements(pin_model, pin_data)
        
        # Get current end-effector pose
        current_pose = pin_data.oMf[ee_frame_id]
        
        # Compute position error
        error_pos = target_pose.translation - current_pose.translation
        
        # Compute orientation error using matrix logarithm
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
        v = np.linalg.solve(JJt + lambda_eye, error)
        dq = Jt @ v
        
        # Update joint positions
        q = pin.integrate(pin_model, q, dq)
        
        # Joint limits handling (if needed)
        q = np.clip(q, pin_model.lowerPositionLimit, pin_model.upperPositionLimit)
    
    print("IK failed to converge")
    return q, False

class TaskSpaceTrajectory:
    """任务空间轨迹规划器"""
    def __init__(self, robot_model: pin.Model, q_init: np.ndarray, target_pos: np.ndarray, duration: float):
        """
        初始化轨迹规划器
        
        参数:
        robot_model: 机器人模型
        q_init: 初始关节角度
        target_pos: 目标位置
        duration: 轨迹持续时间
        """
        self.model = robot_model
        self.data = robot_model.createData()
        self.model.gravity.linear = np.array([0., 0., 0.])
        self.data = robot_model.createData()
        
        pin.forwardKinematics(self.model, self.data, q_init)
        pin.updateFramePlacements(self.model, self.data)
        H_init = self.data.oMf[self.model.getFrameId("cylinder_link")]
        self.p0 = H_init.translation
        
        self.pf = target_pos
        self.T = duration
        
        # 计算五次多项式轨迹参数
        self.a = self._compute_quintic_params()
    
    def _compute_quintic_params(self) -> np.ndarray:
        """计算五次多项式参数"""
        A = np.array([
            [0, 0, 0, 0, 0, 1],
            [self.T**5, self.T**4, self.T**3, self.T**2, self.T, 1],
            [0, 0, 0, 0, 1, 0],
            [5*self.T**4, 4*self.T**3, 3*self.T**2, 2*self.T, 1, 0],
            [0, 0, 0, 2, 0, 0],
            [20*self.T**3, 12*self.T**2, 6*self.T, 2, 0, 0]
        ])
        b = np.array([
            self.p0,
            self.pf,
            np.zeros_like(self.p0),
            np.zeros_like(self.p0),
            np.zeros_like(self.p0),
            np.zeros_like(self.p0)
        ])
        return np.linalg.solve(A, b)
    
    def get_state(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """获取时刻t的位置、速度和加速度"""
        t = np.clip(t, 0, self.T)
        
        pos = (self.a[0]*t**5 + self.a[1]*t**4 + self.a[2]*t**3 + 
               self.a[3]*t**2 + self.a[4]*t + self.a[5])
        
        vel = (5*self.a[0]*t**4 + 4*self.a[1]*t**3 + 3*self.a[2]*t**2 + 
               2*self.a[3]*t + self.a[4])
        
        acc = (20*self.a[0]*t**3 + 12*self.a[1]*t**2 + 6*self.a[2]*t + 
               2*self.a[3])
        
        return pos, vel, acc

class TaskSpaceController:
    """任务空间动力学控制器"""
    def __init__(self, robot_model: pin.Model, dt: float):
        self.model = robot_model
        self.model.gravity.linear = np.array([0., 0., 0.])
        self.data = self.model.createData()
        self.dt = dt
        
        self.Kp = np.diag([0.] * 3)
        self.Kd = np.diag([0.] * 3)
        
        self.q_min = np.array([-2.96706, -2.0944, -2.96706, -2.0944, -2.96706, -2.0944, -3.05433])
        self.q_max = np.array([2.96706, 2.0944, 2.96706, 2.0944, 2.96706, 2.0944, 3.05433])
        self.v_max = np.array([1.4835, 1.4835, 1.7453, 1.3090, 2.2689, 2.3562, 2.3562])
        self.end_effector_id = self.model.getFrameId("cylinder_link")

        self.initial_orientation = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]])
        
    def get_task_space_state(self, q: np.ndarray, v: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        H = self.data.oMf[self.end_effector_id]
        current_pos = H.translation
        current_ori = H.rotation
        
        J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id, pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]
        
        current_vel = J_pos @ v
        
        return current_pos, current_vel, current_ori
    
    def get_task_space_state_with_orientation(self, q: np.ndarray, v: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        H = self.data.oMf[self.end_effector_id]
        current_pos = H.translation
        current_rot = H.rotation

        orientation_error = pin.log3(self.initial_orientation @ current_rot.T)
        
        J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id, pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]
        J_rot = J[3:, :]
        
        current_vel_pos = J_pos @ v
        current_vel_rot = J_rot @ v
        
        return current_pos, current_vel_pos, orientation_error, current_vel_rot 
    
    def compute_control_task_space_with_orientation_and_imp(self, q: np.ndarray, v: np.ndarray, 
                       pos_des: np.ndarray, vel_des: np.ndarray, 
                       acc_des: np.ndarray, current_pos: np.ndarray,
                       current_vel: np.ndarray,
                       force_ext: np.ndarray, torque_ext: np.ndarray) -> np.ndarray:
        pos_cur, vel_pos_cur, ori_err, vel_rot_cur = self.get_task_space_state_with_orientation(q, v)
        vel_rot_err = -vel_rot_cur
        
        end_effector_id = self.model.getFrameId("cylinder_link")
        J = pin.computeFrameJacobian(self.model, self.data, q, end_effector_id, pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]
        J_rot = J[3:, :]
        
        M = pin.crba(self.model, self.data, q)
        M_inv = pinv(M)
        W = np.eye(7)
        
        J_dot = pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.end_effector_id, pin.ReferenceFrame.WORLD)
        
        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C
        C = C @ v.reshape(7, 1)
        C = C.reshape(7)

        force_ext = np.array(force_ext).reshape(3)
        m = 10
        d = 50
        k = 100

        force_desired = np.array([0, 0, 0])

        u_pos = acc_des + (force_ext - force_desired - d * (current_vel - vel_des) - k * (current_pos - pos_des)) / m 

        m2 = 1
        d2 = 10
        k2 = 25

        u_rot = (k2 * (ori_err) + d2 * (vel_rot_err)) / m2 

        if np.linalg.norm(u_rot) > 0.1:
            u_rot[2] = 0.001

        u = np.concatenate([u_pos, u_rot])

        J_full = np.vstack([J_pos, J_rot])
        lambda_ = W @ M_inv.T @ J_full.T @ pinv(J_full @ M_inv @ W @ M_inv.T @ J_full.T)
        
        D_null = 10 * np.eye(7)
        v_null = v
        N = (np.eye(7) - lambda_ @ J_full @ M_inv)
        null_term2 = -N @ D_null @ v_null.reshape(7)
        
        tau = lambda_ @ (u - J_dot @ v + J_full @ M_inv @ (C)) + null_term2
        
        return tau

    def compute_control_task_space_with_orientation_and_imp2(self, q: np.ndarray, v: np.ndarray, 
                       pos_des: np.ndarray, vel_des: np.ndarray, 
                       acc_des: np.ndarray, current_pos: np.ndarray,
                       current_vel: np.ndarray,
                       force_ext: np.ndarray, torque_ext: np.ndarray = np.zeros(3)) -> np.ndarray:
        pos_cur, vel_pos_cur, ori_err, vel_rot_cur = self.get_task_space_state_with_orientation(q, v)
        
        pos_err = pos_des - pos_cur
        vel_pos_err = vel_des - vel_pos_cur
        vel_rot_err = -vel_rot_cur
        
        end_effector_id = self.model.getFrameId("cylinder_link1")
        J = pin.computeFrameJacobian(self.model, self.data, q, end_effector_id, pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]
        J_rot = J[3:, :]

        M = pin.crba(self.model, self.data, q)
        M_inv = pinv(M)
        W = np.eye(7)
        
        J_dot = pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.end_effector_id, pin.ReferenceFrame.WORLD)
        
        J_dot_pos = J_dot[:3, :]
        J_dot_rot = J_dot[3:, :]

        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C
        C = C @ v.reshape(7, 1)
        C = C.reshape(7)
        
        force_ext = np.array(force_ext).reshape(3)
        m = 10 
        d = 20
        k = 100

        force_desired = np.array([0, 0, 0])

        u_pos = acc_des + (-force_desired - d * (current_vel - vel_des) - k * (current_pos - pos_des)) / m 
        u_rot = 1 * (vel_rot_err) + 1 * (ori_err)

        u = np.concatenate([u_pos, u_rot])

        J_full = np.vstack([J_pos, J_rot])
        lambda_ = pinv(J)

        D_null = 1. * np.eye(7)
        v_null = v
        N = (np.eye(7) - lambda_ @ J)
        null_term2 = -N @ D_null @ v_null.reshape(7)

        tau = M @ (lambda_ @ (u - J_dot @ v)) + C
        
        return tau

    def compute_control_task_space_with_orientation(self, q: np.ndarray, v: np.ndarray, 
                       pos_des: np.ndarray, vel_des: np.ndarray, 
                       acc_des: np.ndarray, current_pos: np.ndarray,
                       current_vel: np.ndarray) -> np.ndarray:
        pos_cur, vel_pos_cur, ori_err, vel_rot_cur = self.get_task_space_state_with_orientation(q, v)
        
        pos_err = pos_des - pos_cur
        vel_pos_err = vel_des - vel_pos_cur
        vel_rot_err = -vel_rot_cur
        
        end_effector_id = self.model.getFrameId("cylinder_link")
        J = pin.computeFrameJacobian(self.model, self.data, q, end_effector_id, pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]
        J_rot = J[3:, :]

        M = pin.crba(self.model, self.data, q)
        M_inv = pinv(M)

        W = M_inv
        
        J_dot = pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.end_effector_id, pin.ReferenceFrame.WORLD)
        
        J_dot_pos = J_dot[:3, :]
        J_dot_rot = J_dot[3:, :]

        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C
        C = C @ v.reshape(7, 1)
        C = C.reshape(7)

        u_pos = acc_des + 20 * (vel_des - current_vel) + 100 * (pos_des - current_pos)
        u_rot = 20 * (vel_rot_err) + 100 * (ori_err)

        u = np.concatenate([u_pos, u_rot])

        J_full = np.vstack([J_pos, J_rot])
        lambda_ = M @ M_inv.T @ J_full.T @ pinv(J_full @ M_inv @ M @ M_inv.T @ J_full.T)

        D_null = 1.2 * np.eye(7)
        v_null = v
        N = (np.eye(7) - lambda_ @ J_full @ M_inv)
        null_term2 = -N @ D_null @ v_null.reshape(7)

        tau = lambda_ @ (u - J_dot @ v + J_full @ M_inv @ (C)) + null_term2
        
        return tau

    def compute_control_task_space(self, q: np.ndarray, v: np.ndarray, 
                       pos_des: np.ndarray, vel_des: np.ndarray, 
                       acc_des: np.ndarray, current_pos: np.ndarray,
                       current_vel: np.ndarray) -> np.ndarray:
        pos_cur, vel_cur, _ = self.get_task_space_state(q, v)
        
        pos_err = pos_des - pos_cur
        vel_err = vel_des - vel_cur
        
        end_effector_id = self.model.getFrameId("cylinder_link")
        J = pin.computeFrameJacobian(self.model, self.data, q, end_effector_id, pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]

        M = pin.crba(self.model, self.data, q)
        M_inv = pinv(M)

        W = M_inv

        lambda_ = M @ M_inv.T @ J_pos.T @ pinv(J_pos @ M_inv @ M @ M_inv.T @ J_pos.T)
        
        J_dot_full = pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.end_effector_id, pin.ReferenceFrame.WORLD)
        
        J_dot_full = J_dot_full[:3, :]

        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C
        C = C @ v.reshape(7, 1)
        C = C.reshape(7)

        u = acc_des + 20 * (vel_des - current_vel) + 100 * (pos_des - current_pos)

        grad_m = self.manipulability_gradient(q)
        k = 0.2
        
        ddq_desired = k * grad_m

        tau_null_desired = M @ ddq_desired

        D_null = 1.2 * np.eye(7)
        v_null = v
        N = (np.eye(7) - lambda_ @ J_pos @ M_inv)
        null_term2 = -N @ D_null @ v_null.reshape(7)
        null_term = N @ tau_null_desired.reshape(7)

        tau = lambda_ @ (u - J_dot_full @ v + J_pos @ M_inv @ (C)) + null_term2
        
        return tau

    def _apply_limits(self, tau: np.ndarray, q: np.ndarray, v: np.ndarray) -> np.ndarray:
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

def compute_ik(pin_model, pin_data, target_pose, initial_q=np.ones(7)*0.3, max_iters=1000, eps=1e-6):
    if initial_q is None:
        q = pin.neutral(pin_model)
    else:
        q = initial_q.copy()
        
    ee_frame_id = pin_model.getFrameId("cylinder_link")
    damp = 1e-8
    
    for i in range(max_iters):
        pin.forwardKinematics(pin_model, pin_data, q)
        pin.updateFramePlacements(pin_model, pin_data)
        
        current_pose = pin_data.oMf[ee_frame_id]
        
        error_pos = target_pose.translation - current_pose.translation
        error_rot = pin.log3(target_pose.rotation @ current_pose.rotation.T)
        
        error = np.concatenate([error_pos, error_rot])
        
        if np.linalg.norm(error) < eps:
            print(f"IK converged in {i+1} iterations")
            return q, True
            
        pin.computeJointJacobians(pin_model, pin_data, q)
        J = pin.getFrameJacobian(pin_model, pin_data, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        
        Jt = J.T
        JJt = J @ Jt
        lambda_eye = damp * np.eye(6)
        
        v = np.linalg.solve(JJt + lambda_eye, error)
        dq = Jt @ v
        
        q = pin.integrate(pin_model, q, dq)
        q = np.clip(q, pin_model.lowerPositionLimit, pin_model.upperPositionLimit)
    
    print("IK failed to converge")
    return q, False


class RobotSimulator:
    def __init__(self, robot_model: pin.Model, dt: float):
        self.model = robot_model
        self.model.gravity.linear = np.array([0., 0., 0.])
        self.data = self.model.createData()
        self.dt = dt
        self.end_effector_id = self.model.getFrameId("cylinder_link")
        
    def compute_acceleration(self, q: np.ndarray, v: np.ndarray, tau: np.ndarray) -> np.ndarray:
        pin.computeAllTerms(self.model, self.data, q, v)
        a = pin.aba(self.model, self.data, q, v, tau)
        return a
    
    def step(self, q: np.ndarray, v: np.ndarray, tau: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id)
        J_pos = J[:3, :]
        F_ext = np.zeros(3).reshape(3,1)*0.5

        dt = self.dt  

        tau = tau + J_pos.T @ F_ext.reshape(3)
        
        a1 = self.compute_acceleration(q, v, tau)
        dq1 = v
        dv1 = a1
        
        q_mid = pin.integrate(self.model, q, 0.5 * dt * dq1)
        v_mid = v + 0.5 * dt * dv1
        a2 = self.compute_acceleration(q_mid, v_mid, tau)
        dq2 = v_mid
        dv2 = a2
        
        q_mid = pin.integrate(self.model, q, 0.5 * dt * dq2)
        v_mid = v + 0.5 * dt * dv2
        a3 = self.compute_acceleration(q_mid, v_mid, tau)
        dq3 = v_mid
        dv3 = a3
        
        q_end = pin.integrate(self.model, q, dt * dq3)
        v_end = v + dt * dv3
        a4 = self.compute_acceleration(q_end, v_end, tau)
        dq4 = v_end
        dv4 = a4
        
        dq = (dq1 + 2*dq2 + 2*dq3 + dq4) / 6.0
        dv = (dv1 + 2*dv2 + 2*dv3 + dv4) / 6.0
        
        q_next = pin.integrate(self.model, q, dt * dq)
        v_next = v + dt * dv
        
        return q_next, v_next

def cal_imp_force(current_pos, current_vel, desired_pos, desired_vel):
    K = 1000
    D = 100
    imp_force = K * (current_pos-desired_pos) + D * (current_vel-desired_vel)
    return imp_force

def run_simulation(q_init: np.ndarray):
    model = pin.buildModelFromUrdf("kuka_xml_urdf/urdf/iiwa14.urdf")
    data = model.createData()
    
    assert len(q_init) == model.nq, f"Initial joint angles must have length {model.nq}"
    assert np.all(q_init >= -3.14) and np.all(q_init <= 3.14), "Joint angles must be in radians"
    
    dt = 0.0001
    controller = TaskSpaceController(model, dt)
    simulator = RobotSimulator(model, dt)

    init_pos = np.array([0., 0.5, 0.5])
    init_ori = np.array([
        [1,  0,  0],
        [0, -1,  0],
        [0,  0, -1]
    ])

    q_init, _ = compute_ik(model, data, pin.SE3(init_ori, init_pos))
    
    q = q_init.copy()
    v = np.zeros(model.nv)

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    H_init = data.oMf[model.getFrameId("cylinder_link")]
    print(f"Initial end-effector position: {H_init.translation}")
    
    target_pos = H_init.translation + np.array([0.01, -0.02, -0.2])
    print(f"Target position: {target_pos}")
    
    duration = 5.5
    trajectory = TaskSpaceTrajectory(model, q_init, target_pos, 5)
    
    t_list = []
    pos_actual = []
    pos_desired = []
    joint_angles = []
    joint_velocities = []
    virtual_force_list = []
    tau_hist = []
    
    t = 0.0
    f_imp = np.zeros(3)
    current_pos, current_vel, _ = controller.get_task_space_state(q, v)
    
    while t < duration:
        pos_des, vel_des, acc_des = trajectory.get_state(t)
        
        tau = controller.compute_control_task_space_with_orientation(q, v, pos_des, vel_des, acc_des, current_pos, current_vel)
        tau_hist.append(tau)
        
        q, v = simulator.step(q, v, tau)
        
        current_pos, current_vel, _ = controller.get_task_space_state(q, v)
        current_pos = np.array(current_pos)
        current_vel = np.array(current_vel)

        f_imp = cal_imp_force(current_pos, current_vel, pos_des, vel_des)
        virtual_force_list.append(f_imp)
        t_list.append(t)
        pos_actual.append(current_pos)
        pos_desired.append(pos_des)
        joint_angles.append(q.copy())
        joint_velocities.append(v.copy())
        
        t += dt
    
    plot_results(t_list, pos_actual, pos_desired, joint_angles, joint_velocities, virtual_force_list, tau_hist)

def plot_results(t_list: List[float], pos_actual: List[np.ndarray], 
                pos_desired: List[np.ndarray], joint_angles: List[np.ndarray],
                joint_velocities: List[np.ndarray], virtual_force_list: List[np.ndarray],
                tau_hist: List[np.ndarray]):
    pos_actual = np.array(pos_actual)
    pos_desired = np.array(pos_desired)
    joint_angles = np.array(joint_angles)
    joint_velocities = np.array(joint_velocities)
    virtual_force_list = np.array(virtual_force_list)
    tau_hist = np.array(tau_hist)
    
    fig = plt.figure(figsize=(15, 12))
    gs = plt.GridSpec(3, 2)
    
    ax1 = fig.add_subplot(gs[0, :])
    labels = ['X', 'Y', 'Z']
    for i in range(3):
        ax1.plot(t_list, pos_actual[:, i], '-', label=f'Actual {labels[i]}')
        ax1.plot(t_list, pos_desired[:, i], '--', label=f'Desired {labels[i]}')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Position [m]')
    ax1.legend()
    ax1.grid(True)
    ax1.set_title('End-effector Position Tracking')
    
    ax2 = fig.add_subplot(gs[1, :])
    for i in range(3):
        error = pos_desired[:, i] - pos_actual[:, i]
        ax2.plot(t_list, error, label=f'{labels[i]} Error')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Error [m]')
    ax2.legend()
    ax2.grid(True)
    ax2.set_title('Position Tracking Error')
    
    ax3 = fig.add_subplot(gs[2, 0])
    for i in range(6):
        ax3.plot(t_list, np.rad2deg(joint_angles[:, i]), label=f'Joint {i+1}')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Joint Angle [deg]')
    ax3.legend()
    ax3.grid(True)
    ax3.set_title('Joint Angles')

    ax4 = fig.add_subplot(gs[2, 1])
    for i in range(3):
        ax4.plot(t_list[:], virtual_force_list[:, i], label=f'axis {i+1}')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Virtual Force [N]')
    ax4.legend()
    ax4.grid(True)
    ax4.set_title('Impedance Force')

    plt.show()
    
    fig = plt.figure(figsize=(15, 12))
    gs = plt.GridSpec(7, 1)

    for i in range(7):
        ax1 = fig.add_subplot(gs[i, 0])
        ax1.plot(t_list, tau_hist[:, i], label=f'Joint {i+1}')
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Torque [Nm]')
        ax1.legend()
        ax1.grid(True)
        ax1.set_title('Joint Torques')

    plt.show()

if __name__ == '__main__':
    q_init = np.ones(7) * 0.3
    run_simulation(q_init)  