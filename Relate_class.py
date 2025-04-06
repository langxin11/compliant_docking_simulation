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
        # Verify inputs
        assert start_pos.shape == (3,), "Start position must be 3D vector"
        assert target_pos.shape == (3,), "Target position must be 3D vector"
        assert duration > 0, "Duration must be positive"
        
        self.p0 = start_pos
        self.pf = target_pos
        self.T = duration
        
        # Calculate coefficients for each direction
        self.ax = self._solve_quintic_coefficients(start_pos[0], target_pos[0])
        self.ay = self._solve_quintic_coefficients(start_pos[1], target_pos[1])
        self.az = self._solve_quintic_coefficients(start_pos[2], target_pos[2])
        
        # Store coefficients in a more organized way
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
        # Construct the coefficient matrix
        A = np.array([
            [0, 0, 0, 0, 0, 1],           # position at t=0
            [self.T**5, self.T**4, self.T**3, self.T**2, self.T, 1],  # position at t=T
            [0, 0, 0, 0, 1, 0],           # velocity at t=0
            [5*self.T**4, 4*self.T**3, 3*self.T**2, 2*self.T, 1, 0],  # velocity at t=T
            [0, 0, 0, 2, 0, 0],           # acceleration at t=0
            [20*self.T**3, 12*self.T**2, 6*self.T, 2, 0, 0]           # acceleration at t=T
        ])
        
        # Construct the boundary conditions vector
        b = np.array([p0, pf, 0, 0, 0, 0])
        
        # Solve the system of equations
        return np.linalg.solve(A, b)
    
    def get_state(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get position, velocity, and acceleration at time t
        
        Parameters:
            t: Current time (seconds)
            
        Returns:
            Tuple of (position, velocity, acceleration) each as 3D numpy arrays
        """
        # Ensure time is within bounds
        t = np.clip(t, 0, self.T)
        
        # Initialize arrays for storing results
        pos = np.zeros(3)
        vel = np.zeros(3)
        acc = np.zeros(3)
        
        # Time vector for position
        t_pos = np.array([t**5, t**4, t**3, t**2, t, 1])
        # Time vector for velocity
        t_vel = np.array([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0])
        # Time vector for acceleration
        t_acc = np.array([20*t**3, 12*t**2, 6*t, 2, 0, 0])
        
        # Calculate state for each direction
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
        # Check start conditions
        pos_start, vel_start, acc_start = self.get_state(0)
        # Check end conditions
        pos_end, vel_end, acc_end = self.get_state(self.T)
        
        # Verify all conditions
        conditions = [
            np.allclose(pos_start, self.p0, atol=tol),
            np.allclose(pos_end, self.pf, atol=tol),
            np.allclose(vel_start, np.zeros(3), atol=tol),
            np.allclose(vel_end, np.zeros(3), atol=tol),
            np.allclose(acc_start, np.zeros(3), atol=tol),
            np.allclose(acc_end, np.zeros(3), atol=tol)
        ]
        
        return all(conditions)

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
        
        # 计算初始位置
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
        
        # KUKA iiwa14控制器增益
        self.Kp = np.diag([0.] * 3)  # 位置增益
        self.Kd = np.diag([0.] * 3)   # 速度增益
        
        # KUKA iiwa14关节限位
        self.q_min = np.array([-2.96706, -2.0944, -2.96706, -2.0944, -2.96706, -2.0944, -3.05433])
        self.q_max = np.array([2.96706, 2.0944, 2.96706, 2.0944, 2.96706, 2.0944, 3.05433])
        self.v_max = np.array([1.4835, 1.4835, 1.7453, 1.3090, 2.2689, 2.3562, 2.3562])
        self.end_effector_id = self.model.getFrameId("cylinder_link")

        self.initial_orientation = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1] ])
        
    def get_task_space_state(self, q: np.ndarray, v: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """获取末端执行器的位置和速度"""
        # 更新机器人构型
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        # 获取末端执行器位置
        
        H = self.data.oMf[self.end_effector_id]
        current_pos = H.translation  # 位置向量
        current_ori = H.rotation  # 姿态矩阵
        
        # 计算雅可比矩阵
        J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id,pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]  # 只取位置部分
        
        # 计算末端速度
        current_vel = J_pos @ v
        
        return current_pos, current_vel , current_ori
    
    def get_task_space_state_with_orientation(self, q: np.ndarray, v: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """获取末端执行器的位置和速度"""
        # 更新机器人构型
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        # 获取末端执行器位置
        
        H = self.data.oMf[self.end_effector_id]
        current_pos = H.translation  # 位置向量
        current_rot = H.rotation  # 姿态矩阵

        orientation_error = pin.log3( self.initial_orientation @ current_rot.T)
        
        # 计算雅可比矩阵
        J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id,pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]  # 只取位置部分
        J_rot = J[3:, :]  # 只取姿态部分
        
        # 计算末端速度
        current_vel_pos = J_pos @ v
        current_vel_rot = J_rot @ v

        
        return current_pos, current_vel_pos, orientation_error, current_vel_rot 
    

    
    def compute_control_task_space_with_orientation_and_imp(self, q: np.ndarray, v: np.ndarray, 
                       pos_des: np.ndarray, vel_des: np.ndarray, 
                       acc_des: np.ndarray,current_pos : np.ndarray,
                       current_vel : np.ndarray,
                       force_ext : np.ndarray,torque_ext : np.ndarray)-> np.ndarray:
        """计算控制力矩"""
        # 获取当前末端状态
        pos_cur, vel_pos_cur, ori_err, vel_rot_cur = self.get_task_space_state_with_orientation(q, v)

        vel_rot_err = - vel_rot_cur  # 期望角速度为零


        
        # 获取雅可比矩阵
        end_effector_id = self.model.getFrameId("cylinder_link")
        J = pin.computeFrameJacobian(self.model, self.data, q, end_effector_id,pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]
        J_rot = J[3:, :]



        M = pin.crba(self.model, self.data, q)  # 质量矩阵
        M_inv = pinv(M)

        #lambda_ = M @ pinv(J_pos)

        #lambda_ = M_inv @ J_pos.T @ inv( J_pos @ M_inv @ M_inv @ J_pos.T)
        #lambda_ = M @ J_pos.T @ inv(J_pos @ J_pos.T)
        #lambda_ = J_pos.T @ inv( J_pos @ M_inv @ J_pos.T)
        #print(lambda_.shape)
        #lambda_ = M_inv.T @ J_pos.T @ pinv(J_pos @ M_inv @ M_inv.T @J_pos.T) 

        #W = M_inv #* M_inv

        W =  np.eye(7)

        

        #lambda_ = M @ pinv(J_pos)
        
        J_dot = pin.getFrameJacobianTimeVariation(
            self.model, self.data,
            self.end_effector_id, pin.ReferenceFrame.WORLD)


        # 计算非线性动力学项（科里奥利力和重力）
        # q = np.ones(6)
        # v = np.zeros(6)
        #C = pin.nonLinearEffects(self.model, self.data, q, v)  # 包含科氏力和重力
        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C  # 这是7x7的科氏力矩阵
        C = C @ v.reshape(7,1)  # 乘以速度v
        C = C.reshape(7)  # 变成6x1的矩阵
        # acc_des = np.concatenate([acc_des,np.zeros(3) ])  # 加上重力加速度
        # vel_des = np.concatenate([vel_des,np.zeros(3) ])  # 加上重力速度
        # pos_des = np.concatenate([pos_des,np.zeros(3) ])  # 加上重力位置

        # current_pos = np.concatenate([current_pos,np.zeros(3) ])  # 加上重力位置
        # current_vel = np.concatenate([current_vel,np.zeros(3) ])  # 加上重力速度
        force_ext = np.array(force_ext).reshape(3)
        m = 10
        d = 50
        k = 100

        force_desired = np.array([0,0,0])

        u_pos = acc_des  + (force_ext -force_desired -d*(current_vel - vel_des) -k*(current_pos - pos_des)) / m 

        #u_pos = acc_des  + 20 *(vel_des - current_vel) + 100 * (pos_des - current_pos)

        #print('u_pos:',u_pos)

        m2 = 1
        d2 = 10
        k2 = 25

       #torque_ext = np.zeros(3)

        u_rot = (  k2 * (ori_err) + d2 * (vel_rot_err)) / m2 

        if np.linalg.norm(u_rot) > 0.1:
            u_rot[2] = 0.001

        u = np.concatenate([u_pos, u_rot])

        J_full = np.vstack([J_pos, J_rot])



        lambda_ = W @ M_inv.T @ J_full.T @ pinv( J_full @ M_inv @ W @ M_inv.T @ J_full.T)



        D_null = 10 * np.eye(7)  # 阻尼增益，可以根据需要调整
    
        # 计算零空间速度
        v_null = v  # 这里假设零空间速度等于当前关节速度
        N = (np.eye(7) - lambda_ @ J_full @ M_inv )
        #print(N)

        null_term2 = - N @ D_null @ v_null.reshape(7)  # 7x1

        # print(lambda_.shape)
        # print(J_dot.shape)
        # print(  u.shape)


        # 计算操作空间力
        tau = lambda_ @ ( u - J_dot @ v + J_full @ M_inv @ (C)) + null_term2  #- J_dot_full @ v + J @ M_inv @ C) #+null_term

        #print('tau:',lambda_)
        
        # 映射到关节空间
        #tau = J_pos.T @ F + h
        
        return tau

    def compute_control_task_space_with_orientation_and_imp2(self, q: np.ndarray, v: np.ndarray, 
                       pos_des: np.ndarray, vel_des: np.ndarray, 
                       acc_des: np.ndarray,current_pos : np.ndarray,
                       current_vel : np.ndarray,
                       force_ext : np.ndarray , torque_ext : np.ndarray = np.zeros(3))-> np.ndarray:
        """计算控制力矩"""
        # 获取当前末端状态
        pos_cur, vel_pos_cur, ori_err, vel_rot_cur = self.get_task_space_state_with_orientation(q, v)
        
        # 计算位置误差
        pos_err = pos_des - pos_cur
        vel_pos_err = vel_des - vel_pos_cur

                # 方向误差（我们希望它保持为零，所以直接使用获取的误差）
        # ori_err已经是相对于期望方向（初始方向）的误差
        vel_rot_err = -vel_rot_cur  # 期望角速度为零


        
        # 获取雅可比矩阵
        end_effector_id = self.model.getFrameId("cylinder_link1")
        J = pin.computeFrameJacobian(self.model, self.data, q, end_effector_id,pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]
        J_rot = J[3:, :]





        M = pin.crba(self.model, self.data, q)  # 质量矩阵
        M_inv = pinv(M)

        #lambda_ = M @ pinv(J_pos)

        #lambda_ = M_inv @ J_pos.T @ inv( J_pos @ M_inv @ M_inv @ J_pos.T)
        #lambda_ = M @ J_pos.T @ inv(J_pos @ J_pos.T)
        #lambda_ = J_pos.T @ inv( J_pos @ M_inv @ J_pos.T)
        #print(lambda_.shape)
        #lambda_ = M_inv.T @ J_pos.T @ pinv(J_pos @ M_inv @ M_inv.T @J_pos.T) 

        #W = M_inv #* M_inv

        W =  np.eye(7)

        

        #lambda_ = M @ pinv(J_pos)
        
        J_dot = pin.getFrameJacobianTimeVariation(
            self.model, self.data,
            self.end_effector_id, pin.ReferenceFrame.WORLD)
        
        J_dot_pos = J_dot[:3, :]
        J_dot_rot = J_dot[3:, :]


        # 计算非线性动力学项（科里奥利力和重力）
        # q = np.ones(6)
        # v = np.zeros(6)
        #C = pin.nonLinearEffects(self.model, self.data, q, v)  # 包含科氏力和重力
        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C  # 这是7x7的科氏力矩阵
        C = C @ v.reshape(7,1)  # 乘以速度v
        C = C.reshape(7)  # 变成6x1的矩阵
        # acc_des = np.concatenate([acc_des,np.zeros(3) ])  # 加上重力加速度
        # vel_des = np.concatenate([vel_des,np.zeros(3) ])  # 加上重力速度
        # pos_des = np.concatenate([pos_des,np.zeros(3) ])  # 加上重力位置

        # current_pos = np.concatenate([current_pos,np.zeros(3) ])  # 加上重力位置
        # current_vel = np.concatenate([current_vel,np.zeros(3) ])  # 加上重力速度
        force_ext = np.array(force_ext).reshape(3)
        m = 10 
        d = 20
        k = 100

        force_desired = np.array([0,0,0])

        u_pos = acc_des  + ( -force_desired -d*(current_vel - vel_des) -k*(current_pos - pos_des)) / m 

        #u_rot = np.zeros(3)


        u_rot =  1 * (vel_rot_err) + 1 * (ori_err)

        u = np.concatenate([u_pos, u_rot])

        J_full = np.vstack([J_pos, J_rot])

        lambda_ =  pinv(J)



        D_null = 1. * np.eye(7)  # 阻尼增益，可以根据需要调整
    
        # 计算零空间速度
        v_null = v  # 这里假设零空间速度等于当前关节速度
        N = (np.eye(7) - lambda_ @ J )

        null_term2 = - N @ D_null @ v_null.reshape(7)  # 7x1

        # print(lambda_.shape)
        # print(J_dot.shape)
        # print(  u.shape)


        # 计算操作空间力
        tau = M @ (lambda_ @ ( u - J_dot @ v) ) + C  # + null_term2
        
        # 映射到关节空间
        #tau = J_pos.T @ F + h
        
        return tau
    
    def compute_control_task_space_with_orientation(self, q: np.ndarray, v: np.ndarray, 
                       pos_des: np.ndarray, vel_des: np.ndarray, 
                       acc_des: np.ndarray,current_pos : np.ndarray,
                       current_vel : np.ndarray)-> np.ndarray:
        """计算控制力矩"""
        # 获取当前末端状态
        pos_cur, vel_pos_cur, ori_err, vel_rot_cur = self.get_task_space_state_with_orientation(q, v)
        
        # 计算位置误差
        pos_err = pos_des - pos_cur
        vel_pos_err = vel_des - vel_pos_cur

                # 方向误差（我们希望它保持为零，所以直接使用获取的误差）
        # ori_err已经是相对于期望方向（初始方向）的误差
        vel_rot_err = - vel_rot_cur  # 期望角速度为零


        
        # 获取雅可比矩阵
        end_effector_id = self.model.getFrameId("cylinder_link")
        J = pin.computeFrameJacobian(self.model, self.data, q, end_effector_id,pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]
        J_rot = J[3:, :]



        M = pin.crba(self.model, self.data, q)  # 质量矩阵
        M_inv = pinv(M)

        #lambda_ = M @ pinv(J_pos)

        #lambda_ = M_inv @ J_pos.T @ inv( J_pos @ M_inv @ M_inv @ J_pos.T)
        #lambda_ = M @ J_pos.T @ inv(J_pos @ J_pos.T)
        #lambda_ = J_pos.T @ inv( J_pos @ M_inv @ J_pos.T)
        #print(lambda_.shape)
        #lambda_ = M_inv.T @ J_pos.T @ pinv(J_pos @ M_inv @ M_inv.T @J_pos.T) 

        W = M_inv #* M_inv

        #W =  np.eye(7)

        

        #lambda_ = M @ pinv(J_pos)
        
        J_dot = pin.getFrameJacobianTimeVariation(
            self.model, self.data,
            self.end_effector_id, pin.ReferenceFrame.WORLD)
        
        J_dot_pos = J_dot[:3, :]
        J_dot_rot = J_dot[3:, :]


        # 计算非线性动力学项（科里奥利力和重力）
        # q = np.ones(6)
        # v = np.zeros(6)
        #C = pin.nonLinearEffects(self.model, self.data, q, v)  # 包含科氏力和重力
        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C  # 这是7x7的科氏力矩阵
        C = C @ v.reshape(7,1)  # 乘以速度v
        C = C.reshape(7)  # 变成6x1的矩阵
        # acc_des = np.concatenate([acc_des,np.zeros(3) ])  # 加上重力加速度
        # vel_des = np.concatenate([vel_des,np.zeros(3) ])  # 加上重力速度
        # pos_des = np.concatenate([pos_des,np.zeros(3) ])  # 加上重力位置

        # current_pos = np.concatenate([current_pos,np.zeros(3) ])  # 加上重力位置
        # current_vel = np.concatenate([current_vel,np.zeros(3) ])  # 加上重力速度

        u_pos = acc_des  + 20 *(vel_des - current_vel) + 100 * (pos_des - current_pos)
        u_rot = 20 * (vel_rot_err) + 100 * (ori_err)

        u = np.concatenate([u_pos, u_rot])

        J_full = np.vstack([J_pos, J_rot])

        lambda_ = M @ M_inv.T @ J_full.T @ pinv( J_full @ M_inv @ M @ M_inv.T @ J_full.T)



        D_null = 1.2 * np.eye(7)  # 阻尼增益，可以根据需要调整
    
        # 计算零空间速度
        v_null = v  # 这里假设零空间速度等于当前关节速度
        N = (np.eye(7) - lambda_ @ J_full @ M_inv )

        null_term2 = - N @ D_null @ v_null.reshape(7)  # 7x1

        # print(lambda_.shape)
        # print(J_dot.shape)
        # print(  u.shape)


        # 计算操作空间力
        tau = lambda_ @ ( u - J_dot @ v + J_full @ M_inv @ (C)) + null_term2  #- J_dot_full @ v + J @ M_inv @ C) #+null_term
        
        # 映射到关节空间
        #tau = J_pos.T @ F + h
        
        return tau

    
    def compute_control_task_space(self, q: np.ndarray, v: np.ndarray, 
                       pos_des: np.ndarray, vel_des: np.ndarray, 
                       acc_des: np.ndarray,current_pos : np.ndarray,
                       current_vel : np.ndarray)-> np.ndarray:
        """计算控制力矩"""
        # 获取当前末端状态
        pos_cur, vel_cur,_ = self.get_task_space_state(q, v)
        
        # 计算误差
        pos_err = pos_des - pos_cur
        vel_err = vel_des - vel_cur
        
        # 获取雅可比矩阵
        end_effector_id = self.model.getFrameId("cylinder_link")
        J = pin.computeFrameJacobian(self.model, self.data, q, end_effector_id,pin.ReferenceFrame.WORLD)
        J_pos = J[:3, :]

        M = pin.crba(self.model, self.data, q)  # 质量矩阵
        M_inv = pinv(M)

        #lambda_ = M @ pinv(J_pos)

        #lambda_ = M_inv @ J_pos.T @ inv( J_pos @ M_inv @ M_inv @ J_pos.T)
        #lambda_ = M @ J_pos.T @ inv(J_pos @ J_pos.T)
        #lambda_ = J_pos.T @ inv( J_pos @ M_inv @ J_pos.T)
        #print(lambda_.shape)
        #lambda_ = M_inv.T @ J_pos.T @ pinv(J_pos @ M_inv @ M_inv.T @J_pos.T) 

        W = M_inv #* M_inv

        #W =  np.eye(7)

        lambda_ = M @ M_inv.T @ J_pos.T @ pinv( J_pos @ M_inv @ M @ M_inv.T @ J_pos.T)

        #lambda_ = M @ pinv(J_pos)
        
        J_dot_full = pin.getFrameJacobianTimeVariation(
            self.model, self.data,
            self.end_effector_id, pin.ReferenceFrame.WORLD)
        
        J_dot_full = J_dot_full[:3,:]


        # 计算非线性动力学项（科里奥利力和重力）
        # q = np.ones(6)
        # v = np.zeros(6)
        #C = pin.nonLinearEffects(self.model, self.data, q, v)  # 包含科氏力和重力
        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C  # 这是7x7的科氏力矩阵
        C = C @ v.reshape(7,1)  # 乘以速度v
        C = C.reshape(7)  # 变成6x1的矩阵
        # acc_des = np.concatenate([acc_des,np.zeros(3) ])  # 加上重力加速度
        # vel_des = np.concatenate([vel_des,np.zeros(3) ])  # 加上重力速度
        # pos_des = np.concatenate([pos_des,np.zeros(3) ])  # 加上重力位置

        # current_pos = np.concatenate([current_pos,np.zeros(3) ])  # 加上重力位置
        # current_vel = np.concatenate([current_vel,np.zeros(3) ])  # 加上重力速度

        u = acc_des  + 20 *(vel_des - current_vel) + 100 * (pos_des - current_pos)

        grad_m = self.manipulability_gradient(q)
        #print(grad_m)
        # 设置比例增益
        k = 0.2  # 可以根据需要调整
        
        # 期望关节加速度
        ddq_desired = k * grad_m  # 7x1

        tau_null_desired = M @ ddq_desired  # 7x7 @ 7x1 = 7x1

        D_null = 1.2 * np.eye(7)  # 阻尼增益，可以根据需要调整
    
        # 计算零空间速度
        v_null = v  # 这里假设零空间速度等于当前关节速度
        N = (np.eye(7) - lambda_ @ J_pos @ M_inv )

        null_term2 = - N @ D_null @ v_null.reshape(7)  # 7x1

        null_term =   N @ tau_null_desired.reshape(7) 

        # 计算操作空间力
        tau = lambda_ @ ( u - J_dot_full @ v + J_pos @ M_inv @ (C)) + null_term2  #- J_dot_full @ v + J @ M_inv @ C) #+null_term
        
        # 映射到关节空间
        #tau = J_pos.T @ F + h
        
        return tau

    
    def _apply_limits(self, tau: np.ndarray, q: np.ndarray, 
                     v: np.ndarray) -> np.ndarray:
        """应用关节限位和速度限制"""
        # 位置软限位
        k_limit = 100.0
        tau_limit = np.zeros_like(tau)
        for i in range(len(q)):
            if q[i] < self.q_min[i]:
                tau_limit[i] = k_limit * (self.q_min[i] - q[i])
            elif q[i] > self.q_max[i]:
                tau_limit[i] = k_limit * (self.q_max[i] - q[i])
        
        # 速度限制
        v_scale = np.minimum(1.0, self.v_max / (np.abs(v) + 1e-6))
        tau = tau * v_scale
        
        return tau + tau_limit
    
    def manipulability_gradient(self,q, delta=1e-6):
        """
        计算操控性指标相对于关节角度 q 的梯度。
        使用有限差分法近似梯度。
        """
        grad = np.zeros_like(q)
        # 计算当前操控性
        J_current = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id)[:3, :]
        manipulability_current = np.linalg.det(J_current @ J_current.T)
        
        for i in range(len(q)):
            q_delta = q.copy()
            q_delta[i] += delta  # 增加微小偏移
            J_delta = pin.computeFrameJacobian(self.model, self.data, q_delta, self.end_effector_id)[:3, :]
            manipulability_delta = np.linalg.det(J_delta @ J_delta.T)
            # 近似导数
            grad[i] = (manipulability_delta - manipulability_current) / delta
        return grad  # 形状为 (7,)
    
def compute_ik(pin_model, pin_data, target_pose, initial_q=np.ones(7)*0.3, max_iters=1000, eps=1e-6):
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


class RobotSimulator:
    """简单的机器人仿真器，使用四阶龙格-库塔积分法"""
    def __init__(self, robot_model: pin.Model, dt: float):
        self.model = robot_model
        self.model.gravity.linear = np.array([0., 0., 0.])
        self.data = self.model.createData()
        self.dt = dt
        self.end_effector_id = self.model.getFrameId("cylinder_link")
        
        
    def compute_acceleration(self, q: np.ndarray, v: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """计算加速度 a = M^{-1}(tau - h)"""
        pin.computeAllTerms(self.model, self.data, q, v)
        a = pin.aba(self.model, self.data, q, v, tau)
        return a
    
    def step(self, q: np.ndarray, v: np.ndarray, tau: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """使用RK4积分仿真一个时间步"""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        # 获取雅可比矩阵
        J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id)
        J_pos = J[:3, :]  # 只取位置部分
        F_ext = np.zeros(3).reshape(3,1)*0.5  # 外部力为零


        dt = self.dt  

        tau = tau + J_pos.T @ F_ext.reshape(3)  # 加上一个微小扰动，防止分母为0
        
        # 计算 k1
        a1 = self.compute_acceleration(q, v, tau)
        dq1 = v
        dv1 = a1
        
        # 计算 k2
        q_mid = pin.integrate(self.model, q, 0.5 * dt * dq1)
        v_mid = v + 0.5 * dt * dv1
        a2 = self.compute_acceleration(q_mid, v_mid, tau)
        dq2 = v_mid
        dv2 = a2
        
        # 计算 k3
        q_mid = pin.integrate(self.model, q, 0.5 * dt * dq2)
        v_mid = v + 0.5 * dt * dv2
        a3 = self.compute_acceleration(q_mid, v_mid, tau)
        dq3 = v_mid
        dv3 = a3
        
        # 计算 k4
        q_end = pin.integrate(self.model, q, dt * dq3)
        v_end = v + dt * dv3
        a4 = self.compute_acceleration(q_end, v_end, tau)
        dq4 = v_end
        dv4 = a4
        
        # 组合四个斜率
        dq = (dq1 + 2*dq2 + 2*dq3 + dq4) / 6.0
        dv = (dv1 + 2*dv2 + 2*dv3 + dv4) / 6.0
        
        # 更新状态
        q_next = pin.integrate(self.model, q, dt * dq)
        v_next = v + dt * dv
        
        return q_next, v_next

def cal_imp_force(current_pos,current_vel,desired_pos,desired_vel):
    K = 1000
    D = 100

    imp_force = K * (current_pos-desired_pos) + D * (current_vel-desired_vel)

    return imp_force

def run_simulation(q_init: np.ndarray):
    """运行仿真"""
    # 创建机器人模型
    model = pin.buildModelFromUrdf("kuka_xml_urdf/urdf/iiwa14.urdf")
    data = model.createData()
    
    # 验证初始关节角度
    assert len(q_init) == model.nq, f"Initial joint angles must have length {model.nq}"
    assert np.all(q_init >= -3.14) and np.all(q_init <= 3.14), "Joint angles must be in radians"
    
    # 检查模型是否正确加载
    # assert model.njoints == 8, "KUKA iiwa14 should have 8 joints (including universe)"
    # print(f"Robot model loaded with {model.nv} DoF")
    
    # 初始化控制器和仿真器
    dt = 0.0001
    controller = TaskSpaceController(model, dt)
    simulator = RobotSimulator(model, dt)

    init_pos = np.array([0., 0.5, 0.5])
    init_ori = np.array([
        [1,  0,  0],
        [0, -1,  0],
        [0,  0, -1]
    ])

    q_init , _ = compute_ik(model, data, pin.SE3(init_ori, init_pos))

    
    # 设置初始状态
    q = q_init.copy()
    v = np.zeros(model.nv)

    # 计算并显示初始位置
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    H_init = data.oMf[model.getFrameId("cylinder_link")]
    print(f"Initial end-effector position: {H_init.translation}")
    
    # 设置目标位置（相对运动）
    target_pos = H_init.translation + np.array([0.01, -0.02,-0.2])
    print(f"Target position: {target_pos}")
    
    # 创建轨迹
    duration = 5.5
    trajectory = TaskSpaceTrajectory(model, q_init, target_pos, 5)
    
    # 存储数据用于绘图
    t_list = []
    pos_actual = []
    pos_desired = []
    joint_angles = []
    joint_velocities = []
    virtual_force_list = []

    tau_hist = []
    
    # 仿真循环
    t = 0.0
    f_imp = np.zeros(3)
    current_pos, current_vel ,_= controller.get_task_space_state(q, v)
    while t < duration:
        # 获取期望轨迹
        pos_des, vel_des, acc_des = trajectory.get_state(t)
        
        # 计算控制输入
        #tau = controller.compute_control2(q, v, pos_des, vel_des, acc_des,current_pos,current_vel  )

        tau = controller.compute_control_task_space_with_orientation(q, v, pos_des, vel_des, acc_des ,current_pos,current_vel )

        tau_hist.append(tau)
        
        # 仿真一步
        q, v = simulator.step(q, v, tau)
        
        # 记录数据
        current_pos, current_vel,_ = controller.get_task_space_state(q, v)
        current_pos = np.array(current_pos)
        current_vel = np.array(current_vel)

        f_imp = cal_imp_force(current_pos,current_vel,pos_des,vel_des)
        virtual_force_list.append(f_imp)
        t_list.append(t)
        pos_actual.append(current_pos)
        pos_desired.append(pos_des)
        joint_angles.append(q.copy())
        joint_velocities.append(v.copy())
        
        t += dt
    
    # 绘制结果
    plot_results(t_list, pos_actual, pos_desired, joint_angles, joint_velocities,virtual_force_list,tau_hist)

def plot_results(t_list: List[float], pos_actual: List[np.ndarray], 
                pos_desired: List[np.ndarray], joint_angles: List[np.ndarray],
                joint_velocities: List[np.ndarray],virtual_force_list: List[np.ndarray],
                tau_hist: List[np.ndarray]):
    """绘制仿真结果"""
    pos_actual = np.array(pos_actual)
    pos_desired = np.array(pos_desired)
    joint_angles = np.array(joint_angles)
    joint_velocities = np.array(joint_velocities)

    virtual_force_list = np.array(virtual_force_list)
    tau_hist = np.array(tau_hist)
    
    # 创建三个子图
    fig = plt.figure(figsize=(15, 12))
    gs = plt.GridSpec(3, 2)
    
    # 1. 位置跟踪
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
    
    # 2. 跟踪误差
    ax2 = fig.add_subplot(gs[1, :])
    for i in range(3):
        error = pos_desired[:, i] - pos_actual[:, i]
        ax2.plot(t_list, error, label=f'{labels[i]} Error')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Error [m]')
    ax2.legend()
    ax2.grid(True)
    ax2.set_title('Position Tracking Error')
    
    # 3. 关节角度和速度
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