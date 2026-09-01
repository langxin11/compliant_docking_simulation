"""
main_simulation.py - Robotic Task Space Control Simulation

This module implements a simulation environment for task space control of a KUKA iiwa14 robot
using MuJoCo for physics simulation and Pinocchio for dynamics calculations. It demonstrates
operational space control with impedance for smooth interaction with the environment.

The simulation includes:
- Force/torque sensor integration
- Task space trajectory tracking
- External force compensation
- Visualization of robot motion and performance metrics

Author: langxin11
Date: 2025
"""
import os
# 在无显示环境（如服务器/CI）下，指定 MuJoCo 使用 EGL 离屏渲染后端 /
# In headless environments (e.g., server/CI), set MuJoCo to use EGL offscreen backend
# 若本机有显示并安装了驱动，也可改为 'glfw'；某些环境需使用 'osmesa' /
# If you have a display and drivers, 'glfw' may work; some setups need 'osmesa'
if "DISPLAY" not in os.environ or not os.environ["DISPLAY"]:
    os.environ["MUJOCO_GL"] = "egl"  # 试试 osmesa，也可以改成 egl

import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
from time import sleep
from typing import Callable, Optional

import mujoco.viewer
from typing import Tuple, List

from Relate_class import TaskSpaceController,TaskSpaceTrajectory,DecoupledQuinticTrajectory,compute_ik
from muj_class import MujRobot
from log_class import Log
    


        

def run_simulation(muj_robot:MujRobot,
                   task_dynamics:TaskSpaceController,
                   trajector_planner:TaskSpaceTrajectory,
                   log:Log,
                   duration:float,
                   dt:float,
                   q_init:np.ndarray):
    """
    执行主仿真循环：读取轨迹 → 计算任务空间阻抗控制力矩 → MuJoCo 步进 → 记录/绘图 /
    Run the main simulation loop: sample trajectory → compute task-space impedance torque → MuJoCo step → log/plot

    参数说明 / Args:
    - muj_robot: MuJoCo 封装（写入力矩、推进仿真、可视化/录帧） / MuJoCo wrapper (apply torque, step sim, render/record)
    - task_dynamics: 任务空间控制器（Pinocchio 提供雅可比/动力学项） / Task-space controller (uses Pinocchio for Jacobians/dynamics)
    - trajector_planner: 轨迹规划器（解耦 5 次多项式，输出 pos/vel/acc） / Trajectory planner (decoupled quintic; pos/vel/acc)
    - log: 日志记录与绘图类 / Logger for data capture and plotting
    - duration/dt: 总时长与步长 / Total duration and time step
    - q_init: 初始关节位置（由 IK 求得） / Initial joint configuration (from IK)
    """
    
    log.reset_logs()

    muj_robot.init_simulators(q_init)



    q = q_init
    v = np.zeros(7)

    current_pos, current_vel, _ = task_dynamics.get_task_space_state(q, v)

    force_external = np.zeros(3)
    torque_external = np.zeros(3)


    while muj_robot.data.time < duration:
        # while True:
        t = muj_robot.data.time
        # 1) 根据当前仿真时间采样期望的末端位置/速度/加速度 /
        # 1) Sample desired end-effector pos/vel/acc at current sim time
        pos_des, vel_des, acc_des = trajector_planner.get_state(t)
        #print(f"Current time: {t}, Desired position: {pos_des}, Desired velocity: {vel_des}, Desired acceleration: {acc_des}")

        # 2) 任务空间控制（含平动/姿态阻抗与外力补偿）→ 得到关节力矩 tau /
        # 2) Task-space control (translation/rotation impedance + external force) → joint torques tau
        tau = task_dynamics.compute_control_task_space_with_orientation_and_imp(
            q, v, pos_des, vel_des, acc_des, current_pos, current_vel, force_external, torque_external)
        
        # 力矩限幅以防止过大的控制输入导致碰撞检测失败 /
        # Clamp torques to prevent excessive control inputs that cause collision detection failure
        max_torque = 10.0  # 根据机器人规格调整 / Adjust based on robot specs
        tau = np.clip(tau, -max_torque, max_torque)
                                                                
        # 3) 将 tau 写入 MuJoCo，并推进一步物理仿真 /
        # 3) Apply tau to MuJoCo and advance one simulation step
        try:
            q, v, eef_pos = muj_robot.step(tau)
        except Exception as e:
            print(f"\nSimulation error at t={t:.3f}s: {e}")
            print(f"Torques: {tau}")
            print(f"Joint positions: {q}")
            print("Breaking simulation loop...")
            break
        #print(f"Current time: {t}, End-effector position: {q},tau: {v}",)

        # 4) 使用 Pinocchio 更新当前末端状态（位置/速度/姿态） /
        # 4) Update current end-effector state (pos/vel/orientation) via Pinocchio
        current_pos, current_vel, current_ori = task_dynamics.get_task_space_state(q, v)

        current_pos = np.array(current_pos)
        tau = np.array(tau)

        # 5) 读取力/力矩传感器（根据模型定义：此处取负号与力方向约定相关） /
        # 5) Read force/torque sensors (sign depends on model’s convention)
        force_sensor = -muj_robot.data.sensor("force_sensor").data

        torque_sensor = -muj_robot.data.sensor("torque_sensor").data


        # 6) 将传感器数据通过当前末端旋转矩阵变换到控制参考系下 /
        # 6) Rotate sensor data with current EE rotation to controller’s reference frame
        # 注意：变换方向取决于 `current_ori` 的参考系定义，需与传感器坐标系一致 /
        # Note: direction depends on `current_ori` definition and sensor frame
        force_external = current_ori @ force_sensor 

        torque_external = current_ori @ torque_sensor

        print(f"current_time:{t},force_external:{force_external}")

        #print('current:',current_pos,"current_ori:",current_ori)

        # print('far:',np.linalg.norm(eef_pos-current_pos))

        #log.store_data(t, pos_des, q, v, tau, np.linalg.norm(current_pos - pos_des))

        # 7) 记录关节/末端/控制量/外力等数据，便于后续绘图分析 /
        # 7) Log joint/EE/control/external data for plotting/analysis
        log.store_data(
            t, q, v, current_pos, current_vel,
            np.linalg.norm(current_pos - pos_des),
            pos_des, vel_des, acc_des, tau,
            force_external, torque_external)


    log.plot_results(save_path="figure/")

    if muj_robot.record:
        # 创建绝对路径以确保视频保存在正确位置
        current_dir = os.path.dirname(os.path.abspath(__file__))
        video_dir = os.path.join(current_dir, "video")
        if not os.path.exists(video_dir):
            try:
                os.makedirs(video_dir)
                print(f"Created video directory: {video_dir}")
            except Exception as e:
                print(f"Error creating video directory: {e}")
                
        video_path = os.path.join(video_dir, "docking_update.mp4")
        print(f"Saving video to absolute path: {video_path}")
        muj_robot.to_mp4(video_path)



def main(render=True, record=True, dt=0.001, traj_duration=15.0, duration=20.0):
    """
    Main function that sets up and runs the robot control simulation.
    
    Args:
        render (bool): Whether to render the simulation visually
        record (bool): Whether to record the simulation to a video file
        dt (float): Simulation time step in seconds
        traj_duration (float): Duration of the trajectory execution in seconds
        duration (float): Total simulation duration in seconds
    """
    log = Log()

    # 1) 构建 Pinocchio 模型/数据（用于雅可比/动力学计算） /
    # 1) Build Pinocchio model/data (for Jacobians and dynamics)
    pin_model = pin.buildModelFromUrdf("kuka_xml_urdf/iiwa14_dock.urdf")
    pin_data = pin_model.createData()

    # 注意：TaskSpaceController 当前实现的 __init__ 形参为 (robot_model, dt) /
    # Note: TaskSpaceController __init__ currently expects (robot_model, dt)
    # 这里传入 (pin_model, pin_data) 可能导致运行时不匹配，仅保留为示例；/
    # Passing (pin_model, pin_data) may mismatch at runtime; example only
    # 建议统一构造接口或在此处传入 dt。/
    # Suggest unifying the constructor or pass dt here
    #task_dynamics = TaskSpaceController(pin_model, pin_data)
    # 这里改为传入 dt /
    task_dynamics = TaskSpaceController(pin_model, dt)
    # 计算初始位姿的逆运动学，得到初始关节位置 /
    # Compute IK for initial pose to get initial joint configuration
    init_pos = np.array([0.0, 0.5, 0.5])
    init_ori = np.array([
        [1,  0,  0],
        [0, -1,  0],
        [0,  0, -1]
    ])

    init_pose = pin.SE3(init_ori, init_pos)

    # 使用合适的初始猜测值以帮助IK收敛 /
    # Use a good initial guess to help IK converge
    initial_guess = np.array([0.0, 0.5, 0.0, -1.0, 0.0, 1.5, 0.0])
    q_init, success = compute_ik(pin_model, pin_data, init_pose, initial_q=initial_guess, max_iters=5000)
    
    if not success:
        print("Warning: IK did not converge perfectly, but continuing with best solution found.")
    
    #步进 Pinocchio 以更新数据 /
    # Step Pinocchio to update data
    pin.forwardKinematics(pin_model, pin_data, q_init)
    # 更新所有坐标系位姿
    # Update frame placements
    pin.updateFramePlacements(pin_model, pin_data)
    #获取末端在世界坐标系的SE(3)位姿
    H_init = pin_data.oMf[pin_model.getFrameId("cylinder_link")]
    print(f"Initial end-effector position: {H_init.translation}")
    print(f"Initial joint positions: {q_init}", success)

    # Set target position (relative motion)
    target_pos = init_pos + np.array([0.00, -0.00, -0.18])
    print(f"Target position: {target_pos}")

    # Initialize robot simulation with parameters
    # 2) 初始化 MuJoCo 机器人（写 tau、推进仿真、渲染/录帧） /
    # 2) Initialize MuJoCo robot (apply tau, step sim, render/record)
    muj_robot = MujRobot(
        model_path="kuka_xml_urdf/iiwa14_dock_updated.xml",
        render=render,
        record=record,
        dt=dt,
        target_pos=target_pos
    )

    # Create trajectory planner with specified duration
    # 3) 构建任务空间解耦五次轨迹规划器 /
    # 3) Build decoupled quintic task-space trajectory planner
    trajector_planner = DecoupledQuinticTrajectory(init_pos, target_pos, traj_duration)

    # Run simulation with specified duration
    run_simulation(
        muj_robot,
        task_dynamics,
        trajector_planner,
        log,
        duration=duration,
        dt=dt,
        q_init=q_init
    )

if __name__ == '__main__':
    main(render=False, record=True, dt=0.001, traj_duration=15.0, duration=18.0)

    

