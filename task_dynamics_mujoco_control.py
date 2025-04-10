"""
task_dynamics_mujoco_control.py - Robotic Task Space Control Simulation

This module implements a simulation environment for task space control of a KUKA iiwa14 robot
using MuJoCo for physics simulation and Pinocchio for dynamics calculations. It demonstrates
operational space control with impedance for smooth interaction with the environment.

The simulation includes:
- Force/torque sensor integration
- Task space trajectory tracking
- External force compensation
- Visualization of robot motion and performance metrics

Author: RQM
Date: 2024
"""

import mujoco
import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
from time import sleep
from typing import Callable, Optional
import os
import mujoco.viewer
from typing import Tuple, List

from Relate_class import TaskSpaceController,TaskSpaceTrajectory,DecoupledQuinticTrajectory
from muj_class import MujRobot
from log_class import Log
    


        

def run_simulation(muj_robot:MujRobot,
                   task_dynamics:TaskSpaceController,
                   trajector_planner:TaskSpaceTrajectory,
                   log:Log,duration:float,
                   dt:float,q_init:np.ndarray):
    
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
        pos_des, vel_des, acc_des = trajector_planner.get_state(t)
        #print(f"Current time: {t}, Desired position: {pos_des}, Desired velocity: {vel_des}, Desired acceleration: {acc_des}")

        tau = task_dynamics.compute_control_task_space_with_orientation_and_imp(q, v, pos_des, vel_des, acc_des ,current_pos,current_vel,force_external,torque_external)
                                                                
        q,v,eef_pos = muj_robot.step(tau)
        #print(f"Current time: {t}, End-effector position: {q},tau: {v}",)

        current_pos, current_vel ,current_ori = task_dynamics.get_task_space_state(q, v)

        current_ori_log = pin.log3(current_ori)

        current_pos = np.array(current_pos)
        tau = np.array(tau)

        force_sensor = -muj_robot.data.sensor("force_sensor").data

        torque_sensor = -muj_robot.data.sensor("torque_sensor").data


        force_external = current_ori @ force_sensor 

        torque_external = current_ori @ torque_sensor

        #print(f"current_time:{t},force_external:{force_exteral},current_ori:{current_ori_log}")

        #print('current:',current_pos,"current_ori:",current_ori)

        # print('far:',np.linalg.norm(eef_pos-current_pos))

        #log.store_data(t, pos_des, q, v, tau, np.linalg.norm(current_pos - pos_des))

        log.store_data(t, q, v, current_pos, current_vel, np.linalg.norm(current_pos - pos_des),pos_des,vel_des,acc_des,tau ,force_external,torque_external)
                #    v: np.ndarray, pos_actual: np.ndarray, 
                #    vel_actual: np.ndarray, error: float,
                #    pos_desired: np.ndarray,vel_desired: np.ndarray,
                #    acc_desired: np.ndarray)

    log.plot_results()



def main():

    dt = 0.001

    
    log = Log()

    pin_model = pin.buildModelFromUrdf("kuka_xml_urdf/iiwa14_dock.urdf")
    #pin_model = pin.buildModelFromMJCF("kuka_xml_urdf/iiwa14_dock.xml")
    pin_data = pin_model.createData()


    task_dynamics = TaskSpaceController(pin_model, pin_data)


    init_pos = np.array([0.0, 0.5, 0.5])
    init_ori = np.array([
        [1,  0,  0],
        [0, -1,  0],
        [0,  0, -1]
    ])

    init_pose = pin.SE3(init_ori, init_pos)

    q_init , success = compute_ik(pin_model, pin_data, init_pose)

    pin.forwardKinematics(pin_model, pin_data, q_init)
    pin.updateFramePlacements(pin_model, pin_data)
    H_init = pin_data.oMf[pin_model.getFrameId("cylinder_link")]
    print(f"Initial end-effector position: {H_init.translation}")

    

    print(f"Initial joint positions: {q_init}",success)

    
    # 设置目标位置（相对运动）
    target_pos = init_pos + np.array([0.00, -0.00,-0.18])

    muj_robot = MujRobot(model_path="kuka_xml_urdf/iiwa14_dock.xml",render=True,dt=dt ,target_pos = target_pos)
    print(f"Target position: {target_pos}")
    #target_pos = np.array([0.25,0.25,0.5,])
    traj_duration = 15.0
    trajector_planner = DecoupledQuinticTrajectory(init_pos ,target_pos ,traj_duration)

    run_simulation(muj_robot,task_dynamics,trajector_planner,log,duration=20,dt=dt,q_init = q_init)

if __name__ == '__main__':
    main()

    



