"""
telemetry.py - Simulation Data Logging and Visualization Library

This module provides a comprehensive logging system for robot control simulations.
It captures time-series data for joint angles, velocities, end-effector positions,
control torques, and external forces/torques during simulation runs.

Features:
- Data collection for robot state variables during simulation
- Visualization of tracking performance and control inputs
- Multiple plot types for analyzing different aspects of control performance
- Support for external force/torque measurement visualization

Author: langxin11
Date: 2025
"""

import os

import matplotlib.pyplot as plt
import numpy as np


class Log:
    def __init__(self):
        # 关节数量（KUKA iiwa14 为 7 自由度）
        self.nq = 7

    def reset_logs(self):
        """重置记录数据：在每次仿真开始时调用"""

        self.t_list = []

        self.joint_angles = []
        self.joint_velocities = []

        self.pos_actual = []
        self.vel_actual = []

        self.error =[]

        self.pos_desired = []
        self.vel_desired = []
        self.acc_desired = []

        self.tau_hist = []
        self.force_externals = []
        self.torque_externals = []



    def store_data(self, t: float, q: np.ndarray,
                   v: np.ndarray, pos_actual: np.ndarray,
                   vel_actual: np.ndarray, error: float,
                   pos_desired: np.ndarray, vel_desired: np.ndarray,
                   acc_desired: np.ndarray, tau: np.ndarray,
                   external_force: np.ndarray, external_torque: np.ndarray):
        """
        存储数据（单步）：时间、关节状态、末端状态、期望轨迹、力矩及外力
        Args:
            t: 时间戳（秒）
            q: 当前关节角
            v: 当前关节角速度
            pos_actual/vel_actual: 当前末端位置/速度
            error: 末端位置跟踪误差范数
            pos_desired/vel_desired/acc_desired: 期望末端 pos/vel/acc
            tau: 控制器计算的关节力矩
            external_force/external_torque: 传感器外力/力矩（控制参考系）
        """

        self.t_list.append(t)

        self.joint_angles.append(q)
        self.joint_velocities.append(v)

        self.pos_actual.append(pos_actual)
        self.vel_actual.append(vel_actual)

        self.error.append(error)

        self.pos_desired.append(pos_desired)
        self.vel_desired.append(vel_desired)
        self.acc_desired.append(acc_desired)

        self.tau_hist.append(tau)
        self.force_externals.append(external_force)
        self.torque_externals.append(external_torque)



    def plot_results(self, save_path: str):
        """绘制仿真结果：末端跟踪曲线、误差、外力/外力矩、各关节力矩"""
        pos_actual = np.array(self.pos_actual).reshape(-1, 3)
        pos_desired = np.array(self.pos_desired)

        tau_hist = np.array(self.tau_hist)
        external_forces = np.array(self.force_externals)
        external_torques = np.array(self.torque_externals)

        if len(self.force_externals) < 1:
            external_forces = np.zeros_like(pos_actual)
            external_torques = np.zeros_like(pos_actual)


        # 创建三个子图：末端跟踪、误差、外力/外力矩
        fig = plt.figure(figsize=(15, 12))
        gs = plt.GridSpec(3, 2)

        # 1. 位置跟踪
        ax1 = fig.add_subplot(gs[0, :])
        labels = ['X', 'Y', 'Z']
        color = ['r', 'g', 'b']
        for i in range(3):
            ax1.plot(self.t_list, pos_actual[:, i], '-', label=f'Actual {labels[i]}', color=color[i])
            ax1.plot(self.t_list, pos_desired[:, i], '--', label=f'Desired {labels[i]}', color=color[i])
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Position [m]')
        ax1.legend()
        ax1.grid(True)
        ax1.set_title('End-effector Position Tracking')

        # 2. 跟踪误差
        ax2 = fig.add_subplot(gs[1, :])
        for i in range(3):
            error = pos_desired[:, i] - pos_actual[:, i]
            ax2.plot(self.t_list, error, label=f'{labels[i]} Error')
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Error [m]')
        ax2.legend()
        ax2.grid(True)
        ax2.set_title('Position Tracking Error')

        # 3. 关节角度和速度
        ax3 = fig.add_subplot(gs[2, 0])
        for i in range(3):
            ax3.plot(self.t_list[:], external_forces[:, i], label=f'axis {i+1}')
            ax3.set_xlabel('Time [s]')
            ax3.set_ylabel('Virtual Force [N]')
            ax3.legend()
            ax3.grid(True)
            ax3.set_title('Impedance Force')

        ax4 = fig.add_subplot(gs[2, 1])
        for i in range(3):
            ax4.plot(self.t_list[:], external_torques[:, i], label=f'axis {i+1}')
            ax4.set_xlabel('Time [s]')
            ax4.set_ylabel('Virtual Force [N]')
            ax4.legend()
            ax4.grid(True)
            ax4.set_title('Impedance Torque')

        os.makedirs(save_path, exist_ok=True)

        fig.savefig(save_path + "tracking_error.png",dpi=1200)

        fig = plt.figure(figsize=(15, 12))
        gs = plt.GridSpec(7, 1)

        for i in range(7):
            ax1 = fig.add_subplot(gs[i, 0])
            ax1.plot(self.t_list, tau_hist[:, i], label=f'Joint {i+1}')
            ax1.set_xlabel('Time [s]')
            ax1.set_ylabel('Torque [Nm]')
            ax1.legend()
            ax1.grid(True)
            ax1.set_title('Joint Torques')

        fig.savefig(save_path + "torque.png",dpi=1200)
