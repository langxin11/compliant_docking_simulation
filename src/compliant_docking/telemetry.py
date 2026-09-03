"""
telemetry.py - 数据记录 + SciencePlots IEEE 中文绘图（绘图实现见 plotting.py）/
telemetry.py - Simulation data logging + SciencePlots IEEE plotting (see plotting.py)

This module provides a logging system for robot control simulations.
It captures time-series data for joint angles, velocities, end-effector positions,
control torques, and external forces/torques during simulation runs.

Features:
- Data collection for robot state variables during simulation
- 委托 compliant_docking.plotting 完成 SciencePlots IEEE 中文风格绘图 /
  Delegates plotting to compliant_docking.plotting (SciencePlots IEEE style)

Author: langxin11
Date: 2025
"""

from pathlib import Path

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

        # 末端姿态误差向量（世界系，log(R_d R^T)）；仅在调用方提供时记录，
        # 列表长度可能短于其他列表（metrics 侧按非空判断）
        self.orientation_errors = []



    def store_data(self, t: float, q: np.ndarray,
                   v: np.ndarray, pos_actual: np.ndarray,
                   vel_actual: np.ndarray, error: float,
                   pos_desired: np.ndarray, vel_desired: np.ndarray,
                   acc_desired: np.ndarray, tau: np.ndarray,
                   external_force: np.ndarray, external_torque: np.ndarray,
                   *, orientation_error: np.ndarray | None = None):
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
            orientation_error: 末端姿态误差向量（世界系，可选；None 时不记录）
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

        if orientation_error is not None:
            self.orientation_errors.append(orientation_error)



    def plot_results(self, save_path: str = "figure/",
                     *, scene_name: str | None = None) -> list[Path]:
        """绘制仿真结果（SciencePlots IEEE 中文风格，委托 plotting 模块）/
        Plot simulation results (SciencePlots IEEE CJK style; delegates to plotting)."""
        # 惰性导入：telemetry 在无 scienceplots 的环境下仍可独立 import，不连累 CLI /
        # Lazy import: telemetry stays importable without scienceplots installed
        from compliant_docking.plotting import plot_docking_log

        return plot_docking_log(self, save_path, scene_name=scene_name)
