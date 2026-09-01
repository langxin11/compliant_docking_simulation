"""
pinocchio_sim.py - Pinocchio Robot Dynamics Simulation

This module implements a lightweight physics simulation of robot dynamics
using the Pinocchio robotics library (Runge-Kutta integration).

Key components:
- RobotSimulator: Simple physics simulation of robot dynamics using Runge-Kutta integration

Author: langxin11
Date: 2025
"""

from typing import Tuple

import numpy as np
import pinocchio as pin


class RobotSimulator:
    def __init__(self, robot_model: pin.Model, dt: float):
        # 重力置零由 compliant_docking.models.load_pin_model 负责（加载时统一处理）/
        # Gravity zeroing is owned by compliant_docking.models.load_pin_model
        self.model = robot_model
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
