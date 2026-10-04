"""Straight Cartesian waypoint segments with smooth SO(3) orientation references.

Each segment is rest-to-rest and C2, including angular motion. Translation stays
on the specified line (a screw interpolation could leave a clearance corridor).
"""
from __future__ import annotations

import numpy as np
import pinocchio as pin

from .trajectory import quintic_rest_to_rest_duration


class WaypointPoseTrajectory:
    """Named poses, with independent linear/angular limits for every segment."""

    def __init__(self, poses, names, limits):
        if len(poses) < 2 or len(names) != len(poses) or len(limits) != len(poses) - 1:
            raise ValueError("Waypoints require N poses/names and N-1 limit tuples")
        self.poses = [pin.SE3(pose) for pose in poses]
        self.names = list(names)
        self._segments = []
        for start, end, bounds in zip(self.poses[:-1], self.poses[1:], limits, strict=True):
            if len(bounds) != 4 or not np.all(np.isfinite(bounds)) or min(bounds) <= 0:
                raise ValueError("Waypoint velocity/acceleration limits must be finite and positive")
            delta = end.translation - start.translation
            phi = pin.log3(start.rotation.T @ end.rotation)
            distance, angle = np.linalg.norm(delta), np.linalg.norm(phi)
            durations = [quintic_rest_to_rest_duration(length, speed, accel)
                         for length, speed, accel in
                         [(distance, bounds[0], bounds[1]), (angle, bounds[2], bounds[3])]
                         if length > 1e-12]
            self._segments.append((start, delta, phi, max(durations, default=0.0)))
        self.times = np.r_[0.0, np.cumsum([segment[3] for segment in self._segments])]

    @property
    def total_duration(self):
        return float(self.times[-1])

    def phase(self, t):
        if t >= self.total_duration:
            return "hold"
        index = min(int(np.searchsorted(self.times[1:], max(0.0, t), side="right")),
                    len(self._segments) - 1)
        return self.names[index + 1]

    def _sample(self, t):
        if t >= self.total_duration:
            return self.poses[-1], np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)
        index = min(int(np.searchsorted(self.times[1:], max(0.0, t), side="right")),
                    len(self._segments) - 1)
        start, delta, phi, duration = self._segments[index]
        u = np.clip((t - self.times[index]) / duration, 0.0, 1.0)
        s = 10*u**3 - 15*u**4 + 6*u**5
        sd = (30*u**2 - 60*u**3 + 30*u**4) / duration
        sdd = (60*u - 180*u**2 + 120*u**3) / duration**2
        pose = pin.SE3(start.rotation @ pin.exp3(phi*s), start.translation + delta*s)
        return pose, delta*sd, delta*sdd, phi*sd, phi*sdd

    def get_state(self, t):
        pose, velocity, acceleration, _, _ = self._sample(t)
        return pose.translation.copy(), velocity, acceleration

    def get_pose(self, t):
        return pin.SE3(self._sample(t)[0])

    def get_motion_state(self, t):
        pose, velocity, acceleration, omega, alpha = self._sample(t)
        body_velocity = pose.rotation.T @ velocity
        # d(R^T v)/dt = R^T a - omega x (R^T v).
        body_acceleration = pose.rotation.T @ acceleration - np.cross(omega, body_velocity)
        return pin.SE3(pose), np.r_[body_velocity, omega], np.r_[body_acceleration, alpha]
