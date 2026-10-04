"""Latched, sensor-only yaw and optional lateral stiffness release.

This policy decreases stored spring energy after contact. It retains the
controller's yaw inertia/damping and never uses target truth or contact IDs.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ContactYawSpec:
    force_threshold_N: float = 0.15
    filter_s: float = 0.01
    dwell_s: float = 0.02
    release_s: float = 0.25
    stiffness_after: float = 0.0
    lateral_stiffness_after: float | None = None  # body X/Y, N/m; None keeps both fixed

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == "lateral_stiffness_after" and value is None:
                continue
            is_stiffness = name in ("stiffness_after", "lateral_stiffness_after")
            if not np.isfinite(value) or value < 0 or (not is_stiffness and value == 0):
                raise ValueError(f"contact_yaw.{name} must be finite and "
                                 + ("nonnegative" if is_stiffness else "positive"))


class ContactYawSchedule:
    def __init__(self, spec: ContactYawSpec, stiffness_before: float,
                 lateral_stiffness_before=None):
        if not np.isfinite(stiffness_before) or stiffness_before < spec.stiffness_after:
            raise ValueError("contact_yaw may only decrease a nonnegative yaw stiffness")
        self.spec = spec
        self.stiffness_before = float(stiffness_before)
        self.stiffness = self.stiffness_before
        self.lateral_stiffness_before = None
        self.lateral_stiffness = None
        if spec.lateral_stiffness_after is not None:
            lateral = np.asarray(lateral_stiffness_before, dtype=float)
            if (lateral.shape != (2,) or not np.isfinite(lateral).all()
                    or np.any(lateral < spec.lateral_stiffness_after)):
                raise ValueError("contact_yaw may only decrease both nonnegative lateral stiffnesses")
            self.lateral_stiffness_before = lateral.copy()
            self.lateral_stiffness = lateral.copy()
        self.filtered_force_N = 0.0
        self.trigger_t = None
        self._above_since = None
        self._last_t = None

    def update(self, t: float, phase: str, axial_force_N: float) -> float:
        if not np.isfinite([t, axial_force_N]).all() or (self._last_t is not None and t <= self._last_t):
            raise ValueError("contact_yaw requires finite force and increasing control timestamps")
        dt = 0.0 if self._last_t is None else t-self._last_t
        self._last_t = t
        # Ignore free-space inertial F/T loads. Filtering and dwell restart at
        # the insertion boundary; once triggered, release stays latched.
        if phase not in ("insert", "hold") and self.trigger_t is None:
            self.filtered_force_N = 0.0
            self._above_since = None
        else:
            alpha = -np.expm1(-dt/self.spec.filter_s)
            self.filtered_force_N += alpha*(abs(axial_force_N)-self.filtered_force_N)
            if self.trigger_t is None:
                if self.filtered_force_N >= self.spec.force_threshold_N:
                    if self._above_since is None:
                        self._above_since = t
                    if t-self._above_since >= self.spec.dwell_s-1e-12:
                        self.trigger_t = t
                else:
                    self._above_since = None
        if self.trigger_t is not None:
            u = float(np.clip((t-self.trigger_t)/self.spec.release_s, 0., 1.))
            blend = u*u*(3.-2.*u)  # zero slope at both ends
            self.stiffness = (self.stiffness_before
                              + blend*(self.spec.stiffness_after-self.stiffness_before))
            if self.lateral_stiffness_before is not None:
                self.lateral_stiffness = (self.lateral_stiffness_before
                    + blend*(self.spec.lateral_stiffness_after-self.lateral_stiffness_before))
        return self.stiffness


def control_stride(period: float | None, physics_dt: float) -> int:
    """Require an integer number of physics solves per control update."""
    period = physics_dt if period is None else period
    if not np.isfinite([period, physics_dt]).all() or min(period, physics_dt) <= 0:
        raise ValueError("control period and physics timestep must be finite and positive")
    ratio = period/physics_dt
    stride = int(round(ratio))
    if stride < 1 or not np.isclose(ratio, stride, rtol=0., atol=1e-10):
        raise ValueError("control_period must be an integer multiple of the physics timestep")
    return stride
