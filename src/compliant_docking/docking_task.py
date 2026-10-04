"""Estimated-target approach and insertion; physical truth is evaluation-only.

The initial application is the upright iiwa14 flower-crown interface. Heights
refer to the controlled EE frame, not to a mesh vertex or the flange.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin

from .planning.waypoints import WaypointPoseTrajectory


@dataclass(frozen=True)
class DockingTaskSpec:
    """Explicit estimate, insertion geometry and declared experiment criteria."""

    estimate_pos: tuple[float, float, float]
    estimate_yaw_deg: float = 0.0
    mating_yaw_deg: float = 0.0  # known interface clocking, separate from estimation error
    entry_height: float = 0.070
    clearance: float = 0.060
    command_depth: float = 0.040
    insertion_speed: float = 0.010
    insertion_acceleration: float = 0.030
    hold_s: float = 8.0
    min_depth: float = 0.017
    max_lateral: float = 0.004
    max_tilt_deg: float = 5.0
    max_force: float = 40.0
    max_axial_moment: float = 2.0
    max_linear_speed: float = 0.003
    max_angular_speed_deg: float = 2.0
    max_torque_saturation_ratio: float = 0.01

    def __post_init__(self):
        position = np.asarray(self.estimate_pos, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("docking.estimate_pos must be a finite world XYZ position")
        if not np.isfinite([self.estimate_yaw_deg, self.mating_yaw_deg]).all():
            raise ValueError("docking yaw angles must be finite")
        for key, value in vars(self).items():
            if key not in ("estimate_pos", "estimate_yaw_deg", "mating_yaw_deg"):
                if not np.isfinite(value) or value <= 0:
                    raise ValueError(f"docking.{key} must be finite and positive")
        if self.min_depth > self.command_depth:
            raise ValueError("docking.min_depth cannot exceed command_depth")
        if self.hold_s < 1.0:
            raise ValueError("docking.hold_s must cover the 1 s acceptance window")
        if self.max_torque_saturation_ratio > 1.0:
            raise ValueError("docking.max_torque_saturation_ratio must not exceed 1")


def estimated_target_pose(spec):
    return pin.SE3(pin.exp3(np.array([0.0, 0.0, np.deg2rad(spec.estimate_yaw_deg)])),
                   np.asarray(spec.estimate_pos, dtype=float))


def build_docking_trajectory(task, spec, limits):
    """Build only from the estimate; this function never receives TargetSpec."""
    target = estimated_target_pose(spec)
    rotation = (target.rotation @ pin.exp3(np.array([0., 0., np.deg2rad(spec.mating_yaw_deg)]))
                @ np.diag([1.0, -1.0, -1.0]))
    entry = target.translation + np.array([0.0, 0.0, spec.entry_height])
    above = entry + np.array([0.0, 0.0, spec.clearance])
    final = entry - np.array([0.0, 0.0, spec.command_depth])
    poses = [pin.SE3(task.init_ori, task.init_pos)]
    names = ["start"]
    approach_bounds = (limits.v_max_approach, limits.a_max_approach,
                       limits.omega_max_approach, limits.alpha_max_approach)
    bounds = []
    # Lift at the starting XY before translating if the start is below clearance.
    if task.init_pos[2] < above[2] - 1e-9:
        lift = np.array(task.init_pos, copy=True)
        lift[2] = above[2]
        poses.append(pin.SE3(task.init_ori, lift))
        names.append("lift")
        bounds.append(approach_bounds)
    poses.extend([pin.SE3(rotation, above), pin.SE3(rotation, entry), pin.SE3(rotation, final)])
    names.extend(["approach", "descent", "insert"])
    bounds.extend([approach_bounds,
                   (limits.v_max_docking, limits.a_max_docking,
                    limits.omega_max_docking, limits.alpha_max_docking),
                   (spec.insertion_speed, spec.insertion_acceleration,
                    limits.omega_max_docking, limits.alpha_max_docking)])
    return WaypointPoseTrajectory(poses, names, bounds)


def target_rotation(target):
    return pin.Quaternion(*np.asarray(target.quat, dtype=float)).matrix()


def docking_sample(spec, target, *, t, phase, position, rotation, velocity,
                   angular_velocity, force, moment, interface_contacts, other_contacts):
    """Evaluate against actual geometry. Never feed this dictionary to control."""
    R = target_rotation(target)
    local_pos = R.T @ (position - target.pos)
    local_rot = R.T @ rotation
    local_force, local_moment = R.T @ force, R.T @ moment
    return dict(t=float(t), phase=phase,
                depth_mm=float(1000*(spec.entry_height - local_pos[2])),
                lateral_mm=float(1000*np.linalg.norm(local_pos[:2])),
                yaw_deg=float(np.rad2deg(np.arctan2(local_rot[1, 0], local_rot[0, 0]))),
                tilt_deg=float(np.rad2deg(np.arccos(np.clip(-local_rot[2, 2], -1, 1)))),
                axial_force_N=float(local_force[2]), axial_moment_Nm=float(local_moment[2]),
                force_N=float(np.linalg.norm(force)), speed_m_s=float(np.linalg.norm(velocity)),
                angular_speed_deg_s=float(np.rad2deg(np.linalg.norm(angular_velocity))),
                interface_contacts=int(interface_contacts), other_contacts=int(other_contacts))


def evaluate_docking(log, spec, trajectory, dt):
    """A declared insertion/settling gate, not a CAD seating or locking certificate.

Yaw drift is deliberately measured rather than penalized: passive axial rotation
is required for the crown to accommodate contact. Tilt remains constrained.
"""
    samples = log.docking_samples
    if not samples:
        return dict(status="INCOMPLETE", reasons=["no samples"])
    numeric_keys = [key for key in samples[0] if key != "phase"]
    if not all(np.isfinite([sample[key] for key in numeric_keys]).all() for sample in samples):
        return dict(status="FAIL", reasons=["non-finite telemetry"])
    end = samples[-1]["t"]
    complete = end + dt >= trajectory.total_duration + spec.hold_s
    tail = [sample for sample in samples if sample["t"] > end - 1.0]
    peak_force = max(sample["force_N"] for sample in samples)
    peak_moment = max(abs(sample["axial_moment_Nm"]) for sample in samples)
    early = any(sample["interface_contacts"] and sample["phase"] not in ("insert", "hold")
                for sample in samples)
    reasons = []
    if peak_force > spec.max_force:
        reasons.append("force limit")
    if peak_moment > spec.max_axial_moment:
        reasons.append("axial moment limit")
    if early:
        reasons.append("contact before insertion phase")
    if any(sample["other_contacts"] for sample in samples):
        reasons.append("contact outside tool-target interface")
    if getattr(log, "docking_joint_limit_violation", False):
        reasons.append("joint position limit")
    saturation = float(np.mean(log.torque_saturated))
    if not np.isfinite(saturation) or saturation > spec.max_torque_saturation_ratio:
        reasons.append("torque saturation")
    if complete:
        checks = [(min(s["depth_mm"] for s in tail) >= 1000*spec.min_depth, "insufficient insertion"),
                  (max(s["lateral_mm"] for s in tail) <= 1000*spec.max_lateral, "lateral offset"),
                  (max(s["tilt_deg"] for s in tail) <= spec.max_tilt_deg, "tilt"),
                  (max(s["speed_m_s"] for s in tail) <= spec.max_linear_speed, "not settled (translation)"),
                  (max(s["angular_speed_deg_s"] for s in tail) <= spec.max_angular_speed_deg,
                   "not settled (rotation)"),
                  (np.mean([s["interface_contacts"] > 0 for s in tail]) >= 0.95, "contact not maintained")]
        reasons.extend(reason for passed, reason in checks if not passed)
    contact = next((s for s in samples if s["interface_contacts"]), None)
    yaw = np.rad2deg(np.unwrap(np.deg2rad([s["yaw_deg"] for s in samples])))
    contact_index = samples.index(contact) if contact is not None else len(samples) - 1
    return dict(status="FAIL" if reasons else "PASS" if complete else "INCOMPLETE",
                reasons=reasons, duration_s=end, first_contact_s=contact["t"] if contact else None,
                final_depth_mm=samples[-1]["depth_mm"], final_lateral_mm=samples[-1]["lateral_mm"],
                final_tilt_deg=samples[-1]["tilt_deg"], final_yaw_deg=samples[-1]["yaw_deg"],
                contact_yaw_change_deg=float(yaw[-1] - yaw[contact_index]),
                post_contact_advance_mm=(samples[-1]["depth_mm"] - contact["depth_mm"]
                                         if contact is not None else None),
                peak_force_N=peak_force, peak_axial_moment_Nm=peak_moment,
                torque_saturation_ratio=saturation)
