"""Declared PetalDock seating reference with every-step physical stop contact.

Uses supplied flange separation and mating phase, rather than the old crown's
single visual mesh. This is a simulation seating gate, not a locking certificate.
"""
from __future__ import annotations

import json

import numpy as np

from .docking_task import target_rotation


def evaluate_petal_seating(samples, scene, *, complete):
    if not samples:
        return dict(status="INCOMPLETE", reasons=["no synchronized samples"])
    info = json.loads((scene.tool.mjcf.parent / "model_info.json").read_text())
    R = target_rotation(scene.target)
    times = np.asarray([s["t"] for s in samples])
    tail = [s for s in samples if s["t"] > times[-1]-1.]
    positions = np.asarray([R.T @ (s["position"]-scene.target.pos) for s in tail])
    rotations = np.asarray([R.T @ s["rotation"] for s in tail])
    yaw = np.rad2deg(np.arctan2(rotations[:, 1, 0], rotations[:, 0, 0]))
    nominal = np.asarray(info["mating_relative_quaternion_wxyz"])
    # Nominal quaternion has w=z=0: yaw is twice atan2(y,x).
    nominal_yaw = np.rad2deg(2*np.arctan2(nominal[2], nominal[1]))
    phase_error = np.abs((yaw-nominal_yaw+45.) % 90. - 45.)
    axial_gap = positions[:, 2]-info["nominal_flange_separation_m"]
    lateral = np.linalg.norm(positions[:, :2], axis=1)
    tilt = np.rad2deg(np.arccos(np.clip(-rotations[:, 2, 2], -1., 1.)))
    stop_fraction = np.mean([s["stop_contact_count"] > 0 for s in tail])
    reasons = []
    for failed, reason in [
        (lateral.max() > .0005, "lateral alignment"),
        (tilt.max() > .5, "axis alignment"),
        (phase_error.max() > 2., "mating phase"),
        (np.abs(axial_gap).max() > .00075, "nominal seating height"),
        (stop_fraction < .95, "stop contact not maintained"),
    ]:
        if failed:
            reasons.append(reason)
    return dict(status="INCOMPLETE" if not complete else "NOT_SEATED" if reasons else "SEATED_CANDIDATE",
                reasons=reasons, locking_verified=False,
                reference="supplied flange separation / phase, with loaded stop contacts",
                nominal_separation_mm=1000*info["nominal_flange_separation_m"],
                nominal_yaw_deg=float(nominal_yaw),
                last_second_max_lateral_mm=float(1000*lateral.max()),
                last_second_max_tilt_deg=float(tilt.max()),
                last_second_max_phase_error_deg=float(phase_error.max()),
                last_second_max_abs_axial_gap_mm=float(1000*np.abs(axial_gap).max()),
                last_second_stop_contact_fraction=float(stop_fraction))
