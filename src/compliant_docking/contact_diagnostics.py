"""Passive contact/F-T diagnostics at the solver time, in one world frame.

Call immediately after mj_step, before calling mj_forward or editing mjData.
Contacts, sensors, cvel/cacc and Cartesian transforms still describe the solve
at t; qpos/qvel have already advanced to t+dt. Never feed this channel to control.
"""
from __future__ import annotations

import mujoco
import numpy as np


def contact_wrench(contact, wrench, *, on_geom2, origin):
    """Contact rows are world axes; mj_contactForce acts on geom2."""
    rotation = np.asarray(contact.frame).reshape(3, 3).T
    sign = 1.0 if on_geom2 else -1.0
    force = sign * (rotation @ wrench[:3])
    moment = sign * (rotation @ wrench[3:]) + np.cross(contact.pos-origin, force)
    return np.r_[force, moment]


def inertial_wrench(model, data, body_ids, origin):
    """Sum I*a + v cross* I*v, including MuJoCo's -gravity in cacc.

    cinert/cvel/cacc use the common subtree-COM world frame, angular first.
    Return linear-first world wrench about origin. This is an effective inertial
    load (inertia minus gravity), not an estimate from differenced F/T readings.
    """
    total = np.zeros(6)
    for body_id in body_ids:
        inertia = data.cinert[body_id]
        tensor = np.array([[inertia[0], inertia[3], inertia[4]],
                      [inertia[3], inertia[1], inertia[5]],
                      [inertia[4], inertia[5], inertia[2]]])
        first_moment, mass = inertia[6:9], inertia[9]
        omega, velocity = data.cvel[body_id, :3], data.cvel[body_id, 3:]
        alpha, acceleration = data.cacc[body_id, :3], data.cacc[body_id, 3:]
        angular_momentum = tensor @ omega + np.cross(first_moment, velocity)
        momentum = mass*velocity - np.cross(first_moment, omega)
        force = mass*acceleration - np.cross(first_moment, alpha) + np.cross(omega, momentum)
        moment = (tensor @ alpha + np.cross(first_moment, acceleration)
                  + np.cross(omega, angular_momentum) + np.cross(velocity, momentum))
        com = data.subtree_com[model.body_rootid[body_id]]
        total += np.r_[force, moment + np.cross(com-origin, force)]
    return total


class ContactDiagnostics:
    def __init__(self, model, scene, *, store_events=True):
        self.model = model
        self.store_events = store_events
        self.tool_root = model.body(scene.eef_body).id
        self.site = model.site(scene.sensor_site).id
        self.body_ids = []
        for body_id in range(1, model.nbody):
            ancestor = body_id
            while ancestor and ancestor != self.tool_root:
                ancestor = int(model.body_parentid[ancestor])
            if ancestor == self.tool_root:
                self.body_ids.append(body_id)
        self.tool_geoms = set(np.flatnonzero(np.isin(model.geom_bodyid, self.body_ids)))
        self.target_geoms = {i for i in range(model.ngeom)
                             if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "")
                             .startswith(scene.target.prefix)}
        self.tool_mask = np.isin(np.arange(model.ngeom), list(self.tool_geoms))
        self.target_mask = np.isin(np.arange(model.ngeom), list(self.target_geoms))
        self.stop_mask = np.array(["_stop_" in (model.geom(i).name or "")
                                   for i in range(model.ngeom)])

    def _compact_contacts(self, data, origin):
        """Every-step wrench sum without retaining millions of contact events."""
        g1, g2 = data.contact.geom1, data.contact.geom2
        indices = np.flatnonzero(self.tool_mask[g1] != self.tool_mask[g2])
        local = np.zeros((len(indices), 6))
        for row, index in enumerate(indices):
            mujoco.mj_contactForce(self.model, data, int(index), local[row])
        frames = data.contact.frame[indices].reshape(-1, 3, 3)
        sign = np.where(self.tool_mask[g2[indices]], 1., -1.)[:, None]
        force = sign * np.einsum("nji,nj->ni", frames, local[:, :3])
        moment = (sign * np.einsum("nji,nj->ni", frames, local[:, 3:])
                  + np.cross(data.contact.pos[indices]-origin, force))
        interface = self.target_mask[g1[indices]] | self.target_mask[g2[indices]]
        stop = (interface & self.stop_mask[g1[indices]] & self.stop_mask[g2[indices]]
                & (data.contact.efc_address[indices] >= 0)
                & (data.contact.dist[indices] <= 1e-6) & (local[:, 0] > 1e-8))
        wrenches = np.column_stack((force, moment))
        info = dict(interface_count=int(interface.sum()), stop_contact_count=int(stop.sum()),
                    min_distance_m=float(np.min(data.contact.dist[indices][interface], initial=0.)))
        return wrenches[interface].sum(axis=0), wrenches.sum(axis=0), info

    def capture(self, data, solve_t, q_solve, v_solve):
        origin = data.xpos[self.tool_root].copy()
        rotation = data.xmat[self.tool_root].reshape(3, 3).copy()
        site_R = data.site_xmat[self.site].reshape(3, 3)
        raw = np.r_[data.sensor("force_sensor").data, data.sensor("torque_sensor").data].copy()
        force = -site_R @ raw[:3]
        sensor = np.r_[force, -site_R @ raw[3:]
                       + np.cross(data.site_xpos[self.site]-origin, force)]
        interface, all_contacts = np.zeros(6), np.zeros(6)
        rows = []
        for index, contact in enumerate(data.contact) if self.store_events else ():
            g1, g2 = int(contact.geom1), int(contact.geom2)
            if (g1 in self.tool_geoms) == (g2 in self.tool_geoms):
                continue
            local = np.zeros(6)
            mujoco.mj_contactForce(self.model, data, index, local)
            world = contact_wrench(contact, local, on_geom2=g2 in self.tool_geoms, origin=origin)
            is_interface = g1 in self.target_geoms or g2 in self.target_geoms
            all_contacts += world
            if is_interface:
                interface += world
            rows.append(dict(t=solve_t, index=index, geom1=g1, geom2=g2,
                             interface=is_interface, active=contact.efc_address >= 0,
                             distance_m=float(contact.dist), position_world=contact.pos.copy(),
                             normal_on_tool=(1 if g2 in self.tool_geoms else -1)
                             * contact.frame[:3].copy(), force_world=world[:3],
                             moment_at_tool_world=world[3:]))
        if self.store_events:
            info = dict(interface_count=sum(row["interface"] for row in rows),
                        min_distance_m=min((row["distance_m"] for row in rows if row["interface"]),
                                           default=0.0),
                        stop_contact_count=sum(
                            row["interface"] and self.stop_mask[row["geom1"]]
                            and self.stop_mask[row["geom2"]] and row["active"]
                            and row["distance_m"] <= 1e-6
                            and np.dot(row["force_world"], row["normal_on_tool"]) > 1e-8
                            for row in rows))
        else:
            interface, all_contacts, info = self._compact_contacts(data, origin)
        effective_inertia = inertial_wrench(self.model, data, self.body_ids, origin)
        # Applied body wrenches are about the COM, already in world coordinates.
        applied = np.zeros(6)
        for body_id in self.body_ids:
            value = data.xfrc_applied[body_id]
            applied += np.r_[value[:3], value[3:]
                             + np.cross(data.xipos[body_id]-origin, value[:3])]
        residual = sensor + effective_inertia - all_contacts - applied
        sample = dict(t=float(solve_t), state_t=float(data.time), q=q_solve.copy(), v=v_solve.copy(),
                      position=origin, rotation=rotation, sensor_raw_site=raw,
                      sensor_world=sensor, interface_world=interface,
                      all_contact_world=all_contacts, inertial_world=effective_inertia,
                      applied_world=applied, balance_residual_world=residual,
                      **info)
        return sample, rows


def summarize_diagnostics(samples, contacts, target_rotation):
    if not samples:
        return dict(status="INCOMPLETE")
    R = target_rotation
    def axial(key):
        return np.asarray([R[:, 2] @ s[key][3:] for s in samples])
    sensor, contact, inertia = (axial(key) for key in
                                ("sensor_world", "interface_world", "inertial_world"))
    index = int(np.argmax(np.abs(sensor)))
    t = samples[index]["t"]
    residual = np.asarray([s["balance_residual_world"] for s in samples])
    return dict(status="RECORDED", solve_time_convention="pre-integration t; state_t=t+dt",
                wrench_convention="force on tool; world frame; moment about tool root",
                peak_sensor_axial_moment_Nm=float(np.max(np.abs(sensor))),
                peak_contact_axial_moment_Nm=float(np.max(np.abs(contact))),
                peak_effective_inertial_axial_moment_Nm=float(np.max(np.abs(inertia))),
                peak_sensor_time_s=float(t),
                at_sensor_peak=dict(sensor_Nm=float(sensor[index]), contact_Nm=float(contact[index]),
                                    effective_inertia_Nm=float(inertia[index])),
                peak_balance_force_residual_N=float(np.max(np.linalg.norm(residual[:, :3], axis=1))),
                peak_balance_moment_residual_Nm=float(np.max(np.linalg.norm(residual[:, 3:], axis=1))),
                peak_contacts=[{k: v.tolist() if isinstance(v, np.ndarray) else v
                                for k, v in row.items()} for row in contacts if abs(row["t"]-t)<1e-9])


def evaluate_contact_load(wrenches, spec, target_rotation, *, complete):
    """Apply the same declared load limits to actual interface contact wrenches.

    Keep the original F/T benchmark gate separately: inertial cancellation can
    make F/T pass while the true contact load exceeds the same threshold.
    """
    values = np.asarray(wrenches, dtype=float)
    if values.size == 0:
        return dict(status="INCOMPLETE", reasons=["no contact telemetry"])
    if values.ndim != 2 or values.shape[1] != 6 or not np.isfinite(values).all():
        return dict(status="FAIL", reasons=["invalid contact telemetry"])
    force = float(np.max(np.linalg.norm(values[:, :3], axis=1)))
    moment = float(np.max(np.abs(values[:, 3:] @ target_rotation[:, 2])))
    reasons = []
    if force > spec.max_force:
        reasons.append("contact force limit")
    if moment > spec.max_axial_moment:
        reasons.append("contact axial moment limit")
    return dict(status="FAIL" if reasons else "PASS" if complete else "INCOMPLETE", reasons=reasons,
                peak_contact_force_N=force, peak_contact_axial_moment_Nm=moment,
                force_limit_N=spec.max_force, axial_moment_limit_Nm=spec.max_axial_moment)
