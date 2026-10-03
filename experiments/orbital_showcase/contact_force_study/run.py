"""Isolated vertical mating force study with explicitly simplified seating pads.

Loads the existing vertical scene and a saved pre-mating state without rebuilding
or overwriting either. Pad contacts are real MuJoCo contacts, not weld reactions.
This is a force-control benchmark, not a reproduction of crown micro-contact.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
SCENE = HERE.parent / "assets/vertical_assembly.xml"
ROLLOUT = HERE.parent / "outputs/vertical/rollout.npz"
OUT = HERE / "outputs"
KP = np.array([600., 600., 500., 500., 180., 140., 80.])
KD = np.array([70., 70., 55., 55., 20., 16., 9.])
TORQUE_LIMITS = np.array([320., 320., 176., 176., 110., 40., 40.])
APOTHEM = .115 * np.cos(np.pi / 6)
TIP = np.array([-.42, .40, .18 + 3 * APOTHEM + .10])
ROTATION = np.diag([1., -1., -1.])


@dataclass(frozen=True)
class Case:
    name: str
    target_n: float = 8.
    feedback: bool = True
    pose_feedback: bool = True
    contact_timeconst_s: float = .008
    dt_s: float = .001
    speed_m_s: float = .002
    lateral_offset_m: float = 0.
    duration_s: float = 14.


def make_model(case):
    root = ET.parse(SCENE).getroot()
    root.find("option").set("timestep", str(case.dt_s))
    root.find("option").set("iterations", "100")
    mobile = root.find(".//body[@name='module1_port_4']")
    fixed = root.find(".//body[@name='module2_port_1']")
    # Port origins are 50 mm apart at the original nominal installed pose.
    # 29 mm mobile reach + 21 mm fixed reach closes this separation. These
    # dimensions define a benchmark seat, not measured crown CAD dimensions.
    for i, angle in enumerate(np.linspace(0, 2 * np.pi, 3, endpoint=False)):
        x, y = .028 * np.cos(angle), .028 * np.sin(angle)
        ET.SubElement(mobile, "geom", name=f"seat_mobile_{i}", type="sphere",
                      pos=f"{x} {y} .025", size=".004", contype="2", conaffinity="2",
                      group="3", rgba=".15 .8 .5 1", mass="0")
    ET.SubElement(fixed, "geom", name="seat_fixed", type="cylinder", pos="0 0 .018",
                  size=".042 .003", contype="2", conaffinity="2", group="3",
                  rgba=".15 .8 .5 1", mass="0")
    pairs = root.find("contact")
    if pairs is None:
        pairs = ET.SubElement(root, "contact")
    for i in range(3):
        ET.SubElement(pairs, "pair", geom1="seat_fixed", geom2=f"seat_mobile_{i}",
                      condim="3", friction=".2 .2 .001 .0001 .0001",
                      solref=f"{case.contact_timeconst_s} 1", solimp=".95 .95 .001")
    # Disable an accidental time-only assembly-lock operation: this benchmark
    # measures and holds force with assembly_lock OFF throughout.
    root.find(".//weld[@name='assembly_lock']").set("active", "false")
    xml = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml)
    return model, xml


def seating_wrench(model, data):
    """Sum only fixed-seat/mobile-pad forces ON module1, in world coordinates.

    mj_contactForce returns a wrench in the contact frame on geom2. Contact
    frame axes are rows, so transpose maps vectors to world coordinates.
    Translate each contact moment to the module1 port4 origin before summing.
    """
    fixed = model.geom("seat_fixed").id
    mobile = {model.geom(f"seat_mobile_{i}").id for i in range(3)}
    origin = data.body("module1_port_4").xpos
    force, moment = np.zeros(3), np.zeros(3)
    contacts = 0
    unplanned = []
    penetration = 0.
    for i in range(data.ncon):
        c = data.contact[i]
        if not ((c.geom1 == fixed and c.geom2 in mobile)
                or (c.geom2 == fixed and c.geom1 in mobile)):
            unplanned.append((int(c.geom1), int(c.geom2)))
            continue
        local = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, local)
        sign = 1. if c.geom2 in mobile else -1.
        f = sign * c.frame.reshape(3, 3).T @ local[:3]
        m = sign * c.frame.reshape(3, 3).T @ local[3:]
        force += f
        moment += m + np.cross(c.pos - origin, f)
        contacts += int(np.linalg.norm(f) > 1e-6)
        penetration = max(penetration, -float(c.dist))
    return force, moment, contacts, unplanned, penetration


def reference_ik(model, kin, q, position, rotation):
    q = q.copy()
    for _ in range(4):
        kin.qpos[:7] = q
        mujoco.mj_kinematics(model, kin)
        mujoco.mj_comPos(model, kin)
        site = kin.site("gripper_tip")
        error = np.r_[position - site.xpos,
                      Rotation.from_matrix(rotation @ site.xmat.reshape(3, 3).T).as_rotvec()]
        if np.linalg.norm(error) < 1e-8:
            break
        jac = np.zeros((6, model.nv))
        mujoco.mj_jacSite(model, kin, jac[:3], jac[3:], site.id)
        jac = jac[:, :7]
        q += jac.T @ np.linalg.solve(jac @ jac.T + 1e-8 * np.eye(6), error)
    if np.linalg.norm(error) > 1e-5:
        raise RuntimeError(f"Reference IK residual {np.linalg.norm(error)}")
    return q


def lock_ready(force, target, lateral, moment, speed, position_error, angle_error, contacts):
    """Experimental gate, to be calibrated against real latch tolerances later."""
    return bool(target > 0 and abs(force - target) <= .5 and contacts == 3
                and lateral <= 2. and moment <= .5 and abs(speed) <= .0005
                and position_error <= .001 and angle_error <= np.deg2rad(.5))


def evaluate(case, records, contact_time, faults):
    # Fixed final 3 s window; includes all samples, never drops lost contact.
    final = records["t"] >= case.duration_s - 3.
    f = records["force_z"][final]
    error = f - case.target_n
    result = dict(case=asdict(case), scope="fixed-base, zero-gravity, three-pad seating proxy",
                  assembly_weld_active=False, contact_time_s=contact_time,
                  peak_axial_force_n=float(np.max(records["force_z"])),
                  steady_mean_force_n=float(np.mean(f)),
                  steady_rmse_n=float(np.sqrt(np.mean(error ** 2))),
                  steady_peak_to_peak_n=float(np.ptp(f)),
                  steady_contact_fraction=float(np.mean(records["contacts"][final] > 0)),
                  max_lateral_force_n=float(np.max(records["lateral_force"])),
                  max_contact_moment_nm=float(np.max(records["moment"])),
                  max_penetration_m=float(np.max(records["penetration"])),
                  max_actual_normal_speed_m_s=float(np.max(abs(records["normal_speed"]))),
                  max_mating_position_error_in_final_window_m=float(np.max(records["pose_error"][final])),
                  max_mating_angle_error_in_final_window_deg=float(np.rad2deg(np.max(records["angle_error"][final]))),
                  force_pose_gate_eligible_at_end=bool(records["lock_ready"][-1]),
                  first_force_pose_gate_time_s=(float(records["t"][np.flatnonzero(records["lock_ready"])[0]])
                                               if np.any(records["lock_ready"]) else None),
                  minimum_joint_margin_rad=float(np.min(records["joint_margin"])),
                  max_torque_fraction=float(np.max(records["torque_fraction"])),
                  faults=sorted(faults))
    # These are benchmark acceptance criteria, not certified hardware limits.
    result["benchmark_criteria"] = dict(steady_rmse_n=.5, steady_contact_fraction=.99,
                                        peak_axial_force_n=1.5 * case.target_n,
                                        max_lateral_force_n=2., max_contact_moment_nm=.5)
    result["force_hold_pass"] = bool(contact_time is not None and not faults
                                     and result["steady_rmse_n"] <= .5
                                     and result["steady_contact_fraction"] >= .99
                                     and result["peak_axial_force_n"] <= 1.5 * case.target_n
                                     and result["max_lateral_force_n"] <= 2.
                                     and result["max_contact_moment_nm"] <= .5
                                     and result["force_pose_gate_eligible_at_end"])
    return result


def run(case):
    model, xml = make_model(case)
    data, kin = mujoco.MjData(model), mujoco.MjData(model)
    with np.load(ROLLOUT) as saved:
        index = int(np.argmin(abs(saved["t"] - 28.)))
        data.qpos[:] = saved["qpos"][index]
    data.qvel[:] = 0.
    data.eq_active[:] = [False, True, False]
    mujoco.mj_forward(model, data)
    position = data.site("gripper_tip").xpos.copy()
    position[0] += case.lateral_offset_m
    if case.lateral_offset_m:
        # Initialize arm and carried module together at the prescribed bias;
        # a sudden reference jump would confound the contact-speed comparison.
        data.qpos[:7] = reference_ik(model, kin, data.qpos[:7], position, ROTATION)
        address = int(model.jnt_qposadr[model.joint("module1_free").id])
        data.qpos[address] += case.lateral_offset_m
        mujoco.mj_forward(model, data)
    nominal_xy = TIP[:2].copy()
    rotation_correction = np.zeros(3)
    qref = data.qpos[:7].copy()
    qdot = np.zeros(7)
    filtered, velocity = 0., -case.speed_m_s
    contact_time = None
    gate_dwell = 0.
    faults = set()
    keys = ["t", "force_z", "filtered_force", "target", "lateral_force", "moment",
            "contacts", "normal_speed", "penetration", "joint_margin", "torque_fraction",
            "reference_z", "qpos", "qvel", "pose_error", "angle_error", "lock_ready"]
    records = {key: [] for key in keys}
    # KUKA iiwa14 limits from its supplied URDF; not all MuJoCo joints carry them.
    limit = np.deg2rad([170., 120., 170., 120., 170., 120., 175.])
    interval = max(1, round(.01 / case.dt_s))
    for step in range(round(case.duration_s / case.dt_s) + 1):
        t = step * case.dt_s
        # Refresh dynamics with the current control before reading solved contact
        # forces; avoid stale contact wrenches from the previous integration step.
        mujoco.mj_forward(model, data)
        f, moment, count, unexpected, penetration = seating_wrench(model, data)
        filtered += (1 - np.exp(-case.dt_s / .02)) * (f[2] - filtered)
        if contact_time is None and f[2] >= .15:
            contact_time = t
        target = (case.target_n * min(1., max(0., (t - contact_time) / 1.))
                  if contact_time is not None else 0.)
        if case.feedback and contact_time is not None:
            # Axial admittance: M*zdd + B*zd = F_contact - F_target.
            velocity += case.dt_s * (filtered - target - 400. * velocity) / .5
            velocity = float(np.clip(velocity, -case.speed_m_s, case.speed_m_s))
        else:
            velocity = -case.speed_m_s
        position[2] = max(TIP[2] - .030, position[2] + velocity * case.dt_s)
        if not case.feedback:
            position[2] = max(TIP[2] - .010, position[2])
        if step % interval == 0:
            if case.feedback and case.pose_feedback and contact_time is not None:
                # Slow outer pose compensation rejects load-induced joint-servo
                # deflection. Without it, correct axial force can hide a tilted
                # module and incomplete three-point seating.
                tip = data.site("gripper_tip")
                pose_gain = 3. * interval * case.dt_s
                position[:2] += pose_gain * (nominal_xy - tip.xpos[:2])
                rotation_correction += pose_gain * Rotation.from_matrix(
                    ROTATION @ tip.xmat.reshape(3, 3).T).as_rotvec()
                position[:2] = np.clip(position[:2], nominal_xy - .005, nominal_xy + .005)
                rotation_correction = np.clip(rotation_correction, -.052, .052)
            previous = qref.copy()
            desired_rotation = Rotation.from_rotvec(rotation_correction).as_matrix() @ ROTATION
            qref = reference_ik(model, kin, qref, position, desired_rotation)
            qdot = (qref - previous) / (interval * case.dt_s)
        data.ctrl[:] = KP * qref + KD * qdot + data.qfrc_bias[:7]
        torque = KP * (qref - data.qpos[:7]) + KD * (qdot - data.qvel[:7]) + data.qfrc_bias[:7]
        fraction = float(np.max(abs(torque) / TORQUE_LIMITS))
        margin = float(np.min(limit - abs(data.qpos[:7])))
        jac = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jac, None, model.site("gripper_tip").id)
        speed = float((jac @ data.qvel)[2])
        a, b = data.site("assembly_anchor"), data.site("module1_anchor")
        pose_error = float(np.linalg.norm(a.xpos - b.xpos))
        angle_error = float(np.linalg.norm(Rotation.from_matrix(
            a.xmat.reshape(3, 3) @ b.xmat.reshape(3, 3).T).as_rotvec()))
        ready = target == case.target_n and lock_ready(
            f[2], case.target_n, np.linalg.norm(f[:2]), np.linalg.norm(moment),
            speed, pose_error, angle_error, count)
        gate_dwell = gate_dwell + case.dt_s if ready else 0.
        if unexpected:
            faults.add("unexpected structural contact")
        if margin <= 0:
            faults.add("joint limit")
        if fraction >= 1.:
            faults.add("torque saturation")
        if not np.isfinite(data.qpos).all() or not np.isfinite(f).all():
            raise RuntimeError("Non-finite state")
        if data.eq_active[2]:
            raise RuntimeError("Assembly weld contaminated contact measurement")
        values = [t, f[2], filtered, target, np.linalg.norm(f[:2]), np.linalg.norm(moment),
                  count, speed, penetration, margin, fraction, position[2],
                  data.qpos.copy(), data.qvel.copy(), pose_error, angle_error, gate_dwell >= .5]
        for key, value in zip(keys, values, strict=True):
            records[key].append(value)
        if step < round(case.duration_s / case.dt_s):
            mujoco.mj_step(model, data)
    records = {key: np.array(value) for key, value in records.items()}
    result = evaluate(case, records, contact_time, faults)
    folder = OUT / case.name
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "model.xml").write_text(xml)
    np.savez_compressed(folder / "trace.npz", **records)
    (folder / "metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result), flush=True)
    return records, result


def plot(results):
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True)
    for case, trace, _ in results:
        axes[0].plot(trace["t"], trace["force_z"], label=case.name)
        axes[1].plot(trace["t"], trace["normal_speed"] * 1000, label=case.name)
    for force in [8, 10]:
        axes[0].axhline(force, color="gray", linestyle="--", alpha=.5)
    axes[0].set(ylabel="Axial contact force on module 1 [N]",
                title="Vertical seating force benchmark — simplified three-pad contact")
    axes[1].set(xlabel="Time since pre-mating initialization [s]",
                ylabel="Actual tool normal velocity [mm/s]")
    for ax in axes:
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    fig.savefig(OUT / "force_comparison.png", dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="Only nominal feedback and baseline")
    args = parser.parse_args()
    cases = [Case("position_only", feedback=False), Case("admittance_8N"),
             Case("admittance_10N", target_n=10.)]
    if not args.quick:
        cases += [Case("admittance_8N_stiffer", contact_timeconst_s=.004),
                  Case("admittance_8N_softer", contact_timeconst_s=.016),
                  Case("admittance_8N_half_dt", dt_s=.0005),
                  Case("admittance_8N_offset_1mm", lateral_offset_m=.001),
                  Case("axial_only_8N", pose_feedback=False)]
    results = [(case, *run(case)) for case in cases]
    plot(results)
    (OUT / "summary.json").write_text(json.dumps([r for _, _, r in results], indent=2) + "\n")
    (OUT / "source_manifest.json").write_text(json.dumps({str(p): hashlib.sha256(p.read_bytes()).hexdigest()
         for p in [Path(__file__), SCENE, ROLLOUT]}, indent=2) + "\n")
    (OUT / "runtime.json").write_text(json.dumps(dict(mujoco=mujoco.__version__,
                                                      numpy=np.__version__), indent=2) + "\n")
    if any(not result["force_hold_pass"] for case, _, result in results
           if case.feedback and case.pose_feedback):
        raise SystemExit(2)
    if any(result["force_pose_gate_eligible_at_end"] for case, _, result in results
           if case.name == "axial_only_8N"):
        raise RuntimeError("Incomplete seating was accepted")


if __name__ == "__main__":
    main()
