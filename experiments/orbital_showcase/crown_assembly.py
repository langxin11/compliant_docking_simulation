"""Original convex-crown contact in the full vertical robotic assembly sequence."""
# ruff: noqa: E402
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import xml.etree.ElementTree as ET

import assembly_sequence as seq
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import vertical_assembly as vertical
from contact_force_study.run import reference_ik
from scipy.spatial.transform import Rotation
from side_docking import APOTHEM, quaternion

OUT = seq.HERE / "outputs/crown_integrated"
CROWN = seq.ROOT / "assets/interfaces/convex_crown"
HEIGHT = .0471874034950606


def seating_contact(model, data):
    # Legacy crown has no separately labelled seating stop.
    return True


def configure():
    vertical.configure()
    seq.OUT = OUT
    seq.INSTALLED = vertical.SEED + [0, 0, 2*APOTHEM+HEIGHT]
    seq.INSTALL_TIP = seq.INSTALLED + [0, 0, APOTHEM+seq.ENGAGEMENT]
    seq.phases = phases


def phases():
    result = vertical.phases()
    result[8].end = seq.INSTALL_TIP + [0, 0, .006]
    result[8].title = "09  降至花冠上方 6 mm，准备低速接触"
    result[9].seconds = 14.
    result[9].event = None
    result[9].title = "10  花冠实际接触 · 柔顺插合 · 持续就位后锁定"
    result[10].title = "11  模块间锁定确认，卸力后释放机械臂"
    return result


def build_model():
    root = ET.parse(seq.ASSETS / "vertical_assembly.xml").getroot()
    root.set("model", "vertical_crown_contact_assembly")
    root.find("compiler").set("meshdir", str(seq.ROOT/"assets/iiwa14/assets"))
    # Identical original-crown convex geometry on both active side ports.
    source = ET.parse(CROWN / "male.xml").getroot()
    asset = root.find("asset")
    meshes = source.findall("asset/mesh")[1:]
    for item in meshes:
        ET.SubElement(asset, "mesh", name="contact_"+item.get("name"),
                      file=str((CROWN / item.get("file")).resolve()))
    for module, number in [("module1", 4), ("module2", 1)]:
        body = root.find(f".//body[@name='{module}_port_{number}_crown']")
        if module == "module2":
            # Port roots differ by Rz(180)Rx(180); calibrate the target crown
            # attachment phase to the sampled Rz(4.25)Rx(180) root reference.
            body.set("euler", "0 0 175.75")
        for i, item in enumerate(meshes):
            ET.SubElement(body, "geom", name=f"{module}_contact_{i}", type="mesh",
                          mesh="contact_"+item.get("name"), mass="0", group="3",
                          contype="2", conaffinity="2", friction=".2 .001 .0001",
                          solref=".008 1", solimp=".95 .95 .001")
    root.find(".//site[@name='assembly_anchor']").set("pos", seq.vector([-2*APOTHEM-HEIGHT, 0, 0]))
    OUT.mkdir(parents=True, exist_ok=True)
    xml = ET.tostring(root, encoding="unicode")
    (OUT / "model.xml").write_text(xml)
    return mujoco.MjModel.from_xml_string(xml)


def wrench(model, data):
    mobile = {model.geom(f"module1_contact_{i}").id for i in range(130)}
    fixed = {model.geom(f"module2_contact_{i}").id for i in range(130)}
    f, m = np.zeros(3), np.zeros(3)
    unexpected, depth = [], 0.
    origin = data.body("module1_port_4").xpos
    count = 0
    for i in range(data.ncon):
        c = data.contact[i]
        if not ((c.geom1 in mobile and c.geom2 in fixed) or
                (c.geom2 in mobile and c.geom1 in fixed)):
            unexpected.append((int(c.geom1), int(c.geom2)))
            continue
        local = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, local)
        sign = 1 if c.geom2 in mobile else -1
        fi = sign*c.frame.reshape(3, 3).T@local[:3]
        f += fi
        m += sign*c.frame.reshape(3, 3).T@local[3:]+np.cross(c.pos-origin, fi)
        depth = max(depth, -float(c.dist))
        count += int(np.linalg.norm(fi) > 1e-7)
    return f, m, count, unexpected, depth


def simulate(model, spline, phase_list, planning, quick=False, *,
             contact_wrench=wrench, contact_seated=seating_contact, storage_contact=None,
             contact_force_n=.3):
    data, kin = mujoco.MjData(model), mujoco.MjData(model)
    ends = np.cumsum([p.seconds for p in phase_list])
    starts = np.r_[0, ends[:-1]]
    if quick:
        with np.load(seq.HERE / "outputs/vertical/rollout.npz") as saved:
            i = int(np.argmin(abs(saved["t"]-29.)))
            data.qpos[:] = saved["qpos"][i]
        # Initialization only: shift module and tool together from old 50 mm
        # reference to the calibrated pre-contact gap, then integrate normally.
        mujoco.mj_forward(model, data)
        position = seq.INSTALL_TIP+[0, 0, .006]
        old = data.site("gripper_tip").xpos.copy()
        data.qpos[:7] = reference_ik(model, kin, data.qpos[:7], position, vertical.TIP_ROTATION)
        data.qpos[7:10] += position-old
        data.eq_active[:] = [False, True, False]
    else:
        data.qpos[:7] = spline(0)
    mujoco.mj_forward(model, data)
    position = seq.INSTALL_TIP+[0, 0, .006]
    correction = np.zeros(3)
    qref = data.qpos[:7].copy()
    qdot, filtered, zvelocity = np.zeros(7), 0., -.001
    first_contact, locked, dwell, unload_dwell = None, None, 0., 0.
    events, done, faults = [], set(), set()
    records = {key: [] for key in ["t", "qpos", "qvel", "phase", "locks"]}
    telemetry = []
    storage_telemetry = []
    gripper_id = model.equality("gripper_lock").id
    low, high = np.asarray(planning["joint_limits"])
    start_step = round(starts[9]/seq.DT) if quick else 0
    for step in range(start_step, round(ends[-1]/seq.DT)+1):
        t = step*seq.DT
        k = min(int(np.searchsorted(ends, t, side="right")), len(phase_list)-1)
        mujoco.mj_forward(model, data)
        f, moment, count, unexpected, depth = contact_wrench(model, data)
        seating_condition = contact_seated(model, data)
        physical_error = seq.lock_error(model, data, 2)
        ready = False
        rows = np.flatnonzero((data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
                              & (data.efc_id == gripper_id))
        grip_load = float(np.linalg.norm(data.efc_force[rows[:3]])) if len(rows) else 0.
        if locked is not None and grip_load <= .2 and np.linalg.norm(data.qvel[:7]) <= .002:
            unload_dwell += seq.DT
        else:
            unload_dwell = 0.
        if unexpected:
            faults.add("structural contact")
        if np.any(data.qpos[:7] < low) or np.any(data.qpos[:7] > high):
            faults.add("joint limit")
        contact_mode = k == 9 and locked is None
        if contact_mode:
            filtered += (1-np.exp(-seq.DT/.02))*(f[2]-filtered)
            if first_contact is None and f[2] >= .1:
                first_contact = t
            if first_contact is not None:
                target = contact_force_n*min(1., (t-first_contact)/1.)
                zvelocity += seq.DT*(filtered-target-200*zvelocity)/.5
                zvelocity = float(np.clip(zvelocity, -.001, .001))
            else:
                zvelocity = -.001
            position[2] = max(seq.INSTALL_TIP[2]-.002, position[2]+zvelocity*seq.DT)
            if step % 10 == 0:
                tip = data.site("gripper_tip")
                actual = tip.xmat.reshape(3, 3)
                position[:2] += .03*(seq.INSTALL_TIP[:2]-tip.xpos[:2])
                error = Rotation.from_matrix(vertical.TIP_ROTATION@actual.T).as_rotvec()
                correction[:2] += .03*error[:2]
                correction[:2] = np.clip(correction[:2], -.052, .052)
                yaw = .8*np.arctan2(actual[1, 0], actual[0, 0])
                desired_rotation = Rotation.from_rotvec(correction).as_matrix()@Rotation.from_euler("z", yaw).as_matrix()@vertical.TIP_ROTATION
                qref = reference_ik(model, kin, qref, position, desired_rotation)
                # Retain real servo damping. Feeding measured yaw velocity back
                # as reference velocity cancels damping and permits drift.
                qdot = np.zeros(7)
            desired, velocity = qref, qdot
            error = physical_error
            ready = (first_contact is not None and t-first_contact >= 1. and .15 <= f[2] <= .6
                     and error[0] <= .00075 and error[1] <= np.deg2rad(.5)
                     and error[2] <= .0005 and error[3] <= np.deg2rad(.5)
                     and depth <= .0003 and np.linalg.norm(moment) <= .5
                     and seating_condition)
            dwell = dwell+seq.DT if ready else 0.
            if dwell >= .5:
                # Site weld target matches measured installed pose: no snap.
                anchor = model.site("assembly_anchor").id
                module2 = data.body("module2")
                model.site_pos[anchor] = module2.xmat.reshape(3, 3).T@(data.body("module1").xpos-module2.xpos)
                model.site_quat[anchor] = quaternion(module2.xmat.reshape(3, 3).T@data.body("module1").xmat.reshape(3, 3))
                mujoco.mj_forward(model, data)
                data.eq_active[2] = True
                locked = t
                qref, qdot = data.qpos[:7].copy(), np.zeros(7)
                events.append(dict(t=t, event="assembly_on", force_n=float(f[2]), dwell_s=dwell))
                print(events[-1], flush=True)
        elif k == 9 or k == 10:
            if locked is None:
                faults.add("contact seating timeout")
                break
            desired, velocity = qref, np.zeros(7)
        else:
            desired, velocity = spline(t), spline(t, 1)
            # Seed contact IK from the preceding transport reference, rather
            # than the initial storage pose.
            qref = desired.copy()
        phase = phase_list[k]
        if phase.event and phase.event not in done and t-starts[k] >= phase.seconds*.6:
            if quick and phase.event in {"grip_on", "rack_off"}:
                continue
            if phase.event != "grip_off" or unload_dwell >= .5:
                error = seq.handover(model, data, phase.event)
                events.append(dict(t=t, event=phase.event, error_m=error[0],
                                   gripper_constraint_force_n=grip_load, unloaded_dwell_s=unload_dwell))
                done.add(phase.event)
                print(events[-1], flush=True)
        data.ctrl[:] = seq.KP*desired+seq.KD*velocity+data.qfrc_bias[:7]
        torque = seq.KP*(desired-data.qpos[:7])+seq.KD*(velocity-data.qvel[:7])+data.qfrc_bias[:7]
        if np.any(abs(torque) > seq.TORQUE_LIMITS):
            faults.add("torque saturation")
        telemetry.append([t, f[2], np.linalg.norm(f[:2]), np.linalg.norm(moment), count, depth, dwell,
                          *physical_error, int(data.eq_active[2]), grip_load, unload_dwell,
                          int(seating_condition), int(ready)])
        if storage_contact is not None:
            storage_force, storage_depth, storage_count = storage_contact(model, data)
            separation = float(np.linalg.norm(data.site("module1_port_4_mating").xpos-data.site("storage_dock_mating").xpos))
            storage_telemetry.append([t, *storage_force, storage_depth, storage_count, separation,
                                      *data.eq_active.astype(int)])
        if step % 10 == 0:
            for key, value in [("t", t), ("qpos", data.qpos.copy()), ("qvel", data.qvel.copy()),
                               ("phase", k), ("locks", data.eq_active.copy())]:
                records[key].append(value)
        if not np.isfinite(data.qpos).all():
            raise RuntimeError("Nonfinite state")
        if step < round(ends[-1]/seq.DT):
            mujoco.mj_step(model, data)
    a = np.asarray(telemetry)
    final_error = float(np.linalg.norm(data.body("module1").xpos-seq.INSTALLED))
    if list(data.eq_active) != [False, False, True]:
        faults.add("incomplete handover")
    if final_error > .001:
        faults.add("final module position")
    if sum(w.number for w in data.warning):
        faults.add("simulation warning")
    if a[:, 1].max() > 10 or a[:, 5].max() > .0003:
        faults.add("contact load or penetration limit")
    if storage_telemetry:
        stored = np.asarray(storage_telemetry)
        if np.linalg.norm(stored[:, 1:4], axis=1).max() > 10 or stored[:, 4].max() > .0003:
            faults.add("storage contact load or penetration limit")
    report = dict(status="FAIL" if faults else "PASS", faults=sorted(faults), events=events,
                  peak_axial_force_n=float(a[:, 1].max()), max_penetration_mm=float(a[:, 5].max()*1000),
                  final_module_error_mm=final_error*1000, first_contact_s=first_contact,
                  lock_time_s=locked, module_mass_kg=float(model.body("module1").mass[0]),
                  module_inertia_kg_m2=model.body("module1").inertia.tolist(),
                  duration_s=float(ends[-1]),
                  contact_force_target_n=contact_force_n,
                  scope="full arm and module dynamics; convex guide contact; ideal locks")
    if storage_telemetry:
        report["storage_interface"] = dict(
            module_port="module1_port_4", storage_port="storage_dock_port",
            peak_contact_force_n=float(np.linalg.norm(stored[:, 1:4], axis=1).max()),
            max_penetration_mm=float(stored[:, 4].max()*1000),
            final_clearance_m=float(stored[-1, 6]))
        np.savez_compressed(OUT/"storage_trace.npz", values=stored,
                           columns=np.array(["time", "force_x", "force_y", "force_z", "penetration",
                                             "loaded_contacts", "mating_separation", "storage_locked",
                                             "gripper_locked", "assembly_locked"]))
    np.savez_compressed(OUT/"contact_trace.npz", values=a,
                       columns=np.array(["time", "axial_force", "lateral_force", "moment", "contacts",
                                         "penetration", "seating_dwell", "position_error", "angle_error",
                                         "relative_speed", "relative_spin", "assembly_locked",
                                         "gripper_constraint_force", "unloaded_dwell",
                                         "seating_contact_condition", "seating_eligible"]))
    records = {key: np.asarray(value) for key, value in records.items()}
    np.savez_compressed(OUT/"rollout.npz", **records)
    (OUT/"validation.json").write_text(json.dumps(report, indent=2)+"\n")
    # Persist the accepted anchor frame for correct replay and no pose snapping.
    (OUT/"accepted_anchor.json").write_text(json.dumps(dict(pos=model.site("assembly_anchor").pos.tolist(),
                                                           quat=model.site("assembly_anchor").quat.tolist())))
    print(json.dumps(report), flush=True)
    return records, report


def run_scenario(args, hooks=None):
    model = hooks.build_model() if hooks else build_model()
    phase_list = hooks.phases() if hooks else phases()
    spline, phase_list, planning = seq.plan(model, initial_q=vertical.INITIAL_Q, phase_list=phase_list)
    records, report = simulate(model, spline, phase_list, planning, args.quick,
                               contact_wrench=hooks.wrench if hooks else wrench,
                               contact_seated=hooks.seating_contact if hooks else seating_contact,
                               storage_contact=hooks.storage_wrench if hooks else None,
                               contact_force_n=hooks.contact_force_n if hooks else .3)
    report["module_asset"] = args.module
    (OUT/"validation.json").write_text(json.dumps(report, indent=2)+"\n")
    (OUT/"phases.json").write_text(json.dumps([
        dict(key=p.key, title=p.title, seconds=p.seconds, end=p.end.tolist(), event=p.event)
        for p in phase_list], indent=2, ensure_ascii=False)+"\n")
    (OUT/"runtime.json").write_text(json.dumps(dict(
        python=platform.python_version(), platform=platform.platform(),
        mujoco=mujoco.__version__, numpy=np.__version__, physics_timestep_s=seq.DT,
        state_record_hz=100, contact_record_hz=1000, video_fps=24), indent=2)+"\n")
    if report["status"] != "PASS":
        raise SystemExit(2)
    a = np.load(OUT/"contact_trace.npz")["values"]
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(a[:, 0], a[:, 1])
    axes[0].set_ylabel("Axial contact force [N]")
    axes[1].plot(a[:, 0], a[:, 7]*1000)
    axes[1].set_ylabel("Installation position error [mm]")
    axes[1].set_xlabel("Time [s]")
    for ax in axes:
        ax.axvline(report["lock_time_s"], linestyle="--", color="green")
        ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(OUT/"contact_result.png", dpi=160)
    plt.close(fig)
    if args.video:
        seq.render(model, records, phase_list, True,
                   work_camera=seq.camera([0, .3, .22], 2.8, 85, -23),
                   wide_camera=seq.camera([0, .25, -.02], 6.5, 75, -24), detail_azimuth=90,
                   detail_elevation=-12,
                   base_prelocked=args.module == "hexframe",
                   footer=("默认 HexFrame · 导向与止挡接触 · 理想锁定后卸力释放" if args.module == "hexframe"
                           else "原花冠凸分解接触 · 持续就位后理想锁定 · 卸力释放与撤离"))
    paths = [seq.HERE/"crown_assembly.py", seq.HERE/"assembly_sequence.py", seq.HERE/"vertical_assembly.py",
             seq.HERE/"contact_force_study/run.py", seq.HERE/"default_scene.json",
             seq.ASSETS/"vertical_assembly.xml", seq.ROOT/"uv.lock", OUT/"model.xml", OUT/"phases.json",
             seq.ROOT/"assets/iiwa14/iiwa14_dock.urdf", *CROWN.glob("*.obj")]
    paths += list((seq.ROOT/"assets/iiwa14/assets").glob("*.obj"))
    if args.module == "hexframe":
        paths += [seq.HERE/"hexframe_integration.py", * (seq.HERE/"assets/hexframe_module").rglob("*.obj"),
                  seq.HERE/"assets/hexframe_module/model_info.json",
                  seq.HERE/"assets/hexframe_module/mjcf/assets.xml",
                  seq.HERE/"assets/hexframe_module/mjcf/a_contents.xml"]
    (OUT/"source_manifest.json").write_text(json.dumps({str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}, indent=2))


def default_module():
    setting = json.loads((seq.HERE/"default_scene.json").read_text())["module"]
    if setting not in {"hexframe", "legacy"}:
        raise ValueError(f"Unsupported default module: {setting}")
    return setting


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--out", help="New result directory; existing results are preserved")
    parser.add_argument("--replay", action="store_true")
    parser.add_argument("--module", choices=["hexframe", "legacy"], default=default_module())
    args = parser.parse_args()
    if args.module == "hexframe":
        if args.quick:
            parser.error("HexFrame uses its full trajectory; --quick requires --module legacy")
        from compliant_docking.assembly.runner import run
        from compliant_docking.scene import load_scene
        raise SystemExit(run(load_scene(seq.ROOT/"scenes/hexframe_assembly.yaml"),
                             output=args.out, record=args.video, replay=args.replay))
    else:
        configure()
        run_scenario(args)


if __name__ == "__main__":
    main()
