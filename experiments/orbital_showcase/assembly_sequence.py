"""Dynamic pick-transfer-install demonstration with pose-gated ideal latches.

The module has a free joint throughout the run. Only forces and switchable weld
constraints move it after initialization. Crown micro-contact and latch mechanisms
are deliberately abstracted; this does not replace the compliant contact experiment.
"""
# ruff: noqa: E402
from __future__ import annotations

import argparse
import contextlib
import copy
import hashlib
import io
import json
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT/"src"))

import mujoco
import numpy as np
import pinocchio as pin
from build_assets import ASSETS, beam, build, geom, vector, write_tree
from scipy.interpolate import CubicSpline
from side_docking import APOTHEM, R_F_M, R_W_F, make_module, quaternion

from compliant_docking.models import load_pin_model
from compliant_docking.planning.kinematics import compute_ik

OUT = HERE/"outputs/sequence"
R_MODULE = R_W_F @ R_F_M
PICK = np.array([-.36, .80, .60])
SEED = np.array([.08, 1.10, .60])
# The engagement offset is a nominal seated interface separation, not a
# contact-mechanics result. It keeps the inherited crown visuals intermeshed.
ENGAGEMENT = .050
PICK_TIP = PICK - np.array([0., APOTHEM+ENGAGEMENT, 0.])
INSTALLED = SEED - np.array([0., 2*APOTHEM+ENGAGEMENT, 0.])
INSTALL_TIP = INSTALLED - np.array([0., APOTHEM+ENGAGEMENT, 0.])
DT = .001
TORQUE_LIMITS = np.array([320., 320., 176., 176., 110., 40., 40.])
KP = np.array([600., 600., 500., 500., 180., 140., 80.])
KD = np.array([70., 70., 55., 55., 20., 16., 9.])
LOCK_NAMES = ["storage_lock", "gripper_lock", "assembly_lock"]
LOCK_SITES = [("storage_anchor", "module1_anchor"),
              ("gripper_anchor", "module1_anchor"),
              ("assembly_anchor", "module1_anchor")]


@dataclass
class Phase:
    key: str
    title: str
    seconds: float
    end: np.ndarray
    event: str | None = None


def phases():
    ready = PICK_TIP - [0., .17, 0.]
    above_pick = PICK_TIP + [0., 0., .12]
    pre_install = INSTALL_TIP - [0., .12, 0.]
    return [
        Phase("stowed", "01  存放：模块 1 锁在料架，模块 2 固定于基座", 1.5, ready),
        Phase("approach", "02  机械臂末端接口接近模块 1 的接口 1", 4., PICK_TIP-[0., .04, 0.]),
        Phase("capture", "03  对准接口 1，低速进入捕获位置", 4., PICK_TIP),
        Phase("grip_check", "04  锁定机械臂接口，并确认连接", 1.5, PICK_TIP, "grip_on"),
        Phase("rack_release", "05  抓取确认后，解除模块 1 的存放锁", 1., PICK_TIP, "rack_off"),
        Phase("lift", "06  抬升模块 1，离开存放架", 3., above_pick),
        Phase("transfer", "07  机械臂带模块 1 转运到组装位", 5., pre_install+[0., 0., .12]),
        Phase("align", "08  下降并对准：模块 1 接口 4 → 模块 2 接口 1", 3., pre_install),
        Phase("mate", "09  沿接口法线低速接近安装位置", 6., INSTALL_TIP),
        Phase("assembly_check", "10  锁定模块间连接，并确认安装", 2., INSTALL_TIP, "assembly_on"),
        Phase("gripper_release", "11  安装确认后，解除机械臂接口锁", 1., INSTALL_TIP, "grip_off"),
        Phase("retreat", "12  机械臂撤离，模块 1 留在模块 2 上", 3., INSTALL_TIP-[0., .18, 0.]),
        Phase("complete", "完成：模块 1 与模块 2 组成基座上的装配体", 2., INSTALL_TIP-[0., .18, 0.]),
    ]


def tray(world, center, name):
    x, y, _ = center
    body = ET.SubElement(world, "body", name=name)
    top = geom(body, "box", pos=[x, y, .500], size=[.15, .15, .016], material="orb_dark")
    top.set("contype", "1")
    top.set("conaffinity", "1")
    for dx in [-.135, .135]:
        for dy in [-.135, .135]:
            beam(body, [x+dx, y+dy, .02], [x+dx, y+dy, .484], .008)
        beam(body, [x+dx, y-.135, .025], [x+dx, y+.135, .475], .004)
    geom(body, "box", pos=[x, y, .023], size=[.18, .18, .018], material="orb_metal")
    for dx in [-.125, .125]:
        for dy in [-.10, .10]:
            geom(body, "box", pos=[x+dx, y+dy, .532], size=[.012, .018, .016], material="orb_gold")
    geom(body, "sphere", name=name+"_led", pos=[x, y-.153, .50], size=[.013], material="orb_cyan")


def build_model(filename="assembly_sequence.xml", base_filename="sequence_arm_base.xml"):
    base = build(with_pedestal=False, filename=base_filename)
    tree = ET.parse(base)
    root = tree.getroot()
    root.set("model", "orbital_pick_transfer_install")
    root.remove(root.find("keyframe"))
    asset, world = root.find("asset"), root.find("worldbody")
    asset.find("material[@name='orb_white']").set("rgba", ".68 .75 .82 1")
    ET.SubElement(asset, "mesh", name="crown_mesh", file="dock_1_17_new.STL")
    ET.SubElement(world, "light", pos="0 0 3", dir="0 0 -1", directional="true")
    # Shared workcell with an ordered row of storage fixtures and a seed station.
    deck = ET.SubElement(world, "body", name="workcell")
    floor = geom(deck, "box", pos=[-.25, .72, .019], size=[.70, .64, .011], material="orb_metal")
    floor.set("contype", "1")
    floor.set("conaffinity", "1")
    for x in [-.84, -.50, -.16, .18]:
        beam(deck, [x, .22, .033], [x, 1.26, .033], .0025, "orb_dark")
    for y in [.25, .59, .93, 1.27]:
        beam(deck, [-.89, y, .033], [.32, y, .033], .0025, "orb_dark")
    tray(world, PICK, "storage")
    tray(world, PICK+[-.36, 0, 0], "spare_slot")
    tray(world, SEED, "seed_station")
    # End-effector hardware stays on the arm after release.
    link7 = root.find(".//body[@name='link7']")
    eff = ET.SubElement(link7, "body", name="gripper", pos="0 0 .060")
    ET.SubElement(eff, "inertial", pos="0 0 0", mass=".25", diaginertia=".0003 .0003 .0005")
    geom(eff, "cylinder", pos=[0, 0, -.012], size=[.040, .012], material="orb_dark")
    crown = ET.SubElement(eff, "body", name="gripper_crown", euler="0 0 40")
    geom(crown, "mesh", mesh="crown_mesh", material="orb_blue")
    geom(eff, "sphere", name="gripper_led", pos=[0, -.045, -.014], size=[.009], material="orb_gold")
    ET.SubElement(eff, "site", name="gripper_tip", size=".001")
    ET.SubElement(eff, "site", name="gripper_anchor", pos=vector([0, 0, APOTHEM+ENGAGEMENT]),
                  quat=vector(quaternion(R_F_M)), size=".001")
    # Both modules use the same upright frame and side-port numbering.
    make_module("sequence_module_template", True)
    template = ET.parse(ASSETS/"sequence_module_template.xml").find("worldbody/body/body[@name='module']")
    for name, center, free in [("module1", PICK, True), ("module2", SEED, False)]:
        module = copy.deepcopy(template)
        for element in module.iter():
            if "name" in element.attrib:
                element.set("name", name+"_"+element.get("name"))
        module.set("name", name)
        module.set("pos", vector(center))
        module.set("quat", vector(quaternion(R_MODULE)))
        for parent in module.iter():
            for child in list(parent):
                if child.tag == "geom" and child.get("type") == "sdf":
                    parent.remove(child)
        if free:
            ET.SubElement(module, "freejoint", name="module1_free")
            ET.SubElement(module, "site", name="module1_anchor", size=".001")
        else:
            ET.SubElement(module, "site", name="assembly_anchor", size=".001",
                          pos=vector([-2*APOTHEM-ENGAGEMENT, 0, 0]))
        world.append(module)
    ET.SubElement(world, "site", name="storage_anchor", pos=vector(PICK),
                  quat=vector(quaternion(R_MODULE)), size=".001")
    eq = ET.SubElement(root, "equality")
    for i, (name, sites) in enumerate(zip(LOCK_NAMES, LOCK_SITES, strict=True)):
        ET.SubElement(eq, "weld", name=name, site1=sites[0], site2=sites[1],
                      active="true" if i == 0 else "false", solref=".006 1", solimp=".99 .99 .001")
    for i, motor in enumerate(root.find("actuator")):
        # Native velocity bias lets implicitfast integrate servo damping stably.
        motor.set("biastype", "affine")
        motor.set("biasprm", vector([0, -KP[i], -KD[i]]))
        motor.set("forcelimited", "true")
        motor.set("forcerange", vector([-TORQUE_LIMITS[i], TORQUE_LIMITS[i]]))
    write_tree(tree, filename)
    return mujoco.MjModel.from_xml_path(str(ASSETS/filename))


def plan(model, initial_q=None, phase_list=None):
    pm = load_pin_model(ROOT/"assets/iiwa14/iiwa14_dock.urdf")
    pd = pm.createData()
    q = np.array([.83567448, .26176348, 1.11070619, -1.80861519,
                  .70674746, -.55862481, .69454423])
    if initial_q is not None:
        q = np.asarray(initial_q).copy()
    data = mujoco.MjData(model)
    if phase_list is None:
        phase_list = phases()
    start = phase_list[0].end.copy()
    qs, ts, phase_ids = [], [], []
    time = 0.
    max_fk = 0.
    with contextlib.redirect_stdout(io.StringIO()):
        for k, phase in enumerate(phase_list):
            for tau in np.linspace(0., phase.seconds, max(2, int(phase.seconds*30)+1)):
                if ts and tau == 0:
                    continue
                u = tau/phase.seconds
                s = 10*u**3 - 15*u**4 + 6*u**5
                p = start + s*(phase.end-start)
                q, ok = compute_ik(pm, pd, pin.SE3(R_W_F, p), initial_q=q, max_iters=1000)
                if not ok:
                    raise ValueError(f"Unreachable trajectory: {phase.key}, {tau}")
                data.qpos[:7] = q
                mujoco.mj_forward(model, data)
                tip = data.site("gripper_tip")
                max_fk = max(max_fk, float(np.linalg.norm(tip.xpos-p)))
                qs.append(q.copy())
                ts.append(time+tau)
                phase_ids.append(k)
            start = phase.end.copy()
            time += phase.seconds
    assert max_fk < 1e-6
    qs, ts = np.array(qs), np.array(ts)
    spline = CubicSpline(ts, qs, bc_type=((1, np.zeros(7)), (1, np.zeros(7))))
    return spline, phase_list, dict(ik_samples=len(ts), fk_max_error_m=max_fk,
                                  joint_limits=[pm.lowerPositionLimit.tolist(), pm.upperPositionLimit.tolist()])


def lock_error(model, data, index):
    a, b = [data.site(name) for name in LOCK_SITES[index]]
    dp = float(np.linalg.norm(a.xpos-b.xpos))
    dr = float(np.linalg.norm(pin.log3(a.xmat.reshape(3, 3)@b.xmat.reshape(3, 3).T)))
    jacobians = []
    for site in [a, b]:
        j = np.zeros((6, model.nv))
        mujoco.mj_jacSite(model, data, j[:3], j[3:], site.id)
        jacobians.append(j)
    relative = (jacobians[0]-jacobians[1])@data.qvel
    return dp, dr, float(np.linalg.norm(relative[:3])), float(np.linalg.norm(relative[3:]))


def handover(model, data, event):
    """Enable the receiving latch before releasing the current support."""
    if event not in {"grip_on", "rack_off", "assembly_on", "grip_off"}:
        raise ValueError(event)
    index = 1 if event.startswith("grip") else 0 if event == "rack_off" else 2
    if event.endswith("on"):
        error = lock_error(model, data, index)
        if not (error[0] < .001 and error[1] < np.deg2rad(.5)
                and error[2] < .003 and error[3] < np.deg2rad(2.)):
            raise RuntimeError(f"Latch gate failed: {event}: {error}")
        data.eq_active[index] = True
    else:
        support = 1 if event == "rack_off" else 2
        error = lock_error(model, data, support)
        if not data.eq_active[support] or error[0] > .001 or error[1] > np.deg2rad(.5):
            raise RuntimeError(f"Cannot release {event}: receiving latch unconfirmed")
        data.eq_active[index] = False
    assert np.any(data.eq_active)
    return error


def verify_interlocks(model, initial_q):
    """Reject remote capture and both unsafe release operations."""
    data = mujoco.MjData(model)
    data.qpos[:7] = initial_q
    mujoco.mj_forward(model, data)
    rejected = []
    for event in ["grip_on", "rack_off", "grip_off"]:
        before = data.eq_active.copy()
        try:
            handover(model, data, event)
        except RuntimeError:
            np.testing.assert_array_equal(data.eq_active, before)
            rejected.append(event)
        else:
            raise AssertionError(f"Unsafe event accepted: {event}")
    return rejected


def simulate(model, spline, phase_list, planning):
    data = mujoco.MjData(model)
    data.qpos[:7] = spline(0)
    mujoco.mj_forward(model, data)
    interlocks = verify_interlocks(model, spline(0))
    ends = np.cumsum([p.seconds for p in phase_list])
    starts = np.r_[0., ends[:-1]]
    events, done = [], set()
    records = {key: [] for key in ["t", "qpos", "qvel", "phase", "locks"]}
    saturation = 0
    max_tracking = max_weld = 0.
    min_contact = 0.
    contacts = set()
    low, high = np.asarray(planning["joint_limits"])
    joint_limit_violation = False
    for step in range(int(round(ends[-1]/DT))+1):
        t = step*DT
        k = min(int(np.searchsorted(ends, t, side="right")), len(phase_list)-1)
        phase = phase_list[k]
        # Dwell before evaluating each handover. Never rewrite qpos at an event.
        if phase.event and phase.event not in done and t-starts[k] >= phase.seconds*.6:
            event = phase.event
            error = handover(model, data, event)
            events.append(dict(t=t, event=event, error_m=error[0], error_deg=float(np.rad2deg(error[1])),
                               active_locks=data.eq_active.tolist()))
            done.add(event)
            print(events[-1], flush=True)
        desired, velocity = spline(t), spline(t, 1)
        torque = KP*(desired-data.qpos[:7])+KD*(velocity-data.qvel[:7])+data.qfrc_bias[:7]
        saturation += int(np.any(abs(torque) > TORQUE_LIMITS))
        data.ctrl[:] = KP*desired+KD*velocity+data.qfrc_bias[:7]
        max_tracking = max(max_tracking, float(np.max(abs(desired-data.qpos[:7]))))
        joint_limit_violation |= bool(np.any(data.qpos[:7] < low) or np.any(data.qpos[:7] > high))
        for i, active in enumerate(data.eq_active):
            if active:
                # Position residual is enough for a compact continuous audit;
                # full pose and velocity gates are applied at each latch event.
                a, b = [data.site(name) for name in LOCK_SITES[i]]
                max_weld = max(max_weld, float(np.linalg.norm(a.xpos-b.xpos)))
        for contact in data.contact[:data.ncon]:
            min_contact = min(min_contact, float(contact.dist))
            pair = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g])
                    for g in [contact.geom1, contact.geom2]]
            contacts.add(tuple(sorted(pair)))
        if step % 10 == 0:
            for key, value in [("t", t), ("qpos", data.qpos.copy()), ("qvel", data.qvel.copy()),
                               ("phase", k), ("locks", data.eq_active.copy())]:
                records[key].append(value)
        assert np.isfinite(data.qpos).all()
        if step < int(round(ends[-1]/DT)):
            mujoco.mj_step(model, data)
    final_error = float(np.linalg.norm(data.body("module1").xpos-INSTALLED))
    reasons = []
    if len(events) != 4 or list(data.eq_active) != [False, False, True]:
        reasons.append("incomplete latch handover")
    if final_error > .001:
        reasons.append("installed module position error")
    if joint_limit_violation:
        reasons.append("joint limit")
    if contacts:
        reasons.append("unplanned structural contact")
    if saturation:
        reasons.append("torque saturation")
    report = dict(status="FAIL" if reasons else "PASS", reasons=reasons,
                  scope="dynamic rigid-body transfer with ideal pose-gated latches; crown contact disabled",
                  collision_coverage="robot proxies, module rails, fixture tops, workcell floor; trim visual only",
                  duration_s=float(ends[-1]), planning=planning, events=events,
                  rejected_unsafe_events=interlocks,
                  module_free_joint=True, qpos_changes_after_initialization="MuJoCo integration only",
                  max_joint_tracking_error_rad=max_tracking, max_latch_position_error_m=max_weld,
                  final_module_position_error_m=final_error, torque_saturated_steps=saturation,
                  structural_contact_pairs=sorted(contacts), deepest_contact_m=min_contact,
                  nominal_crown_engagement_m=ENGAGEMENT)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT/"validation.json").write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n")
    (OUT/"phases.json").write_text(json.dumps([
        dict(key=p.key, title=p.title, start=float(starts[i]), end=float(ends[i]), event=p.event)
        for i, p in enumerate(phase_list)], indent=2, ensure_ascii=False)+"\n")
    np.savez_compressed(OUT/"rollout.npz", **{key: np.array(value) for key, value in records.items()})
    print(json.dumps(report, ensure_ascii=False), flush=True)
    return {key: np.array(value) for key, value in records.items()}, report


def camera(lookat, distance, azimuth=135, elevation=-27):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    return cam


def render(model, records, phase_list, video, *, work_camera=None, wide_camera=None,
           detail_azimuth=175, detail_elevation=-18,
           base_prelocked=False,
           footer="刚体动力学流程演示 · 理想锁定约束 · 花冠微接触未模拟"):
    from compliant_docking.assembly.rendering import render as render_assembly
    return render_assembly(model, records, phase_list, video, output=OUT,
                           work_camera=work_camera, wide_camera=wide_camera,
                           detail_azimuth=detail_azimuth, detail_elevation=detail_elevation,
                           base_prelocked=base_prelocked, footer=footer)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--replay", action="store_true")
    parser.add_argument("--no-render", action="store_true")
    args = parser.parse_args()
    model = build_model()
    if args.replay:
        with np.load(OUT/"rollout.npz") as saved:
            records = {key: saved[key] for key in saved.files}
        report = json.loads((OUT/"validation.json").read_text())
        phase_list = phases()
    else:
        OUT.mkdir(parents=True, exist_ok=True)
        paths = [Path(__file__), HERE/"build_assets.py", HERE/"side_docking.py",
                 ROOT/"src/compliant_docking/planning/kinematics.py",
                 ROOT/"assets/iiwa14/iiwa14_dock.urdf", ASSETS/"assembly_sequence.xml"]
        (OUT/"source_manifest.json").write_text(json.dumps({str(path.relative_to(ROOT)):
            hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}, indent=2)+"\n")
        spline, phase_list, planning = plan(model)
        records, report = simulate(model, spline, phase_list, planning)
    if not args.no_render:
        render(model, records, phase_list, args.video)
        print(OUT/"storyboard.png", flush=True)
    if report["status"] != "PASS":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
