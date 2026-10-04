"""Rigid six-side-port modules and an isolated horizontal contact experiment.

Frames: M is the natural upright hexagonal prism frame; F is the flange frame.
Port 1 is on -M.x, port 4 on +M.x. F.z points along +M.x. The robot holds
port 1, while port 4 is the controlled contact frame. No top/bottom ports exist.
"""
from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, replace

import mujoco
import numpy as np
import pinocchio as pin
import yaml
from build_assets import ASSETS, HERE, ROOT, beam, geom, palette, vector, write_tree

from compliant_docking.config import DockingConfig, SE3ImpedanceConfig
from compliant_docking.docking_task import evaluate_docking
from compliant_docking.models import load_pin_model
from compliant_docking.planning.kinematics import compute_ik
from compliant_docking.planning.waypoints import WaypointPoseTrajectory
from compliant_docking.scene import RobotSpec, Scene, TargetSpec, TaskSpec, ToolSpec, load_scene

RADIUS = .115
HALF_HEIGHT = .080
APOTHEM = RADIUS*np.cos(np.pi/6)
MASS = 2.0
# F.x=M.y, F.y=M.z, F.z=M.x; both frames are right-handed.
R_F_M = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
# In world coordinates the modules stand upright and mate along world +Y.
R_W_F = np.array([[-1., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
R_W_TARGET = R_W_F @ np.diag([1., -1., -1.])
TARGET_POS = np.array([0., 1.00, .60])
I_PLANAR = MASS*(5*RADIUS**2/24+(2*HALF_HEIGHT)**2/12)
I_AXIAL = MASS*5*RADIUS**2/12
INERTIA_F = np.array([I_PLANAR, I_AXIAL, I_PLANAR])


def quaternion(rotation):
    q = pin.Quaternion(rotation)
    return np.array([q.w, q.x, q.y, q.z])


def port_pose(number):
    theta = np.pi+(number-1)*np.pi/3
    normal = np.array([np.cos(theta), np.sin(theta), 0.])
    tangent = np.array([-np.sin(theta), np.cos(theta), 0.])
    return pin.SE3(np.column_stack([tangent, [0., 0., 1.], normal]), APOTHEM*normal)


def label_port(parent, number):
    # A compact raised numeral above the port, made from original line geometry.
    strokes = {1: [((0, 0), (0, 1))],
               2: [((0, 1), (1, 1)), ((1, 1), (1, .5)), ((1, .5), (0, .5)),
                   ((0, .5), (0, 0)), ((0, 0), (1, 0))],
               3: [((0, 1), (1, 1)), ((0, .5), (1, .5)), ((0, 0), (1, 0)), ((1, 0), (1, 1))],
               4: [((0, 1), (0, .5)), ((0, .5), (1, .5)), ((1, 0), (1, 1))],
               5: [((1, 1), (0, 1)), ((0, 1), (0, .5)), ((0, .5), (1, .5)),
                   ((1, .5), (1, 0)), ((1, 0), (0, 0))],
               6: [((1, 1), (0, 1)), ((0, 1), (0, 0)), ((0, 0), (1, 0)),
                   ((1, 0), (1, .5)), ((1, .5), (0, .5))]}
    for a, b in strokes[number]:
        beam(parent, [(a[0]-.5)*.009, .060+a[1]*.014, .004],
             [(b[0]-.5)*.009, .060+b[1]*.014, .004], .001, "orb_dark")


def make_module(name, moving):
    """Aggregate inertia and all six ports belong to one rigid module body."""
    root = ET.Element("mujoco", model=name)
    ET.SubElement(root, "compiler", angle="degree", meshdir=str(ROOT/"assets/iiwa14/assets"))
    asset = ET.SubElement(root, "asset")
    palette(asset)
    ET.SubElement(asset, "mesh", name="crown_mesh", file="dock_1_17_new.STL")
    world = ET.SubElement(root, "worldbody")
    dock = ET.SubElement(world, "body", name="dock")
    if moving:
        ET.SubElement(dock, "site", name="sensor_site", size=".001")
        center = np.array([0., 0., APOTHEM])
        rotation = R_F_M
    else:
        center = np.array([0., 0., -APOTHEM])
        rotation = R_F_M @ np.diag([-1., -1., 1.])
    module = ET.SubElement(dock, "body", name="module", pos=vector(center),
                           quat=vector(quaternion(rotation)))
    # Natural-frame inertia is symmetric about the upright prism axis.
    ET.SubElement(module, "inertial", mass=str(MASS), pos="0 0 0",
                  diaginertia=vector([I_PLANAR, I_PLANAR, I_AXIAL]))
    vertices = np.column_stack([np.cos(np.arange(6)*np.pi/3+5*np.pi/6),
                               np.sin(np.arange(6)*np.pi/3+5*np.pi/6), np.zeros(6)])*RADIUS
    upper, lower = vertices+[0, 0, HALF_HEIGHT], vertices-[0, 0, HALF_HEIGHT]
    for i in range(6):
        j = (i+1) % 6
        for a, b in [(upper[i], upper[j]), (lower[i], lower[j]), (upper[i], lower[i])]:
            g = geom(module, "capsule", fromto=[*a, *b], size=[.0035])
            # Structural rails use actual capsule collision proxies.
            g.set("contype", "1")
            g.set("conaffinity", "1")
        pose = port_pose(i+1)
        body = ET.SubElement(module, "body", name=f"port_{i+1}", pos=vector(pose.translation),
                             quat=vector(quaternion(pose.rotation)))
        ET.SubElement(body, "site", name=f"port_{i+1}_frame", size=".001")
        label_port(body, i+1)
        # Four spokes transfer load from the interface rim into each side frame.
        for x, y in [(-.050, -.068), (.050, -.068), (-.050, .068), (.050, .068)]:
            length = np.hypot(x, y)
            beam(body, [x, y, 0], [x*.048/length, y*.048/length, 0], .0025)
        active = (moving and i+1 == 4) or (not moving and i+1 == 1)
        rev = ET.SubElement(body, "body", name=f"port_{i+1}_crown",
                            euler="0 0 40" if moving and i+1 == 4 else "0 0 0")
        geom(rev, "mesh", name=f"port_{i+1}_visual", mesh="crown_mesh", material="orb_blue")
        if active:
            ET.SubElement(rev, "geom", name=f"port_{i+1}_contact", type="sdf", mesh="crown_mesh",
                          rgba="0 0 0 0", mass="0", group="3")
    # Keep the shared scene builder's tracking-camera anchor available.
    ET.SubElement(dock, "body", name="rev", pos=vector([0, 0, 2*APOTHEM] if moving else [0, 0, 0]))
    write_tree(ET.ElementTree(root), f"{name}.xml")


def build_side_assets():
    make_module("side_tool", True)
    make_module("side_target", False)
    tree = ET.parse(ROOT/"assets/iiwa14/iiwa14_dock.urdf")
    robot = tree.getroot()
    # Existing cylinder_link becomes port 4; the whole module inertia is expressed
    # relative to that frame. The flange->port1 adapter is the existing 15 mm.
    robot.find("joint[@name='cylinder_joint']/origin").set("xyz", vector([0, 0, .060+2*APOTHEM]))
    tool = robot.find("link[@name='cylinder_link']")
    for child in list(tool):
        if child.tag != "inertial":
            tool.remove(child)
    inertia = tool.find("inertial")
    inertia.find("origin").set("xyz", vector([0, 0, -APOTHEM]))
    inertia.find("mass").set("value", str(MASS))
    tensor = inertia.find("inertia")
    tensor.attrib.update(ixx=str(INERTIA_F[0]), iyy=str(INERTIA_F[1]), izz=str(INERTIA_F[2]),
                         ixy="0", ixz="0", iyz="0")
    ET.indent(tree)
    tree.write(ASSETS/"side_arm.urdf", encoding="unicode")


@dataclass(frozen=True)
class SideScene(Scene):
    @property
    def eef_body(self):
        return "tool_port_4"


def make_scene():
    base = load_scene(ROOT/"scenes/iiwa14_compliant_insertion.yaml")
    kwargs = {field: getattr(base, field) for field in base.__dataclass_fields__}
    spec = replace(base.docking, estimate_pos=tuple(TARGET_POS.tolist()), estimate_yaw_deg=0.)
    kwargs.update(name="orbital_side_docking", path=HERE/"scene.yaml",
                  robot=RobotSpec(ASSETS/"arm.xml", ASSETS/"side_arm.urdf", "attachment_site", "cylinder_link"),
                  tool=ToolSpec(ASSETS/"side_tool.xml", "tool_", np.array([0., 0., .015]), np.array([1., 0., 0., 0.])),
                  target=TargetSpec(ASSETS/"side_target.xml", "target_", TARGET_POS, quaternion(R_W_TARGET)),
                  task=TaskSpec(np.array([-.09, .76, .63]), R_W_F,
                                np.array([.83567448, .26176348, 1.11070619, -1.80861519,
                                          .70674746, -.55862481, .69454423]), np.array([0., .04, 0.])),
                  docking=spec,
                  se3_impedance=replace(base.se3_impedance,
                                        d_diag=[120., 120., 160., 18., 18., 20.]))
    scene = SideScene(**kwargs)
    config = dict(scene={"name": scene.name, "runner": "side_docking.py", "controller": "se3_lie"},
                  target={"contact_port": 1, "pos": TARGET_POS.tolist(), "quat": quaternion(R_W_TARGET).tolist()},
                  module1={"robot_port": 1, "docking_port": 4, "mass_kg": MASS,
                           "radius_m": RADIUS, "height_m": 2*HALF_HEIGHT,
                           "flange_to_port1_m": .015, "port1_to_port4_m": float(2*APOTHEM)},
                  module2={"docking_port": 1, "fixed": True},
                  task={"init_pos": scene.task.init_pos.tolist(),
                        "init_ori": scene.task.init_ori.tolist(),
                        "ik_guess": scene.task.ik_guess.tolist()},
                  se3_impedance=asdict(scene.se3_impedance),
                  docking=asdict(spec),
                  assets={"robot": "assets/arm.xml", "pin_model": "assets/side_arm.urdf",
                          "module1": "assets/side_tool.xml", "module2": "assets/side_target.xml"})
    (HERE/"scene.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
    return scene


def trajectory(scene):
    spec = scene.docking
    normal = R_W_TARGET[:, 2]
    target = np.asarray(spec.estimate_pos)
    rotation = R_W_TARGET @ np.diag([1., -1., -1.])
    entry = target+normal*spec.entry_height
    clearance = entry+normal*spec.clearance
    final = entry-normal*spec.command_depth
    poses = [pin.SE3(scene.task.init_ori, scene.task.init_pos)]
    poses += [pin.SE3(rotation, p) for p in [clearance, entry, final]]
    limits = scene.trajectory
    return WaypointPoseTrajectory(poses, ["start", "approach", "align", "insert"],
        [(limits.v_max_approach, limits.a_max_approach, .2, .2),
         (limits.v_max_docking, limits.a_max_docking, .05, .05),
         (spec.insertion_speed, spec.insertion_acceleration, .05, .05)])


def validate(scene):
    model, pin_model = scene.build_mjmodel(), load_pin_model(scene.robot.pin_model)
    data, pd = mujoco.MjData(model), pin_model.createData()
    path = trajectory(scene)
    q = scene.task.ik_guess.copy()
    matrix_error = pose_error = 0.
    poses = []
    for t in np.linspace(0, path.total_duration, 50):
        q, ok = compute_ik(pin_model, pd, path.get_pose(t), initial_q=q, max_iters=1000,
                           ee_frame=scene.robot.ee_frame)
        if not ok:
            raise ValueError(f"Side-docking trajectory is unreachable at t={t:.3f}")
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        pin.forwardKinematics(pin_model, pd, q)
        pin.updateFramePlacements(pin_model, pd)
        h = pd.oMf[pin_model.getFrameId(scene.robot.ee_frame)]
        body = data.body(scene.eef_body)
        pose_error = max(pose_error, float(np.max(abs(h.translation-body.xpos))),
                         float(np.max(abs(h.rotation-body.xmat.reshape(3, 3)))))
        mj_mass = np.zeros((model.nv, model.nv))
        mujoco.mj_fullM(model, data, mj_mass)
        matrix_error = max(matrix_error, float(np.max(abs(mj_mass-pin.crba(pin_model, pd, q)))))
        for prefix in ["tool_", "target_"]:
            module = data.body(prefix+"module")
            R = module.xmat.reshape(3, 3)
            for number in range(1, 7):
                p = data.body(prefix+f"port_{number}")
                local = R.T@(p.xpos-module.xpos)
                np.testing.assert_allclose(local, port_pose(number).translation, atol=1e-12)
                assert abs(local[2]) < 1e-12  # Every port lies on a side face.
        port1 = data.body("tool_port_1")
        attach = data.site("attachment_site")
        np.testing.assert_allclose(port1.xpos, attach.xpos+attach.xmat.reshape(3, 3)@[0, 0, .015], atol=1e-12)
        np.testing.assert_allclose(data.body("target_port_1").xmat.reshape(3, 3), R_W_TARGET, atol=1e-12)
        poses.append(q.copy())
    np.testing.assert_allclose(matrix_error, 0., atol=1e-8)
    np.testing.assert_allclose(pose_error, 0., atol=1e-8)
    return dict(status="PASS", mass_matrix_max_error=matrix_error, fk_max_error=pose_error,
                reachable_path_samples=len(poses), module1_mass_kg=MASS,
                all_ports_on_side_faces=True, rigid_chain="arm -> M1:P1 -> M1:P4 -> M2:P1"), poses[0]


def simulate(scene):
    from compliant_docking.cli import _load_run_docking
    from compliant_docking.control.se3_impedance import SE3LieImpedanceController
    from compliant_docking.simulation.mujoco_env import MujRobot
    from compliant_docking.telemetry import Log

    pin_model = load_pin_model(scene.robot.pin_model)
    model = scene.build_mjmodel()
    robot = MujRobot(model, render=False, record=False, eef_body=scene.eef_body,
                     target_pos=scene.target.pos)
    gains = scene.se3_impedance
    config = SE3ImpedanceConfig(A_diag=np.array(gains.a_diag), D_diag=np.array(gains.d_diag),
                              K_diag=np.array(gains.k_diag), null_damping=gains.null_damping)
    controller = SE3LieImpedanceController(pin_model, .001, config, ee_frame=scene.robot.ee_frame)
    path = trajectory(scene)
    q, ok = compute_ik(pin_model, pin_model.createData(), path.get_pose(0),
                       initial_q=scene.task.ik_guess, ee_frame=scene.robot.ee_frame)
    assert ok
    cfg = replace(DockingConfig(), duration=path.total_duration+scene.docking.hold_s)
    log = Log()
    _load_run_docking().run_simulation(robot, controller, path, log, cfg=cfg, q_init=q,
                                     scene=scene, r_des=scene.task.init_ori, plot=False)
    log.docking_gate = evaluate_docking(log, scene.docking, path, cfg.dt)
    log.docking_trajectory = path
    return log


def geometry_manifest(scene, out):
    record = dict(chain="arm -> module1 port1 -> module1 port4 -> module2 port1",
                  module_axes="M.z: prism axis; all side ports have M.z=0",
                  port_frames={str(i): port_pose(i).homogeneous.tolist() for i in range(1, 7)},
                  flange_to_port4_m=float(.015+2*APOTHEM),
                  target_port_world=pin.SE3(R_W_TARGET, TARGET_POS).homogeneous.tolist(),
                  tool_mass_kg=MASS, tool_inertia_diagonal_F=INERTIA_F.tolist())
    (out/"geometry_manifest.json").write_text(json.dumps(record, indent=2)+"\n")
