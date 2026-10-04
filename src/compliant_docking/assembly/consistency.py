"""Same-source arm/tool and nominal carried-payload model verification.

The carried model is a rigid nominal grasp for model checking only. The physical
run retains module1's free joint and solves the switchable weld in MuJoCo.
"""
from __future__ import annotations

import copy
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from compliant_docking.models import load_pin_model

from .geometry import quaternion, vector


def physical_xml(model):
    root = ET.Element("mujoco", model="exact_assembly_dynamics")
    ET.SubElement(root, "compiler", angle="radian")
    ET.SubElement(root, "option", gravity=vector(model.opt.gravity), timestep=str(model.opt.timestep))
    elements = {0: ET.SubElement(root, "worldbody")}
    for i in range(1, model.nbody):
        body = ET.SubElement(elements[int(model.body_parentid[i])], "body", name=model.body(i).name,
                             pos=vector(model.body_pos[i]), quat=vector(model.body_quat[i]))
        elements[i] = body
        if model.body_mass[i] > 0:
            ET.SubElement(body, "inertial", mass=str(model.body_mass[i]),
                          pos=vector(model.body_ipos[i]), quat=vector(model.body_iquat[i]),
                          diaginertia=vector(model.body_inertia[i]))
        for j in np.flatnonzero(model.jnt_bodyid == i):
            kind = {int(mujoco.mjtJoint.mjJNT_HINGE): "hinge", int(mujoco.mjtJoint.mjJNT_SLIDE): "slide"}[int(model.jnt_type[j])]
            ET.SubElement(body, "joint", name=model.joint(j).name, type=kind,
                          pos=vector(model.jnt_pos[j]), axis=vector(model.jnt_axis[j]),
                          limited="true" if model.jnt_limited[j] else "false", range=vector(model.jnt_range[j]))
        for j in np.flatnonzero(model.site_bodyid == i):
            ET.SubElement(body, "site", name=model.site(j).name, pos=vector(model.site_pos[j]),
                          quat=vector(model.site_quat[j]))
    return root


def prepare_models(r, samples=12):
    source = ET.parse(r.output / "model.xml").getroot()
    result = {}
    for carried in [False, True]:
        root = copy.deepcopy(source)
        mobile = root.find("worldbody/body[@name='module1']")
        for parent in root.iter():
            for child in list(parent):
                if child.get("name") in {"module1", "module2"} or child.tag in {"equality", "actuator", "sensor"}:
                    parent.remove(child)
        if carried:
            mobile.remove(mobile.find("freejoint"))
            mobile.set("pos", vector([0, 0, r.resource.apothem + r.resource.gap]))
            mobile.set("quat", vector(quaternion(r.tip_rotation.T @ r.module_rotation)))
            root.find(".//body[@name='gripper']").append(mobile)
        name = "carried" if carried else "arm_tool"
        path = r.output / f"{name}_model.xml"
        # Expand physical fields at full precision. This avoids MJCF importer
        # limitations with inherited default classes and MjSpec XML rounding.
        mj = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        root = physical_xml(mj)
        path.write_text(ET.tostring(root, encoding="unicode"))
        pm = load_pin_model(path)
        # The local MJCF importer omits cumulative fixed-body placement for
        # nested site frames. Reconstruct sites from body frames and the source
        # local transform; body inertias and joint transforms stay parser-owned.
        for body in root.iter("body"):
            if not pm.existFrame(body.get("name", "")):
                continue
            parent_frame = pm.frames[pm.getFrameId(body.get("name"))]
            for site in body.findall("site"):
                if not pm.existFrame(site.get("name", "")):
                    continue
                local_pos = np.fromstring(site.get("pos", "0 0 0"), sep=" ")
                local_rot = Rotation.from_quat(np.fromstring(site.get("quat", "1 0 0 0"), sep=" "), scalar_first=True).as_matrix()
                frame = pm.frames[pm.getFrameId(site.get("name"))]
                frame.placement = parent_frame.placement * pin.SE3(local_rot, local_pos)
                frame.parentJoint = parent_frame.parentJoint
        assert pm.nq == mj.nq == pm.nv == mj.nv == 7
        md, pd = mujoco.MjData(mj), pm.createData()
        frames = ["gripper_tip"] + (["module1_port_4_frame"] if carried else [])
        maximum = {key: 0. for key in ["position", "rotation", "jacobian", "mass_matrix", "bias"]}
        rng = np.random.default_rng(315)
        for _ in range(samples):
            q = rng.uniform(-.8, .8, 7)
            v = rng.uniform(-.1, .1, 7)
            md.qpos[:], md.qvel[:] = q, v
            mujoco.mj_forward(mj, md)
            pin.forwardKinematics(pm, pd, q, v)
            pin.updateFramePlacements(pm, pd)
            pin.computeJointJacobians(pm, pd, q)
            for frame in frames:
                site = md.site(frame)
                pose = pd.oMf[pm.getFrameId(frame)]
                maximum["position"] = max(maximum["position"], float(np.max(abs(site.xpos - pose.translation))))
                maximum["rotation"] = max(maximum["rotation"], float(np.max(abs(site.xmat.reshape(3, 3) - pose.rotation))))
                jac = np.zeros((6, 7))
                mujoco.mj_jacSite(mj, md, jac[:3], jac[3:], site.id)
                pj = pin.getFrameJacobian(pm, pd, pm.getFrameId(frame), pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                maximum["jacobian"] = max(maximum["jacobian"], float(np.max(abs(jac - pj))))
            mm = np.zeros((7, 7))
            try:
                mujoco.mj_fullM(mj, md, mm)
            except TypeError:  # MuJoCo < 3.12
                mujoco.mj_fullM(mj, mm, md.qM)
            maximum["mass_matrix"] = max(maximum["mass_matrix"], float(np.max(abs(mm - pin.crba(pm, pd, q)))))
            maximum["bias"] = max(maximum["bias"], float(np.max(abs(md.qfrc_bias - pin.nonLinearEffects(pm, pd, q, v)))))
        for key, value in maximum.items():
            if value > 1e-8:
                raise RuntimeError(f"{name}: {key} MuJoCo/Pinocchio mismatch {value}")
        pinmass = float(sum(i.mass for i in pm.inertias[1:]))
        # Physical moving subtrees include fixed gripper and optional payload.
        moving_mass = float(mj.body_subtreemass[mj.body("link1").id])
        assert abs(moving_mass-pinmass) < 1e-8, (moving_mass, pinmass)
        if carried:
            body = mj.body("module1")
            assert abs(body.mass[0]-r.resource.info["body_mass_kg"]) < 1e-10
        else:
            r.pin_model = pm
        result[name] = dict(status="PASS", samples=samples, maximum_absolute_errors=maximum,
                            control_frames=frames, moving_mass_kg=pinmass,
                            gripper_mass_kg=float(mj.body("gripper").mass[0]),
                            module_mass_kg=r.resource.info["body_mass_kg"] if carried else 0.)
    result["scope"] = "Same-source seven-joint rigid arm/tool and nominal rigid grasp; not a free-floating spacecraft model or runtime weld-force equivalence"
    (r.output / "model_consistency.json").write_text(json.dumps(result, indent=2)+"\n")
    return result
