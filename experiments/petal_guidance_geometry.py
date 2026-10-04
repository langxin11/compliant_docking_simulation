"""Independent native first-contact and contact-direction probes for guide shapes."""
from __future__ import annotations

import copy
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from compliant_docking.contact_diagnostics import contact_wrench
from compliant_docking.scene import REPO_ROOT

DIRECTORIES = {"original": REPO_ROOT/"assets/interfaces/petal_dock100",
               "narrow": REPO_ROOT/"assets/interfaces/petal_guidance/narrow",
               "radial": REPO_ROOT/"assets/interfaces/petal_guidance/radial"}


def pair_model(directory):
    """Exactly the generated convex geoms; free active body, fixed passive."""
    fragment = ET.parse(Path(directory)/"active.xml").getroot()
    root = ET.Element("mujoco", model="guide_pair_probe")
    ET.SubElement(root,"compiler",angle="radian")
    ET.SubElement(root,"size",memory="128M")  # dense radial cells at the overlap bracket
    ET.SubElement(root,"option",timestep="0.00025",gravity="0 0 0",integrator="implicitfast",
                  cone="elliptic",iterations="60",tolerance="1e-10")
    for tag in ("default","asset"):
        root.append(copy.deepcopy(fragment.find(tag)))
    world = ET.SubElement(root,"worldbody")
    for prefix in ("passive","active"):
        body = copy.deepcopy(fragment.find("worldbody/body"))
        body.set("name",prefix)
        for child in body:
            if child.get("name"):
                child.set("name",prefix+"_"+child.get("name"))
        if prefix == "active":
            ET.SubElement(body,"freejoint",name="active_free")
            body.set("pos","0 0 .07")
        world.append(body)
    assets = {mesh.get("file"):(Path(directory)/mesh.get("file")).read_bytes()
              for mesh in root.findall("./asset/mesh")}
    model = mujoco.MjModel.from_xml_string(ET.tostring(root,encoding="unicode"),assets=assets)
    assert model.nq == 7 and model.neq == 0
    return model


def orientation(yaw_deg):
    return Rotation.from_euler("z",45.+yaw_deg,degrees=True).as_matrix() @ np.diag([1.,-1.,-1.])


def position(model,data,xy,z,R):
    data.qpos[:3] = [*xy,z]
    q = Rotation.from_matrix(R).as_quat()
    data.qpos[3:] = q[[3,0,1,2]]
    data.qvel[:] = 0.
    mujoco.mj_fwdPosition(model,data)


def first_contact(model,data,xy,R):
    low,high = .0460,.0710
    position(model,data,xy,low,R)
    assert data.ncon and min(data.contact.dist) < 0., "Lower bracket has no overlap"
    position(model,data,xy,high,R)
    assert data.ncon == 0, "Upper bracket has contact"
    while high-low > 5e-8:
        middle = (low+high)/2
        position(model,data,xy,middle,R)
        if data.ncon and min(data.contact.dist) < 0.:
            low = middle
        else:
            high = middle
    return (low+high)/2


def directions(model,data,xy,R,height,friction):
    """Instantaneous free-body solver probe, not a robot rollout or hold gate."""
    model.geom_friction[:,0] = friction
    position(model,data,xy,height-2e-6,R)
    data.qfrc_applied[:] = [0.,0.,-10.,0.,0.,0.]
    mujoco.mj_forward(model,data)
    origin,normal,moment,wrench,angles = data.body("active").xpos.copy(),[],[],[],[]
    for i,contact in enumerate(data.contact):
        name = model.geom(contact.geom2).name or ""
        on2 = name.startswith("active_")
        n = contact.frame[:3]*(1. if on2 else -1.)
        normal.append(n)
        moment.append(np.cross(contact.pos-origin,n))
        angles.append(float(np.rad2deg(np.arccos(np.clip(abs(n[2]),0.,1.)))))
        local = np.zeros(6)
        mujoco.mj_contactForce(model,data,i,local)
        wrench.append(contact_wrench(contact,local,on_geom2=on2,origin=origin))
    combined = np.sum(wrench,axis=0) if wrench else np.zeros(6)
    assert np.isfinite(combined).all()
    return dict(friction=friction, applied_axial_N=10.,penetration_below_first_touch_m=2e-6,
        contacts=data.ncon,normal_on_active=normal,normal_moment_arm_m=moment,
        effective_slope_deg=angles,interface_wrench_world=combined.tolist(),
        interpretation="instantaneous free-body solver probe; not observed robotic capture")


def validate(directory,points):
    import prepare_petal_guidance as generate
    info = json.loads((Path(directory)/"model_info.json").read_text())
    model,data = pair_model(directory),None
    data = mujoco.MjData(model)
    nominal = first_contact(model,data,(0.,0.),orientation(0.))
    assert abs(nominal-info["nominal_flange_separation_m"]) < 1e-6
    result = dict(status="PASS",nominal_first_touch_m=nominal,
        assets_manifest_sha256=hashlib.sha256((Path(directory)/"manifest.json").read_bytes()).hexdigest(),
        validator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        nominal_separation_m=info["nominal_flange_separation_m"],
        unchanged_stop_reference=True,free_body_dofs=model.nv,equality_constraints=model.neq,
        native_geom_count=model.ngeom,geometry_checks={},continuous_capture_verified=False)
    if info["parameters"]["guide_profile"] == "rounded_linear":
        p = info["parameters"]
        r = np.linspace(32.,50.,71)[:,None]
        a = np.linspace(-180.,180.,1401)[None,:]
        h = generate.surface_height(r,a,p)
        paired = h+generate.surface_height(r,45.-a,p)
        error = float(abs(paired-p["guide_height_mm"]).max())
        assert error < 1e-10
        assert h.min() >= -1e-10 and h.max() <= 18.+1e-10
        result["analytic_pair_height_error_mm"] = error
        result["surface_height_range_mm"] = [float(h.min()),float(h.max())]
    for case,(x,y,yaw) in points.items():
        xy,R = np.array([x,y])/1000.,orientation(yaw)
        height = first_contact(model,data,xy,R)
        samples = []
        for axis in range(2):
            perturbation = np.zeros(2)
            perturbation[axis] = .0005
            samples.append((first_contact(model,data,xy+perturbation,R)
                            -first_contact(model,data,xy-perturbation,R))/.001)
        da = np.deg2rad(1.)
        slope = (first_contact(model,data,xy,orientation(yaw+1.))
                 -first_contact(model,data,xy,orientation(yaw-1.)))/(2*da)
        probes = [directions(model,data,xy,R,height,mu) for mu in (0.,.15,.3)]
        gradient = np.asarray(samples)
        result["geometry_checks"][case] = dict(error=dict(x_mm=x,y_mm=y,yaw_deg=yaw),
            first_touch_gap_mm=1000*(height-info["nominal_flange_separation_m"]),
            height_gradient_xy=gradient.tolist(),height_gradient_yaw_m_rad=float(slope),
            axial_potential_lateral_dot_error=float(-gradient@xy),
            axial_potential_yaw_times_error=float(-slope*np.deg2rad(yaw)),solver_probes=probes)
    return result
