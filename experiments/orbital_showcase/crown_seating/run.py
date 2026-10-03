"""Six-DOF original-crown seating experiment; no robot or position teleporting."""
from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
ASSETS = ROOT / "assets/interfaces/convex_crown"
REFERENCE = ROOT / "runs/convex_contact_20261002/geometry_reference.json"
HEIGHT = .0471874034950606
PHASE = 4.25


def gate(lateral, gap, angle, speed, spin, force, penetration):
    """Research thresholds, not hardware latch tolerances."""
    return bool(lateral <= .0005 and abs(gap) <= .00075 and angle <= .5
                and speed <= .0005 and spin <= np.deg2rad(.5)
                and .1 <= force <= 10 and penetration <= .0003)


def model_xml(dt):
    root = ET.Element("mujoco", model="original_crown_pair")
    ET.SubElement(root, "option", timestep=str(dt), gravity="0 0 0",
                  integrator="implicitfast", iterations="100", cone="elliptic")
    asset = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    carrier = ET.SubElement(world, "body", name="carrier")
    ET.SubElement(carrier, "freejoint")
    ET.SubElement(carrier, "inertial", mass="2", pos="0 0 0", diaginertia=".012 .012 .012")
    for source, prefix, parent in [("male.xml", "mobile_", carrier), ("female.xml", "fixed_", world)]:
        fragment = ET.parse(ASSETS / source).getroot()
        for element in fragment.iter():
            for key in ("name", "mesh", "material"):
                if key in element.attrib:
                    element.set(key, prefix + element.get(key))
            if element.tag == "mesh" and element.get("file"):
                element.set("file", str((ASSETS / element.get("file")).resolve()))
            if element.tag == "geom":
                element.set("mass", "0")
                element.set("friction", ".2 .001 .0001")
                element.set("solref", ".008 1")
                element.set("solimp", ".95 .95 .001")
        for body in fragment.iter("body"):
            for inertia in list(body.findall("inertial")):
                body.remove(inertia)
        asset.extend(fragment.find("asset"))
        parent.append(fragment.find("worldbody/body"))
    equality = ET.SubElement(root, "equality")
    ET.SubElement(equality, "weld", name="latch", body1="fixed_dock", body2="carrier",
                  active="false", solref=".01 1")
    return ET.tostring(root, encoding="unicode")


def run(name, xy=0., yaw=0., dt=.001, stiff=False, out=HERE / "outputs"):
    folder = out / name
    folder.mkdir(parents=True, exist_ok=True)
    xml = model_xml(dt)
    (folder / "model.xml").write_text(xml)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    desired = Rotation.from_euler("z", PHASE, degrees=True).as_matrix() @ np.diag([1., -1., -1.])
    initial = Rotation.from_euler("z", yaw, degrees=True).as_matrix() @ desired
    data.qpos[:3] = [xy, -xy, HEIGHT + .006]
    data.qpos[3:] = Rotation.from_matrix(initial).as_quat(scalar_first=True)
    body = model.body("carrier").id
    mobile = {i for i in range(model.ngeom) if model.geom(i).name.startswith("mobile_")}
    rows, poses, hold, locked, lock_pose = [], [], 0., None, None
    control_stride = round(.001 / dt)
    applied = np.zeros(6)
    duration = 12.
    for step in range(round(duration / dt)):
        mujoco.mj_step1(model, data)
        t = float(data.time)
        p, R = data.qpos[:3].copy(), data.xmat[body].reshape(3, 3).copy()
        v, omega = data.qvel[:3].copy(), R @ data.qvel[3:]
        # Fixed erroneous lateral/yaw command tests passive accommodation, not
        # target-truth correction. Axial spring descends 1 mm/s then preloads.
        reference = np.array([xy, -xy, max(HEIGHT - .001, HEIGHT + .006 - .001*t)])
        error = Rotation.from_matrix(initial @ R.T).as_rotvec()
        force = np.array([100., 100., 1000.]) * (reference-p) - np.array([25., 25., 90.])*v
        torque = np.array([2., 2., 2. if stiff else 0.]) * error - .15*omega
        if step % control_stride == 0:
            applied = np.r_[force, torque]
        data.xfrc_applied[body] = applied if locked is None else np.zeros(6)
        mujoco.mj_step2(model, data)
        contact_force, penetration = np.zeros(3), 0.
        for i in range(data.ncon):
            c = data.contact[i]
            if (c.geom1 in mobile) == (c.geom2 in mobile):
                continue
            wrench = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, wrench)
            contact_force += (1 if c.geom2 in mobile else -1) * c.frame.reshape(3, 3).T @ wrench[:3]
            penetration = max(penetration, -float(c.dist))
        angle = np.rad2deg(np.linalg.norm(Rotation.from_matrix(desired @ R.T).as_rotvec()))
        values = (np.linalg.norm(p[:2]), p[2]-HEIGHT, angle,
                  np.linalg.norm(v), np.linalg.norm(omega), np.linalg.norm(contact_force), penetration)
        eligible = gate(*values)
        hold = hold + dt if eligible else 0.
        rows.append([t, *p, *values, int(eligible), int(locked is not None)])
        poses.append(np.r_[p, Rotation.from_matrix(R).as_quat(scalar_first=True)])
        if locked is None and hold >= .5:
            # Preserve the accepted physical pose; no snapping to nominal CAD.
            model.eq_data[0, 3:6] = data.qpos[:3]
            model.eq_data[0, 6:10] = data.qpos[3:]
            data.eq_active[0] = True
            locked, lock_pose = t+dt, data.qpos.copy()
    a = np.asarray(rows)
    np.savez_compressed(folder / "trace.npz", values=a, qpos=np.asarray(poses),
                       columns=np.array(["time", "x", "y", "z", "lateral", "gap", "angle_deg",
                                         "speed", "spin", "force", "penetration", "eligible", "locked"]))
    tail = a[a[:, 0] >= duration-1]
    result = dict(name=name, timestep=dt, control_period_s=.001, xy_each_axis_m=xy, initial_yaw_error_deg=yaw,
                  stiff_yaw=stiff, lock_time_s=locked, peak_force_n=float(a[:, 9].max()),
                  max_penetration_mm=float(a[:, 10].max()*1000),
                  final_lateral_mm=float(tail[:, 4].max()*1000),
                  final_abs_gap_mm=float(np.abs(tail[:, 5]).max()*1000),
                  final_angle_deg=float(tail[:, 6].max()),
                  warnings=int(sum(w.number for w in data.warning)),
                  latch_position_drift_m=None if lock_pose is None else float(np.linalg.norm(data.qpos[:3]-lock_pose[:3])),
                  status="REJECTED" if locked is None else "LATCHED_IDEAL_CONSTRAINT",
                  physical_latch_verified=False)
    if locked is not None and (result["warnings"] or result["latch_position_drift_m"] > 1e-5
                               or result["peak_force_n"] > 10
                               or result["max_penetration_mm"] > .3
                               or result["final_lateral_mm"] > .5
                               or result["final_abs_gap_mm"] > .75
                               or result["final_angle_deg"] > .5):
        result["status"] = "POST_LOCK_VALIDATION_FAILED"
    (folder / "metrics.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result), flush=True)
    return result, a


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()
    out = HERE / "outputs"
    # Verify that the existing sampled geometric reference still matches CAD.
    ref = json.loads(REFERENCE.read_text())
    for path, digest in ref["assets"].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise RuntimeError(f"Stale geometry reference: {path}")
    cases = [("nominal", {}), ("xy1mm_yaw2deg", dict(xy=.001, yaw=2.)),
             ("xy1mm_yaw2deg_stiff", dict(xy=.001, yaw=2., stiff=True)),
             ("xy1mm_yaw2deg_halfstep", dict(xy=.001, yaw=2., dt=.0005)),
             ("xy1mm_yaw2deg_stiff_halfstep", dict(xy=.001, yaw=2., dt=.0005, stiff=True))]
    if args.quick:
        cases = cases[:1]
    results = []
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for name, kwargs in cases:
        result, a = run(name, **kwargs)
        results.append(result)
        for ax, col, scale in zip(axes, [9, 5, 6], [1, 1000, 1], strict=True):
            ax.plot(a[:, 0], a[:, col]*scale, label=name, linewidth=1)
    for ax, label in zip(axes, ["Contact force [N]", "Seating gap [mm]", "Orientation error [deg]"], strict=True):
        ax.set_ylabel(label)
        ax.grid(alpha=.25)
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(out / "comparison.png", dpi=160)
    plt.close(fig)
    (out / "summary.json").write_text(json.dumps(results, indent=2)+"\n")
    sources = [Path(__file__), REFERENCE, *ASSETS.glob("*.xml"), *ASSETS.glob("*.obj"),
               ROOT / "assets/iiwa14/assets/dock_1_17_new.STL"]
    manifest = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    (out / "source_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    (out / "runtime.json").write_text(json.dumps(dict(mujoco=mujoco.__version__, numpy=np.__version__), indent=2)+"\n")


if __name__ == "__main__":
    main()
