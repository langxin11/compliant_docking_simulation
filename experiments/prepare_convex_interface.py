"""Generate optional crown collision proxies, then gate them against the CAD mesh.

CoACD is required only here. Runtime uses the generated OBJ/MJCF files.
uv run --group geometry python experiments/prepare_convex_interface.py
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import time
import xml.etree.ElementTree as ET
from importlib.metadata import version
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("OMP_NUM_THREADS", "4")

import matplotlib.pyplot as plt
import mujoco
import numpy as np
from scipy.spatial import ConvexHull

from compliant_docking.collision_geometry import (
    CONVEX_DIRECTORY,
    collision_triangles,
    connected_meshes,
    oriented_hull,
    portable_asset_fingerprints,
    sampled_solid_error,
    signed_volume,
)
from compliant_docking.interface_geometry import (
    InterfaceGeometry,
    build_interface_pair,
)
from compliant_docking.plotting import COLORS, apply_style
from compliant_docking.scene import REPO_ROOT, load_scene

PARAMETERS = dict(threshold=.01, preprocess_mode="off", resolution=10000,
                  mcts_iterations=150, mcts_nodes=20, mcts_max_depth=3, seed=42,
                  merge=True, decimate=False, extrude=False, real_metric=False)
# These are research validation tolerances, not a CAD/manufacturing certificate.
LIMITS = dict(solid_error_m=.00075, onset_height_error_m=.0002,
              phase_error_deg=.5, max_parts=200)


def write_obj(path, vertices, faces):
    rows = ["# Convex collision proxy; SI metres, original target-root coordinates."]
    rows += ["v "+" ".join(f"{x:.17g}" for x in row) for row in vertices]
    rows += ["f "+" ".join(str(int(x)+1) for x in row) for row in faces]
    path.write_text("\n".join(rows)+"\n")


def fragment(source, out, files):
    tree = ET.parse(source)
    root = tree.getroot()
    compiler = root.find("compiler")
    source_meshdir = compiler.get("meshdir", "")
    compiler.set("meshdir", ".")
    assets = root.find("asset")
    for mesh in assets.findall("mesh"):
        original = (source.parent / source_meshdir / mesh.get("file")).resolve()
        mesh.set("file", os.path.relpath(original, out.parent))
    reference = root.find('.//geom[@name="dock_geom"]')
    material = reference.get("material")
    # Preserve the full CAD mesh under the old name for independent evaluation.
    # Explicit body inertia is retained, so collision parts do not alter mass.
    reference.set("type", "mesh")
    reference.set("contype", "0")
    reference.set("conaffinity", "0")
    reference.set("group", "3")
    body = root.find('.//body[@name="rev"]')
    for index, file in enumerate(files):
        name = f"crown_part_{index:03d}"
        ET.SubElement(assets, "mesh", name=name, file=file)
        attributes = dict(name=f"convex_{index:03d}", type="mesh", mesh=name, group="3")
        if material:
            attributes["material"] = material
        ET.SubElement(body, "geom", **attributes)
    ET.indent(tree, space="  ")
    tree.write(out, encoding="unicode")


def proxy_geometry(scene):
    model = build_interface_pair(scene)
    data = mujoco.MjData(model)
    data.qpos[:] = [0, 0, 0, 1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    geometry = InterfaceGeometry(scene)
    geometry.tool = collision_triangles(model, data, scene.tool.prefix, scene.tool.prefix+"dock")
    geometry.target = collision_triangles(model, data, scene.target.prefix, scene.target.prefix+"dock")
    geometry._grids.clear()
    return geometry


def onset_scan(scene, poses):
    model = build_interface_pair(scene)
    data = mujoco.MjData(model)
    import pinocchio as pin
    rows = []
    for yaw, xy, required in poses:
        quat = pin.Quaternion(InterfaceGeometry.rotation(yaw)).coeffs()[[3, 0, 1, 2]]
        onset = None
        error = None
        for height in np.arange(required+.015, required-.00101, -.0001):
            data.qpos[:] = np.r_[xy, height, quat]
            try:
                mujoco.mj_forward(model, data)
            except mujoco.FatalError as exc:
                error = str(exc)
                break
            if min((c.dist for c in data.contact), default=0.) < -1e-6:
                onset = float(height)
                break
        rows.append(dict(yaw_deg=yaw, xy_m=list(xy), mesh_height_m=required,
                         first_penetration_height_m=onset,
                         onset_error_m=None if onset is None else onset-required,
                         height_step_m=.0001, penetration_threshold_m=1e-6, error=error))
    return rows


def validate(original, candidate, components, report_dir):
    reference = InterfaceGeometry(original)
    calibration = reference.calibrate()
    proxy = proxy_geometry(candidate)
    proxy_calibration = proxy.calibrate()
    poses = [(float(a), (0., 0.)) for a in np.arange(0., 360., 5.)]
    for c in calibration["candidates"]:
        poses.extend((float(a % 360), (x, y))
                     for a in np.arange(c["yaw_deg"]-2, c["yaw_deg"]+2.01, .25)
                     for x in [-.002, 0., .002] for y in [-.002, 0., .002])
    rows = []
    for yaw, xy in poses:
        R = reference.rotation(yaw)
        true_h = reference.required_height(R, xy, spacing=.0005)
        proxy_h = proxy.required_height(R, xy, spacing=.0005)
        rows.append(dict(yaw_deg=yaw, xy_m=list(xy), mesh_height_m=true_h,
                         proxy_height_m=proxy_h, error_m=proxy_h-true_h))
    # Refine every coarse violation, plus the 20 largest differences, at 0.25 mm.
    selected = set(np.argsort([abs(r["error_m"]) for r in rows])[-20:].tolist())
    selected.update(i for i, r in enumerate(rows) if abs(r["error_m"]) > LIMITS["onset_height_error_m"])
    for i in selected:
        r = rows[i]
        R = reference.rotation(r["yaw_deg"])
        r["mesh_height_m"] = reference.required_height(R, r["xy_m"], spacing=.00025)
        r["proxy_height_m"] = proxy.required_height(R, r["xy_m"], spacing=.00025)
        r["error_m"] = r["proxy_height_m"]-r["mesh_height_m"]
        r["refined_grid_m"] = .00025
    phase_errors = []
    for c in calibration["candidates"]:
        distance = min(abs((c["yaw_deg"]-p["yaw_deg"]+180) % 360-180)
                       for p in proxy_calibration["candidates"])
        phase_errors.append(float(distance))
    native_poses = [(a, xy, reference.required_height(reference.rotation(a), xy, spacing=.00025))
                    for a in [0., 45., *[c["yaw_deg"] for c in calibration["candidates"]]]
                    for xy in [(0., 0.), (.002, -.002)]]
    native = onset_scan(candidate, native_poses)
    sdf = onset_scan(original, native_poses)
    count = sum(c["parts"] for c in components)
    excess = max(c["sampled_solid_error"]["sampled_excess_m"] for c in components)
    missing = max(c["sampled_solid_error"]["sampled_missing_m"] for c in components)
    height_error = max(abs(r["error_m"]) for r in rows)
    native_error = max((abs(r["onset_error_m"]) for r in native if r["onset_error_m"] is not None), default=float("inf"))
    reasons = []
    for condition, reason in [
        (count > LIMITS["max_parts"], "too many convex parts"),
        (max(excess, missing) > LIMITS["solid_error_m"], "sampled solid error"),
        (height_error > LIMITS["onset_height_error_m"], "sampled approach height"),
        (max(phase_errors) > LIMITS["phase_error_deg"], "compact reference phase"),
        (native_error > LIMITS["onset_height_error_m"] or any(r["error"] or r["onset_error_m"] is None for r in native), "native convex collision onset"),
    ]:
        if condition:
            reasons.append(reason)
    validation = dict(status="FAIL" if reasons else "PASS", reasons=reasons,
                      limits=LIMITS, convex_parts=count, sampled_excess_m=excess,
                      sampled_missing_m=missing, max_height_error_m=height_error,
                      max_phase_error_deg=max(phase_errors), max_native_onset_error_m=native_error,
                      poses_checked=len(rows), original_calibration=calibration,
                      proxy_calibration=proxy_calibration, heights=rows,
                      native_onsets=native, sdf_onsets=sdf, continuous_certificate=False,
                      limitations=["sampled geometry; no manufacturing or locking certificate",
                                   "solid samples test each connected component independently",
                                   "height grid excludes tilt; dynamic tilt evaluated by the original mesh reference"])
    report_dir.mkdir(parents=True, exist_ok=True)
    (report_dir / "geometry_validation.json").write_text(json.dumps(validation, indent=2)+"\n")
    plot_validation(report_dir, validation)
    render_parts(candidate, report_dir)
    return validation


def plot_validation(out, validation):
    apply_style("report", cjk_first=True)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4), constrained_layout=True)
    for key, label, color in [("original_calibration", "原始网格", COLORS["text"]),
                              ("proxy_calibration", "凸碰撞模型", COLORS["planned"])]:
        c = validation[key]
        axes[0].plot(c["coarse_yaw_deg"], np.asarray(c["coarse_height_m"])*1000, label=label, color=color)
    axes[0].set(xlabel="相对绕轴角 [°]", ylabel="无重叠根间高度 [mm]", title="同轴配合几何")
    axes[0].legend(frameon=False)
    rows = validation["heights"]
    axes[1].scatter([r["yaw_deg"] for r in rows], [1000*r["error_m"] for r in rows],
                    s=8, color=COLORS["planned"], alpha=.45)
    limit = 1000*LIMITS["onset_height_error_m"]
    axes[1].axhline(limit, color="#888888", linestyle="--")
    axes[1].axhline(-limit, color="#888888", linestyle="--")
    axes[1].set(xlabel="相对绕轴角 [°]", ylabel="凸模型 − 原始网格 [mm]",
                title="配合相位附近的 XY 偏移对照")
    for ax in axes:
        ax.grid(alpha=.7)
    for suffix in ["png", "pdf"]:
        fig.savefig(out / f"geometry_validation.{suffix}", dpi=180)
    plt.close(fig)


def render_parts(candidate, out):
    """Use the same isolated model/cameras for CAD and collision proxies."""
    model = build_interface_pair(candidate, presentation=True)
    from compliant_docking.rendering import apply_render_theme
    apply_render_theme(model)
    data = mujoco.MjData(model)
    data.qpos[:] = [0, 0, .085, 0, 1, 0, 0]
    mujoco.mj_forward(model, data)
    apply_style("report", cjk_first=True)
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.4))
    fig.subplots_adjust(left=.01, right=.99, bottom=.09, top=.88, wspace=.03)
    palette = ["#4477AA", "#66CCEE", "#228833", "#CCBB44", "#EE6677", "#AA3377"]
    from matplotlib.colors import to_rgba
    for index in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index) or ""
        if name.endswith("dock_geom"):
            model.geom_group[index] = 4
        elif "convex_" in name:
            model.geom_rgba[index] = to_rgba(palette[int(name.rsplit("_", 1)[1]) % len(palette)])
        elif name.endswith("_visual"):
            model.geom_rgba[index] = to_rgba("#D4A65A" if name.startswith(candidate.tool.prefix) else "#7197B0")
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, .045]
    camera.distance = .30
    camera.azimuth = 135
    camera.elevation = -22
    model.vis.global_.offheight = 640
    model.vis.global_.offwidth = 640
    with mujoco.Renderer(model, height=640, width=640) as renderer:
        for index, axis in enumerate(axes):
            option = mujoco.MjvOption()
            option.geomgroup[:] = False
            option.geomgroup[2 if index == 0 else 3] = True
            option.sitegroup[:] = False
            renderer.update_scene(data, camera=camera, scene_option=option)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            axis.imshow(renderer.render())
            axis.set_title("原始视觉网格" if index == 0 else "凸碰撞分块（仅显示代理）")
            axis.axis("off")
    fig.text(.03, .025, "相同位姿与视角；安装角保持原值。分块颜色仅用于检查，运行时显示原始网格。", fontsize=10)
    for suffix in ["png", "pdf"]:
        fig.savefig(out / f"collision_parts.{suffix}", dpi=180)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default="scenes/iiwa14_compliant_insertion.yaml")
    parser.add_argument("--threshold", type=float, default=.01, help="CoACD normalized concavity; validated independently in metres")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--report", type=Path, default=REPO_ROOT / "runs/convex_geometry_20261002")
    args = parser.parse_args(argv)
    source_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    scene = load_scene(args.scene)
    if scene.tool.mjcf.name != "male_cone.xml" or scene.target.mjcf.name != "female_socket.xml":
        parser.error("This generator is scoped to the existing crown fragments")
    out = CONVEX_DIRECTORY
    out.mkdir(parents=True, exist_ok=True)
    manifest_path = out / "manifest.json"
    source_assets = portable_asset_fingerprints(scene)
    if args.validate_only:
        manifest = json.loads(manifest_path.read_text())
        if manifest["source_assets"] != source_assets or manifest["generated_assets"] != portable_asset_fingerprints(candidate_scene(scene)):
            raise ValueError("Assets changed; regenerate before validation")
    else:
        import coacd
        coacd.set_log_level("warn")
        params = dict(PARAMETERS, threshold=args.threshold)
        geometry = InterfaceGeometry(scene)
        components = []
        files = []
        for index, (vertices, faces) in enumerate(connected_meshes(geometry.target)):
            triangles = vertices[faces]
            volume = signed_volume(triangles)
            if volume <= 0:
                raise ValueError(f"Component {index}: expected positive oriented volume")
            hull = ConvexHull(vertices)
            concavity = hull.volume/volume-1
            start = time.monotonic()
            print(f"Component {index}: decomposing...", flush=True)
            if concavity < 1e-6:
                parts = [oriented_hull(vertices)]
            else:
                parts = coacd.run_coacd(coacd.Mesh(vertices, faces), **params)
                parts = [oriented_hull(v) for v, _ in parts]
            error = sampled_solid_error(triangles, parts)
            print(f"Component {index}: {len(parts)} parts; sampled error "
                  f"{1000*max(error['sampled_excess_m'], error['sampled_missing_m']):.3f} mm", flush=True)
            components.append(dict(component=index, source_volume_m3=volume,
                                   hull_extra_volume_fraction=concavity, parts=len(parts),
                                   sampled_solid_error=error, generation_seconds=time.monotonic()-start))
            for part, (v, f) in enumerate(parts):
                file = f"component_{index:02d}_part_{part:03d}.obj"
                write_obj(out / file, v, f)
                files.append(file)
        fragment(scene.tool.mjcf, out / "male.xml", files)
        fragment(scene.target.mjcf, out / "female.xml", files)
        manifest = dict(generator=str(Path(__file__).relative_to(REPO_ROOT)),
                        generator_sha256=source_sha256, source_capture="start of generation",
                        packages={p: version(p) for p in ["coacd", "mujoco", "numpy", "scipy"]},
                        parameters=params, source_assets=source_assets, components=components,
                        coordinates="SI metres in the source target rev frame; original male 40 degree mount retained",
                        generated_assets=portable_asset_fingerprints(candidate_scene(scene)))
    candidate = candidate_scene(scene)
    manifest["validation"] = validate(scene, candidate, manifest["components"], args.report)
    manifest["validation_source_sha256"] = source_sha256
    manifest["validation_helper_sha256"] = hashlib.sha256(
        (REPO_ROOT / "src/compliant_docking/collision_geometry.py").read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest, indent=2)+"\n")
    print(json.dumps({k: v for k, v in manifest["validation"].items()
                      if k in ["status", "reasons", "convex_parts", "max_height_error_m", "max_phase_error_deg", "sampled_excess_m", "sampled_missing_m"]}, indent=2))
    return 0 if manifest["validation"]["status"] == "PASS" else 2


def candidate_scene(scene):
    from dataclasses import replace
    return replace(scene, tool=replace(scene.tool, mjcf=CONVEX_DIRECTORY / "male.xml"),
                   target=replace(scene.target, mjcf=CONVEX_DIRECTORY / "female.xml"))


if __name__ == "__main__":
    raise SystemExit(main())
