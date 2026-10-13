"""Generate an original, simplified four-petal interface; CAD in mm, MJCF in m.

No HOTDOCK CAD is copied. Collision cells come directly from the surface
construction, so generic approximate convex decomposition is unnecessary.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cadquery as cq
import numpy as np
from scipy.spatial import ConvexHull
import trimesh

ROOT = Path(__file__).resolve().parent


def fmt(values):
    return " ".join(f"{float(x):.10g}" for x in np.atleast_1d(values))


def write_xml(root, path):
    ET.indent(root, space="  ")
    path.write_text(ET.tostring(root, encoding="unicode") + "\n", encoding="utf-8")


def cyl(radius, z0, height):
    return cq.Workplane("XY").workplane(offset=z0).circle(radius).extrude(height).val()


def hole(radius, x, y, z0, height):
    return cyl(radius, z0, height).translate((x, y, 0))


def angular_point(radius, deg, z):
    a = math.radians(deg)
    return [radius * math.cos(a), radius * math.sin(a), z]


def wave_height(angle, p):
    d = abs((angle + 45) % 90 - 45)
    a = p["guide_tip_half_angle_deg"]
    t = np.clip((45 - a - d) / (45 - 2 * a), 0, 1)
    return p["guide_height_mm"] * t**3 * (10 + t * (-15 + 6 * t))


def convex_mesh(points):
    points = np.unique(np.asarray(points, dtype=float), axis=0)
    hull = ConvexHull(points)
    mesh = trimesh.Trimesh(points, hull.simplices, process=True)
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals()
    if mesh.volume <= 1e-8 or not mesh.is_watertight:
        raise ValueError("Invalid convex collision cell")
    return mesh


def radial_profile(p, samples=5):
    """Circular shoulders at the peak; scaled vertically with local guide height."""
    ri, ro = p["guide_inner_diameter_mm"] / 2, p["outer_diameter_mm"] / 2
    edge, height = p["guide_edge_round_mm"], p["guide_height_mm"]
    t = np.linspace(0, np.pi / 2, samples)
    radii = np.r_[ri + edge * (1 - np.cos(t)), ro - edge + edge * np.sin(t)]
    factor = np.r_[1 - edge / height * (1 - np.sin(t)),
                   1 - edge / height * (1 - np.cos(t))]
    return radii, factor


def petal_cells(center, p):
    z0 = p["adapter_thickness_mm"] + p["head_thickness_mm"]
    step = 360 / p["guide_segments"]
    half = 45 - p["guide_tip_half_angle_deg"]
    angles = np.linspace(center - half, center + half, int(round(2 * half / step)) + 1)
    radii, factor = radial_profile(p)
    cells = []
    for a0, a1 in zip(angles[:-1], angles[1:]):
        points = []
        for a in [a0, a1]:
            h = float(wave_height(a, p))
            points.extend(angular_point(r, a, z0 + h * q) for r, q in zip(radii, factor))
            points.extend(angular_point(r, a, z0) for r in [radii[0], radii[-1]])
        cells.append(convex_mesh(points))
    return cells


def petal_visual_mesh(center, p):
    """Closed mesh sampled from the smooth design, with no buried overlap faces."""
    zb = p["adapter_thickness_mm"] + p["head_thickness_mm"]
    half = 45 - p["guide_tip_half_angle_deg"]
    angles = np.linspace(center - half, center + half, 209)
    radii, factor = radial_profile(p, samples=13)
    n = len(radii)
    vertices = [angular_point(r, a, zb + float(wave_height(a, p)) * q)
                for a in angles for r, q in zip(radii, factor)]
    offset = len(vertices)
    vertices += [angular_point(r, a, zb) for a in angles for r in radii]
    faces = []
    for i in range(len(angles) - 1):
        for j in range(n - 1):
            k = i*n+j
            faces.extend([[k,k+1,k+n+1],[k,k+n+1,k+n]])
            b = offset+k
            faces.extend([[b,b+n+1,b+1],[b,b+n,b+n+1]])
        k = i*n
        faces.extend([[offset+k,k,k+n],[offset+k,k+n,offset+k+n]])
        k = (i+1)*n-1
        faces.extend([[offset+k,offset+k+n,k+n],[offset+k,k+n,k]])
    mesh = trimesh.Trimesh(vertices, faces, process=True)
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals()
    if not mesh.is_watertight:
        raise ValueError('Non-watertight smooth visual petal')
    return mesh


def petal_solid(center, p):
    """Smooth B-rep loft with angular C2 target profile and rounded radial shoulders.

    Sections extend 1 mm into the head plate for a robust solid union. The
    circular R-edge shoulder is exact at the peak and scales with local height.
    The loft is a spline approximation; collisions use the analytic target.
    """
    zb = p["adapter_thickness_mm"] + p["head_thickness_mm"]
    ri, ro = p["guide_inner_diameter_mm"] / 2, p["outer_diameter_mm"] / 2
    edge, H = p["guide_edge_round_mm"], p["guide_height_mm"]
    def section(a):
        h = max(float(wave_height(a, p)), 1e-5)
        def v(r, z):
            return cq.Vector(*angular_point(r, a, z))
        edges = [cq.Edge.makeLine(v(ri, zb - 1), v(ro, zb - 1)),
                 cq.Edge.makeLine(v(ro, zb - 1), v(ro, zb + h * (1 - edge / H)))]
        edges.append(cq.Edge.makeSpline([
            v(ro - edge + edge * np.cos(t), zb + h * (1 - edge / H + edge / H * np.sin(t)))
            for t in np.linspace(0, np.pi / 2, 9)]))
        edges.append(cq.Edge.makeLine(v(ro - edge, zb + h), v(ri + edge, zb + h)))
        edges.append(cq.Edge.makeSpline([
            v(ri + edge - edge * np.sin(t), zb + h * (1 - edge / H + edge / H * np.cos(t)))
            for t in np.linspace(0, np.pi / 2, 9)]))
        edges.append(cq.Edge.makeLine(v(ri, zb + h * (1 - edge / H)), v(ri, zb - 1)))
        return cq.Wire.assembleEdges(edges)
    half = 45 - p["guide_tip_half_angle_deg"]
    angles = np.linspace(center - half, center + half, p["guide_cad_sections"])
    return cq.Solid.makeLoft([section(a) for a in angles], ruled=False)


def mesh_to_solid(mesh):
    faces = []
    for f in mesh.faces:
        poly = [cq.Vector(*v) for v in mesh.vertices[f]]
        faces.append(cq.Face.makeFromWires(cq.Wire.makePolygon(poly, close=True)))
    solid = cq.Solid.makeSolid(cq.Shell.makeShell(faces))
    if not solid.isValid():
        raise ValueError("Invalid STEP petal solid")
    return solid


def shape_to_mesh(shape):
    vertices, faces = shape.tessellate(0.025, 0.06)
    mesh = trimesh.Trimesh([[v.x, v.y, v.z] for v in vertices], faces, process=True)
    mesh.fix_normals()
    return mesh


def validate_parameters(p):
    if p["guide_profile"] != "quintic":
        raise ValueError("Only the quintic guide profile is supported")
    if not 0 < p["guide_edge_round_mm"] < min(p["guide_height_mm"], (p["outer_diameter_mm"] - p["guide_inner_diameter_mm"]) / 4):
        raise ValueError("Guide shoulder radius is outside the valid range")
    if p["guide_cad_sections"] < 25 or p["guide_cad_sections"] % 2 != 1:
        raise ValueError("guide_cad_sections must be odd and at least 25")
    if not 0 < p["stop_round_mm"] < min((p["guide_height_mm"] + p["guide_axial_clearance_mm"]) / 2,
                                       (p["stop_outer_diameter_mm"] - p["center_bore_diameter_mm"]) / 2):
        raise ValueError("Invalid stop fillet radius")
    if p["stop_collision_segments"] < 16:
        raise ValueError("At least 16 stop sectors are required")
    if p["guide_segments"] < 32 or p["guide_segments"] % 16:
        raise ValueError("guide_segments must be a multiple of 16, at least 32")
    if not 0 < p["guide_tip_half_angle_deg"] < 22.5:
        raise ValueError("guide_tip_half_angle_deg must be between 0 and 22.5")
    step = 360 / p["guide_segments"]
    if abs(p["guide_tip_half_angle_deg"] / step - round(p["guide_tip_half_angle_deg"] / step)) > 1e-8:
        raise ValueError("Align guide_tip_half_angle_deg to a guide mesh angular node")
    if not p["stop_outer_diameter_mm"] < p["guide_inner_diameter_mm"] < p["outer_diameter_mm"]:
        raise ValueError("Invalid radial sizes")
    if p["guide_axial_clearance_mm"] <= 0:
        raise ValueError("Guide clearance must be positive")
    if p["back_relief_mm"] >= p["adapter_thickness_mm"]:
        raise ValueError("Back relief must leave a positive adapter thickness")
    if p["robot_counterbore_depth_mm"] >= p["adapter_thickness_mm"]:
        raise ValueError("Robot counterbores cannot cut the adapter through")
    if p["head_counterbore_depth_mm"] >= p["head_thickness_mm"]:
        raise ValueError("Head counterbores cannot cut the head through")


def make_cad(p, out):
    ro = p["outer_diameter_mm"] / 2
    ta, th = p["adapter_thickness_mm"], p["head_thickness_mm"]
    zr, zb = p["back_relief_mm"], ta + th
    mate_z = zb + (p["guide_height_mm"] + p["guide_axial_clearance_mm"]) / 2
    adapter = cyl(p["flange_support_diameter_mm"] / 2, 0, zr)
    adapter = adapter.fuse(cyl(ro, zr, ta - zr))
    adapter = adapter.fuse(cyl(p["pilot_diameter_mm"] / 2, -p["pilot_length_mm"], p["pilot_length_mm"]))
    adapter = adapter.cut(cyl(p["center_bore_diameter_mm"] / 2, -p["pilot_length_mm"] - 1, ta + p["pilot_length_mm"] + 2))
    bolt_angles = [(p["index_angle_deg"] + k * 45) % 360 for k in range(1, 8)]
    for a in bolt_angles:
        x, y, _ = angular_point(p["robot_bolt_pcd_mm"] / 2, a, 0)
        adapter = adapter.cut(hole(p["robot_bolt_clearance_mm"] / 2, x, y, -1, ta + 2))
        adapter = adapter.cut(hole(p["robot_counterbore_diameter_mm"] / 2, x, y,
                                   ta - p["robot_counterbore_depth_mm"], p["robot_counterbore_depth_mm"] + 1))
    x, y, _ = angular_point(p["robot_bolt_pcd_mm"] / 2, p["index_angle_deg"], 0)
    adapter = adapter.cut(hole(p["index_clearance_mm"] / 2, x, y, -1, ta + 2))
    head = cyl(ro, ta, th).cut(cyl(p["center_bore_diameter_mm"] / 2, ta - 1, th + 2))
    for a in [45, 135, 225, 315]:
        x, y, _ = angular_point(p["head_bolt_pcd_mm"] / 2, a, 0)
        adapter = adapter.cut(hole(p["head_adapter_tap_drill_mm"] / 2, x, y, zr - 1, ta - zr + 2))
        head = head.cut(hole(p["head_bolt_clearance_mm"] / 2, x, y, ta - 1, th + 2))
        head = head.cut(hole(p["head_counterbore_diameter_mm"] / 2, x, y,
                             zb - p["head_counterbore_depth_mm"], p["head_counterbore_depth_mm"] + 1))
    stop = cyl(p["stop_outer_diameter_mm"] / 2, zb, mate_z - zb)
    outer_edges = [e for e in stop.Edges() if abs(e.Center().z - mate_z) < 1e-6]
    stop = stop.fillet(p["stop_round_mm"], outer_edges)
    stop = stop.cut(cyl(p["center_bore_diameter_mm"] / 2, zb - 1, mate_z - zb + 2))
    meshes, solids, collision = {}, {"adapter": adapter, "head_plate": head, "stop_land": stop}, []
    for name, solid in solids.items():
        meshes[name] = shape_to_mesh(solid)
    for k in range(4):
        solid = petal_solid(0, p) if k == 0 else solids["petal_0"].rotate((0, 0, 0), (0, 0, 1), k * 90)
        mesh, cells = petal_visual_mesh(k * 90, p), petal_cells(k * 90, p)
        name = f"petal_{k}"
        meshes[name] = mesh
        solids[name] = solid
        collision.extend(cells)
    guide_cell_count = len(collision)
    # Convex annular stop sectors preserve the center aperture at the mating face.
    for k in range(p["stop_collision_segments"]):
        a0, a1 = [360 * i / p["stop_collision_segments"] for i in [k, k + 1]]
        ri, rs = p["center_bore_diameter_mm"] / 2, p["stop_outer_diameter_mm"] / 2
        rf = p["stop_round_mm"]
        profile = [(ri, zb), (rs, zb), (rs, mate_z - rf)]
        profile += [(rs - rf + rf * np.cos(t), mate_z - rf + rf * np.sin(t))
                    for t in np.linspace(0, np.pi / 2, 7)]
        profile += [(ri, mate_z)]
        pts = [angular_point(r, a, z) for a in [a0, a1] for r, z in profile]
        collision.append(convex_mesh(pts))
    assembly = cq.Assembly(name="PetalDock100")
    colors = {"adapter": cq.Color(0.22, 0.25, 0.30), "head_plate": cq.Color(0.52, 0.57, 0.63),
              "stop_land": cq.Color(0.72, 0.76, 0.81)}
    for name, solid in solids.items():
        if not solid.isValid():
            raise ValueError(f"Invalid CAD solid: {name}")
        color = colors.get(name, cq.Color(0.66, 0.72, 0.79))
        assembly.add(solid, name=name, color=color)
        cq.exporters.export(solid, str(out / "cad" / f"{name}_mm.step"))
        cq.exporters.export(solid, str(out / "cad" / f"{name}_mm.stl"), tolerance=0.08, angularTolerance=0.15)
    docking_head = head.fuse(stop, *[solids[f"petal_{k}"] for k in range(4)], tol=1e-5).clean()
    if not docking_head.isValid() or len(docking_head.Solids()) != 1:
        raise ValueError("The docking head must be one connected CAD solid")
    cq.exporters.export(docking_head, str(out / "cad" / "docking_head_mm.step"))
    cq.exporters.export(docking_head, str(out / "cad" / "docking_head_mm.stl"), tolerance=0.08, angularTolerance=0.15)
    manufacturing_assembly = cq.Assembly(name="PetalDock100")
    manufacturing_assembly.add(adapter, name="flange_adapter", color=colors["adapter"])
    manufacturing_assembly.add(docking_head, name="docking_head", color=cq.Color(.52, .64, .76))
    manufacturing_assembly.export(str(out / "cad" / "dock_complete_mm.step"))
    compound = cq.Compound.makeCompound(list(solids.values()))
    cq.exporters.export(compound, str(out / "cad" / "dock_complete_mm.stl"), tolerance=0.08, angularTolerance=0.15)
    for name, mesh in meshes.items():
        mesh_m = mesh.copy()
        mesh_m.apply_scale(0.001)
        mesh_m.export(out / "meshes" / "visual" / f"{name}.obj", include_normals=True)
    base_proxy = convex_mesh([angular_point(ro, a, z) for a in np.linspace(0, 360, 65)[:-1]
                              for z in [zr, zb]])
    base_proxy.apply_scale(.001)
    base_proxy.export(out / "meshes" / "collision" / "base_proxy.obj")
    for i, mesh in enumerate(collision):
        mesh_m = mesh.copy()
        mesh_m.apply_scale(0.001)
        mesh_m.export(out / "meshes" / "collision" / f"cell_{i:03}.obj")
    mass_mesh = trimesh.util.concatenate([shape_to_mesh(adapter), shape_to_mesh(docking_head)])
    mass_mesh.apply_scale(0.001)
    mass_mesh.density = p["density_kg_m3"]
    properties = mass_mesh.mass_properties
    meta = {
        "name": "PetalDock100", "version": "2.0", "cad_units": "mm", "mjcf_units": "m",
        "origin": "robot flange mounting face", "approach_axis": "+Z",
        "mating_site_z_m": mate_z / 1000,
        "nominal_flange_separation_m": 2 * mate_z / 1000,
        "mating_relative_quaternion_wxyz": [0, math.cos(math.pi / 8), math.sin(math.pi / 8), 0],
        "body_mass_kg": float(properties.mass),
        "body_com_m": properties.center_mass.tolist(),
        "body_inertia_about_com_kg_m2": properties.inertia.tolist(),
        "cad_volume_mm3": adapter.Volume() + docking_head.Volume(),
        "cad_valid_solids": len(solids),
        "cad_assembly_solids": 2,
        "docking_head_single_solid": True,
        "guide_convex_cells": guide_cell_count,
        "stop_convex_cells": len(collision) - guide_cell_count,
        "robot_bolt_angles_deg": bolt_angles,
        "hole_clocking_note": "index_angle_deg is configurable; verify your media flange drawing before manufacture",
        "collision_note": "guide cells approximate the analytic rounded quintic surface; STEP is a smooth loft; small mounting holes and rear bore omitted from primitive collision proxies",
        "parameters": p,
    }
    (out / "model_info.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return meshes, collision, meta


def add_body_contents(body, prefix, p, names, meta, collision_count):
    inertia = np.asarray(meta["body_inertia_about_com_kg_m2"])
    ET.SubElement(body, "inertial", mass=fmt(meta["body_mass_kg"]), pos=fmt(meta["body_com_m"]),
                  fullinertia=fmt([inertia[0, 0], inertia[1, 1], inertia[2, 2],
                                   inertia[0, 1], inertia[0, 2], inertia[1, 2]]))
    for name in names:
        material = "pd_dark" if name == "adapter" else "pd_silver" if name == "stop_land" else "pd_blue"
        ET.SubElement(body, "geom", name=f"{prefix}_visual_{name}", mesh=f"pd_v_{name}",
                      type="mesh", **{"class": "pd_visual", "material": material})
    # All collision geoms stay on the same rigid body.
    for i in range(collision_count):
        kind = "guide" if i < meta["guide_convex_cells"] else "stop"
        ET.SubElement(body, "geom", name=f"{prefix}_{kind}_{i:03}", mesh=f"pd_c_{i:03}",
                      type="mesh", **{"class": "pd_collision"})
    base_z = p["back_relief_mm"]
    total_base = p["adapter_thickness_mm"] + p["head_thickness_mm"]
    cylinders = [
        
        ("support", p["flange_support_diameter_mm"] / 2, 0, base_z),
        ("pilot", p["pilot_diameter_mm"] / 2, -p["pilot_length_mm"], 0),
    ]
    # A polygonal convex base avoids deep-overlap cylinder/mesh CCD artifacts.
    ET.SubElement(body, "geom", name=f"{prefix}_base_collision", type="mesh",
                  mesh="pd_base_proxy", **{"class": "pd_collision"})
    for name, radius, z0, z1 in cylinders:
        ET.SubElement(body, "geom", name=f"{prefix}_{name}_collision", type="cylinder",
                      size=fmt([radius / 1000, (z1 - z0) / 2000]),
                      pos=fmt([0, 0, (z0 + z1) / 2000]), **{"class": "pd_collision"})
    ET.SubElement(body, "site", name=f"{prefix}_flange", pos="0 0 0", size="0.002", group="4", rgba="1 0.6 0 1")
    ET.SubElement(body, "site", name=f"{prefix}_mating", pos=fmt([0, 0, meta["mating_site_z_m"]]),
                  size="0.002", group="4", rgba="0 0.8 0.5 1")
    ET.SubElement(body, "site", name=f"{prefix}_contact_zone", type="cylinder", group="5",
                  pos=fmt([0, 0, (total_base + p["guide_height_mm"] / 2) / 1000]),
                  size=fmt([p["outer_diameter_mm"] / 2000 + 0.001, p["guide_height_mm"] / 2000 + 0.001]),
                  rgba="0 1 0 0.05")


def make_mjcf(p, names, meta, collision_count, out):
    inc = ET.Element("mujocoinclude")
    defaults = ET.SubElement(inc, "default")
    visual = ET.SubElement(defaults, "default", {"class": "pd_visual"})
    ET.SubElement(visual, "geom", contype="0", conaffinity="0", group="2", density="0")
    collision = ET.SubElement(defaults, "default", {"class": "pd_collision"})
    ET.SubElement(collision, "geom", contype="1", conaffinity="1", group="3", rgba="0.35 0.65 0.85 0.3",
                  condim="3", friction="0.15 0.003 0.0001", margin="0", gap="0", density="0",
                  solref="0.003 1", solimp="0.95 0.99 0.0002")
    assets = ET.SubElement(inc, "asset")
    for name, color in [("pd_dark", ".20 .24 .30 1"), ("pd_blue", ".34 .48 .64 1"), ("pd_silver", ".68 .74 .81 1")]:
        ET.SubElement(assets, "material", name=name, rgba=color, specular="0.35", shininess="0.35")
    ET.SubElement(assets, "mesh", name="pd_base_proxy", file="../meshes/collision/base_proxy.obj")
    for name in names:
        ET.SubElement(assets, "mesh", name=f"pd_v_{name}", file=f"../meshes/visual/{name}.obj")
    for i in range(collision_count):
        ET.SubElement(assets, "mesh", name=f"pd_c_{i:03}", file=f"../meshes/collision/cell_{i:03}.obj")
    write_xml(inc, out / "mjcf" / "dock_assets.xml")
    for prefix in ["active", "passive"]:
        contents = ET.Element("mujocoinclude")
        add_body_contents(contents, prefix, p, names, meta, collision_count)
        write_xml(contents, out / "mjcf" / f"{prefix}_contents.xml")
    root = ET.Element("mujoco", model="PetalDock100_pair")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep="0.0005", gravity="0 0 0", integrator="implicitfast",
                  solver="Newton", iterations="60", tolerance="1e-10", cone="elliptic")
    ET.SubElement(root, "include", file="dock_assets.xml")
    display_assets = ET.SubElement(root, "asset")
    ET.SubElement(display_assets, "texture", name="pd_background", type="skybox", builtin="gradient",
                  rgb1="0.96 0.97 0.99", rgb2="0.84 0.88 0.94", width="256", height="1536")
    vis = ET.SubElement(root, "visual")
    ET.SubElement(vis, "global", offwidth="1200", offheight="900")
    ET.SubElement(vis, "headlight", ambient="0.45 0.45 0.45", diffuse="0.55 0.55 0.55", specular="0.15 0.15 0.15")
    ET.SubElement(root, "statistic", center="0 0 0.145", extent="0.18")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0.2 -0.2 0.5", dir="-0.2 0.2 -1", diffuse="0.7 0.7 0.7")
    ET.SubElement(world, "geom", type="plane", size="0.3 0.3 0.01", rgba="0.92 0.94 0.97 1", contype="0", conaffinity="0")
    ET.SubElement(world, "geom", type="cylinder", size="0.0315 0.05", pos="0 0 0.05", rgba=".18 .21 .27 1", contype="0", conaffinity="0")
    passive = ET.SubElement(world, "body", name="passive", pos="0 0 0.1")
    ET.SubElement(passive, "include", file="passive_contents.xml")
    nominal = meta["nominal_flange_separation_m"]
    active = ET.SubElement(world, "body", name="active", pos=fmt([0, 0, 0.1 + nominal + .035]),
                           quat=fmt(meta["mating_relative_quaternion_wxyz"]))
    ET.SubElement(active, "freejoint", name="active_free")
    ET.SubElement(active, "include", file="active_contents.xml")
    eq = ET.SubElement(root, "equality")
    ET.SubElement(eq, "weld", name="dock_lock", body1="passive", body2="active", active="false",
                  relpose=fmt([0, 0, nominal] + meta["mating_relative_quaternion_wxyz"]),
                  torquescale="0.05", solref="0.003 1", solimp="0.95 0.99 0.0002")
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "framepos", name="active_tcp_position", objtype="site", objname="active_mating")
    ET.SubElement(sensors, "framequat", name="active_tcp_orientation", objtype="site", objname="active_mating")
    ET.SubElement(sensors, "touch", name="interface_touch", site="active_contact_zone")
    write_xml(root, out / "mjcf" / "demo.xml")
    single = ET.fromstring(ET.tostring(root, encoding="unicode"))
    single.set("model", "PetalDock100_single")
    single.find("worldbody").remove(single.find("worldbody/body[@name='active']"))
    single.remove(single.find("equality"))
    single.remove(single.find("sensor"))
    single.find("statistic").set("center", "0 0 0.115")
    write_xml(single, out / "mjcf" / "single.xml")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=ROOT / "config.json")
    ap.add_argument("--output", type=Path, default=ROOT.parents[4] / "runs/generated_assets/petaldock_interface")
    args = ap.parse_args()
    p = json.loads(args.config.read_text(encoding="utf-8"))
    validate_parameters(p)
    out = args.output.resolve()
    for sub in ["cad", "meshes/visual", "meshes/collision", "mjcf", "results"]:
        (out / sub).mkdir(parents=True, exist_ok=True)
    # Remove generated cells when the angular resolution changes.
    for path in (out / "meshes/collision").glob("cell_*.obj"):
        path.unlink()
    meshes, cells, meta = make_cad(p, out)
    make_mjcf(p, list(meshes), meta, len(cells), out)
    print(json.dumps({k: meta[k] for k in ["body_mass_kg", "mating_site_z_m", "nominal_flange_separation_m",
                                          "cad_valid_solids", "guide_convex_cells", "stop_convex_cells"]}, indent=2))


if __name__ == "__main__":
    main()
