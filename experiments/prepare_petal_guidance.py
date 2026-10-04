"""Parametric simulation candidates with paired angular/radial guide surfaces.

Reuses the reviewed mounting plate and stops. No attachment script is executed,
and no STEP/manufacturing validity is claimed. New guide OBJ surfaces, convex
cells and full inertia deltas are generated using NumPy/SciPy alone.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from compliant_docking.collision_geometry import oriented_hull
from compliant_docking.scene import REPO_ROOT

SOURCE = REPO_ROOT / "assets/interfaces/petal_dock100"
DESTINATION = REPO_ROOT / "assets/interfaces/petal_guidance"
VARIANTS = ("narrow", "radial")


def smooth_ramp(u, blend=.1):
    """C1 rounded linear ramp; f(u)+f(1-u)=1, zero slope at ends."""
    u = np.clip(np.asarray(u, dtype=float), 0., 1.)
    if not 0 < blend < .5:
        raise ValueError("Ramp blend must be between zero and one half")
    return np.where(u < blend, u*u/(2*blend*(1-blend)),
        np.where(u > 1-blend, 1-(1-u)**2/(2*blend*(1-blend)),
                 (u-blend/2)/(1-blend)))


def angular_fraction(angle, p):
    d = abs((np.asarray(angle)+45.) % 90.-45.)
    a = p["guide_tip_half_angle_deg"]
    return smooth_ramp((45.-a-d)/(45.-2*a), p["guide_blend_fraction"])


def radial_amplitude(radius, p):
    half = p["guide_height_mm"]/2
    width, drop = p["radial_lead_width_mm"], p["radial_crest_drop_mm"]
    if width == 0.:
        return np.zeros_like(np.asarray(radius), dtype=float)+half
    ri, ro = p["guide_inner_diameter_mm"]/2, p["outer_diameter_mm"]/2
    distance = np.minimum(np.asarray(radius)-ri, ro-np.asarray(radius))
    # Equal fall/rise of crest/valley preserves paired nominal separation.
    return half-drop*(1-smooth_ramp(distance/width, p["radial_blend_fraction"]))


def surface_height(radius, angle, p):
    s = angular_fraction(angle, p)
    return p["guide_height_mm"]/2 + radial_amplitude(radius, p)*(2*s-1)


def shoulder_profile(p, samples=13):
    ri, ro = p["guide_inner_diameter_mm"]/2, p["outer_diameter_mm"]/2
    edge = p["guide_edge_round_mm"]
    t = np.linspace(0, np.pi/2, samples)
    radii = np.r_[ri+edge*(1-np.cos(t)), ro-edge+edge*np.sin(t)]
    factors = np.r_[1-edge/p["guide_height_mm"]*(1-np.sin(t)),
                     1-edge/p["guide_height_mm"]*(1-np.cos(t))]
    if p["radial_lead_width_mm"]:
        radii = np.r_[radii, np.linspace(ri+edge, ro-edge, 65),
                      ri+p["radial_lead_width_mm"], ro-p["radial_lead_width_mm"]]
        factors = np.r_[factors, np.ones(67)]
    order = np.argsort(radii)
    radii, index = np.unique(radii[order], return_index=True)
    return radii, factors[order][index]


def xyz(r, a, z):
    a = np.deg2rad(a)
    return np.column_stack((np.asarray(r)*np.cos(a), np.asarray(r)*np.sin(a), z))


def clean_mesh(vertices, faces):
    vertices, inverse = np.unique(np.round(vertices, 13), axis=0, return_inverse=True)
    faces = inverse[np.asarray(faces)]
    tri = vertices[faces]
    valid = np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1) > 1e-18
    faces = faces[valid]
    _, inverse, count = np.unique(np.sort(faces, axis=1), axis=0, return_inverse=True, return_counts=True)
    faces = faces[count[inverse] == 1]  # cancel coincident top/bottom at zero height
    used = np.unique(faces)
    remap = np.full(len(vertices), -1)
    remap[used] = np.arange(len(used))
    return vertices[used], remap[faces]


def guide_mesh(center, p, angular_samples=257, radial_samples=13):
    extent = 45. if p["radial_lead_width_mm"] else 45.-p["guide_tip_half_angle_deg"]
    angles = np.linspace(center-extent, center+extent, angular_samples)
    radii, factor = shoulder_profile(p, radial_samples)
    n, na = len(radii), len(angles)
    base = p["adapter_thickness_mm"]+p["head_thickness_mm"]
    top = np.concatenate([xyz(radii, a, base+surface_height(radii, a, p)*factor) for a in angles])
    bottom = top.copy()
    bottom[:, 2] = base
    offset, faces = len(top), []
    for i in range(na-1):
        for j in range(n-1):
            k, b = i*n+j, offset+i*n+j
            faces.extend(((k,k+1,k+n+1),(k,k+n+1,k+n),
                          (b,b+n+1,b+1),(b,b+n,b+n+1)))
        k = i*n
        faces.extend(((offset+k,k,k+n),(offset+k,k+n,offset+k+n)))
        k = (i+1)*n-1
        faces.extend(((offset+k,offset+k+n,k+n),(offset+k,k+n,k)))
    for j in range(n-1):
        faces.extend(((offset+j,offset+j+1,j+1),(offset+j,j+1,j)))
        k = (na-1)*n+j
        faces.extend(((offset+k,k,k+1),(offset+k,k+1,offset+k+1)))
    return clean_mesh(np.concatenate((top,bottom))*.001, faces)


def mesh_moments(mesh, density):
    """Signed tetrahedral volume, first moment and inertia about origin."""
    vertices, faces = mesh
    t = vertices[faces]
    volume = np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1],t[:, 2]))/6
    s = t.sum(axis=1)
    mass = density*volume.sum()
    first = density*np.einsum("i,ij->j", volume, s)/4
    second = density*(np.einsum("i,ij,ik->jk",volume,s,s)
                      +np.einsum("i,ikj,ikl->jl",volume,t,t))/20
    return float(mass), first, np.trace(second)*np.eye(3)-second


def read_obj(path):
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        values = line.split()
        if values and values[0] == "v":
            vertices.append([float(v) for v in values[1:4]])
        elif values and values[0] == "f":
            ids = [int(v.split("/")[0])-1 for v in values[1:]]
            faces.extend((ids[0], ids[i], ids[i+1]) for i in range(1,len(ids)-1))
    return np.asarray(vertices), np.asarray(faces)


def write_obj(path, mesh):
    vertices, faces = mesh
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("# Parametric PetalDock simulation geometry; SI units\n")
        for vertex in vertices:
            f.write("v "+" ".join(f"{x:.12g}" for x in vertex)+"\n")
        for face in faces:
            f.write("f "+" ".join(str(int(i)+1) for i in face)+"\n")


def convex_cells(center, p):
    ri, ro = p["guide_inner_diameter_mm"]/2, p["outer_diameter_mm"]/2
    edge = p["guide_edge_round_mm"]
    extent = 45. if p["radial_lead_width_mm"] else 45.-p["guide_tip_half_angle_deg"]
    count = int(round(2*extent/(360/p["guide_segments"])))
    angles = np.linspace(center-extent, center+extent, count+1)
    radii, factors = shoulder_profile(p, 5)
    bands = [(ri,ro)]
    if p["radial_lead_width_mm"]:
        width = p["radial_lead_width_mm"]
        knots = [ri,ri+edge,ri+width,ro-width,ro-edge,ro]
        bands = list(zip(knots[:-1],knots[1:],strict=True))
    base = p["adapter_thickness_mm"]+p["head_thickness_mm"]
    for a0,a1 in zip(angles[:-1],angles[1:],strict=True):
        for r0,r1 in bands:
            selected = (radii >= r0-1e-8) & (radii <= r1+1e-8)
            r, factor = radii[selected], factors[selected]
            points = np.concatenate([xyz(r,a,base+surface_height(r,a,p)*factor) for a in (a0,a1)]
                                    +[xyz(np.array([r0,r1]),a,np.array([base,base])) for a in (a0,a1)])*.001
            points = np.unique(np.round(points,13),axis=0)
            if np.ptp(points[:,2]) < 1e-10:
                continue
            yield oriented_hull(points)


def candidate_parameters(original, variant):
    if variant not in VARIANTS:
        raise ValueError("Unknown guide candidate")
    p = dict(original, guide_tip_half_angle_deg=4.21875, guide_profile="rounded_linear",
             guide_blend_fraction=.1, radial_blend_fraction=.1,
             radial_lead_width_mm=6. if variant == "radial" else 0.,
             radial_crest_drop_mm=3. if variant == "radial" else 0.)
    return p


def fmt(values):
    return " ".join(f"{x:.16g}" for x in np.atleast_1d(values))


def generate(variant, destination=None):
    destination = DESTINATION/variant if destination is None else Path(destination)
    if destination.exists():
        raise ValueError("Candidate directory exists; preserve it or select a new destination")
    source_manifest = json.loads((SOURCE/"manifest.json").read_text())
    for name,digest in source_manifest["imported_files"].items():
        assert hashlib.sha256((SOURCE/name).read_bytes()).hexdigest() == digest, name
    info = json.loads((SOURCE/"model_info.json").read_text())
    p = candidate_parameters(info["parameters"],variant)
    meshes = [guide_mesh(k*90,p) for k in range(4)]
    refined = [guide_mesh(k*90,p,513,25) for k in range(4)]
    old = [read_obj(SOURCE/f"meshes/visual/petal_{k}.obj") for k in range(4)]
    density = p["density_kg_m3"]
    def moments(items):
        values = [mesh_moments(m,density) for m in items]
        return tuple(sum(v[i] for v in values) for i in range(3))
    previous, generated, fine = moments(old), moments(meshes), moments(refined)
    delta = [generated[i]-previous[i] for i in range(3)]
    mass = info["body_mass_kg"]+delta[0]
    center0 = np.asarray(info["body_com_m"])
    center = (info["body_mass_kg"]*center0+delta[1])/mass
    inertia0 = np.asarray(info["body_inertia_about_com_kg_m2"])
    def shift(m,c):
        return m*(np.dot(c,c)*np.eye(3)-np.outer(c,c))
    inertia = inertia0+shift(info["body_mass_kg"],center0)+delta[2]-shift(mass,center)
    convergence = dict(mass_delta_kg=float(abs(fine[0]-generated[0])),
        first_moment_delta_kg_m=float(np.max(abs(fine[1]-generated[1]))),
        inertia_at_origin_delta_kg_m2=float(np.max(abs(fine[2]-generated[2]))),
        mesh_mass_positive=all(mesh_moments(m,density)[0]>0 for m in meshes))
    assert convergence["mass_delta_kg"] < 5e-5
    assert convergence["inertia_at_origin_delta_kg_m2"] < 1e-7
    assert mass > 0 and np.linalg.eigvalsh(inertia).min() > 0
    destination.mkdir(parents=True)
    for name in ("adapter","head_plate","stop_land"):
        target = destination/f"meshes/visual/{name}.obj"
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(SOURCE/f"meshes/visual/{name}.obj",target)
    base = destination/"meshes/collision/base_proxy.obj"
    base.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(SOURCE/"meshes/collision/base_proxy.obj",base)
    for k,mesh in enumerate(meshes):
        write_obj(destination/f"meshes/visual/petal_{k}.obj",mesh)
    cells = [cell for k in range(4) for cell in convex_cells(k*90,p)]
    for k,cell in enumerate(cells):
        write_obj(destination/f"meshes/collision/guide_{k:04}.obj",cell)
    for k in range(info["stop_convex_cells"]):
        shutil.copy2(SOURCE/f"meshes/collision/cell_{info['guide_convex_cells']+k:03}.obj",
                     destination/f"meshes/collision/stop_{k:03}.obj")
    for side in ("active","passive"):
        root = ET.parse(SOURCE/f"{side}.xml").getroot()
        root.set("model",f"PetalDock100_guidance_{variant}_{side}")
        asset,body = root.find("asset"),root.find("worldbody/body")
        for child in list(asset):
            if (child.get("name") or "").startswith("pd_c_"):
                asset.remove(child)
        for child in list(body):
            if (child.get("name") or "").startswith(("guide_","stop_")):
                body.remove(child)
        for kind,count in (("guide",len(cells)),("stop",info["stop_convex_cells"])):
            for k in range(count):
                token = f"{k:04}" if kind == "guide" else f"{k:03}"
                name = f"pd_{kind}_{token}"
                ET.SubElement(asset,"mesh",name=name,file=f"meshes/collision/{kind}_{token}.obj")
                ET.SubElement(body,"geom",name=f"{kind}_{token}",mesh=name,type="mesh",**{"class":"pd_collision"})
        body.find("inertial").attrib.update(mass=fmt(mass),pos=fmt(center),
            fullinertia=fmt([inertia[0,0],inertia[1,1],inertia[2,2],inertia[0,1],inertia[0,2],inertia[1,2]]))
        ET.indent(root)
        ET.ElementTree(root).write(destination/f"{side}.xml",encoding="utf-8",xml_declaration=True)
    meta = copy.deepcopy(info)
    for key in list(meta):
        if key.startswith("cad_") or key == "docking_head_single_solid":
            meta.pop(key)
    meta.update(version=f"3.0-simulation-{variant}",body_mass_kg=mass,body_com_m=center.tolist(),
        body_inertia_about_com_kg_m2=inertia.tolist(),guide_convex_cells=len(cells),parameters=p,
        collision_note="Direct convex guide cells; original stops/base preserved. No new STEP export.",
        manufacturing_cad_generated=False, inertia_method="original assembly plus oriented guide-mesh delta",
        inertia_convergence=convergence)
    (destination/"model_info.json").write_text(json.dumps(meta,indent=2)+"\n")
    manifest = dict(interface=f"PetalDock100 guidance {variant}",source_files=source_manifest["imported_files"],
        generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        imported_files={str(f.relative_to(destination)):hashlib.sha256(f.read_bytes()).hexdigest()
                        for f in sorted(destination.rglob("*")) if f.is_file()},
        changes=["narrow flat crest plus rounded linear angular slope",
                 "paired radial lead" if variant == "radial" else "original radial shoulders",
                 "full assembly inertia updated from mesh delta"],bundled_manifest_used=False)
    (destination/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    return meta


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant",nargs="+",choices=VARIANTS,default=list(VARIANTS))
    args = parser.parse_args()
    for name in args.variant:
        meta = generate(name)
        print(name,meta["guide_convex_cells"],meta["body_mass_kg"],meta["inertia_convergence"])
