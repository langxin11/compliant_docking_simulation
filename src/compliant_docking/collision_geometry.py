"""Offline collision-geometry utilities; no decomposition package at runtime."""
from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import ConvexHull

from .interface_geometry import _triangles, asset_fingerprints
from .scene import REPO_ROOT

CONVEX_DIRECTORY = REPO_ROOT / "assets/interfaces/convex_crown"


def portable_asset_fingerprints(scene):
    """Keep generated asset manifests usable after moving/cloning the repository."""
    return {str(Path(path).relative_to(REPO_ROOT)): digest
            for path, digest in asset_fingerprints(scene).items()}


def connected_meshes(triangles):
    """Return independent closed-surface candidates in stable vertex order."""
    vertices, inverse = np.unique(np.asarray(triangles).reshape(-1, 3), axis=0,
                                  return_inverse=True)
    faces = inverse.reshape(-1, 3)
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                       shape=(len(vertices), len(vertices)))
    count, labels = connected_components(graph, directed=False)
    result = []
    for component in range(count):
        selected = faces[labels[faces[:, 0]] == component]
        indices = np.unique(selected)
        remap = np.full(len(vertices), -1)
        remap[indices] = np.arange(len(indices))
        result.append((vertices[indices], remap[selected]))
    return result


def oriented_hull(vertices):
    """Triangulate a convex hull with outward-facing normals for OBJ/MuJoCo."""
    vertices = np.asarray(vertices, dtype=float)
    hull = ConvexHull(vertices)
    faces = hull.simplices.copy()
    t = vertices[faces]
    normals = np.cross(t[:, 1]-t[:, 0], t[:, 2]-t[:, 0])
    reverse = np.einsum("ij,ij->i", normals, hull.equations[:, :3]) < 0
    faces[reverse] = faces[reverse][:, [0, 2, 1]]
    return vertices, faces


def signed_volume(triangles):
    t = np.asarray(triangles)
    return float(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum()/6)


def surface_samples(triangles, subdivisions=3):
    """Deterministic barycentric samples, not a continuous distance certificate."""
    t = np.asarray(triangles)
    weights = np.array([(i/subdivisions, j/subdivisions, 1-(i+j)/subdivisions)
                        for i in range(subdivisions+1)
                        for j in range(subdivisions+1-i)])
    return np.unique(np.einsum("sk,tkd->tsd", weights, t).reshape(-1, 3), axis=0)


def point_mesh_distance(points, triangles):
    """Euclidean closest distance to triangles, including face interiors."""
    points, t = np.asarray(points), np.asarray(triangles)
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    u, v = b-a, c-a
    normal = np.cross(u, v)
    n2 = np.einsum("ij,ij->i", normal, normal)
    uu, uv, vv = (np.einsum("ij,ij->i", left, right) for left, right in [(u, u), (u, v), (v, v)])
    det = uu*vv-uv*uv
    output = []
    for start in range(0, len(points), 128):
        delta = points[start:start+128, None, :]-a
        du, dv = np.einsum("ptd,td->pt", delta, u), np.einsum("ptd,td->pt", delta, v)
        beta = (du*vv-dv*uv)/np.maximum(det, 1e-30)
        gamma = (dv*uu-du*uv)/np.maximum(det, 1e-30)
        inside = (beta >= -1e-10) & (gamma >= -1e-10) & (beta+gamma <= 1+1e-10) & (det > 1e-30)
        face = np.einsum("ptd,td->pt", delta, normal)**2/np.maximum(n2, 1e-30)
        distance2 = np.where(inside, face, np.inf)
        for left, right in [(a, b), (b, c), (c, a)]:
            edge = right-left
            difference = points[start:start+128, None, :]-left
            factor = np.clip(np.einsum("ptd,td->pt", difference, edge)
                             /np.maximum(np.einsum("ij,ij->i", edge, edge), 1e-30), 0, 1)
            nearest = difference-factor[:, :, None]*edge
            distance2 = np.minimum(distance2, np.einsum("ptd,ptd->pt", nearest, nearest))
        output.extend(np.sqrt(distance2.min(axis=1)))
    return np.asarray(output)


def inside_closed_mesh(points, triangles):
    """Solid-angle winding test; requires an oriented, closed input surface."""
    output = []
    for start in range(0, len(points), 128):
        vectors = np.asarray(triangles)[None, :, :, :]-np.asarray(points)[start:start+128, None, None, :]
        lengths = np.linalg.norm(vectors, axis=-1)
        a, b, c = vectors[:, :, 0], vectors[:, :, 1], vectors[:, :, 2]
        numerator = np.einsum("ptd,ptd->pt", a, np.cross(b, c))
        denominator = (lengths.prod(axis=2)
                       + np.einsum("ptd,ptd->pt", a, b)*lengths[:, :, 2]
                       + np.einsum("ptd,ptd->pt", b, c)*lengths[:, :, 0]
                       + np.einsum("ptd,ptd->pt", c, a)*lengths[:, :, 1])
        winding = (2*np.arctan2(numerator, denominator)).sum(axis=1)
        output.extend(np.abs(winding) > 2*np.pi)
    return np.asarray(output)


def sampled_solid_error(original, hulls):
    """Sampled excess and missing material per original connected component.

    Internal decomposition faces are inside the original solid and count as zero
    excess. Distances to vertices alone would wrongly penalize large flat faces.
    """
    original = np.asarray(original)
    generated = np.concatenate([v[f] for v, f in hulls])
    generated_points = surface_samples(generated)
    distance = point_mesh_distance(generated_points, original)
    excess = np.where(inside_closed_mesh(generated_points, original), 0., distance)
    original_points = surface_samples(original)
    inside_union = np.zeros(len(original_points), dtype=bool)
    for vertices, _ in hulls:
        equations = ConvexHull(vertices).equations
        inside_union |= np.all(original_points @ equations[:, :3].T+equations[:, 3] <= 1e-8, axis=1)
    missing = np.where(inside_union, 0., point_mesh_distance(original_points, generated))
    return dict(sampled_excess_m=float(excess.max()), sampled_missing_m=float(missing.max()),
                original_sample_count=len(original_points), generated_sample_count=len(generated_points),
                barycentric_subdivisions=3, continuous_certificate=False)


def collision_triangles(model, data, prefix, root):
    triangles = []
    for index in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index) or ""
        if name.startswith(prefix) and (model.geom_contype[index] or model.geom_conaffinity[index]):
            if model.geom_type[index] != mujoco.mjtGeom.mjGEOM_MESH:
                raise ValueError(f"{name}: expected convex mesh proxy")
            triangles.append(_triangles(model, data, name, root))
    if not triangles:
        raise ValueError("No convex collision proxies found")
    return np.concatenate(triangles)


def with_convex_interface(scene):
    """Swap fragments only after the generated geometry has passed its gate."""
    manifest_path = CONVEX_DIRECTORY / "manifest.json"
    if not manifest_path.exists():
        raise ValueError("Generate and validate convex_crown assets before selecting convex collisions")
    manifest = json.loads(manifest_path.read_text())
    if manifest["validation"]["status"] != "PASS":
        raise ValueError("Convex interface geometry did not pass validation")
    if manifest["source_assets"] != portable_asset_fingerprints(scene):
        raise ValueError("Convex assets were generated from a different interface; regenerate them")
    candidate = replace(scene, tool=replace(scene.tool, mjcf=CONVEX_DIRECTORY / "male.xml"),
                        target=replace(scene.target, mjcf=CONVEX_DIRECTORY / "female.xml"))
    if portable_asset_fingerprints(candidate) != manifest["generated_assets"]:
        raise ValueError("Generated collision fragments or meshes changed after validation")
    return candidate
