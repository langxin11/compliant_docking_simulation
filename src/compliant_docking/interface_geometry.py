"""Mesh-based, evaluation-only reference for an upright crown interface.

Vertical rays sample the upper target envelope and lower tool envelope. Their
maximum difference is the first non-overlapping root separation on a downward
coaxial approach. A minimum over yaw is a compact seating *candidate*, not a
certificate of locking, manufacturing clearance or a continuous collision test.
SDF collision detection is deliberately not used to define this reference.
"""
from __future__ import annotations

import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def vertical_envelope(triangles, x, y, *, upper):
    """Rasterize exact triangle heights at grid rays; ignore vertical faces."""
    result = np.full((len(y), len(x)), -np.inf if upper else np.inf)
    for triangle in triangles:
        A, B, C = triangle
        u, v = B[:2]-A[:2], C[:2]-A[:2]
        det = u[0]*v[1]-u[1]*v[0]
        if abs(det) < 1e-14:
            continue
        lo, hi = triangle[:, :2].min(axis=0), triangle[:, :2].max(axis=0)
        i0, j0 = np.searchsorted(x, lo[0]-1e-10), np.searchsorted(y, lo[1]-1e-10)
        i1, j1 = np.searchsorted(x, hi[0]+1e-10, side="right"), np.searchsorted(y, hi[1]+1e-10, side="right")
        if i0 >= i1 or j0 >= j1:
            continue
        dx, dy = x[None, i0:i1]-A[0], y[j0:j1, None]-A[1]
        b = (dx*v[1]-dy*v[0])/det
        c = (dy*u[0]-dx*u[1])/det
        inside = (b >= -1e-8) & (c >= -1e-8) & (b+c <= 1+1e-8)
        z = A[2]+b*(B[2]-A[2])+c*(C[2]-A[2])
        block = result[j0:j1, i0:i1]
        function = np.maximum if upper else np.minimum
        block[inside] = function(block[inside], z[inside])
    return result


def build_interface_pair(scene, *, octree_maxdepth=None, presentation=False):
    """Exact interface fragments with a free tool carrier, no arm or floor."""
    spec = mujoco.MjSpec.from_string(
        '<mujoco><option gravity="0 0 0"/><worldbody>'
        '<body name="carrier"><freejoint/></body></worldbody></mujoco>')
    for path, prefix, frame in [(scene.tool.mjcf, scene.tool.prefix, spec.body("carrier").add_frame()),
                                (scene.target.mjcf, scene.target.prefix, spec.worldbody.add_frame())]:
        part = mujoco.MjSpec.from_file(str(path))
        if octree_maxdepth is not None:
            for mesh in part.meshes:
                mesh.octree_maxdepth = octree_maxdepth
        frame.attach_body(part.body("dock"), prefix=prefix)
    if presentation:
        spec.add_texture(name="presentation_sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                         builtin=mujoco.mjtBuiltin.mjBUILTIN_FLAT,
                         rgb1=[.94, .95, .97], rgb2=[.94, .95, .97], width=256, height=1536)
        spec.worldbody.add_light(pos=[.2, -.2, .4], dir=[-.2, .2, -.4],
                                 type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                                 diffuse=[.7, .7, .7], ambient=[.25, .25, .25], castshadow=False)
    spec.option.sdf_iterations = scene.physics.sdf_iterations
    spec.option.sdf_initpoints = scene.physics.sdf_initpoints
    return spec.compile()


def _triangles(model, data, geom_name, root_name):
    geom = model.geom(geom_name).id
    mesh = int(model.geom_dataid[geom])
    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    # Compiled mesh vertices are recentered/rotated. geom_xmat/xpos include this
    # transform; reading raw vertices as CAD coordinates would silently be wrong.
    vertices = model.mesh_vert[start:start+count] @ data.geom_xmat[geom].reshape(3, 3).T
    vertices += data.geom_xpos[geom]
    root = model.body(root_name).id
    vertices = (vertices-data.xpos[root]) @ data.xmat[root].reshape(3, 3)
    first, number = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
    return vertices[model.mesh_face[first:first+number]]


def asset_fingerprints(scene):
    files = set()
    for fragment in (scene.tool.mjcf, scene.target.mjcf):
        fragment = Path(fragment)
        files.add(fragment)
        root = ET.parse(fragment).getroot()
        compiler = root.find("compiler")
        meshdir = compiler.get("meshdir", "") if compiler is not None else ""
        for mesh in root.findall("./asset/mesh"):
            if mesh.get("file"):
                files.add((fragment.parent / meshdir / mesh.get("file")).resolve())
    return {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(files)}


def mesh_topology(triangles):
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    edges = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    _, incidence = np.unique(edges, axis=0, return_counts=True)
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(len(vertices), len(vertices)))
    count, labels = connected_components(graph, directed=False)
    volumes = []
    for component in range(count):
        parts = triangles[labels[faces[:, 0]] == component]
        volume = np.sum(np.einsum("ij,ij->i", parts[:, 0], np.cross(parts[:, 1], parts[:, 2])))/6
        volumes.append(float(volume))
    return dict(vertices=int(len(vertices)), triangles=int(len(triangles)), components=int(count),
                boundary_edges=int(np.sum(incidence == 1)), nonmanifold_edges=int(np.sum(incidence > 2)),
                component_signed_volumes_m3=volumes,
                limitations="Closed-edge counts do not prove no self-intersections or no overlapping components")


class InterfaceGeometry:
    def __init__(self, scene):
        self.model = build_interface_pair(scene)
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:] = [0, 0, 0, 1, 0, 0, 0]
        mujoco.mj_forward(self.model, self.data)
        self.tool = _triangles(self.model, self.data, scene.tool.prefix+"dock_geom", scene.tool.prefix+"dock")
        self.target = _triangles(self.model, self.data, scene.target.prefix+"dock_geom", scene.target.prefix+"dock")
        self.scene = scene
        self._grids = {}
        self.calibration = None

    def _grid(self, spacing):
        if spacing not in self._grids:
            extent = float(np.max(np.abs(self.target[:, :, :2])))+spacing
            extent = np.ceil(extent/spacing)*spacing
            axis = np.arange(-extent, extent+spacing/2, spacing)
            self._grids[spacing] = axis, vertical_envelope(self.target, axis, axis, upper=True)
        return self._grids[spacing]

    def required_height(self, rotation, xy=(0., 0.), *, spacing=.0005):
        axis, top = self._grid(spacing)
        triangles = self.tool @ np.asarray(rotation).T
        triangles[:, :, :2] += np.asarray(xy)
        bottom = vertical_envelope(triangles, axis, axis, upper=False)
        shared = np.isfinite(top) & np.isfinite(bottom)
        if not shared.any():
            raise ValueError("Tool and target have no sampled XY overlap")
        return float(np.max(top[shared]-bottom[shared]))

    @staticmethod
    def rotation(yaw_deg):
        return pin.exp3(np.array([0., 0., np.deg2rad(yaw_deg)])) @ np.diag([1., -1., -1.])

    def calibrate(self):
        angles = np.arange(0., 360., 5.)
        heights = np.array([self.required_height(self.rotation(a), spacing=.001) for a in angles])
        minima = [i for i, h in enumerate(heights)
                  if h < heights[(i-1) % len(heights)] and h <= heights[(i+1) % len(heights)]]
        candidates = []
        for index in minima:
            fine_angles = np.arange(angles[index]-5., angles[index]+5.01, .5)
            fine = [self.required_height(self.rotation(a), spacing=.0005) for a in fine_angles]
            phase = float(fine_angles[np.argmin(fine)])
            finest_angles = np.arange(phase-1., phase+1.01, .25)
            finest = [self.required_height(self.rotation(a), spacing=.00025) for a in finest_angles]
            phase = float(finest_angles[np.argmin(finest)]) % 360
            height = float(min(finest))
            grid_change = abs(height-self.required_height(self.rotation(phase), spacing=.0005))
            candidates.append(dict(yaw_deg=phase, root_height_m=height,
                                   grid_refinement_change_m=grid_change))
        if not candidates:
            raise ValueError("Yaw scan found no compact reference minima")
        self.calibration = dict(
            status="SAMPLED_MESH_REFERENCE", locking_verified=False,
            method="upper/lower triangle-envelope rays; compact coaxial yaw minima",
            reference_frame="tool root relative to target root; Rz(yaw) Rx(180deg)",
            assets=asset_fingerprints(self.scene), coarse_yaw_deg=angles.tolist(),
            mesh_topology=dict(tool=mesh_topology(self.tool), target=mesh_topology(self.target)),
            coarse_height_m=heights.tolist(), candidates=candidates,
            fine_yaw_step_deg=.25, fine_grid_m=.00025,
            approximate_90deg_repeat_error_m=float(np.max(np.abs(heights-np.roll(heights, 18)))),
            tolerances=dict(lateral_m=.0005, tilt_deg=.5, yaw_deg=2., axial_m=.00075),
            limitations=["sampled rays can miss narrow features", "coaxial geometric reference only",
                         "research tolerances are not manufacturing tolerances",
                         "no locking or manufacturing-clearance certificate"])
        return self.calibration

    def compare_collision_onsets(self):
        """Quasi-static downward scans against the independent CAD envelope."""
        if self.calibration is None:
            raise ValueError("Calibrate before comparing collision onset")
        opt = self.model.opt
        original = int(opt.sdf_iterations), int(opt.sdf_initpoints)
        angles = sorted({0., 45., *(c["yaw_deg"] for c in self.calibration["candidates"])})
        results = []
        is_sdf = np.any(self.model.geom_type == int(mujoco.mjtGeom.mjGEOM_SDF))
        settings = ([("baseline", 1, None), ("sdf_refined", 2, None), ("octree_refined", 1, 8)]
                    if is_sdf else [("baseline", 1, None)])
        try:
            for setting, factor, depth in settings:
                model = self.model if depth is None else build_interface_pair(self.scene, octree_maxdepth=depth)
                data = self.data if depth is None else mujoco.MjData(model)
                opt = model.opt
                opt.sdf_iterations, opt.sdf_initpoints = original[0]*factor, original[1]
                for angle in angles:
                    R = self.rotation(angle)
                    quat = pin.Quaternion(R).coeffs()[[3, 0, 1, 2]]
                    required = self.required_height(R, spacing=.00025)
                    onset = None
                    error = None
                    mujoco.mj_resetData(model, data)
                    for height in np.arange(.075, required-.002, -.00025):
                        data.qpos[:] = np.r_[0., 0., height, quat]
                        try:
                            mujoco.mj_forward(model, data)
                        except mujoco.FatalError as exc:
                            error = str(exc)
                            break
                        distance = min((c.dist for c in data.contact), default=0.)
                        if distance < -1e-6:
                            onset = float(height)
                            break
                    results.append(dict(setting=setting, yaw_deg=angle, mesh_height_m=required,
                                        sdf_iterations=int(opt.sdf_iterations), sdf_initpoints=int(opt.sdf_initpoints),
                                        octree_depth_override=depth,
                                        error=error, backend="sdf" if is_sdf else "convex",
                                        first_penetration_height_m=onset,
                                        sdf_first_penetration_height_m=onset if is_sdf else None,
                                        collision_minus_mesh_m=None if onset is None else onset-required,
                                        sdf_minus_mesh_m=None if not is_sdf or onset is None else onset-required,
                                        height_step_m=.00025, penetration_threshold_m=1e-6))
        finally:
            self.model.opt.sdf_iterations, self.model.opt.sdf_initpoints = original
        return results

    def compare_sdf_onsets(self):
        """Compatibility alias for the original SDF experiment."""
        return self.compare_collision_onsets()

    def evaluate(self, samples, target_position, target_rotation):
        if not samples:
            return dict(status="INCOMPLETE", reasons=["no synchronized samples"])
        if self.calibration is None:
            raise ValueError("Calibrate geometry before evaluating a rollout")
        tol = self.calibration["tolerances"]
        # Inspect five states spanning the last second. This geometry verdict is
        # sampled and separate from the every-step dynamic/load gate.
        times = np.array([s["t"] for s in samples])
        indices = np.unique([np.argmin(abs(times-t)) for t in np.linspace(max(times[0], times[-1]-1), times[-1], 5)])
        measurements = []
        for index in indices:
            sample = samples[index]
            position = target_rotation.T @ (sample["position"]-target_position)
            rotation = target_rotation.T @ sample["rotation"]
            yaw = np.rad2deg(np.arctan2(rotation[1, 0], rotation[0, 0]))
            candidate = min(self.calibration["candidates"],
                            key=lambda c: abs((yaw-c["yaw_deg"]+180) % 360-180))
            required = self.required_height(rotation, position[:2], spacing=.00025)
            measurements.append(dict(t=sample["t"], root_height_mm=1000*position[2],
                                     lateral_mm=1000*np.linalg.norm(position[:2]),
                                     tilt_deg=np.rad2deg(np.arccos(np.clip(-rotation[2, 2], -1, 1))),
                                     yaw_deg=yaw, candidate_yaw_deg=candidate["yaw_deg"],
                                     phase_error_deg=abs((yaw-candidate["yaw_deg"]+180) % 360-180),
                                     compact_height_gap_mm=1000*(position[2]-candidate["root_height_m"]),
                                     mesh_clearance_mm=1000*(position[2]-required)))
        reasons = []
        checks = [(max(m["lateral_mm"] for m in measurements) > 1000*tol["lateral_m"], "lateral alignment"),
                  (max(m["tilt_deg"] for m in measurements) > tol["tilt_deg"], "axis alignment"),
                  (max(m["phase_error_deg"] for m in measurements) > tol["yaw_deg"], "crown phase"),
                  (max(m["compact_height_gap_mm"] for m in measurements) > 1000*tol["axial_m"], "above compact reference"),
                  (min(m["mesh_clearance_mm"] for m in measurements) < -1000*tol["axial_m"], "sampled mesh overlap")]
        reasons.extend(reason for failed, reason in checks if failed)
        complete = times[-1]-times[0] >= 1.
        return dict(status="INCOMPLETE" if not complete else "NOT_SEATED" if reasons else "SEATED_CANDIDATE",
                    reasons=reasons, samples=measurements, locking_verified=False)
