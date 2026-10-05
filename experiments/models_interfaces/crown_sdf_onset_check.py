"""原冠形 STL 首触核对：完整表面三角形与 SDF 的同姿态静态比较。

计算投影三角形交集上高度差的最大值，避免把导向首触与最终紧凑落座混同。
只运行四个固定姿态，不积分动力学、不调整 SDF 精度或控制参数。
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin
from scipy.spatial import cKDTree

from compliant_docking.interface_geometry import InterfaceGeometry, asset_fingerprints
from compliant_docking.scene import REPO_ROOT, load_scene


def cross2(a, b):
    """二维叉积标量。"""
    return a[0]*b[1]-a[1]*b[0]


def intersection(a, b):
    """用凸多边形裁剪求两个 XY 三角形的交集顶点，单位 m。"""
    polygon = list(a)
    if cross2(b[1]-b[0], b[2]-b[0]) < 0:
        b = b[::-1]
    for start, end in zip(b, np.roll(b, -1, axis=0), strict=True):
        if not polygon:
            break
        result = []
        previous = polygon[-1]
        dp = cross2(end-start, previous-start)
        for current in polygon:
            dc = cross2(end-start, current-start)
            if (dc >= 0) != (dp >= 0):
                result.append(previous+(current-previous)*dp/(dp-dc))
            if dc >= 0:
                result.append(current)
            previous, dp = current, dc
        polygon = result
    return np.asarray(polygon)


def exact_height(tool, target):
    """求竖直下降时原网格首触根间距，并返回见证接触点。

    对有面积的投影面片，z 为 XY 的仿射函数；两个面片交集上的高度差
    最大值在交集顶点取得。闭合网格的竖直边界由相邻非竖直面片覆盖。
    """
    def prepare(triangles):
        delta = triangles[:, 1:, :2]-triangles[:, :1, :2]
        determinants = delta[:, 0, 0]*delta[:, 1, 1]-delta[:, 0, 1]*delta[:, 1, 0]
        indices = np.flatnonzero(abs(determinants) > 1e-14)
        t = triangles[indices]
        matrix = np.concatenate((t[:, :, :2], np.ones((len(t), 3, 1))), axis=2)
        coefficients = np.linalg.solve(matrix, t[:, :, 2, None])[:, :, 0]
        return t, coefficients, indices
    a, ap, ai = prepare(tool)
    b, bp, bi = prepare(target)
    blo, bhi = b[:, :, :2].min(axis=1), b[:, :, :2].max(axis=1)
    maximum, witness, pairs = -np.inf, None, 0
    for index, triangle in enumerate(a):
        lo, hi = triangle[:, :2].min(axis=0), triangle[:, :2].max(axis=0)
        candidates = np.flatnonzero(np.all(bhi >= lo, axis=1) & np.all(blo <= hi, axis=1))
        for j in candidates:
            poly = intersection(triangle[:, :2], b[j, :, :2])
            if len(poly) == 0:
                continue
            pairs += 1
            xy1 = np.column_stack((poly, np.ones(len(poly))))
            heights = xy1@(bp[j]-ap[index])
            k = np.argmax(heights)
            if heights[k] > maximum:
                maximum = float(heights[k])
                witness = dict(xy_m=poly[k].tolist(), tool_triangle=int(ai[index]),
                               target_triangle=int(bi[j]), target_z_m=float(xy1[k]@bp[j]),
                               tool_z_before_translation_m=float(xy1[k]@ap[index]))
    assert np.isfinite(maximum)
    return dict(root_height_m=maximum, witness=witness, intersecting_projected_pairs=pairs)


def raw_stl(path):
    """读取本项目二进制 STL 三角形，不经过 SDF 或凸包。"""
    content = path.read_bytes()
    n = int.from_bytes(content[80:84], 'little')
    assert len(content) == 84+n*50
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    return np.frombuffer(content[84:], dtype=dtype)['vertices'].astype(float)


def main():
    """按固定姿态验证安装变换、原表面首触及 SDF 接触，不评价动态稳定性。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists() and any(args.out.iterdir()):
        raise ValueError('Use a new output directory')
    args.out.mkdir(parents=True, exist_ok=True)
    scene = load_scene('scenes/iiwa14_compliant_insertion.yaml')
    geometry = InterfaceGeometry(scene)
    geometry.model.opt.timestep = .001
    original = raw_stl(REPO_ROOT/'assets/iiwa14/assets/dock_1_17_new.STL')
    rotation40 = pin.exp3(np.array([0., 0., np.deg2rad(40.)]))
    transforms = {}
    for label, raw, compiled in [('tool', original@rotation40.T, geometry.tool),
                                 ('target', original, geometry.target)]:
        tree = cKDTree(compiled.reshape(-1, 3))
        error = float(tree.query(raw.reshape(-1, 3))[0].max())
        assert error < 1e-8
        transforms[label] = dict(raw_vs_compiled_vertex_error_m=error)
    plan = [(0., (0., 0.)), (45., (0., 0.)), (4.25, (0., 0.)), (4.25, (.002, -.002))]
    rows = []
    for yaw, xy in plan:
        rotation = geometry.rotation(yaw)
        tool = geometry.tool@rotation.T
        tool[:, :, :2] += xy
        exact = exact_height(tool, geometry.target)
        grid = geometry.required_height(rotation, xy, spacing=.00025)
        quat = pin.Quaternion(rotation).coeffs()[[3, 0, 1, 2]]
        previous, onset, contacts = None, None, []
        for height in np.arange(.08, exact['root_height_m']-.001, -.0001):
            mujoco.mj_resetData(geometry.model, geometry.data)
            geometry.data.qpos[:] = np.r_[xy, height, quat]
            mujoco.mj_forward(geometry.model, geometry.data)
            active = [c for c in geometry.data.contact if c.dist < -1e-6]
            if active:
                onset = float(height)
                contacts = [dict(geoms=[geometry.model.geom(int(c.geom1)).name,
                                       geometry.model.geom(int(c.geom2)).name],
                                 distance_mm=float(c.dist*1000), position_m=c.pos.tolist()) for c in active]
                break
            previous = float(height)
        row = dict(yaw_deg=yaw, xy_m=xy, exact_mesh=exact,
                   grid_height_m=grid, grid_minus_exact_m=grid-exact['root_height_m'],
                   sdf_first_penetration_height_m=onset, previous_no_penetration_height_m=previous,
                   sdf_minus_same_pose_mesh_m=None if onset is None else onset-exact['root_height_m'],
                   contacts=contacts)
        rows.append(row)
        (args.out/'partial.json').write_text(json.dumps(rows, indent=2)+'\n')
        print(yaw, xy, 'mesh', exact['root_height_m'], 'sdf', onset, flush=True)
    result = dict(rows=rows, transforms=transforms, tool_internal_yaw_deg=40.,
        control_to_tool_local_translation_m=scene.tool.pose_pos.tolist(),
        comparison_frame='tool dock root relative to target dock root; internal mount included',
        controlled_frame_height_equals_tool_root_height_plus_m=.015,
        scan_step_m=.0001, penetration_threshold_m=1e-6,
        sdf_iterations=geometry.model.opt.sdf_iterations, sdf_initpoints=geometry.model.opt.sdf_initpoints,
        margin_m=geometry.model.geom_margin.tolist(), gap_m=geometry.model.geom_gap.tolist(),
        mujoco_version=mujoco.__version__, static_only=True, dynamics_stability_verified=False,
        assets_sha256=asset_fingerprints(scene),
        sources_sha256={str(p.relative_to(REPO_ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in [Path(__file__).resolve(), REPO_ROOT/'src/compliant_docking/interface_geometry.py',
                                  scene.path]})
    (args.out/'summary.json').write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
