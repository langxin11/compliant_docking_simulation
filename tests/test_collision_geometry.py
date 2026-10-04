import json
from dataclasses import replace

import mujoco
import numpy as np
import pytest

from compliant_docking.collision_geometry import (
    CONVEX_DIRECTORY,
    connected_meshes,
    inside_closed_mesh,
    oriented_hull,
    point_mesh_distance,
    sampled_solid_error,
    with_convex_interface,
)
from compliant_docking.interface_geometry import InterfaceGeometry, build_interface_pair
from compliant_docking.scene import load_scene


def box(lo, hi):
    vertices = np.array([[x, y, z] for x in [lo[0], hi[0]]
                         for y in [lo[1], hi[1]] for z in [lo[2], hi[2]]])
    return oriented_hull(vertices)


def test_surface_distance_and_winding_include_face_interiors():
    vertices, faces = box([0, 0, 0], [1, 1, 1])
    triangles = vertices[faces]
    points = np.array([[.5, .5, .5], [.5, .5, 1.25], [-.3, -.4, .5]])
    np.testing.assert_allclose(point_mesh_distance(points, triangles), [.5, .25, .5])
    np.testing.assert_array_equal(inside_closed_mesh(points, triangles), [True, False, False])
    assert sampled_solid_error(triangles, [(vertices, faces)])["sampled_excess_m"] < 1e-12


def test_convex_hull_filling_a_recess_is_detected():
    # An L-shaped union. Internal faces in the original triangle soup cancel
    # their signed winding; a convex hull fills the missing top-right corner.
    first = box([0, 0, 0], [2, 1, 1])
    second = box([0, 1, 0], [1, 2, 1])
    original = np.concatenate([v[f] for v, f in [first, second]])
    filled = oriented_hull(np.concatenate([first[0], second[0]]))
    error = sampled_solid_error(original, [filled])
    assert error["sampled_excess_m"] == pytest.approx(1/3)
    correct = sampled_solid_error(original, [first, second])
    assert correct["sampled_excess_m"] < 1e-12
    assert correct["sampled_missing_m"] < 1e-12
    assert len(connected_meshes(np.concatenate([first[0][first[1]], second[0][second[1]]]+[original+[10, 0, 0]]))) == 2


def test_convex_fragments_preserve_reference_mount_mass_and_sensor():
    base = load_scene('scenes/iiwa14_compliant_insertion.yaml')
    convex = with_convex_interface(base)
    assert convex.tool.mjcf.parent == CONVEX_DIRECTORY
    original_model, convex_model = build_interface_pair(base), build_interface_pair(convex)
    for field in ['body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'body_pos', 'body_quat',
                  'body_parentid', 'site_pos', 'site_quat']:
        np.testing.assert_allclose(getattr(convex_model, field), getattr(original_model, field), atol=1e-12)
    original, modified = InterfaceGeometry(base), InterfaceGeometry(convex)
    np.testing.assert_allclose(original.tool, modified.tool, atol=1e-9)
    np.testing.assert_allclose(original.target, modified.target, atol=1e-9)
    assert not np.any(convex_model.geom_type == int(mujoco.mjtGeom.mjGEOM_SDF))
    for name in ['tool_dock_geom', 'target_dock_geom', 'tool_dock_visual', 'target_dock_visual']:
        geom = convex_model.geom(name).id
        assert convex_model.geom_contype[geom] == convex_model.geom_conaffinity[geom] == 0
    assert (CONVEX_DIRECTORY/'manifest.json').exists()
    assert json.loads((CONVEX_DIRECTORY/'manifest.json').read_text())["validation"]["status"] == "PASS"


def test_convex_selection_rejects_a_different_interface():
    base = load_scene('scenes/iiwa14_compliant_insertion.yaml')
    wrong = replace(base, target=replace(base.target, mjcf=base.tool.mjcf))
    with pytest.raises(ValueError, match='different interface'):
        with_convex_interface(wrong)
