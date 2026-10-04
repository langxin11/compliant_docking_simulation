import numpy as np
import pytest

from compliant_docking.interface_geometry import InterfaceGeometry, vertical_envelope
from compliant_docking.scene import load_scene


def test_triangle_ray_heights_with_vertical_and_sloped_faces():
    triangles = np.array([[[0, 0, 1], [1, 0, 2], [0, 1, 3]],
                          [[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                          [[0, 0, 0], [0, 0, 1], [0, 1, 0]]], dtype=float)
    x, y = np.array([.1, .2, 2.]), np.array([.1, .2])
    top = vertical_envelope(triangles, x, y, upper=True)
    bottom = vertical_envelope(triangles, x, y, upper=False)
    np.testing.assert_allclose(top[:, :2], [[1.3, 1.4], [1.5, 1.6]])
    np.testing.assert_allclose(bottom[:, :2], 0.)
    assert np.all(np.isneginf(top[:, 2]))
    assert np.all(np.isposinf(bottom[:, 2]))


def test_required_height_matches_known_prism_and_xy_translation():
    scene = load_scene('scenes/iiwa14_compliant_insertion.yaml')
    geometry = InterfaceGeometry(scene)
    # Independent rectangular surfaces: target z=2cm, flipped tool bottom=-3cm.
    plane = np.array([[[-.02, -.02, 0], [.02, -.02, 0], [.02, .02, 0]],
                      [[-.02, -.02, 0], [.02, .02, 0], [-.02, .02, 0]]])
    geometry.target = plane + [0, 0, .02]
    geometry.tool = plane + [0, 0, .03]
    geometry._grids.clear()
    assert geometry.required_height(geometry.rotation(0)) == pytest.approx(.05)
    assert geometry.required_height(geometry.rotation(10), xy=(.003, 0)) == pytest.approx(.05)
    with pytest.raises(ValueError, match="no sampled XY overlap"):
        geometry.required_height(geometry.rotation(0), xy=(1, 0))


def test_compiled_mesh_restores_physical_root_coordinates():
    geometry = InterfaceGeometry(load_scene('scenes/iiwa14_compliant_insertion.yaml'))
    vertices = geometry.target.reshape(-1, 3)
    # These millimetre-scale CAD bounds are in the fragment root, not its
    # recentered principal-inertia mesh coordinates (whose z spans +/-55mm).
    assert vertices[:, 2].min() == pytest.approx(0., abs=1e-7)
    assert vertices[:, 2].max() == pytest.approx(.0335, abs=1e-7)
    assert geometry.required_height(geometry.rotation(5), spacing=.001) < .049
    assert geometry.required_height(geometry.rotation(45), spacing=.001) > .066
