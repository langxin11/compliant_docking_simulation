"""面片交集首触算法用解析平面验证，防止把投影采样误差当作 SDF 偏差。"""
import numpy as np
import pytest

from experiments.models_interfaces.crown_sdf_onset_check import exact_height


def test_parallel_triangles_have_known_vertical_gap():
    triangle = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
    target = triangle.copy()
    target[:, :, 2] = 2.
    assert exact_height(triangle, target)['root_height_m'] == pytest.approx(2.)
    assert exact_height(triangle[:, ::-1], target[:, ::-1])['root_height_m'] == pytest.approx(2.)


def test_maximum_at_edge_intersection_not_mesh_vertex():
    tool = np.array([[[0., 0., 0.], [2., 0., 0.], [0., 2., 0.]]])
    # target z=x；交集最右点 (1.5,0.5) 是两条边的交点，而非任一原三角形顶点。
    target = np.array([[[.5, .5, .5], [2., .5, 2.], [.5, 2., .5]]])
    result = exact_height(tool, target)
    assert result['root_height_m'] == pytest.approx(1.5)
    np.testing.assert_allclose(result['witness']['xy_m'], [1.5, .5])


def test_no_projected_overlap_is_rejected():
    tool = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
    with pytest.raises(AssertionError):
        exact_height(tool, tool+np.array([3., 3., 0.]))
