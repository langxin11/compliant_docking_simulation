"""MuJoCo 场景背景与视频轨迹叠加测试。"""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from compliant_docking.scene import load_scene
from compliant_docking.simulation.mujoco_env import MujRobot

REPO_ROOT = Path(__file__).resolve().parents[1]


def _fr3_robot() -> MujRobot:
    scene = load_scene(REPO_ROOT / "scenes" / "fr3_docking.yaml")
    return MujRobot(
        model=scene.build_mjmodel(),
        render=False,
        record=False,
        target_pos=scene.task.init_pos + scene.task.stroke,
        eef_body=scene.eef_body,
    )


def test_fr3_scene_has_gradient_skybox():
    """FR3 不应回落到默认黑色背景。"""
    robot = _fr3_robot()
    assert np.any(robot.model.tex_type == mujoco.mjtTexture.mjTEXTURE_SKYBOX)


def test_trajectory_overlay_downsamples_and_adds_geometries():
    robot = _fr3_robot()
    path = np.column_stack([
        np.linspace(0.0, 1.0, 50),
        np.zeros(50),
        np.linspace(0.5, 0.3, 50),
    ])
    robot.set_trajectory_visualization(path, max_points=10)
    robot.set_desired_position([0.2, 0.0, 0.4])
    robot.actual_path = [np.array([0.0, 0.0, 0.5]), np.array([0.1, 0.0, 0.45])]

    render_scene = mujoco.MjvScene(robot.model, maxgeom=64)
    robot._add_trajectory_overlays(render_scene)

    assert robot.planned_path.shape == (10, 3)
    # 10 个规划路径点 + 2 个实际轨迹采样点 + 1 个期望点。
    assert render_scene.ngeom == 13
    desired_geom = render_scene.geoms[render_scene.ngeom - 1]
    assert desired_geom.type == mujoco.mjtGeom.mjGEOM_SPHERE
    np.testing.assert_allclose(desired_geom.pos, [0.2, 0.0, 0.4])


@pytest.mark.parametrize("bad_path", [np.zeros(3), np.zeros((4, 2)), [[0.0, np.nan, 0.0]]])
def test_trajectory_overlay_rejects_invalid_paths(bad_path):
    robot = _fr3_robot()
    with pytest.raises(ValueError):
        robot.set_trajectory_visualization(bad_path)
