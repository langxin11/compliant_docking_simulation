"""DecoupledQuinticTrajectory 单元测试：端点约束、单调性、不同时长的正确性。"""
import numpy as np
import pytest

from compliant_docking.planning.trajectory import DecoupledQuinticTrajectory

START = np.array([0.0, 0.5, 0.5])
STROKE = np.array([0.0, 0.0, -0.18])
TARGET = START + STROKE


@pytest.mark.parametrize("T", [1.0, 15.0])
class TestQuinticBoundary:
    """端点位置精确、端点速度/加速度为零（自带校验 + 显式 numpy 断言）。"""

    def test_builtin_verification(self, T):
        traj = DecoupledQuinticTrajectory(START, TARGET, T)
        assert traj.verify_boundary_conditions()

    def test_endpoints_exact(self, T):
        traj = DecoupledQuinticTrajectory(START, TARGET, T)

        pos0, vel0, acc0 = traj.get_state(0.0)
        np.testing.assert_allclose(pos0, START, atol=1e-12)
        np.testing.assert_allclose(vel0, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(acc0, np.zeros(3), atol=1e-12)

        posT, velT, accT = traj.get_state(T)
        np.testing.assert_allclose(posT, TARGET, atol=1e-12)
        np.testing.assert_allclose(velT, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(accT, np.zeros(3), atol=1e-12)

    def test_clamped_beyond_T(self, T):
        """t 超过 T 时应被钳制在终点（get_state 内部 clip）。"""
        traj = DecoupledQuinticTrajectory(START, TARGET, T)
        pos_beyond, vel_beyond, acc_beyond = traj.get_state(T + 100.0)
        np.testing.assert_allclose(pos_beyond, TARGET, atol=1e-12)
        np.testing.assert_allclose(vel_beyond, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(acc_beyond, np.zeros(3), atol=1e-12)


@pytest.mark.parametrize("T", [1.0, 15.0])
def test_straight_line_stroke_is_monotonic(T):
    """直线行程：每一轴沿期望方向单调推进（无回退、无超调）。"""
    traj = DecoupledQuinticTrajectory(START, TARGET, T)

    ts = np.linspace(0.0, T, 501)
    positions = np.array([traj.get_state(t)[0] for t in ts])

    diffs = np.diff(positions, axis=0)
    # 平移行程为零的轴（x/y）应始终不动
    assert np.allclose(diffs[:, 0], 0.0, atol=1e-12)
    assert np.allclose(diffs[:, 1], 0.0, atol=1e-12)
    # z 轴向下行程：差分应全部 <= 0（单调递减）
    assert np.all(diffs[:, 2] <= 1e-12)

    # 中间点应落在起点与终点之间（无越界）
    assert np.all(positions[:, 2] <= START[2] + 1e-12)
    assert np.all(positions[:, 2] >= TARGET[2] - 1e-12)


def test_nonzero_multi_axis_stroke_is_monotonic_per_axis():
    """三轴同时运动时，各轴仍单调到达目标。"""
    start = np.array([0.1, -0.2, 0.3])
    target = np.array([-0.1, 0.2, -0.1])
    traj = DecoupledQuinticTrajectory(start, target, 5.0)

    ts = np.linspace(0.0, 5.0, 301)
    positions = np.array([traj.get_state(t)[0] for t in ts])
    diffs = np.diff(positions, axis=0)

    for axis in range(3):
        direction = np.sign(target[axis] - start[axis])
        if direction > 0:
            assert np.all(diffs[:, axis] >= -1e-12)
        elif direction < 0:
            assert np.all(diffs[:, axis] <= 1e-12)
