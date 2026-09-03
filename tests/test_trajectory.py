"""DecoupledQuinticTrajectory 单元测试：端点约束、单调性、不同时长的正确性。"""
import numpy as np
import pytest

from compliant_docking.planning.trajectory import (
    DecoupledQuinticTrajectory,
    TwoPhaseDockingTrajectory,
)

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


# ---- TwoPhaseDockingTrajectory：两段式对接轨迹（接近段 + 对接段） ----

# 两段轨迹测试几何（垂直下压对接）：P0 z=0.5 → Pf z=0.32，总行程 0.18 m，
# standoff=0.06 → 预对接点 z=0.38；接近段实际长度 L1 = 0.5 - 0.38 = 0.12 m，
# 对接段长度 L2 = standoff = 0.06 m
TP_START = np.array([0.0, 0.5, 0.5])
TP_FINAL = np.array([0.0, 0.5, 0.32])
TP_STANDOFF = 0.06
TP_PRE_DOCK = np.array([0.0, 0.5, 0.38])
TP_L1 = 0.12  # ‖pre_dock - start‖
TP_L2 = 0.06  # ‖final - pre_dock‖
TP_V_APP, TP_A_APP = 0.10, 0.20
TP_V_DOCK, TP_A_DOCK = 0.02, 0.05


def _make_two_phase() -> TwoPhaseDockingTrajectory:
    return TwoPhaseDockingTrajectory(
        TP_START, TP_FINAL, standoff=TP_STANDOFF,
        v_max_approach=TP_V_APP, a_max_approach=TP_A_APP,
        v_max_docking=TP_V_DOCK, a_max_docking=TP_A_DOCK,
    )


class TestTwoPhaseConstruction:
    """预对接点几何与两段时长反推（T = max(1.875L/v_max, sqrt(5.7735L/a_max))）。"""

    def test_pre_dock_and_axis(self):
        traj = _make_two_phase()
        np.testing.assert_allclose(traj.pre_dock_pos, TP_PRE_DOCK, atol=1e-12)
        # 接近轴 = 推进方向反向（"上方"）
        np.testing.assert_allclose(traj.axis, [0.0, 0.0, 1.0], atol=1e-12)

    def test_durations_from_limits(self):
        traj = _make_two_phase()
        t1, t2 = traj.durations

        # L1=0.12: max(1.875*0.12/0.10, sqrt(5.7735*0.12/0.20)) = max(2.25, 1.8612) = 2.25
        t1_expected = max(1.875 * TP_L1 / TP_V_APP, np.sqrt(5.7735 * TP_L1 / TP_A_APP))
        # L2=0.06: max(1.875*0.06/0.02, sqrt(5.7735*0.06/0.05)) = max(5.625, 2.6321) = 5.625
        t2_expected = max(1.875 * TP_L2 / TP_V_DOCK, np.sqrt(5.7735 * TP_L2 / TP_A_DOCK))

        assert t1 == pytest.approx(t1_expected, abs=1e-12)
        assert t1 == pytest.approx(2.25, abs=1e-9)  # 速度限起作用
        assert t2 == pytest.approx(t2_expected, abs=1e-12)
        assert t2 == pytest.approx(5.625, abs=1e-9)  # 速度限起作用
        assert traj.total_duration == pytest.approx(t1 + t2, abs=1e-12)
        assert traj.total_duration == pytest.approx(7.875, abs=1e-9)


class TestTwoPhaseLimits:
    """数值扫描全程：各段峰值速度/加速度不超限，且与五次解析峰值一致。"""

    def test_velocity_and_acceleration_limits_scan(self):
        traj = _make_two_phase()
        t1, t2 = traj.durations

        dt = 0.01
        ts = np.arange(0.0, traj.total_duration + dt / 2, dt)
        speeds = np.array([np.linalg.norm(traj.get_state(t)[1]) for t in ts])
        accels = np.array([np.linalg.norm(traj.get_state(t)[2]) for t in ts])

        in_approach = ts < t1
        in_docking = ~in_approach

        v_app = speeds[in_approach].max()
        v_dock = speeds[in_docking].max()
        a_app = accels[in_approach].max()
        a_dock = accels[in_docking].max()

        # 硬上限：各段峰值 ≤ 对应限速（留 1e-6 相对容差）
        assert v_app <= TP_V_APP * (1 + 1e-6)
        assert v_dock <= TP_V_DOCK * (1 + 1e-6)
        assert a_app <= TP_A_APP * (1 + 1e-6)
        assert a_dock <= TP_A_DOCK * (1 + 1e-6)

        # 与解析峰值一致：v_peak = 1.875L/T、a_peak = 5.7735L/T²
        # （速度限起作用时 v_peak 精确等于 v_max；扫描 0.01s 步长应命中峰值附近）
        assert v_app == pytest.approx(1.875 * TP_L1 / t1, rel=1e-3)
        assert v_dock == pytest.approx(1.875 * TP_L2 / t2, rel=1e-3)
        assert a_app == pytest.approx(5.7735 * TP_L1 / t1**2, rel=5e-3)
        assert a_dock == pytest.approx(5.7735 * TP_L2 / t2**2, rel=5e-3)


class TestTwoPhaseWaypoints:
    """路径点通过与接合点 C2 连续（rest-to-rest 自然衔接）。"""

    def test_waypoints_and_segment_junction(self):
        traj = _make_two_phase()
        t1, t2 = traj.durations

        # 起点（含 t<0 钳制）
        pos0, vel0, acc0 = traj.get_state(0.0)
        np.testing.assert_allclose(pos0, TP_START, atol=1e-12)
        np.testing.assert_allclose(vel0, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(acc0, np.zeros(3), atol=1e-12)

        pos_neg, vel_neg, acc_neg = traj.get_state(-1.0)
        np.testing.assert_allclose(pos_neg, TP_START, atol=1e-12)
        np.testing.assert_allclose(vel_neg, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(acc_neg, np.zeros(3), atol=1e-12)

        # 预对接点：t = t1 处位置精确，速度/加速度 ≈ 0（段间瞬时停顿）
        pos1, vel1, acc1 = traj.get_state(t1)
        np.testing.assert_allclose(pos1, TP_PRE_DOCK, atol=1e-9)
        np.testing.assert_allclose(vel1, np.zeros(3), atol=1e-9)
        np.testing.assert_allclose(acc1, np.zeros(3), atol=1e-9)

        # 从接近段一侧逼近接合点同样应停在预对接点（位置连续）
        pos1l, vel1l, _ = traj.get_state(t1 - 1e-9)
        np.testing.assert_allclose(pos1l, TP_PRE_DOCK, atol=1e-9)
        np.testing.assert_allclose(vel1l, np.zeros(3), atol=1e-6)

        # 终点（含超时钳制）
        posf, velf, accf = traj.get_state(t1 + t2)
        np.testing.assert_allclose(posf, TP_FINAL, atol=1e-12)
        np.testing.assert_allclose(velf, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(accf, np.zeros(3), atol=1e-12)

        posb, velb, accb = traj.get_state(t1 + t2 + 10.0)
        np.testing.assert_allclose(posb, TP_FINAL, atol=1e-12)
        np.testing.assert_allclose(velb, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(accb, np.zeros(3), atol=1e-12)

    def test_path_is_monotonic_descent(self):
        """垂直下压：z 全程单调下降且不越过 final（两段都朝目标推进）。"""
        traj = _make_two_phase()
        ts = np.linspace(0.0, traj.total_duration, 801)
        positions = np.array([traj.get_state(t)[0] for t in ts])

        assert np.allclose(positions[:, :2], TP_START[:2], atol=1e-12)
        assert np.all(np.diff(positions[:, 2]) <= 1e-12)
        assert positions[:, 2].max() <= TP_START[2] + 1e-12
        assert positions[:, 2].min() >= TP_FINAL[2] - 1e-12
