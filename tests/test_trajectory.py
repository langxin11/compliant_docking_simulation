"""DecoupledQuinticTrajectory 单元测试：端点约束、单调性、不同时长的正确性。"""
import numpy as np
import pytest

from compliant_docking.planning.trajectory import (
    CircleFigure8Trajectory,
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


# ---- CircleFigure8Trajectory：圆+8字跟踪测试轨迹（过渡→竖直圆→过渡→平面8字→保持） ----

# 默认参数下圆心在 START 正下方 0.06 m；f_c·T_c = f_8·T_8 = 1（整周期）
CF8_CENTER = START + np.array([0.0, 0.0, -0.06])
CF8_P1 = CF8_CENTER + np.array([0.0, 0.10, 0.0])  # 圆起点（整圈时圆终点 P2 = P1）


def test_circle_figure8_analytic_matches_finite_difference():
    """随机参数实例：全时程 0.01s 采样，解析 vel/acc 与中心差分 pos 导数一致。

    分段表达式虽在交界处 C2 连续，但中心差分跨越两套浮点计算路径；这里跳过
    极小交界邻域，把有限差分测试聚焦于各段解析导数，边界连续性由专门测试覆盖。
    """
    rng = np.random.default_rng(2026)
    traj = CircleFigure8Trajectory(
        START + rng.uniform(-0.05, 0.05, 3),
        transition_duration=float(rng.uniform(0.8, 2.0)),
        circle_duration=float(rng.uniform(3.0, 6.0)),
        circle_radius=float(rng.uniform(0.05, 0.15)),
        circle_frequency=float(rng.uniform(0.1, 0.4)),
        circle_center_offset=float(rng.uniform(-0.10, -0.02)),
        figure8_duration=float(rng.uniform(3.0, 6.0)),
        figure8_radius_x=float(rng.uniform(0.05, 0.15)),
        figure8_radius_y=float(rng.uniform(0.04, 0.12)),
        figure8_frequency=float(rng.uniform(0.1, 0.4)),
    )
    total = traj.total_duration
    interior_junctions = np.array([seg[2] for seg in traj.segments][:-1])  # t1/t2/t3
    h = 1e-5
    guard = 1e-3

    checked = 0
    for t in np.arange(0.0, total, 0.01):
        if t < guard or t > total - guard:
            continue
        if np.min(np.abs(interior_junctions - t)) < guard:
            continue
        pos_p, vel_p, _ = traj.get_state(t + h)
        pos_m, vel_m, _ = traj.get_state(t - h)
        _, vel, acc = traj.get_state(t)
        np.testing.assert_allclose(
            vel, (pos_p - pos_m) / (2 * h), rtol=1e-4, atol=1e-6, err_msg=f"vel at t={t}")
        np.testing.assert_allclose(
            acc, (vel_p - vel_m) / (2 * h), rtol=1e-4, atol=1e-6, err_msg=f"acc at t={t}")
        checked += 1
    assert checked > 800  # 采样基本覆盖全时程


def test_circle_figure8_boundary_conditions():
    """端点与段交界：t=0 停在起点；交界位置左右极限连续且 vel/acc≈0；
    t≥total 停在 8 字结束点（默认整周期 = 圆心 C）。"""
    traj = CircleFigure8Trajectory(START)
    t_tr, t_c, _ = traj.durations
    total = traj.total_duration

    # t=0（含 t<0 钳制）：pos=start_pos、vel=acc=0
    for t_zero in (0.0, -1.0):
        pos0, vel0, acc0 = traj.get_state(t_zero)
        np.testing.assert_allclose(pos0, START, atol=1e-12)
        np.testing.assert_allclose(vel0, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(acc0, np.zeros(3), atol=1e-12)

    # 四个段交界（T_tr、T_tr+T_c、2T_tr+T_c、total）：位置左右极限连续（atol=1e-12）
    # get_state(tj) 落在后一段（区间 [t0,t1) 约定），nextafter 取前一段极限
    for tj in (t_tr, t_tr + t_c, 2 * t_tr + t_c, total):
        pos_at, _, _ = traj.get_state(tj)
        pos_left, _, _ = traj.get_state(np.nextafter(tj, -np.inf))
        np.testing.assert_allclose(pos_at, pos_left, atol=1e-12, err_msg=f"junction t={tj}")

    # 交界位置即关键路点：P1（圆起点）、P2（圆终点）、C（8 字中心）
    theta_end = 2.0 * np.pi * 0.2 * 5.0
    p2 = CF8_CENTER + 0.10 * np.array([0.0, np.cos(theta_end), np.sin(theta_end)])
    np.testing.assert_allclose(traj.get_state(t_tr)[0], CF8_P1, atol=1e-12)
    np.testing.assert_allclose(traj.get_state(t_tr + t_c)[0], p2, atol=1e-12)
    np.testing.assert_allclose(traj.get_state(2 * t_tr + t_c)[0], CF8_CENTER, atol=1e-12)

    # 交界处五次时间缩放使两侧速度/加速度均为零；此处检查关键边界样本，
    # 左右两侧的完整 C2 对比由下一项专门测试覆盖。
    _, vel_l1, acc_l1 = traj.get_state(np.nextafter(t_tr, -np.inf))  # 过渡1 结束
    np.testing.assert_allclose(vel_l1, np.zeros(3), atol=1e-9)
    np.testing.assert_allclose(acc_l1, np.zeros(3), atol=1e-9)
    _, vel_j2, acc_j2 = traj.get_state(t_tr + t_c)  # 过渡2 起点（u=0，恰为 0）
    np.testing.assert_allclose(vel_j2, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(acc_j2, np.zeros(3), atol=1e-12)
    _, vel_l3, acc_l3 = traj.get_state(np.nextafter(2 * t_tr + t_c, -np.inf))  # 过渡2 结束
    np.testing.assert_allclose(vel_l3, np.zeros(3), atol=1e-9)
    np.testing.assert_allclose(acc_l3, np.zeros(3), atol=1e-9)

    # t ≥ total：停在 8 字结束点（默认参数 f_8·T_8=1 整周期 → 结束点 = 圆心 C）
    for t_hold in (total, total + 5.0):
        pos_h, vel_h, acc_h = traj.get_state(t_hold)
        np.testing.assert_allclose(pos_h, CF8_CENTER, atol=1e-12)
        np.testing.assert_allclose(vel_h, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(acc_h, np.zeros(3), atol=1e-12)


def test_circle_figure8_is_c2_at_every_segment_boundary():
    """圆/8 字相位同样经五次时间缩放，所有段边界应 C2 连续。"""
    traj = CircleFigure8Trajectory(START)
    for _, _, boundary in traj.segments:
        _, vel_at, acc_at = traj.get_state(boundary)
        _, vel_left, acc_left = traj.get_state(np.nextafter(boundary, -np.inf))
        np.testing.assert_allclose(vel_left, vel_at, atol=1e-8)
        np.testing.assert_allclose(acc_left, acc_at, atol=1e-7)


def test_circle_figure8_segment_structure():
    """durations/total_duration/segments 数值与结构正确；start_pos 存副本。"""
    traj = CircleFigure8Trajectory(START)
    assert traj.durations == (1.5, 5.0, 5.0)
    assert traj.total_duration == pytest.approx(13.0, abs=1e-12)
    assert traj.segments == (
        ("过渡1", 0.0, 1.5),
        ("圆周", 1.5, 6.5),
        ("过渡2", 6.5, 8.0),
        ("8字", 8.0, 13.0),
    )

    # start_pos 存副本：外部修改传入数组不影响轨迹
    src = START.copy()
    traj2 = CircleFigure8Trajectory(src)
    src[:] = 99.0
    np.testing.assert_allclose(traj2.start_pos, START, atol=1e-15)
    np.testing.assert_allclose(traj2.get_state(0.0)[0], START, atol=1e-12)


def test_circle_figure8_geometry():
    """默认参数几何：圆周段半径恒 =r、x 恒 =C_x；8 字段 z 恒 =C_z 且 x/y 包络不超 (ax, ay)。"""
    traj = CircleFigure8Trajectory(START)
    r, ax8, ay8 = 0.10, 0.10, 0.07

    # 圆周段 [1.5, 6.5)：到圆心距离恒 = r，x 恒 = C_x
    pos_circle = np.array([traj.get_state(t)[0] for t in np.arange(1.5, 6.5, 0.01)])
    np.testing.assert_allclose(
        np.linalg.norm(pos_circle - CF8_CENTER, axis=1), r, atol=1e-12)
    np.testing.assert_allclose(pos_circle[:, 0], CF8_CENTER[0], atol=1e-15)

    # 8 字段 [8.0, 13.0)：z 恒 = C_z，x/y 包络 ≤ (ax, ay) + 1e-12
    pos_fig = np.array([traj.get_state(t)[0] for t in np.arange(8.0, 13.0, 0.01)])
    np.testing.assert_allclose(pos_fig[:, 2], CF8_CENTER[2], atol=1e-15)
    assert np.all(np.abs(pos_fig[:, 0] - CF8_CENTER[0]) <= ax8 + 1e-12)
    assert np.all(np.abs(pos_fig[:, 1] - CF8_CENTER[1]) <= ay8 + 1e-12)
