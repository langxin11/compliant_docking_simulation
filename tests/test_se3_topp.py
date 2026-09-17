"""SE(3)-TOPP 轨迹单元测试：解析时长、边界条件、限幅、测地线几何。"""
import numpy as np
import pinocchio as pin
import pytest

from compliant_docking.planning.se3_topp import (
    SE3ToppTrajectory,
    _topp_duration,
    _topp_profile_exact,
)

# 论文 Table 8 限速：接近 0.10 m/s / 0.20 rad/s / 0.20 m/s²；
# 对接 0.02 m/s / 0.05 rad/s / 0.05 m/s²
LIMITS = dict(
    v_max_approach=0.10, a_max_approach=0.20,
    omega_max_approach=0.20, alpha_max_approach=0.20,
    v_max_docking=0.02, a_max_docking=0.05,
    omega_max_docking=0.05, alpha_max_docking=0.05,
)


def _make(start_z=0.50, stroke=-0.18, standoff=0.10):
    start = np.array([0.0, 0.5, start_z])
    final = start + np.array([0.0, 0.0, stroke])
    return SE3ToppTrajectory(start, np.eye(3), final, np.eye(3),
                             standoff=standoff, **LIMITS)


def test_topp_duration_analytic_cases():
    """解析校验：梯形 T = 1/v + v/a；三角形 T = 2/sqrt(a)。"""
    # 梯形：v=0.2, a=0.5 → T = 5 + 0.4 = 5.4
    assert _topp_duration(0.2, 0.5) == pytest.approx(5.4)
    # 三角形：v_max=0.9 ≥ sqrt(0.5) → T = 2/sqrt(0.5)
    assert _topp_duration(0.9, 0.5) == pytest.approx(2.0 * np.sqrt(2.0))


def test_durations_match_hand_computed_topp():
    """iiwa 场景几何（接近 0.08 m，对接 0.10 m）手工 TOPP 时长复算。

    接近段：ṡ_max=0.10/0.08=1.25，s̈_max=0.20/0.08=2.5，v²/a=0.625<1
    → 梯形 T = 1/1.25 + 1.25/2.5 = 1.3 s
    对接段：ṡ_max=0.02/0.10=0.2，s̈_max=0.05/0.10=0.5，v²/a=0.08<1
    → 梯形 T = 1/0.2 + 0.2/0.5 = 5.4 s
    """
    traj = _make()
    t1, t2 = traj.durations
    assert t1 == pytest.approx(1.3)
    assert t2 == pytest.approx(5.4)
    assert traj.total_duration == pytest.approx(6.7)


def test_rest_to_rest_boundaries_and_limits():
    """边界静止（速度为零）；加速度为 bang-bang 剖面、段边界有界不连续。"""
    traj = _make()
    t1 = traj.durations[0]
    total = traj.total_duration

    for t in (0.0, t1, total):
        _, vel, _ = traj.get_state(t)
        assert np.linalg.norm(vel) < 1e-9  # rest-to-rest：边界速度为零
        # TOPP 的 s̈ 为 bang-bang（±a_max），世界系线加速度有界但不连续，
        # 边界处 |acc| = a_max_lin = 0.2 属预期

    pos_end, vel_end, _ = traj.get_state(total + 5.0)
    pos_final, _, _ = traj.get_state(total)
    np.testing.assert_allclose(pos_end, pos_final, atol=1e-12)
    assert np.linalg.norm(vel_end) == 0.0

    ts = np.linspace(0.0, total, 2001)
    vmax = amax = 0.0
    for t in ts:
        _, vel, acc = traj.get_state(t)
        vmax = max(vmax, float(np.linalg.norm(vel)))
        amax = max(amax, float(np.linalg.norm(acc)))
    assert vmax <= 0.10 + 1e-9   # 接近段线速度上限（全程最宽松界）
    assert amax <= 0.20 + 1e-9   # 接近段线加速度上限


def test_identity_rotation_is_straight_line_through_waypoint():
    """姿态恒定时退化为直线：路径逐点共线，且途经预对接点。"""
    traj = _make()
    start = np.array([0.0, 0.5, 0.50])
    final = np.array([0.0, 0.5, 0.32])
    direction = (final - start) / np.linalg.norm(final - start)
    for t in np.linspace(0.0, traj.total_duration, 50):
        pos, _, _ = traj.get_state(t)
        assert np.linalg.norm(np.cross(pos - start, direction)) < 1e-12

    pos_appr_end, _, _ = traj.get_state(traj.durations[0] - 1e-6)
    assert abs(pos_appr_end[2] - 0.42) < 1e-4  # 预对接点 z = 0.32 + 0.10


def test_rotating_segment_geodesic_and_omega_limit():
    """平移+90° 旋转组合段：角速度限幅成立、姿态路径为测地线（slerp）。"""
    R0 = np.eye(3)
    R1 = np.array(pin.exp3(np.pi / 2 * np.array([0.0, 0.0, 1.0])))
    start = np.array([0.0, 0.0, 0.5])
    final = start + np.array([0.5, 0.0, 0.0])
    traj = SE3ToppTrajectory(
        start, R0, final, R1, standoff=1.0,
        v_max_approach=0.10, a_max_approach=0.20,
        omega_max_approach=0.25, alpha_max_approach=0.25,
        v_max_docking=0.10, a_max_docking=0.20,
        omega_max_docking=0.25, alpha_max_docking=0.25)
    total = traj.total_duration

    # 角速度限幅：相邻位姿差分的角速度 ≤ 0.25 rad/s
    ts = np.linspace(0.0, total, 2001)
    dt = ts[1] - ts[0]
    ang_rate = 0.0
    prev = traj.get_pose(ts[0]).rotation
    for t in ts[1:]:
        R = traj.get_pose(t).rotation
        dR = R @ prev.T
        ang_rate = max(ang_rate, float(np.linalg.norm(pin.log3(dR))) / dt)
        prev = R
    assert ang_rate <= 0.25 + 1e-6

    # 姿态路径 = 测地线（slerp）：T2 姿态同终点 → 旋转在接近段完成，
    # 接近段中点旋转角恰为 45°，接近段末为 90°
    t1 = traj.durations[0]
    R_mid = traj.get_pose(t1 / 2).rotation
    theta_mid = float(np.linalg.norm(pin.log3(R_mid @ R0.T)))
    assert theta_mid == pytest.approx(np.pi / 4, abs=1e-6)
    R_appr_end = traj.get_pose(t1).rotation
    assert float(np.linalg.norm(pin.log3(R_appr_end @ R0.T))) == \
        pytest.approx(np.pi / 2, abs=1e-6)

    # 线速度限幅仍成立（平移分量 0.10 m/s）
    for t in ts:
        _, vel, _ = traj.get_state(t)
        assert np.linalg.norm(vel) <= 0.10 + 1e-9


# ======================================================================
# 有限差分审计（Stage B）：get_state 的速度/加速度必须与 get_pose 的
# 位置中心差分一致；get_motion_state 的 T_d/V_d/Vdot_d 必须满足
# Td⁻¹Ṫd = [V_d] 且 dV_d/dt = Vdot_d（body 量）
# ======================================================================
def _rotating_traj():
    """平移+90° 旋转组合段轨迹（姿态变化段才能暴露加速度耦合项错误）。"""
    R1 = np.array(pin.exp3(np.pi / 2 * np.array([0.0, 0.0, 1.0])))
    start = np.array([0.0, 0.0, 0.5])
    final = start + np.array([0.5, 0.0, 0.0])
    return SE3ToppTrajectory(
        start, np.eye(3), final, R1, standoff=1.0,
        v_max_approach=0.10, a_max_approach=0.20,
        omega_max_approach=0.25, alpha_max_approach=0.25,
        v_max_docking=0.10, a_max_docking=0.20,
        omega_max_docking=0.25, alpha_max_docking=0.25)


def test_get_state_velocity_matches_pose_finite_difference():
    """速度一致性：ṗ 的中心差分 == get_state 的 vel（旋转与纯平移段皆测）。"""
    traj = _rotating_traj()
    h = 1e-5
    for t in np.linspace(0.05, traj.total_duration - 0.05, 37):
        p_p = np.array(traj.get_pose(t + h).translation)
        p_m = np.array(traj.get_pose(t - h).translation)
        _, vel, _ = traj.get_state(t)
        np.testing.assert_allclose((p_p - p_m) / (2 * h), vel,
                                   atol=1e-6, err_msg=f"t={t}")


def test_get_state_acceleration_matches_pose_finite_difference():
    """加速度一致性：p̈ 的中心差分 == get_state 的 acc。

    T(s)=Ta·Exp(ξs) 的世界线加速度应为
    p̈ = R(ω×v)ṡ² + R·ξ_v·s̈，其中 ω=ξ_w ṡ、v=ξ_v ṡ（叉乘项已含 ṡ²，
    不得再乘 ṡ²）。该测试在姿态变化段审计此公式。
    """
    traj = _rotating_traj()
    h = 1e-4
    for t in np.linspace(0.2, traj.total_duration - 0.2, 29):
        p_pp = np.array(traj.get_pose(t + h).translation)
        p_0 = np.array(traj.get_pose(t).translation)
        p_mm = np.array(traj.get_pose(t - h).translation)
        _, _, acc = traj.get_state(t)
        np.testing.assert_allclose((p_pp - 2 * p_0 + p_mm) / h**2, acc,
                                   atol=5e-4, err_msg=f"t={t}")


def test_get_motion_state_body_twist_consistency():
    """get_motion_state 的 V_d 必须是 body twist：vee(Td⁻¹Ṫd) == V_d。"""
    traj = _rotating_traj()
    h = 1e-6
    for t in np.linspace(0.03, traj.total_duration - 0.03, 23):
        T_d, V_d, Vdot_d = traj.get_motion_state(t)
        T_p = traj.get_motion_state(t + h)[0]
        T_m = traj.get_motion_state(t - h)[0]
        Tdot = (np.array(T_p.homogeneous) - np.array(T_m.homogeneous)) / (2 * h)
        Tinv = np.array(T_d.inverse().homogeneous)
        # body twist：[V] = Td⁻¹·Ṫd（不是 Ṫd·Td⁻¹——那是 spatial twist）
        Vb = Tinv @ Tdot
        S = Vb[:3, :3]  # 反对称块，直接提取 ω（不得喂给 log3）
        V_fd = np.concatenate([Vb[:3, 3], [S[2, 1], S[0, 2], S[1, 0]]])
        np.testing.assert_allclose(V_fd, V_d, atol=1e-5, err_msg=f"t={t}")


def test_get_motion_state_twist_derivative_consistency():
    """get_motion_state 的 Vdot_d == d(V_d)/dt（body twist 向量的导数）。"""
    traj = _rotating_traj()
    h = 1e-6
    for t in np.linspace(0.03, traj.total_duration - 0.03, 23):
        _, _, Vdot_d = traj.get_motion_state(t)
        V_p = traj.get_motion_state(t + h)[1]
        V_m = traj.get_motion_state(t - h)[1]
        np.testing.assert_allclose((V_p - V_m) / (2 * h), Vdot_d,
                                   atol=1e-5, err_msg=f"t={t}")


def test_get_motion_state_pose_matches_get_pose_and_rest_boundaries():
    """get_motion_state 的 T_d 与 get_pose 一致；边界处 rest-to-rest。"""
    traj = _rotating_traj()
    for t in np.linspace(0.0, traj.total_duration + 0.5, 17):
        T_d, V_d, Vdot_d = traj.get_motion_state(t)
        T_ref = traj.get_pose(t)
        np.testing.assert_allclose(np.array(T_d.homogeneous),
                                   np.array(T_ref.homogeneous), atol=1e-12)
    # 边界（段切换点）与超时点：速度为零。段内起点的 s̈=±a_max 是 TOPP
    # bang-bang 剖面的合法行为，只有"停驻"（超时/负时刻）参考加速度才为零
    for t in (0.0, traj.durations[0], traj.total_duration,
              traj.total_duration + 1.0, -0.5):
        _, V_d, _ = traj.get_motion_state(t)
        assert np.linalg.norm(V_d) < 1e-12
    for t in (traj.total_duration, traj.total_duration + 1.0, -0.5):
        _, _, Vdot_d = traj.get_motion_state(t)
        assert np.linalg.norm(Vdot_d) < 1e-12


def test_get_motion_state_body_twist_analytic():
    """段内常螺旋的解析关系：V_d = ξ·ṡ、Vdot_d = ξ·s̈（ξ 在段内为常量）。"""
    traj = _rotating_traj()
    segs = traj._segments
    for seg_idx, t_local in ((0, 0.4 * segs[0][5]), (1, 0.5 * segs[1][5])):
        T_a, _, xi, s_dot_max, s_ddot_max, T = segs[seg_idx]
        s, s_dot, s_ddot = _topp_profile_exact(
            t_local, s_dot_max, s_ddot_max, T)
        t = (traj.durations[0] if seg_idx else 0.0) + t_local
        _, V_d, Vdot_d = traj.get_motion_state(t)
        np.testing.assert_allclose(V_d, xi * s_dot, atol=1e-12)
        np.testing.assert_allclose(Vdot_d, xi * s_ddot, atol=1e-12)
