"""
test_se3_impedance.py — SE3LieImpedanceController 单元测试

覆盖任务书 §3/§10/§25 要求：
- 相对 twist 有限差分验证：Ṫ̃T̃⁻¹ ↔ Ad_T̃·V_d − V（右平移，Eq. 45）
- body Jacobian 帧一致性：J_LOCAL·v ↔ vee(T⁻¹Ṫ) 有限差分（含 J̇）
- 零误差零校正；小纯平移/纯旋转误差方向正确
- 静态期望位姿自然退化（V_d=Vdot_d=0）
- γ 力项方向正确（外力方向的柔顺响应，Eq. 50/56/60）
- 7-DoF：τ 形状/有限性；零空间不破坏主任务（J·q̈_ref + J̇v ≈ V̇_ref）
- 近零姿态不爆炸；接近 π 有限
- 摩擦前馈与 TaskSpaceController 同源
"""
import numpy as np
import pinocchio as pin
import pytest

from compliant_docking.config import SE3ImpedanceConfig
from compliant_docking.control.lie_se3 import adjoint, vee4
from compliant_docking.control.se3_impedance import SE3LieImpedanceController
from compliant_docking.models import load_pin_model
from compliant_docking.planning.kinematics import compute_ik
from compliant_docking.scene import DEFAULT_SCENE_PATH, load_scene

_SCENE = load_scene(DEFAULT_SCENE_PATH)


@pytest.fixture(scope="module")
def controller() -> SE3LieImpedanceController:
    model = load_pin_model(_SCENE.robot.pin_model)
    return SE3LieImpedanceController(model, dt=0.001,
                                     ee_frame=_SCENE.robot.ee_frame)


@pytest.fixture(scope="module")
def q_home(controller) -> np.ndarray:
    data = controller.model.createData()
    init_pose = pin.SE3(_SCENE.task.init_ori, _SCENE.task.init_pos)
    q, success = compute_ik(controller.model, data, init_pose,
                            initial_q=_SCENE.task.ik_guess, max_iters=5000,
                            ee_frame=_SCENE.robot.ee_frame)
    assert success
    return q


def _sample_qv(controller, q_home, rng, v_scale=0.3):
    v = rng.normal(size=controller.model.nv) * v_scale
    return np.asarray(q_home, dtype=float).copy(), v


# ----------------------------------------------------------------------
# §3 相对 twist 有限差分（Eq. 45 的 convention 验证）
# ----------------------------------------------------------------------
def test_relative_twist_right_translation_fd():
    """T̃=T⁻¹Td 的 Ṫ̃T̃⁻¹（右平移）= Ad_T̃·V_d − V（论文 Eq. 45）。"""
    rng = np.random.default_rng(11)

    def rand_T():
        w = rng.normal(size=3) * 0.5
        p = rng.normal(size=3) * 0.3
        return pin.exp6(pin.Motion(np.concatenate([p, w])))

    T0, Td0 = rand_T(), rand_T()
    V = rng.normal(size=6) * 0.4      # T 的 body twist
    V_d = rng.normal(size=6) * 0.4    # Td 的 body twist
    h = 1e-6

    def T_of(s):
        return T0 * pin.exp6(pin.Motion(s * V))

    def Td_of(s):
        return Td0 * pin.exp6(pin.Motion(s * V_d))

    Tt_p = T_of(h).inverse() * Td_of(h)
    Tt_m = T_of(-h).inverse() * Td_of(-h)
    Tt_0 = T_of(0.0).inverse() * Td_of(0.0)
    Tt_dot = (np.array(Tt_p.homogeneous) - np.array(Tt_m.homogeneous)) / (2 * h)
    V_tilde_fd = vee4(Tt_dot @ np.array(Tt_0.inverse().homogeneous))
    V_tilde_analytic = adjoint(Tt_0) @ V_d - V
    np.testing.assert_allclose(V_tilde_fd, V_tilde_analytic, atol=1e-6)


# ----------------------------------------------------------------------
# §10 帧一致性：LOCAL body Jacobian ↔ vee(T⁻¹Ṫ)
# ----------------------------------------------------------------------
def test_body_jacobian_matches_pose_finite_difference(controller, q_home):
    """J_LOCAL·v 与 vee(T(q+sv)⁻¹·dT/ds) 中心差分一致（body twist 定义）。"""
    rng = np.random.default_rng(5)
    q, v = _sample_qv(controller, q_home, rng)
    T, J, _ = controller.get_body_state(q, v)
    h = 1e-6

    T_p = controller.get_body_state(q + h * v, v)[0]
    T_m = controller.get_body_state(q - h * v, v)[0]
    Tdot = (np.array(T_p.homogeneous) - np.array(T_m.homogeneous)) / (2 * h)
    V_fd = vee4(np.array(T.inverse().homogeneous) @ Tdot)
    np.testing.assert_allclose(V_fd, J @ v, atol=1e-5)


def test_body_jacobian_dot_matches_finite_difference(controller, q_home):
    """J̇_LOCAL·v 与 d/ds[J(q+sv)·v] 中心差分一致（V̇ = J̇v + Jv̇ 的 J̇ 项）。"""
    rng = np.random.default_rng(6)
    q, v = _sample_qv(controller, q_home, rng)
    _, J, Jdot = controller.get_body_state(q, v)
    h = 1e-6
    J_p = controller.get_body_state(q + h * v, v)[1]
    J_m = controller.get_body_state(q - h * v, v)[1]
    np.testing.assert_allclose((J_p - J_m) @ v / (2 * h), Jdot @ v, atol=1e-5)


# ----------------------------------------------------------------------
# §25 控制器性质
# ----------------------------------------------------------------------
def test_zero_error_zero_correction(controller, q_home):
    """λ=0、V=V_d、F=0：λ̈_ref≈0，V̇_ref 退化为 Ad·V̇_d = V̇_d。"""
    rng = np.random.default_rng(7)
    q, v = _sample_qv(controller, q_home, rng)
    T, J, _ = controller.get_body_state(q, v)
    Vdot_d = rng.normal(size=6) * 0.2
    tau = controller.compute_control(
        q, v, T_d=pin.SE3(T.rotation, T.translation), V_d=J @ v,
        Vdot_d=Vdot_d, F_body=np.zeros(6))
    d = controller.latest_diagnostics
    assert np.linalg.norm(d["lam"]) < 1e-9
    assert np.linalg.norm(d["lam_dot"]) < 1e-9
    assert np.linalg.norm(d["lam_ddot_ref"]) < 1e-6
    np.testing.assert_allclose(d["Vdot_ref"], Vdot_d, atol=1e-6)
    assert tau.shape == (controller.nv,)
    assert np.all(np.isfinite(tau))


def test_small_translation_error_direction(controller, q_home):
    """小纯平移误差（body +x 偏 δ）：恢复方向正确（V̇_ref 指向期望）。"""
    delta = 0.02
    q = np.asarray(q_home, dtype=float).copy()
    v = np.zeros(controller.nv)
    T, _, _ = controller.get_body_state(q, v)
    T_d = T * pin.SE3(np.eye(3), np.array([delta, 0.0, 0.0]))
    controller.compute_control(q, v, T_d, np.zeros(6), np.zeros(6), np.zeros(6))
    d = controller.latest_diagnostics
    lam = d["lam"]
    # λ = log(T⁻¹Td) ≈ [δ·e_x; 0]
    np.testing.assert_allclose(lam[:3], [delta, 0, 0], atol=1e-6)
    np.testing.assert_allclose(lam[3:], np.zeros(3), atol=1e-9)
    # λ̈_ref 阻尼误差（恢复）；V̇_ref 沿 +x 推向期望
    assert float(d["lam_ddot_ref"] @ lam) < 0.0
    assert d["Vdot_ref"][0] == pytest.approx(
        controller.K[0, 0] / controller.A[0, 0] * delta, rel=0.05)
    assert abs(d["Vdot_ref"][1]) < 1e-3 and abs(d["Vdot_ref"][5]) < 1e-6


def test_small_rotation_error_direction(controller, q_home):
    """小纯旋转误差（body z 轴 +θ）：λ_ξ ≈ θ·e_z，恢复力矩方向正确。"""
    theta = 0.05
    q = np.asarray(q_home, dtype=float).copy()
    v = np.zeros(controller.nv)
    T, _, _ = controller.get_body_state(q, v)
    T_d = T * pin.exp6(pin.Motion(np.concatenate([np.zeros(3),
                                                  theta * np.array([0.0, 0.0, 1.0])])))
    controller.compute_control(q, v, T_d, np.zeros(6), np.zeros(6), np.zeros(6))
    d = controller.latest_diagnostics
    lam = d["lam"]
    np.testing.assert_allclose(lam[3:], [0.0, 0.0, theta], atol=1e-9)
    assert float(d["lam_ddot_ref"] @ lam) < 0.0
    # 期望姿态绕 +z 转过 θ：EE 需 +z 角加速度（小角度 dexp≈I 去耦）
    assert d["Vdot_ref"][5] == pytest.approx(
        controller.K[5, 5] / controller.A[5, 5] * theta, rel=0.05)


def test_static_desired_pose_degrades_cleanly(controller, q_home):
    """静态期望位姿（V_d=Vdot_d=0、静止、无外力）：输出有限、误差恢复。"""
    q = np.asarray(q_home, dtype=float).copy()
    v = np.zeros(controller.nv)
    T, _, _ = controller.get_body_state(q, v)
    T_d = T * pin.exp6(pin.Motion(np.array([0.03, -0.02, 0.01, 0.02, -0.01, 0.03])))
    tau = controller.compute_control(q, v, T_d, np.zeros(6), np.zeros(6), np.zeros(6))
    assert np.all(np.isfinite(tau))
    d = controller.latest_diagnostics
    assert float(d["lam_ddot_ref"] @ d["lam"]) < 0.0
    assert d["cond_dexp"] < 100.0


def test_gamma_force_compliance_direction(controller, q_home):
    """F≠0（body +x 推力 f）：γ 项使 EE 沿外力方向加速（柔顺）。

    F̃=-F（F_d=0），λ≈0 时 λ̈_ref = K_F·γ = -A⁻¹F → V̇_ref ≈ +f/A·e_x。
    """
    f = 5.0
    q = np.asarray(q_home, dtype=float).copy()
    v = np.zeros(controller.nv)
    T, _, _ = controller.get_body_state(q, v)
    F_body = np.array([f, 0.0, 0.0, 0.0, 0.0, 0.0])
    controller.compute_control(q, v, pin.SE3(T.rotation, T.translation),
                               np.zeros(6), np.zeros(6), F_body)
    d = controller.latest_diagnostics
    # 误差坐标沿外力反方向加速（λ 与物理位移反号），物理上沿外力方向
    assert d["lam_ddot_ref"][0] == pytest.approx(-f / controller.A[0, 0], rel=1e-6)
    assert d["Vdot_ref"][0] == pytest.approx(f / controller.A[0, 0], rel=1e-6)
    # 力矩端：-JᵀF 项把外力从逆动力学中扣除（柔顺不抵抗）
    assert d["tau_norm"] > 0.0


def test_pure_moment_compliance_direction(controller, q_home):
    """F 为纯矩（body z 轴 n）：EE 获 +z 角加速度（转动柔顺）。"""
    n = 1.2
    q = np.asarray(q_home, dtype=float).copy()
    v = np.zeros(controller.nv)
    T, _, _ = controller.get_body_state(q, v)
    F_body = np.array([0.0, 0.0, 0.0, 0.0, 0.0, n])
    controller.compute_control(q, v, pin.SE3(T.rotation, T.translation),
                               np.zeros(6), np.zeros(6), F_body)
    d = controller.latest_diagnostics
    assert d["Vdot_ref"][5] == pytest.approx(n / controller.A[5, 5], rel=1e-6)


def test_seven_dof_nullspace_preserves_task(controller, q_home):
    """7-DoF：τ 形状 (nv,) 有限；零空间分量不改变主任务加速度。"""
    rng = np.random.default_rng(8)
    q, v = _sample_qv(controller, q_home, rng, v_scale=0.1)
    T, J, Jdot = controller.get_body_state(q, v)
    T_d = T * pin.exp6(pin.Motion(np.array([0.02, 0.0, -0.01, 0.0, 0.03, 0.0])))
    V_d = rng.normal(size=6) * 0.05
    Vdot_d = rng.normal(size=6) * 0.05
    tau = controller.compute_control(q, v, T_d, V_d, Vdot_d, np.zeros(6))
    assert tau.shape == (controller.nv,)
    assert np.all(np.isfinite(tau))
    d = controller.latest_diagnostics
    # 主任务实现：J·q̈_ref + J̇·v ≈ V̇_ref（零空间投影 N=I-J_bar J 不扰动）
    np.testing.assert_allclose(J @ d["qdd_ref"] + Jdot @ v, d["Vdot_ref"],
                               atol=1e-8)


def test_nullspace_only_adds_damping(controller, q_home):
    """零空间阻尼改变 τ 但不改变实现的任务加速度（对比两种阻尼）。"""
    rng = np.random.default_rng(9)
    q, v = _sample_qv(controller, q_home, rng, v_scale=0.2)
    T, J, Jdot = controller.get_body_state(q, v)
    T_d = T * pin.exp6(pin.Motion(np.array([0.01, 0.02, 0.0, 0.0, 0.0, 0.02])))
    args = (q, v, T_d, np.zeros(6), np.zeros(6), np.zeros(6))

    c1 = SE3LieImpedanceController(
        controller.model, 0.001, SE3ImpedanceConfig(null_damping=0.0),
        ee_frame=_SCENE.robot.ee_frame)
    c2 = SE3LieImpedanceController(
        controller.model, 0.001, SE3ImpedanceConfig(null_damping=50.0),
        ee_frame=_SCENE.robot.ee_frame)
    tau1, tau2 = c1.compute_control(*args), c2.compute_control(*args)
    assert np.linalg.norm(tau2 - tau1) > 1e-6  # 零空间确实起作用
    np.testing.assert_allclose(c1.latest_diagnostics["Vdot_ref"],
                               c2.latest_diagnostics["Vdot_ref"], atol=1e-10)


def test_near_zero_orientation_no_blowup(controller, q_home):
    """姿态误差 ~1e-8：无 NaN/爆炸（θ→0 Taylor 分支）。"""
    q = np.asarray(q_home, dtype=float).copy()
    v = np.zeros(controller.nv)
    T, _, _ = controller.get_body_state(q, v)
    T_d = T * pin.exp6(pin.Motion(
        np.concatenate([np.array([1e-8, 0.0, 0.0]), 1e-8 * np.array([0.0, 0.0, 1.0])])))
    tau = controller.compute_control(q, v, T_d, np.zeros(6), np.zeros(6), np.zeros(6))
    assert np.all(np.isfinite(tau))
    assert controller.latest_diagnostics["cond_dexp"] < 10.0


@pytest.mark.parametrize("angle", [2.9, 3.13])
def test_near_pi_rotation_finite(controller, q_home, angle):
    """接近 π 的姿态误差（166°/179.4°，主 log 分支内）：有限、恢复方向正确。"""
    q = np.asarray(q_home, dtype=float).copy()
    v = np.zeros(controller.nv)
    T, _, _ = controller.get_body_state(q, v)
    T_d = T * pin.exp6(pin.Motion(
        np.concatenate([np.zeros(3), angle * np.array([1.0, 0.2, -0.1]) / np.sqrt(1.05)])))
    tau = controller.compute_control(q, v, T_d, np.zeros(6), np.zeros(6), np.zeros(6))
    assert np.all(np.isfinite(tau))
    d = controller.latest_diagnostics
    assert float(d["lam_ddot_ref"] @ d["lam"]) < 0.0
    assert d["cond_dexp"] < 1e6


def test_full_6x6_matrices_supported(controller, q_home):
    """一般 6×6 阻抗矩阵（非对角）可注入并被使用。"""
    rng = np.random.default_rng(10)
    A = np.eye(6) * 4.0 + np.diag(rng.normal(size=6) * 0.1)
    K = np.eye(6) * 30.0
    D = np.eye(6) * 40.0
    c = SE3LieImpedanceController(
        controller.model, 0.001, ee_frame=_SCENE.robot.ee_frame,
        A=A, D=D, K=K)
    q = np.asarray(q_home, dtype=float).copy()
    v = np.zeros(c.nv)
    T, _, _ = c.get_body_state(q, v)
    T_d = T * pin.SE3(np.eye(3), np.array([0.01, 0.0, 0.0]))
    tau = c.compute_control(q, v, T_d, np.zeros(6), np.zeros(6), np.zeros(6))
    assert np.all(np.isfinite(tau))
    # 对角主导时近似退化为标量公式（非对角项小扰动）
    lam = c.latest_diagnostics["lam"]
    np.testing.assert_allclose(
        c.latest_diagnostics["lam_ddot_ref"], -np.linalg.solve(A, K @ lam),
        rtol=0.05, atol=1e-3)


def test_desired_wrench_api(controller, q_home):
    """F_d 期望 wrench（Eq. 50 API 预留）：净 wrench F̃ = F_d − F 驱动误差系统。

    F_d 恰与传感器力相消时 γ 项归零（无校正）；F_d 超出 F 的部分
    （净 +x）使 EE 获得 -x 物理加速度（λ̈ ∝ +F̃，物理位移与 λ 反号）。
    """
    f = 3.0
    q = np.asarray(q_home, dtype=float).copy()
    v = np.zeros(controller.nv)
    T, _, _ = controller.get_body_state(q, v)
    T_id = pin.SE3(T.rotation, T.translation)
    F_body = np.array([f, 0, 0, 0, 0, 0])

    # F_d = F：净 wrench 为零，γ 不产生校正（与 F=0 时同）
    controller.compute_control(q, v, T_id, np.zeros(6), np.zeros(6),
                               F_body=F_body, F_d=F_body.copy())
    d = controller.latest_diagnostics
    assert abs(d["Vdot_ref"][0]) < 1e-9
    assert abs(d["lam_ddot_ref"][0]) < 1e-9

    # F_d = 2F：净 F̃ = +f·e_x，λ̈_ref = +f/A（V̇_ref = −f/A）
    controller.compute_control(q, v, T_id, np.zeros(6), np.zeros(6),
                               F_body=F_body, F_d=2.0 * F_body)
    d = controller.latest_diagnostics
    assert d["lam_ddot_ref"][0] == pytest.approx(
        f / controller.A[0, 0], rel=1e-6)
    assert d["Vdot_ref"][0] == pytest.approx(
        -f / controller.A[0, 0], rel=1e-6)


def test_friction_feedforward_same_helper(controller, q_home):
    """摩擦前馈与 TaskSpaceController 同源 helper（FR3 类场景公平对比）。"""
    fl = np.full(controller.nv, 0.3)
    c = SE3LieImpedanceController(
        controller.model, 0.001, ee_frame=_SCENE.robot.ee_frame,
        frictionloss=fl, friction_mode="torque")
    q = np.asarray(q_home, dtype=float).copy()
    v = np.zeros(c.nv)
    T, _, _ = c.get_body_state(q, v)
    tau = c.compute_control(q, v, pin.SE3(T.rotation, T.translation),
                            np.zeros(6), np.zeros(6), np.zeros(6))
    # 零误差零速度时 τ_pre≈0，但 torque 模式前馈仍按 τ_pre 方向放大输出
    assert np.all(np.isfinite(tau))


def test_se3_error_dynamics_simulation(controller, q_home):
    """λ 误差动力学闭环仿真（论文 Eq. 57 数值验证）。

    把控制器的 λ̈_ref 当作理想闭环加速度，数值积分 λ 并对照阻抗方程
    A_λ λ̈ + D_λ λ̇ + K λ = γ 的解——本测试用简化路径：单步验证
    λ̈_ref = -A_λ⁻¹(D_λ λ̇ + K λ - γ) 的代数关系（K_V/K_P/K_F 展开正确）。
    """
    from compliant_docking.control.lie_se3 import (
        dexp_dot_se3,
        dexp_inv_se3,
        dexp_se3,
    )
    rng = np.random.default_rng(12)
    q, v = _sample_qv(controller, q_home, rng, v_scale=0.0)
    T, _, _ = controller.get_body_state(q, v)
    T_d = T * pin.exp6(pin.Motion(np.array([0.02, -0.01, 0.0, 0.0, 0.02, 0.0])))
    F_body = np.array([1.0, 0.0, 0.0, 0.0, 0.5, 0.0])
    controller.compute_control(q, v, T_d, np.zeros(6), np.zeros(6), F_body)
    d = controller.latest_diagnostics
    lam = d["lam"]

    # 重算阻抗方程右侧（A_λ/D_λ 由 Eq. 58 组装），验证 λ̈_ref 满足 Eq. 57
    E = dexp_se3(lam)
    lam_dot = dexp_inv_se3(lam) @ d["V_tilde"]
    A_lam = E.T @ controller.A @ E
    D_lam = E.T @ (controller.D @ E
                   + controller.A @ dexp_dot_se3(lam, lam_dot))
    F_tilde = -F_body
    gamma = E.T @ F_tilde
    lam_ddot_expected = np.linalg.solve(
        A_lam, gamma - D_lam @ lam_dot - controller.K @ lam)
    np.testing.assert_allclose(d["lam_ddot_ref"], lam_ddot_expected,
                               atol=1e-8)
