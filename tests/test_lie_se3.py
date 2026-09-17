"""
test_lie_se3.py — SE(3)/SO(3) dexp 工具集的数学单元测试

以有限差分、矩阵恒等式、共轭作用与功率守恒为 oracle，逐条验证
Kim et al. 2025（T-RO, Vol. 41）Section II-C/II-D 的闭式公式：

- 逆一致性：dexp @ dexp⁻¹ = I（SO(3) 与 SE(3)，覆盖 0/小角度/中等/近 π）
- 右平凡化微分：vee(Ṫ T⁻¹) = dexp(λ) λ̇（不是 T⁻¹Ṫ 的左平凡化版本！）
- 逆微分：λ̇ = dexp⁻¹(λ) V，用 Log(Exp(λ)·Exp(tV)) 的差分验证
- d/dt dexp 与 d/dt dexp⁻¹ 的解析实现 vs 中心差分；并交叉验证链式法则
  d(M⁻¹)/dt = -M⁻¹ Ṁ M⁻¹
- Adjoint：hat(Ad_T V) = T hat(V) T⁻¹（共轭）+ Pinocchio oracle
- wrench 对偶：V'ᵀF' = VᵀF 与论文 Eq. 56 λ̇ᵀγ = ṼᵀF̃
- θ→0 数值稳定（0, 1e-10, 1e-8, 1e-6, 1e-4 无 NaN/inf，且收敛到
  论文 Eq. 42-43 的解析极限）
- SE(3) dexp 的旋转-平移耦合块非零（不得退化为 block_diag）
"""
import numpy as np
import pinocchio as pin
import pytest

from compliant_docking.control.lie_se3 import (
    ad6,
    adjoint,
    adjoint_wrench,
    dexp_inv_dot_se3,
    dexp_inv_dot_so3,
    dexp_inv_se3,
    dexp_inv_so3,
    dexp_dot_se3,
    dexp_dot_so3,
    dexp_se3,
    dexp_so3,
    hat4,
    skew3,
    vee4,
)

# 中心差分步长：指数坐标量级 ~1，h=1e-6 时截断误差 ~h² ≈ 1e-12，
# 舍入误差 ~eps/h ≈ 1e-10，总量级 1e-9 以内
_FD_H = 1e-6


def _rand_lam(rng, theta, trans_scale=0.3):
    """构造旋转角精确为 theta、平移随机尺度的 λ = [η; ξ]。"""
    u = rng.normal(size=3)
    u /= np.linalg.norm(u)
    xi = theta * u
    eta = rng.normal(size=3) * trans_scale
    return np.concatenate([eta, xi])


def _sample_lambdas(rng):
    """覆盖 0 / 小角度 / 中等 / 近 π（不越过主 log 分支）。"""
    return [
        np.zeros(6),
        _rand_lam(rng, 1e-4),
        _rand_lam(rng, 0.3),
        _rand_lam(rng, 1.2),
        _rand_lam(rng, np.pi - 1e-3),
    ]


# ----------------------------------------------------------------------
# 1. 逆一致性
# ----------------------------------------------------------------------
@pytest.mark.parametrize("theta", [0.0, 1e-4, 0.3, 1.2, np.pi - 1e-3])
def test_so3_inverse_consistency(rng_seeded, theta):
    lam = _rand_lam(rng_seeded, theta)
    xi = lam[3:]
    E, Einv = dexp_so3(xi), dexp_inv_so3(xi)
    np.testing.assert_allclose(E @ Einv, np.eye(3), atol=1e-10)
    np.testing.assert_allclose(Einv @ E, np.eye(3), atol=1e-10)


@pytest.mark.parametrize("idx", range(5))
def test_se3_inverse_consistency(rng_seeded, idx):
    lam = _sample_lambdas(rng_seeded)[idx]
    E, Einv = dexp_se3(lam), dexp_inv_se3(lam)
    np.testing.assert_allclose(E @ Einv, np.eye(6), atol=1e-9)
    np.testing.assert_allclose(Einv @ E, np.eye(6), atol=1e-9)


def test_se3_dexp_translation_coupling_present():
    """一般 λ 下 C_ξ(η) ≠ 0：dexp 不得退化为 block_diag(dexp_ξ, dexp_ξ)。"""
    lam = np.array([0.2, -0.1, 0.15, 0.1, 0.2, 0.25])
    E = dexp_se3(lam)
    assert np.linalg.norm(E[:3, 3:]) > 1e-6
    E_decoupled = np.zeros((6, 6))
    E3 = dexp_so3(lam[3:])
    E_decoupled[:3, :3] = E3
    E_decoupled[3:, 3:] = E3
    assert np.linalg.norm(E - E_decoupled) > 1e-3


def test_dexp_exp6_translation_identity():
    """论文 Eq. 10：Exp(λ) 的平移部分 = dexp_ξ·η（用 pin.exp6 做 oracle）。"""
    rng = np.random.default_rng(7)
    for _ in range(10):
        lam = _rand_lam(rng, rng.uniform(0.05, 3.0))
        T = pin.exp6(pin.Motion(lam))
        np.testing.assert_allclose(
            np.array(T.translation), dexp_so3(lam[3:]) @ lam[:3], atol=1e-10)


# ----------------------------------------------------------------------
# 2. 右平凡化微分关系（Ṫ T⁻¹，不是 T⁻¹Ṫ）
# ----------------------------------------------------------------------
def test_right_trivialized_differential_so3(rng_seeded):
    """R(t)=Exp(ξ+tξ̇)：vee(Ṙ Rᵀ) = dexp_ξ ξ̇（空间角速度，右平凡化）。"""
    rng = rng_seeded
    for theta in (0.3, 1.5, np.pi - 1e-2):
        lam = _rand_lam(rng, theta)
        xi, xi_dot = lam[3:], rng.normal(size=3)

        def R_of(s):
            return np.array(pin.exp3(xi + s * xi_dot))

        h = _FD_H
        Rdot = (R_of(h) - R_of(-h)) / (2.0 * h)
        w_spatial = vee4(np.block([[Rdot @ np.linalg.inv(R_of(0.0)), np.zeros((3, 1))],
                                   [np.zeros((1, 3)), 0.0]]))[3:]
        np.testing.assert_allclose(w_spatial, dexp_so3(xi) @ xi_dot, atol=1e-7)


@pytest.mark.parametrize("idx", range(5))
def test_right_trivialized_differential_se3(rng_seeded, idx):
    """T(t)=Exp(λ+tλ̇)：vee(Ṫ T⁻¹) = dexp_λ λ̇（右平凡化/空间 twist）。"""
    rng = rng_seeded
    lam = _sample_lambdas(rng)[idx]
    lam_dot = np.concatenate([rng.normal(size=3) * 0.5, rng.normal(size=3) * 0.5])

    def T_of(s):
        return pin.exp6(pin.Motion(lam + s * lam_dot))

    h = _FD_H
    Tdot = (np.array(T_of(h).homogeneous) - np.array(T_of(-h).homogeneous)) / (2 * h)
    Tinv = np.array(T_of(0.0).inverse().homogeneous)
    V_spatial = vee4(Tdot @ Tinv)
    np.testing.assert_allclose(V_spatial, dexp_se3(lam) @ lam_dot, atol=1e-7)


# ----------------------------------------------------------------------
# 3. 逆微分关系
# ----------------------------------------------------------------------
@pytest.mark.parametrize("idx", range(5))
def test_inverse_differential_se3(rng_seeded, idx):
    """λ̇ = dexp⁻¹(λ)·V（空间 twist V）：用 Log(Exp(λ)·Exp(tV)) 差分验证。"""
    rng = rng_seeded
    lam = _sample_lambdas(rng)[idx]
    V = np.concatenate([rng.normal(size=3) * 0.4, rng.normal(size=3) * 0.4])
    T0 = pin.exp6(pin.Motion(lam))

    # 路径 T(t) = Exp(tV)·T0 满足 Ṫ = [V]T（右平凡化/spatial twist 恰为 V）；
    # 若误用右乘 T0·Exp(tV)，得到的是 Ad_{T0}V，会测到另一套 convention。
    h = 1e-6
    Tp = pin.exp6(pin.Motion(h * V)) * T0
    Tm = pin.exp6(pin.Motion(-h * V)) * T0
    lam_dot_fd = (pin.log6(Tp).vector - pin.log6(Tm).vector) / (2 * h)
    np.testing.assert_allclose(lam_dot_fd, dexp_inv_se3(lam) @ V, atol=1e-6)


def test_inverse_differential_so3(rng_seeded):
    rng = rng_seeded
    xi = _rand_lam(rng, 1.0)[3:]
    w = rng.normal(size=3) * 0.5
    R0 = pin.exp3(xi)
    h = 1e-6
    Rp = pin.exp3(h * w) @ R0
    Rm = pin.exp3(-h * w) @ R0
    xi_dot_fd = (pin.log3(Rp) - pin.log3(Rm)) / (2 * h)
    np.testing.assert_allclose(xi_dot_fd, dexp_inv_so3(xi) @ w, atol=1e-6)


# ----------------------------------------------------------------------
# 4. dexp / dexp⁻¹ 的时间导数（解析 vs 中心差分 + 链式法则交叉验证）
# ----------------------------------------------------------------------
@pytest.mark.parametrize("idx", range(5))
def test_dexp_dot_se3_central_difference(rng_seeded, idx):
    rng = rng_seeded
    lam = _sample_lambdas(rng)[idx]
    lam_dot = np.concatenate([rng.normal(size=3) * 0.7, rng.normal(size=3) * 0.7])
    h = _FD_H
    dexp_fd = (dexp_se3(lam + h * lam_dot) - dexp_se3(lam - h * lam_dot)) / (2 * h)
    np.testing.assert_allclose(dexp_fd, dexp_dot_se3(lam, lam_dot), atol=1e-7)


@pytest.mark.parametrize("idx", range(5))
def test_dexp_inv_dot_se3_central_difference(rng_seeded, idx):
    rng = rng_seeded
    lam = _sample_lambdas(rng)[idx]
    lam_dot = np.concatenate([rng.normal(size=3) * 0.7, rng.normal(size=3) * 0.7])
    h = _FD_H
    fd = (dexp_inv_se3(lam + h * lam_dot) - dexp_inv_se3(lam - h * lam_dot)) / (2 * h)
    np.testing.assert_allclose(fd, dexp_inv_dot_se3(lam, lam_dot), atol=1e-7)


@pytest.mark.parametrize("idx", range(5))
def test_dexp_dot_so3_central_difference(rng_seeded, idx):
    rng = rng_seeded
    lam = _sample_lambdas(rng)[idx]
    xi, xi_dot = lam[3:], rng.normal(size=3) * 0.7
    h = _FD_H
    fd = (dexp_so3(xi + h * xi_dot) - dexp_so3(xi - h * xi_dot)) / (2 * h)
    np.testing.assert_allclose(fd, dexp_dot_so3(xi, xi_dot), atol=1e-7)


@pytest.mark.parametrize("idx", range(5))
def test_dexp_inv_dot_so3_central_difference(rng_seeded, idx):
    rng = rng_seeded
    lam = _sample_lambdas(rng)[idx]
    xi, xi_dot = lam[3:], rng.normal(size=3) * 0.7
    h = _FD_H
    fd = (dexp_inv_so3(xi + h * xi_dot) - dexp_inv_so3(xi - h * xi_dot)) / (2 * h)
    np.testing.assert_allclose(fd, dexp_inv_dot_so3(xi, xi_dot), atol=1e-7)


@pytest.mark.parametrize("idx", range(5))
def test_inv_dot_chain_rule_cross_check(rng_seeded, idx):
    """链式法则交叉验证：d(M⁻¹)/dt = -M⁻¹ Ṁ M⁻¹。"""
    rng = rng_seeded
    lam = _sample_lambdas(rng)[idx]
    lam_dot = np.concatenate([rng.normal(size=3) * 0.7, rng.normal(size=3) * 0.7])
    E_inv, E_dot = dexp_inv_se3(lam), dexp_dot_se3(lam, lam_dot)
    expected = -E_inv @ E_dot @ E_inv
    np.testing.assert_allclose(expected, dexp_inv_dot_se3(lam, lam_dot), atol=1e-9)


# ----------------------------------------------------------------------
# 5. Adjoint
# ----------------------------------------------------------------------
def test_adjoint_conjugation(rng_seeded):
    """hat(Ad_T V) = T hat(V) T⁻¹（SE(3) 共轭作用，论文 Eq. 5-6）。"""
    rng = rng_seeded
    for _ in range(10):
        T = pin.exp6(pin.Motion(_rand_lam(rng, rng.uniform(0.1, 2.5))))
        V = np.concatenate([rng.normal(size=3), rng.normal(size=3)])
        T4 = np.array(T.homogeneous)
        lhs = hat4(adjoint(T) @ V)
        rhs = T4 @ hat4(V) @ np.linalg.inv(T4)
        np.testing.assert_allclose(lhs, rhs, atol=1e-10)


def test_adjoint_matches_pinocchio(rng_seeded):
    """Pinocchio oracle：SE3.act(Motion) 即 Ad_T V（[v; ω] 顺序一致）。"""
    rng = rng_seeded
    for _ in range(10):
        T = pin.exp6(pin.Motion(_rand_lam(rng, rng.uniform(0.1, 2.5))))
        V = np.concatenate([rng.normal(size=3), rng.normal(size=3)])
        np.testing.assert_allclose(
            np.array(T.act(pin.Motion(V)).vector), adjoint(T) @ V, atol=1e-10)


def test_ad6_matches_adjoint_infinitesimal():
    """Ad_Exp(tV) = I + t·ad_V + O(t²)：ad 是 Ad 的无穷小生成元。"""
    rng = np.random.default_rng(3)
    V = np.concatenate([rng.normal(size=3), rng.normal(size=3)])
    h = 1e-5
    lhs = (adjoint(pin.exp6(pin.Motion(h * V)))
           - adjoint(pin.exp6(pin.Motion(-h * V)))) / (2 * h)
    np.testing.assert_allclose(lhs, ad6(V), atol=1e-9)


# ----------------------------------------------------------------------
# 6. 虚功率 / wrench 对偶
# ----------------------------------------------------------------------
def test_wrench_duality_power_invariance(rng_seeded):
    """V' = Ad_T V、F' = Ad_T^{-T} F 时 V'ᵀF' = VᵀF（功率不变）。"""
    rng = rng_seeded
    for _ in range(10):
        T = pin.exp6(pin.Motion(_rand_lam(rng, rng.uniform(0.1, 2.5))))
        V = np.concatenate([rng.normal(size=3), rng.normal(size=3)])
        F = np.concatenate([rng.normal(size=3), rng.normal(size=3)])
        V2 = adjoint(T) @ V
        F2 = adjoint_wrench(T) @ F
        np.testing.assert_allclose(V2 @ F2, V @ F, rtol=1e-10, atol=1e-10)


def test_wrench_transform_moment_shift(rng_seeded):
    """纯力 + 参考点平移：目标点矩 = 源点矩 + (p_S - p_T) × f_W。"""
    rng = rng_seeded
    R_W = np.array(pin.exp3(0.3 * np.array([1.0, 2.0, -1.0]) / np.sqrt(6)))
    f_S = np.array([1.0, 0.5, -0.3])
    p_S, p_E = np.array([0.1, 0.2, 0.3]), np.array([-0.1, 0.0, 0.2])
    # 传感器系 S → 世界 W（矩参考点移到世界原点）：f_W = R_W f_S，
    # n_W = 0 + p_S × f_W（纯力在源点矩为零，换参考点产生平移矩）
    F_W = adjoint_wrench(pin.SE3(R_W, p_S)) @ np.concatenate([f_S, np.zeros(3)])
    f_W = F_W[:3]
    np.testing.assert_allclose(f_W, R_W @ f_S, atol=1e-12)
    np.testing.assert_allclose(F_W[3:], np.cross(p_S, f_W), atol=1e-12)
    # 世界 W → 末端系 E（E 与 S 同朝向 R_W）：
    # n_E = R_Wᵀ(n_W_at_E) = R_Wᵀ(0 + (p_S - p_E) × f_W)
    F_E = adjoint_wrench(pin.SE3(R_W, p_E).inverse()) @ F_W
    n_expected = R_W.T @ np.cross(p_S - p_E, f_W)
    np.testing.assert_allclose(F_E[:3], f_S, atol=1e-12)
    np.testing.assert_allclose(F_E[3:], n_expected, atol=1e-12)


def test_eq56_gamma_power_duality(rng_seeded):
    """论文 Eq. 56：λ̇ᵀγ = ṼᵀF̃，其中 Ṽ = dexp λ̇、γ = dexpᵀ F̃。"""
    rng = rng_seeded
    for theta in (0.2, 1.0, np.pi - 0.1):
        lam = _rand_lam(rng, theta)
        lam_dot = np.concatenate([rng.normal(size=3), rng.normal(size=3)])
        F_tilde = np.concatenate([rng.normal(size=3), rng.normal(size=3)])
        E = dexp_se3(lam)
        V_tilde = E @ lam_dot
        gamma = E.T @ F_tilde
        np.testing.assert_allclose(lam_dot @ gamma, V_tilde @ F_tilde,
                                   rtol=1e-10, atol=1e-10)


# ----------------------------------------------------------------------
# 7. θ→0 数值稳定性
# ----------------------------------------------------------------------
@pytest.mark.parametrize("theta", [0.0, 1e-10, 1e-8, 1e-6, 1e-4])
def test_small_angle_all_finite_and_continuous(rng_seeded, theta):
    """θ→0 时所有函数无 NaN/inf，且数值连续地趋近解析极限。"""
    rng = rng_seeded
    if theta == 0.0:
        xi = np.zeros(3)
    else:
        u = rng.normal(size=3)
        xi = theta * u / np.linalg.norm(u)
    eta = rng.normal(size=3) * 0.1
    eta_dot = rng.normal(size=3) * 0.1
    xi_dot = rng.normal(size=3) * 0.1
    lam = np.concatenate([eta, xi])
    lam_dot = np.concatenate([eta_dot, xi_dot])

    mats = [
        dexp_so3(xi), dexp_inv_so3(xi),
        dexp_se3(lam), dexp_inv_se3(lam),
        dexp_dot_so3(xi, xi_dot), dexp_inv_dot_so3(xi, xi_dot),
        dexp_dot_se3(lam, lam_dot), dexp_inv_dot_se3(lam, lam_dot),
    ]
    for M in mats:
        assert np.all(np.isfinite(M))

    # θ→0 极限（论文 Eq. 42-43 与 C_0(y)=½[y]、D_0(y)=-½[y]）。
    # 收敛为 O(θ)：θ=1e-4 时偏差 ~1e-6，容差取 1e-3——公式级错误（符号/
    # 系数）会产生 O(0.01-0.1) 偏差仍被捕获；精确系数由中等角度的
    # 中心差分测试把关
    tol = 0.0 if theta == 0.0 else 1e-3
    np.testing.assert_allclose(
        dexp_se3(lam)[:3, 3:], 0.5 * skew3(eta), atol=max(tol, 1e-12))
    np.testing.assert_allclose(
        dexp_inv_se3(lam)[:3, 3:], -0.5 * skew3(eta), atol=max(tol, 1e-12))
    anti = skew3(xi_dot) @ skew3(eta) + skew3(eta) @ skew3(xi_dot)
    np.testing.assert_allclose(
        dexp_dot_se3(lam, lam_dot)[:3, 3:], 0.5 * skew3(eta_dot) + anti / 6.0,
        atol=max(tol, 1e-12))
    np.testing.assert_allclose(
        dexp_inv_dot_se3(lam, lam_dot)[:3, 3:], -0.5 * skew3(eta_dot) + anti / 12.0,
        atol=max(tol, 1e-12))
    # 逆一致性在小角度同时成立
    np.testing.assert_allclose(
        dexp_se3(lam) @ dexp_inv_se3(lam), np.eye(6), atol=max(tol, 1e-12))


def test_zero_lambda_exact_limits():
    """λ = 0 的精确值：dexp = I、d/dt dexp 对角块 = C_0(ξ̇) = ½[ξ̇]。"""
    lam = np.zeros(6)
    lam_dot = np.array([0.1, -0.2, 0.3, 0.4, 0.5, -0.6])
    np.testing.assert_allclose(dexp_se3(lam), np.eye(6), atol=0.0)
    np.testing.assert_allclose(dexp_inv_se3(lam), np.eye(6), atol=0.0)
    expected_diag = 0.5 * skew3(lam_dot[3:])
    np.testing.assert_allclose(dexp_dot_se3(lam, lam_dot)[:3, :3], expected_diag, atol=1e-15)
    np.testing.assert_allclose(dexp_inv_dot_se3(lam, lam_dot)[3:, 3:],
                               -0.5 * skew3(lam_dot[3:]), atol=1e-15)


def test_near_pi_finite_and_invertible(rng_seeded):
    """接近 π（不越过主 log 分支）：全部有限且 dexp 可逆。"""
    lam = _rand_lam(rng_seeded, np.pi - 1e-6)
    lam_dot = np.concatenate([rng_seeded.normal(size=3), rng_seeded.normal(size=3)])
    for M in (dexp_se3(lam), dexp_inv_se3(lam),
              dexp_dot_se3(lam, lam_dot), dexp_inv_dot_se3(lam, lam_dot)):
        assert np.all(np.isfinite(M))
    np.testing.assert_allclose(dexp_se3(lam) @ dexp_inv_se3(lam), np.eye(6), atol=1e-7)


def test_inverse_domain_guard():
    """dexp⁻¹ 族在 ‖ξ‖ ≥ 2π 明确拒绝（不静默产生 inf）。"""
    xi_2pi = np.array([0.0, 0.0, 2.0 * np.pi])
    with pytest.raises(ValueError):
        dexp_inv_so3(xi_2pi)
    with pytest.raises(ValueError):
        dexp_inv_se3(np.concatenate([np.zeros(3), xi_2pi]))


# ----------------------------------------------------------------------
# fixture：固定种子
# ----------------------------------------------------------------------
@pytest.fixture
def rng_seeded():
    return np.random.default_rng(20260917)
