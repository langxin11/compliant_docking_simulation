"""
test_wrench.py — wrench 坐标系/参考点变换测试

验证 transform_wrench / wrench_to_body 的三个物理语义与一个守恒律：
- 纯力旋转（无偏移时只做旋转映射）
- 纯力因参考点平移产生矩 (p_S - p_T) × f_W
- 纯矩直接旋转映射（不因平移改变）
- 功率一致性：V_TᵀF_T = V_SᵀF_S（twist 同步变换时功率不变）
- 与 lie_se3.adjoint_wrench 的 co-adjoint 作用数值等价
"""
import numpy as np
import pinocchio as pin

from compliant_docking.control.lie_se3 import adjoint, adjoint_wrench
from compliant_docking.wrench import transform_wrench, wrench_to_body

_RNG = np.random.default_rng(42)


def _rand_pose(scale=0.3):
    w = _RNG.normal(size=3)
    w *= 0.8 / max(np.linalg.norm(w), 1e-9)
    R = np.array(pin.exp3(w))
    p = _RNG.normal(size=3) * scale
    return R, p


def test_pure_force_rotation_only():
    """源/目标 frame 原点重合：纯力只被旋转，无附加矩。"""
    R_S, p = _rand_pose()
    R_T, _ = _rand_pose()
    f_S = np.array([1.0, -2.0, 0.5])
    f_T, n_T = transform_wrench(f_S, np.zeros(3), p, R_S, p, R_T)
    np.testing.assert_allclose(f_T, R_T.T @ R_S @ f_S, atol=1e-14)
    np.testing.assert_allclose(n_T, np.zeros(3), atol=1e-14)


def test_pure_force_point_offset_generates_moment():
    """纯力 + 参考点平移：目标矩 = (p_S - p_T) × f_W。"""
    R_S, p_S = _rand_pose()
    R_T, p_T = _rand_pose()
    f_S = np.array([3.0, 1.0, -2.0])
    f_W = R_S @ f_S
    f_T, n_T = transform_wrench(f_S, np.zeros(3), p_S, R_S, p_T, R_T)
    np.testing.assert_allclose(f_T, R_T.T @ f_W, atol=1e-14)
    np.testing.assert_allclose(n_T, R_T.T @ np.cross(p_S - p_T, f_W), atol=1e-14)


def test_pure_moment_unchanged_by_point_offset():
    """纯矩与参考点无关：只做旋转映射。"""
    R_S, p_S = _rand_pose()
    R_T, p_T = _rand_pose()
    n_S = np.array([0.3, -0.7, 1.1])
    f_T, n_T = transform_wrench(np.zeros(3), n_S, p_S, R_S, p_T, R_T)
    np.testing.assert_allclose(f_T, np.zeros(3), atol=1e-14)
    np.testing.assert_allclose(n_T, R_T.T @ R_S @ n_S, atol=1e-14)


def test_power_consistency_with_twist_transform():
    """功率不变：V_TᵀF_T = V_SᵀF_S（V_T = Ad_{T_T_S} V_S 同步变换）。"""
    for _ in range(10):
        R_S, p_S = _rand_pose()
        R_T, p_T = _rand_pose()
        f_S = _RNG.normal(size=3)
        n_S = _RNG.normal(size=3)
        V_S = np.concatenate([_RNG.normal(size=3), _RNG.normal(size=3)])

        f_T, n_T = transform_wrench(f_S, n_S, p_S, R_S, p_T, R_T)
        # 同一物理 twist 的 S/T 坐标：Ad_{T_W_S}V_S = Ad_{T_W_T}V_T
        # ⟹ V_T = Ad_{T_T_S} V_S（T_T_S = T_WE⁻¹·T_WS）
        T_T_S = pin.SE3(R_T, p_T).inverse() * pin.SE3(R_S, p_S)
        V_T = adjoint(T_T_S) @ V_S

        power_S = V_S @ np.concatenate([f_S, n_S])
        power_T = V_T @ np.concatenate([f_T, n_T])
        np.testing.assert_allclose(power_T, power_S, rtol=1e-10, atol=1e-10)


def test_equivalent_to_coadjoint_action():
    """transform_wrench ≡ adjoint_wrench(T_E_S) @ [f_S; n_S]（数值等价）。"""
    for _ in range(10):
        R_S, p_S = _rand_pose()
        R_T, p_T = _rand_pose()
        f_S = _RNG.normal(size=3)
        n_S = _RNG.normal(size=3)
        f_T, n_T = transform_wrench(f_S, n_S, p_S, R_S, p_T, R_T)
        T_E_S = pin.SE3(R_T, p_T).inverse() * pin.SE3(R_S, p_S)
        F_E = adjoint_wrench(T_E_S) @ np.concatenate([f_S, n_S])
        np.testing.assert_allclose(np.concatenate([f_T, n_T]), F_E, atol=1e-12)


def test_wrench_to_body_matches_transform():
    """wrench_to_body 便利封装与 transform_wrench 逐位一致，输出 [f; n]。"""
    R_S, p_S = _rand_pose()
    R_E, p_E = _rand_pose()
    f_S = _RNG.normal(size=3)
    n_S = _RNG.normal(size=3)
    F_body = wrench_to_body(f_S, n_S, p_S, R_S, pin.SE3(R_E, p_E))
    f_E, n_E = transform_wrench(f_S, n_S, p_S, R_S, p_E, R_E)
    assert F_body.shape == (6,)
    np.testing.assert_allclose(F_body, np.concatenate([f_E, n_E]), atol=1e-15)
