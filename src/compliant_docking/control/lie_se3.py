"""
lie_se3.py — SE(3)/SO(3) 微分指数映射（dexp）闭式工具集

实现 Kim et al. 2025（IEEE T-RO, Vol. 41）"Impedance Control Design Framework
Using Commutative Map Between SE(3) and se(3)" Section II-C / II-D（Eq. 26-43）：

- Lemma 1（Eq. 26-27）：SO(3) 的 dexp / dexp⁻¹
- Lemma 2（Eq. 28-31）：SE(3) 的 dexp / dexp⁻¹（含旋转-平移耦合块 C_ξ(η)、D_ξ(η)）
- Lemma 3（Eq. 34-35）：d/dt dexp_ξ = C_ξ(ξ̇) 与 d/dt dexp_ξ⁻¹ = D_ξ(ξ̇)
- Lemma 4（Eq. 36-43）：SE(3) 的 d/dt dexp_λ 与 d/dt dexp_λ⁻¹（解析实现）

约定（论文 Eq. 1-5，与 Pinocchio ``Motion`` 一致）：

- twist ``V = [v; ω]``（线量在前、角量在后），wrench ``F = [f; n]``
- ceiling form ``[V] = [[ω], v; 0, 0]``
- ``ad_V = [[ω], [v]; 0, [ω]]``，``Ad_T = [R, [r]R; 0, R]``
- 标量系数（Eq. 9）：``α = sinθ/θ``、``β = 2(1-cosθ)/θ²``、``γ = α/β = (θ/2)cot(θ/2)``

dexp 约定（本模块实现的是论文 Eq. 17a/22 的右平凡化版本）::

    vee(Ṫ T⁻¹) = dexp(λ) λ̇，   T = Exp(λ)

即 dexp 把指数坐标速度映射为空间（spatial）twist。论文控制器（Eq. 48）中
相对位姿 T̃ 以当前末端系 {b} 为参考"空间系"，因此 ``Ṽ = dexp_λ λ̇`` 恰是用
{b} 系表达的相对 twist——接线见 control/se3_impedance.py。

数值稳定性：θ→0 时 (1-α)/θ² 等标量系数存在灾难性消去，统一在
θ < ``_TAYLOR_EPS`` 时切换显式 Taylor 展开（含 Γ1..Γ5 与 2(1-γ/β)/θ⁴）。
dexp⁻¹ 族函数在 ‖ξ‖ ≥ 2π 抛 ``ValueError``（论文 Lemma 1：定义域 ‖ξ‖ < 2π，
2kπ 处 β→0 奇异；控制器经 log6 取得的 λ 满足 ‖ξ‖ < π，不会触界）。
"""
from __future__ import annotations

from typing import NamedTuple

import numpy as np

# 标量系数 Taylor 分支阈值：θ < 1e-2 走级数（截断误差 ~θ⁶ 相对量 < 1e-13），
# 直算公式在 θ=1e-2 处的消去误差亦 ~1e-11 相对量，两分支在边界衔接连续。
_TAYLOR_EPS = 1e-2
# dexp⁻¹ 族的定义域上界（论文：‖ξ‖ < 2π，γ、1/β 在 2kπ 奇异）
_TWO_PI = 2.0 * np.pi
_INVERSE_DOMAIN_MARGIN = 1e-9


# ----------------------------------------------------------------------
# 基础算子（论文 Eq. 1-5）
# ----------------------------------------------------------------------
def skew3(x: np.ndarray) -> np.ndarray:
    """⌈x⌉ ∈ so(3)（论文 Eq. 1）。"""
    x = np.asarray(x, dtype=float).reshape(3)
    return np.array([
        [0.0, -x[2], x[1]],
        [x[2], 0.0, -x[0]],
        [-x[1], x[0], 0.0],
    ])


def vee3(S: np.ndarray) -> np.ndarray:
    """so(3) → R³，skew3 的逆算子。"""
    S = np.asarray(S, dtype=float).reshape(3, 3)
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def hat4(V: np.ndarray) -> np.ndarray:
    """[V] ∈ se(3) 的 4×4 ceiling form（论文 Eq. 2），V = [v; ω]。"""
    V = np.asarray(V, dtype=float).reshape(6)
    M = np.zeros((4, 4))
    M[:3, :3] = skew3(V[3:])
    M[:3, 3] = V[:3]
    return M


def vee4(M: np.ndarray) -> np.ndarray:
    """se(3) 4×4 矩阵 → [v; ω]，hat4 的逆算子。"""
    M = np.asarray(M, dtype=float).reshape(4, 4)
    return np.concatenate([M[:3, 3], vee3(M[:3, :3])])


def ad6(V: np.ndarray) -> np.ndarray:
    """伴随算子 ad_V（论文 Eq. 3-4），V = [v; ω]。"""
    V = np.asarray(V, dtype=float).reshape(6)
    w_skew = skew3(V[3:])
    return np.block([
        [w_skew, skew3(V[:3])],
        [np.zeros((3, 3)), w_skew],
    ])


def _Rp(T) -> tuple[np.ndarray, np.ndarray]:
    """从 pin.SE3 对象或 4×4 齐次矩阵提取 (R, p)。"""
    if hasattr(T, "rotation") and hasattr(T, "translation"):
        return np.array(T.rotation, dtype=float), np.array(T.translation, dtype=float)
    M = np.asarray(T, dtype=float)
    return M[:3, :3].copy(), M[:3, 3].copy()


def adjoint(T) -> np.ndarray:
    """伴随变换 Ad_T（论文 Eq. 5）。

    T = ^A T_B（B 在 A 中的位姿）时满足 ``V_A = Ad_T V_B``。
    """
    R, p = _Rp(T)
    return np.block([
        [R, skew3(p) @ R],
        [np.zeros((3, 3)), R],
    ])


def adjoint_wrench(T) -> np.ndarray:
    """wrench 变换矩阵 ``Ad_T^{-T}``（co-adjoint 作用）。

    T = ^A T_B 时满足 ``F_A = Ad_T^{-T} F_B`` 且功率不变
    ``V_Aᵀ F_A = V_Bᵀ F_B``。显式形式 ``[[R, 0], [[p]R, R]]``
    的左下块即矩平移项 ``p × f``。
    """
    R, p = _Rp(T)
    return np.block([
        [R, np.zeros((3, 3))],
        [skew3(p) @ R, R],
    ])


# ----------------------------------------------------------------------
# 标量系数（论文 Eq. 9；Γ_i 为 Eq. 40；g6 为 Eq. 39 的独立项系数）
# ----------------------------------------------------------------------
class _Coeffs(NamedTuple):
    alpha: float   # α = sinθ/θ
    beta: float    # β = 2(1-cosθ)/θ² = sinc²(θ/2)
    gamma: float   # γ = α/β = (θ/2)cot(θ/2)
    inv_beta: float
    g1: float      # Γ1 = (1-α)/θ²
    g2: float      # Γ2 = (α-β)/θ²
    g3: float      # Γ3 = (β/2 - 3Γ1)/θ²
    g4: float      # Γ4 = (1-γ)/θ²
    g5: float      # Γ5 = (1/β + γ - 2)/θ⁴（Eq. 40e：两个 θ² 因子叠乘）
    g6: float      # 2(1-γ/β)/θ⁴（Ḋ 的独立项系数，Eq. 39）


def _coeffs(theta: float, *, inverse: bool = False) -> _Coeffs:
    """按 θ 取稳定分支计算全部标量系数。

    Args:
        theta: ‖ξ‖（非负）
        inverse: True 时校验 dexp⁻¹ 定义域（θ < 2π）
    """
    if inverse and theta >= _TWO_PI - _INVERSE_DOMAIN_MARGIN:
        raise ValueError(
            f"dexp⁻¹ 定义域为 ‖ξ‖ < 2π（γ、1/β 在 2kπ 奇异），当前 θ={theta:.6f}")
    if theta < _TAYLOR_EPS:
        # Taylor 展开（推导见模块级 docstring 的系数说明；截断到 θ⁴）
        t2 = theta * theta
        t4 = t2 * t2
        return _Coeffs(
            alpha=1.0 - t2 / 6.0 + t4 / 120.0,
            beta=1.0 - t2 / 12.0 + t4 / 360.0,
            gamma=1.0 - t2 / 12.0 - t4 / 720.0,
            inv_beta=1.0 + t2 / 12.0 + t4 / 240.0,
            g1=1.0 / 6.0 - t2 / 120.0 + t4 / 5040.0,
            g2=-1.0 / 12.0 + t2 / 180.0 - t4 / 6720.0,
            g3=-1.0 / 60.0 + t2 / 1260.0 - t4 / 60480.0,
            g4=1.0 / 12.0 + t2 / 720.0 + t4 / 30240.0,
            g5=1.0 / 360.0 + t2 / 7560.0 + t4 / 201600.0,
            g6=1.0 / 120.0 + t2 / 1512.0,
        )
    alpha = float(np.sinc(theta / np.pi))          # sinθ/θ
    beta = float(np.sinc(theta / (2.0 * np.pi))) ** 2  # sinc²(θ/2) = 2(1-cosθ)/θ²
    gamma = alpha / beta
    inv_beta = 1.0 / beta
    return _Coeffs(
        alpha=alpha, beta=beta, gamma=gamma, inv_beta=inv_beta,
        g1=(1.0 - alpha) / theta**2,
        g2=(alpha - beta) / theta**2,
        g3=(0.5 * beta - 3.0 * (1.0 - alpha) / theta**2) / theta**2,
        g4=(1.0 - gamma) / theta**2,
        g5=(inv_beta + gamma - 2.0) / theta**4,
        g6=2.0 * (1.0 - gamma * inv_beta) / theta**4,
    )


def _anti(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """⌈a,b⌉ = [a][b] + [b][a]（论文 Eq. 30 记号），对 a,b 对称。"""
    return A @ B + B @ A


# ----------------------------------------------------------------------
# C_ξ / D_ξ 耦合块与时间导数（论文 Eq. 30-31、38-43）
# ----------------------------------------------------------------------
def _C_block(xi: np.ndarray, y: np.ndarray, coeffs: _Coeffs | None = None) -> np.ndarray:
    """C_ξ(y)（论文 Eq. 30，亦写成 Γ 形式，附录 A）。

    dexp_λ 的右上耦合块；按 Lemma 3（Eq. 34），d/dt dexp_ξ = C_ξ(ξ̇)
    复用同一函数（y 换为 ξ̇）。
    """
    xi = np.asarray(xi, dtype=float).reshape(3)
    y = np.asarray(y, dtype=float).reshape(3)
    c = coeffs if coeffs is not None else _coeffs(float(np.linalg.norm(xi)))
    S_xi = skew3(xi)
    xy = float(np.dot(xi, y))
    return ((0.5 * c.beta) * skew3(y)
            + c.g1 * _anti(skew3(y), S_xi)
            + c.g2 * xy * S_xi
            + c.g3 * xy * (S_xi @ S_xi))


def _D_block(xi: np.ndarray, y: np.ndarray) -> np.ndarray:
    """D_ξ(y)（论文 Eq. 31，Γ 形式见附录 A）。

    dexp_λ⁻¹ 的右上耦合块；按 Lemma 3（Eq. 35），d/dt dexp_ξ⁻¹ = D_ξ(ξ̇)。
    """
    xi = np.asarray(xi, dtype=float).reshape(3)
    y = np.asarray(y, dtype=float).reshape(3)
    c = _coeffs(float(np.linalg.norm(xi)), inverse=True)
    S_xi = skew3(xi)
    xy = float(np.dot(xi, y))
    return (-0.5 * skew3(y)
            + c.g4 * _anti(skew3(y), S_xi)
            + c.g5 * xy * (S_xi @ S_xi))


def _zeta(xi: np.ndarray, xi_dot: np.ndarray, y: np.ndarray, theta: float) -> float:
    """ζ(y) = (ξᵀy)(ξᵀξ̇)/‖ξ‖²（论文 Eq. 38 记号）。

    θ→0 时 ζ 一般有非零方向极限，但其全部用例都伴随因子 [ξ] 或 [ξ]²，
    故 θ 精确为 0 附近置 0 即可（该项整体消失）。
    """
    if theta < 1e-12:
        return 0.0
    return float(np.dot(xi, y)) * float(np.dot(xi, xi_dot)) / (theta * theta)


def _C_dot_block(xi: np.ndarray, xi_dot: np.ndarray,
                 y: np.ndarray, y_dot: np.ndarray) -> np.ndarray:
    """Ċ_ξ,ξ̇(y, ẏ)（论文 Eq. 38，δ1/δ2 见 Eq. 41a/41b）。"""
    xi = np.asarray(xi, dtype=float).reshape(3)
    xi_dot = np.asarray(xi_dot, dtype=float).reshape(3)
    y = np.asarray(y, dtype=float).reshape(3)
    y_dot = np.asarray(y_dot, dtype=float).reshape(3)
    theta = float(np.linalg.norm(xi))
    c = _coeffs(theta)
    S_xi = skew3(xi)
    S_xi2 = S_xi @ S_xi
    S_y = skew3(y)
    S_ydot = skew3(y_dot)
    S_xdot = skew3(xi_dot)
    zeta = _zeta(xi, xi_dot, y, theta)
    d0 = float(np.dot(xi_dot, y) + np.dot(xi, y_dot))
    xy = float(np.dot(xi, y))
    xxd = float(np.dot(xi, xi_dot))
    delta1 = xy * S_xdot + xxd * S_y + (d0 - 4.0 * zeta) * S_xi
    delta2 = (xy * _anti(S_xi, S_xdot) + xxd * _anti(S_xi, S_y)
              + (d0 - 5.0 * zeta) * S_xi2)
    return ((0.5 * c.beta) * (S_ydot - zeta * S_xi)
            + c.g1 * (_anti(S_ydot, S_xi) + _anti(S_y, S_xdot) + zeta * S_xi)
            + c.g2 * (delta1 + zeta * S_xi2)
            + c.g3 * delta2)


def _D_dot_block(xi: np.ndarray, xi_dot: np.ndarray,
                 y: np.ndarray, y_dot: np.ndarray) -> np.ndarray:
    """Ḋ_ξ,ξ̇(y, ẏ)（论文 Eq. 39，δ3 见 Eq. 41c）。"""
    xi = np.asarray(xi, dtype=float).reshape(3)
    xi_dot = np.asarray(xi_dot, dtype=float).reshape(3)
    y = np.asarray(y, dtype=float).reshape(3)
    y_dot = np.asarray(y_dot, dtype=float).reshape(3)
    theta = float(np.linalg.norm(xi))
    c = _coeffs(theta, inverse=True)
    S_xi = skew3(xi)
    S_xi2 = S_xi @ S_xi
    S_ydot = skew3(y_dot)
    zeta = _zeta(xi, xi_dot, y, theta)
    d0 = float(np.dot(xi_dot, y) + np.dot(xi, y_dot))
    xy = float(np.dot(xi, y))
    xxd = float(np.dot(xi, xi_dot))
    delta3 = (xy * _anti(S_xi, skew3(xi_dot)) + xxd * _anti(S_xi, skew3(y))
              + (d0 - 3.0 * zeta) * S_xi2)
    return (-0.5 * S_ydot
            + c.g6 * zeta * S_xi2
            + c.g4 * (_anti(S_ydot, S_xi) + _anti(skew3(y), skew3(xi_dot)))
            + c.g5 * delta3)


# ----------------------------------------------------------------------
# 公开 API：SO(3)
# ----------------------------------------------------------------------
def dexp_so3(xi: np.ndarray) -> np.ndarray:
    """dexp_ξ（论文 Eq. 26，右平凡化）：``vee(Ṙ Rᵀ) = dexp_ξ ξ̇``。"""
    xi = np.asarray(xi, dtype=float).reshape(3)
    c = _coeffs(float(np.linalg.norm(xi)))
    S = skew3(xi)
    return np.eye(3) + 0.5 * c.beta * S + c.g1 * (S @ S)


def dexp_inv_so3(xi: np.ndarray) -> np.ndarray:
    """dexp_ξ⁻¹（论文 Eq. 27），dexp_so3 的矩阵逆。"""
    xi = np.asarray(xi, dtype=float).reshape(3)
    c = _coeffs(float(np.linalg.norm(xi)), inverse=True)
    S = skew3(xi)
    return np.eye(3) - 0.5 * S + c.g4 * (S @ S)


def dexp_dot_so3(xi: np.ndarray, xi_dot: np.ndarray) -> np.ndarray:
    """d/dt dexp_ξ（论文 Lemma 3 / Eq. 34）= C_ξ(ξ̇)。"""
    return _C_block(xi, xi_dot)


def dexp_inv_dot_so3(xi: np.ndarray, xi_dot: np.ndarray) -> np.ndarray:
    """d/dt dexp_ξ⁻¹（论文 Lemma 3 / Eq. 35）= D_ξ(ξ̇)。"""
    return _D_block(xi, xi_dot)


# ----------------------------------------------------------------------
# 公开 API：SE(3)
# ----------------------------------------------------------------------
def dexp_se3(lambda_: np.ndarray) -> np.ndarray:
    """dexp_λ（论文 Lemma 2 / Eq. 28）。

    满足右平凡化微分关系 ``vee(Ṫ T⁻¹) = dexp_λ λ̇``（λ = [η; ξ]）。
    注意右上耦合块 C_ξ(η) 不可省略——block_diag(dexp_ξ, dexp_ξ) 会丢掉
    旋转-平移耦合。
    """
    lam = np.asarray(lambda_, dtype=float).reshape(6)
    eta, xi = lam[:3], lam[3:]
    theta = float(np.linalg.norm(xi))
    c = _coeffs(theta)
    S = skew3(xi)
    E = np.eye(3) + 0.5 * c.beta * S + c.g1 * (S @ S)
    out = np.zeros((6, 6))
    out[:3, :3] = E
    out[:3, 3:] = _C_block(xi, eta, coeffs=c)
    out[3:, 3:] = E
    return out


def dexp_inv_se3(lambda_: np.ndarray) -> np.ndarray:
    """dexp_λ⁻¹（论文 Lemma 2 / Eq. 29），dexp_se3 的矩阵逆。"""
    lam = np.asarray(lambda_, dtype=float).reshape(6)
    eta, xi = lam[:3], lam[3:]
    theta = float(np.linalg.norm(xi))
    c = _coeffs(theta, inverse=True)
    S = skew3(xi)
    E = np.eye(3) - 0.5 * S + c.g4 * (S @ S)
    out = np.zeros((6, 6))
    out[:3, :3] = E
    out[:3, 3:] = _D_block(xi, eta)
    out[3:, 3:] = E
    return out


def dexp_dot_se3(lambda_: np.ndarray, lambda_dot: np.ndarray) -> np.ndarray:
    """d/dt dexp_λ（论文 Lemma 4 / Eq. 36），解析实现。"""
    lam = np.asarray(lambda_, dtype=float).reshape(6)
    lam_dot = np.asarray(lambda_dot, dtype=float).reshape(6)
    eta, xi = lam[:3], lam[3:]
    eta_dot, xi_dot = lam_dot[:3], lam_dot[3:]
    diag = _C_block(xi, xi_dot)
    out = np.zeros((6, 6))
    out[:3, :3] = diag
    out[:3, 3:] = _C_dot_block(xi, xi_dot, eta, eta_dot)
    out[3:, 3:] = diag
    return out


def dexp_inv_dot_se3(lambda_: np.ndarray, lambda_dot: np.ndarray) -> np.ndarray:
    """d/dt dexp_λ⁻¹（论文 Lemma 4 / Eq. 37），解析实现。"""
    lam = np.asarray(lambda_, dtype=float).reshape(6)
    lam_dot = np.asarray(lambda_dot, dtype=float).reshape(6)
    eta, xi = lam[:3], lam[3:]
    eta_dot, xi_dot = lam_dot[:3], lam_dot[3:]
    diag = _D_block(xi, xi_dot)
    out = np.zeros((6, 6))
    out[:3, :3] = diag
    out[:3, 3:] = _D_dot_block(xi, xi_dot, eta, eta_dot)
    out[3:, 3:] = diag
    return out
