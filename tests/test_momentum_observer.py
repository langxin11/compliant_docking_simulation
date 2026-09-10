"""PI 动量观测器单元测试（精确模型收敛性 + 基本契约）。"""
import numpy as np
import pinocchio as pin
import pytest

from compliant_docking.control.momentum_observer import MomentumObserver
from compliant_docking.models import load_pin_model


@pytest.fixture(scope="module")
def pin_model():
    return load_pin_model()


def _coriolis_bias(model, data, q, v):
    """返回 (C·q̇, g)：恒速轨迹合成 τ_applied 用。"""
    g = np.asarray(pin.computeGeneralizedGravity(model, data, q))
    pin.computeCoriolisMatrix(model, data, q, v)
    C_v = np.asarray(data.C) @ v
    return C_v, g


def test_rest_zero_residual(pin_model):
    """静止 + 零施加力矩：残差保持接近零（模型一致性数值检查）。"""
    obs = MomentumObserver(pin_model, "cylinder_link", 0.001)
    q = np.zeros(7)
    v = np.zeros(7)
    for _ in range(200):
        tau_ext = obs.update(q, v, np.zeros(7))
    assert np.all(np.isfinite(tau_ext))
    assert np.linalg.norm(tau_ext) < 1e-6
    assert obs.wrench.shape == (6,)
    assert np.linalg.norm(obs.force) < 1e-6


def test_pi_converges_to_constant_unmodeled_torque(pin_model):
    """恒速轨迹 + 常值未建模关节力矩：PI 残差应收敛到该值（积分项持有）。

    恒速时 q̈=0，真实动力学 τ_applied = C·q̇ + g + τ_dist；
    观测器精确模型下稳态 Δp→0，τ̃_ext → τ_dist。
    """
    dt = 0.001
    obs = MomentumObserver(pin_model, "cylinder_link", dt, kp=20.0, ki=40.0)
    data = pin_model.createData()
    q = np.array([0.1, 0.3, -0.4, 0.2, 0.5, -0.3, 0.2])
    v = np.full(7, 0.05)
    tau_dist = np.array([0.5, -0.3, 0.2, 0.4, -0.1, 0.15, -0.05])

    tau_ext = np.zeros(7)
    for _ in range(5000):  # 5 s 收敛
        C_v, g = _coriolis_bias(pin_model, data, q, v)
        # 真实场景：执行器给出精确模型项，环境另加扰动 τ_dist
        tau_applied = C_v + g - tau_dist
        tau_ext = obs.update(q, v, tau_applied)
        q = q + dt * v
    assert np.allclose(tau_ext, tau_dist, atol=5e-3)
    # 稳态时动量误差趋零（PI 无静差）：p̃ ≈ p = M(q)·v
    M = np.array(pin.crba(pin_model, data, q))
    M = np.triu(M) + np.triu(M, 1).T
    assert np.linalg.norm(obs.momentum_hat - M @ v) < 1e-6


def test_reset_reinitializes_state(pin_model):
    """reset 后积分与残差清零、动量估计对齐当前状态。"""
    obs = MomentumObserver(pin_model, "cylinder_link", 0.001)
    q = np.zeros(7)
    v = np.full(7, 0.1)
    obs.reset(q, v)
    assert np.linalg.norm(obs.momentum_hat) > 0.0
    assert np.linalg.norm(obs.integral_err) == 0.0
    assert np.linalg.norm(obs.tau_ext) == 0.0


def test_contact_estimate_subtracts_known_dissipation(pin_model):
    """提供耗散模型时，纯接触估计应把已知摩擦/阻尼从残差中扣除。

    扰动恰为耗散模型项（恒速 v 下 f·tanh(v/v₀)+d·v）：含摩擦残差收敛到
    该值，而纯接触估计收敛到 0。
    """
    dt = 0.001
    fl = np.array([1.137, 1.137, 1.137, 1.137, 0.763, 0.44, 0.248])
    damping = np.full(7, 0.21)
    obs = MomentumObserver(pin_model, "cylinder_link", dt, kp=20.0, ki=40.0,
                           frictionloss=fl, damping=damping)
    data = pin_model.createData()
    q = np.array([0.1, 0.3, -0.4, 0.2, 0.5, -0.3, 0.2])
    v = np.full(7, 0.05)
    tau_dist = fl * np.tanh(v / 0.01) + damping * v

    tau_ext = np.zeros(7)
    for _ in range(5000):
        g = np.asarray(pin.computeGeneralizedGravity(pin_model, data, q))
        pin.computeCoriolisMatrix(pin_model, data, q, v)
        C_v = np.asarray(data.C) @ v
        tau_applied = C_v + g - tau_dist
        tau_ext = obs.update(q, v, tau_applied)
        q = q + dt * v
    assert np.allclose(tau_ext, tau_dist, atol=5e-3)
    assert np.linalg.norm(obs.wrench_contact) < 1e-3
