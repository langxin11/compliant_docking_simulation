"""MuJoCo 未进入 Pinocchio 模型的关节耗散项前馈补偿测试。"""
import numpy as np
import pytest

from compliant_docking.config import ImpedanceConfig
from compliant_docking.control.task_space import TaskSpaceController
from compliant_docking.models import load_pin_model


@pytest.fixture(scope="module")
def pin_model():
    return load_pin_model()


def _run(ctrl, q, v):
    pos_des = np.array([0.0, 0.5, 0.5])
    ctrl.get_task_space_state(q, v)
    cur_pos, cur_vel, _ = ctrl.get_task_space_state(q, v)
    return ctrl.compute_control_task_space_with_orientation_and_imp(
        q, v, pos_des, np.zeros(3), np.zeros(3), cur_pos, cur_vel,
        np.zeros(3), np.zeros(3))


def test_zero_friction_is_noop(pin_model):
    """frictionloss=None 与全零向量行为完全一致（iiwa14 场景路径不受影响）。"""
    q = np.array([0.1, 0.3, -0.4, 0.2, 0.5, -0.3, 0.2])
    v = np.array([0.05, -0.02, 0.03, 0.01, -0.04, 0.02, 0.01])
    tau_none = _run(TaskSpaceController(pin_model, 0.001, ImpedanceConfig(), frictionloss=None), q, v)
    tau_zero = _run(TaskSpaceController(pin_model, 0.001, ImpedanceConfig(), frictionloss=np.zeros(7)), q, v)
    np.testing.assert_array_equal(tau_none, tau_zero)


def test_friction_feedforward_matches_tanh_model(pin_model):
    """前馈力矩差值应等于 frictionloss·tanh(q̇/0.01)（关闭 I 项以隔离前馈）。"""
    q = np.array([0.1, 0.3, -0.4, 0.2, 0.5, -0.3, 0.2])
    v = np.array([0.05, -0.02, 0.03, 0.01, -0.04, 0.02, 0.01])
    fl = np.array([1.137, 1.137, 1.137, 1.137, 0.763, 0.44, 0.248])
    tau_base = _run(TaskSpaceController(pin_model, 0.001, ImpedanceConfig(), friction_integral_gain=0.0), q, v)
    tau_ff = _run(TaskSpaceController(pin_model, 0.001, ImpedanceConfig(), frictionloss=fl,
                                      friction_integral_gain=0.0), q, v)
    expected = fl * np.tanh(v / 0.01)
    np.testing.assert_allclose(tau_ff - tau_base, expected, atol=1e-10)
    # 大速度下（同一状态对比）前馈趋近库仑摩擦幅值 tanh(50)≈1
    v_fast = np.full(7, 0.5)
    tau_base_fast = _run(TaskSpaceController(pin_model, 0.001, ImpedanceConfig(), friction_integral_gain=0.0), q, v_fast)
    tau_ff_fast = _run(TaskSpaceController(pin_model, 0.001, ImpedanceConfig(), frictionloss=fl,
                                           friction_integral_gain=0.0), q, v_fast)
    np.testing.assert_allclose(tau_ff_fast - tau_base_fast, fl, atol=1e-3)


def test_damping_feedforward_matches_mujoco_model(pin_model):
    """阻尼前馈力矩差值应精确等于 ``dof_damping * qdot``。"""
    q = np.array([0.1, 0.3, -0.4, 0.2, 0.5, -0.3, 0.2])
    v = np.array([0.05, -0.02, 0.03, 0.01, -0.04, 0.02, 0.01])
    damping = np.array([0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
    tau_base = _run(
        TaskSpaceController(pin_model, 0.001, ImpedanceConfig(), friction_integral_gain=0.0),
        q,
        v,
    )
    tau_ff = _run(
        TaskSpaceController(
            pin_model,
            0.001,
            ImpedanceConfig(),
            damping=damping,
            friction_integral_gain=0.0,
        ),
        q,
        v,
    )
    np.testing.assert_allclose(tau_ff - tau_base, damping * v, atol=1e-10)


def test_integral_gain_auto_gating(pin_model):
    """I 项自动门控：摩擦非零自动启用（150），零摩擦恒 0（iiwa 路径行为不变）。"""
    q = np.array([0.1, 0.3, -0.4, 0.2, 0.5, -0.3, 0.2])
    ctrl_ff = TaskSpaceController(pin_model, 0.001, ImpedanceConfig(),
                                  frictionloss=np.full(7, 1.137))
    assert ctrl_ff._ki == 150.0
    ctrl_zero = TaskSpaceController(pin_model, 0.001, ImpedanceConfig())
    assert ctrl_zero._ki == 0.0
    # 零增益（I 项关闭）+ 摩擦参数：与无摩擦输出之差恰为 tanh 前馈
    v_moving = np.full(7, 0.05)
    t0 = _run(ctrl_zero, q, v_moving)
    ctrl_ff0 = TaskSpaceController(pin_model, 0.001, ImpedanceConfig(),
                                   frictionloss=np.full(7, 1.137), friction_integral_gain=0.0)
    np.testing.assert_allclose(_run(ctrl_ff0, q, v_moving) - t0,
                               np.full(7, 1.137) * np.tanh(0.05 / 0.01), atol=1e-10)


def test_torque_mode_feedforward_matches_formula(pin_model):
    """torque 模式：前馈差值 = f·tanh(scale·τ_pre/f)，τ_pre 为无摩擦基线力矩。"""
    q = np.array([0.1, 0.3, -0.4, 0.2, 0.5, -0.3, 0.2])
    v = np.array([0.05, -0.02, 0.03, 0.01, -0.04, 0.02, 0.01])
    fl = np.array([1.137, 1.137, 1.137, 1.137, 0.763, 0.44, 0.248])
    tau_base = _run(TaskSpaceController(pin_model, 0.001, ImpedanceConfig(),
                                        friction_integral_gain=0.0, friction_mode="torque"),
                    q, v)
    tau_ff = _run(TaskSpaceController(pin_model, 0.001, ImpedanceConfig(), frictionloss=fl,
                                      friction_integral_gain=0.0, friction_mode="torque"),
                  q, v)
    np.testing.assert_allclose(tau_ff - tau_base, fl * np.tanh(2.0 * tau_base / fl), atol=1e-10)


def test_torque_mode_breaks_stiction_deadzone(pin_model):
    """零速 + |τ_pre|>f 时：velocity 模式补偿为 0（死区），torque 模式 ≈ ±f。"""
    q = np.array([0.1, 0.3, -0.4, 0.2, 0.5, -0.3, 0.2])
    v = np.zeros(7)  # 关节静止：velocity 模式的死区场景
    fl = np.full(7, 1.137)
    # 用期望位置偏移制造非零 τ_pre（正方向推）
    pos_des = np.array([0.0, 0.5, 0.45])  # 比当前末端低 5cm，阻抗产生正向推力矩
    ctrl_zero = TaskSpaceController(pin_model, 0.001, ImpedanceConfig(),
                                    friction_integral_gain=0.0, friction_mode="torque")
    cur_pos, cur_vel, _ = ctrl_zero.get_task_space_state(q, v)
    tau_base = ctrl_zero.compute_control_task_space_with_orientation_and_imp(
        q, v, pos_des, np.zeros(3), np.zeros(3), cur_pos, cur_vel, np.zeros(3), np.zeros(3))
    ctrl_vel = TaskSpaceController(pin_model, 0.001, ImpedanceConfig(), frictionloss=fl,
                                   friction_integral_gain=0.0, friction_mode="velocity")
    ctrl_tor = TaskSpaceController(pin_model, 0.001, ImpedanceConfig(), frictionloss=fl,
                                   friction_integral_gain=0.0, friction_mode="torque")
    tau_vel = ctrl_vel.compute_control_task_space_with_orientation_and_imp(
        q, v, pos_des, np.zeros(3), np.zeros(3), cur_pos, cur_vel, np.zeros(3), np.zeros(3))
    tau_tor = ctrl_tor.compute_control_task_space_with_orientation_and_imp(
        q, v, pos_des, np.zeros(3), np.zeros(3), cur_pos, cur_vel, np.zeros(3), np.zeros(3))
    ff_vel = tau_vel - tau_base
    ff_tor = tau_tor - tau_base
    # 零速下 velocity 模式补偿恒为零（死区），torque 模式给出同向非零补偿：
    # |τ_pre|≥f 的关节为满额 ±f，小力矩关节按 tanh 平滑取部分值
    assert np.all(ff_vel == 0.0)
    assert np.all(np.sign(ff_tor) == np.sign(tau_base))
    assert np.all(np.abs(ff_tor) > 0.0)
    np.testing.assert_allclose(ff_tor, fl * np.tanh(2.0 * tau_base / fl), atol=1e-10)


def test_invalid_friction_mode_rejected(pin_model):
    with pytest.raises(ValueError, match="friction_mode"):
        TaskSpaceController(pin_model, 0.001, ImpedanceConfig(), friction_mode="bogus")
