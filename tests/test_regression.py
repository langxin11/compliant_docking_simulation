"""物理回归测试：复刻 canonical 阻抗控制对接回路（与 experiments/run_docking.py 一致）。

运行方式 / How to run:
    pytest -m "not slow"   # 快速套件（默认）：3 s 无接触跟踪
    pytest -m slow         # 慢速套件：12 s 全接触回归（约 2-4 分钟），数值锚点不可调整

数值锚点为重构前实测的行为基准（refactor 行为不变的判据）；若失败说明重构改变了行为。
"""
import numpy as np
import pinocchio as pin
import pytest

from compliant_docking.config import ImpedanceConfig
from compliant_docking.control.task_space import TaskSpaceController
from compliant_docking.models import load_pin_model
from compliant_docking.planning.kinematics import compute_ik
from compliant_docking.planning.trajectory import DecoupledQuinticTrajectory
from compliant_docking.scene import DEFAULT_SCENE_PATH, load_scene
from compliant_docking.simulation.mujoco_env import MujRobot

# 场景驱动：初始条件与模型资产来自默认场景（数值与重构前硬编码值完全一致）
_SCENE = load_scene(DEFAULT_SCENE_PATH)
INIT_POS = _SCENE.task.init_pos
INIT_ORI = _SCENE.task.init_ori
IK_GUESS = _SCENE.task.ik_guess
STROKE = _SCENE.task.stroke


@pytest.fixture(scope="module")
def q_init():
    """标准初始位姿的 IK 解（模块内共享，避免重复求解）。"""
    pin_model = load_pin_model(_SCENE.robot.urdf)
    pin_data = pin_model.createData()
    init_pose = pin.SE3(INIT_ORI, INIT_POS)
    q, success = compute_ik(pin_model, pin_data, init_pose, initial_q=IK_GUESS, max_iters=5000,
                            ee_frame=_SCENE.robot.ee_frame)
    assert success, "标准初始位姿的 IK 应收敛"
    return q


def run_docking_loop(duration: float, q_init: np.ndarray, dt: float = 0.001,
                     traj_duration: float = 15.0, max_torque: float = 10.0) -> dict:
    """canonical 对接回路（无渲染/无录帧/无逐行打印）。

    load_pin_model → TaskSpaceController(model, dt, ImpedanceConfig()) → 轨迹规划
    → MujRobot → 循环：采样轨迹 → 阻抗控制 → ±max_torque 限幅 → step → 更新状态
    → f_ext = cur_ori @ (-sensor('force_sensor').data)
    """
    pin_model = load_pin_model(_SCENE.robot.urdf)
    controller = TaskSpaceController(pin_model, dt, ImpedanceConfig(), ee_frame=_SCENE.robot.ee_frame)

    target_pos = INIT_POS + STROKE
    traj = DecoupledQuinticTrajectory(INIT_POS, target_pos, traj_duration)

    muj_robot = MujRobot(model=_SCENE.build_mjmodel(), render=False, record=False,
                         dt=dt, target_pos=target_pos, eef_body=_SCENE.eef_body)
    muj_robot.init_simulators(q_init)

    q = q_init
    v = np.zeros(7)
    current_pos, current_vel, _ = controller.get_task_space_state(q, v)
    force_external = np.zeros(3)
    torque_external = np.zeros(3)

    final_err = 0.0
    final_contact = 0.0
    max_contact = 0.0
    steps = 0
    while muj_robot.data.time < duration:
        t = muj_robot.data.time
        pos_des, vel_des, acc_des = traj.get_state(t)

        tau = controller.compute_control_task_space_with_orientation_and_imp(
            q, v, pos_des, vel_des, acc_des, current_pos, current_vel,
            force_external, torque_external)
        tau = np.clip(tau, -max_torque, max_torque)

        q, v, _ = muj_robot.step(tau)

        current_pos, current_vel, current_ori = controller.get_task_space_state(q, v)
        force_external = current_ori @ (-muj_robot.data.sensor("force_sensor").data)

        final_err = float(np.linalg.norm(current_pos - pos_des))
        final_contact = float(np.linalg.norm(force_external))
        max_contact = max(max_contact, final_contact)
        steps += 1

    return {
        "final_err_mm": final_err * 1000.0,
        "max_contact_N": max_contact,
        "final_contact_N": final_contact,
        "steps": steps,
    }


def test_impedance_tracking_no_contact_3s(q_init):
    """快速回归（默认套件）：3 s 时尚未接触，跟踪误差 < 0.5 mm，接触力 < 0.1 N。"""
    result = run_docking_loop(duration=3.0, q_init=q_init)

    assert result["steps"] == pytest.approx(3000, abs=5)
    assert result["final_err_mm"] < 0.5, f"3s 跟踪误差超限: {result['final_err_mm']:.4f} mm"
    assert result["final_contact_N"] < 0.1, f"3s 不应有接触力: {result['final_contact_N']:.4f} N"


@pytest.mark.slow
def test_contact_regression_12s(q_init):
    """慢速回归：12 s 全接触行为锚点（阈值即基准，失败说明重构改变了行为，不可调整）。"""
    result = run_docking_loop(duration=12.0, q_init=q_init)

    assert result["final_err_mm"] == pytest.approx(68.124, abs=0.5), (
        f"final_err_mm={result['final_err_mm']:.3f} 偏离锚点 68.124"
    )
    assert result["max_contact_N"] == pytest.approx(18.785, abs=0.5), (
        f"max_contact_N={result['max_contact_N']:.3f} 偏离锚点 18.785"
    )
    assert result["final_contact_N"] == pytest.approx(2.135, abs=0.1), (
        f"final_contact_N={result['final_contact_N']:.3f} 偏离锚点 2.135"
    )
