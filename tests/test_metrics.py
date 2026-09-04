"""metrics 套件测试：2 s 短仿真（无接触）产出的 Log 上验证各层指标与格式化。

回路写法复用 tests/test_regression.py 的 canonical 对接回路（场景驱动，
无渲染/无录帧/无逐行打印），并以 log3(R_d R^T) 逐 step 记录世界系姿态误差，
供姿态指标断言。运行方式：pytest -m "not slow"（默认套件）。
"""
from dataclasses import replace

import numpy as np
import pinocchio as pin
import pytest

from compliant_docking.config import ImpedanceConfig
from compliant_docking.control.task_space import TaskSpaceController
from compliant_docking.metrics import (
    TrackingThresholds,
    compute_metrics,
    compute_tracking_metrics,
    evaluate_tracking_gate,
    format_metrics,
    format_tracking_gate,
    tracking_summary,
)
from compliant_docking.models import load_pin_model
from compliant_docking.planning.kinematics import compute_ik
from compliant_docking.planning.trajectory import DecoupledQuinticTrajectory
from compliant_docking.scene import DEFAULT_SCENE_PATH, load_scene
from compliant_docking.simulation.mujoco_env import MujRobot
from compliant_docking.telemetry import Log

# 场景驱动：与 test_regression.py 同一初始条件来源
_SCENE = load_scene(DEFAULT_SCENE_PATH)
INIT_POS = _SCENE.task.init_pos
INIT_ORI = _SCENE.task.init_ori
IK_GUESS = _SCENE.task.ik_guess
STROKE = _SCENE.task.stroke
AXIS = STROKE / np.linalg.norm(STROKE)

DURATION = 2.0  # 2 s 时尚未接触（接触约发生在 9-11 s），用于无接触指标的确定性


@pytest.fixture(scope="module")
def metrics_result() -> tuple:
    """2 s 短仿真产出 (DockingMetrics, Log, pin_model)，模块内共享一次仿真。"""
    pin_model = load_pin_model(_SCENE.robot.pin_model)
    pin_data = pin_model.createData()
    init_pose = pin.SE3(INIT_ORI, INIT_POS)
    q_init, success = compute_ik(pin_model, pin_data, init_pose, initial_q=IK_GUESS,
                                 max_iters=5000, ee_frame=_SCENE.robot.ee_frame)
    assert success, "标准初始位姿的 IK 应收敛"

    dt = 0.001
    controller = TaskSpaceController(pin_model, dt, ImpedanceConfig(),
                                     ee_frame=_SCENE.robot.ee_frame)
    traj = DecoupledQuinticTrajectory(INIT_POS, INIT_POS + STROKE, 15.0)
    muj_robot = MujRobot(model=_SCENE.build_mjmodel(), render=False, record=False,
                         dt=dt, target_pos=INIT_POS + STROKE, eef_body=_SCENE.eef_body)
    muj_robot.init_simulators(q_init)

    log = Log()
    log.reset_logs()
    q = q_init
    v = np.zeros(7)
    current_pos, current_vel, _ = controller.get_task_space_state(q, v)
    force_external = np.zeros(3)
    torque_external = np.zeros(3)

    while muj_robot.data.time < DURATION:
        t = muj_robot.data.time
        pos_des, vel_des, acc_des = traj.get_state(t)
        tau = controller.compute_control_task_space_with_orientation_and_imp(
            q, v, pos_des, vel_des, acc_des, current_pos, current_vel,
            force_external, torque_external)
        tau = np.clip(tau, -10.0, 10.0)
        q, v, _ = muj_robot.step(tau)
        current_pos, current_vel, current_ori = controller.get_task_space_state(q, v)
        force_external = current_ori @ (-muj_robot.data.sensor("force_sensor").data)
        torque_external = current_ori @ (-muj_robot.data.sensor("torque_sensor").data)
        log.store_data(
            t, q, v, current_pos, current_vel,
            float(np.linalg.norm(current_pos - pos_des)),
            pos_des, vel_des, acc_des, tau,
            force_external, torque_external,
            orientation_error=pin.log3(INIT_ORI @ current_ori.T))

    metrics = compute_metrics(log, pin_model, axis=AXIS, ee_frame=_SCENE.robot.ee_frame)
    return metrics, log, pin_model


def test_contact_safety_no_contact_2s(metrics_result):
    """2 s 未接触：峰值轴向力应接近 0（< 0.5 N）。"""
    metrics, _, _ = metrics_result
    assert metrics.peak_axial_force_N is not None
    assert metrics.peak_axial_force_N < 0.5, (
        f"2s 不应有接触：peak_axial={metrics.peak_axial_force_N:.4f} N"
    )
    assert metrics.steady_axial_force_N is not None
    assert abs(metrics.steady_axial_force_N) < 0.5


def test_pos_tracking_rms(metrics_result):
    """跟踪精度：2 s 内位置跟踪 RMS 应远小于 2 mm。"""
    metrics, _, _ = metrics_result
    assert metrics.pos_tracking_rms_m is not None
    assert metrics.pos_tracking_rms_m < 0.002, (
        f"位置跟踪 RMS 超限: {metrics.pos_tracking_rms_m:.6f} m"
    )


def test_joint_limit_percentage(metrics_result):
    """内部安全：2 s 内关节角度不应超限，占比有限且 < 100%。"""
    metrics, _, _ = metrics_result
    assert metrics.max_joint_pos_pct is not None
    assert np.isfinite(metrics.max_joint_pos_pct)
    assert metrics.max_joint_pos_pct < 100.0, (
        f"关节角度占比超限: {metrics.max_joint_pos_pct:.2f}%"
    )
    assert metrics.max_joint_pos_joint is not None
    assert metrics.max_joint_vel_pct is not None
    assert np.isfinite(metrics.max_joint_vel_pct)


def test_min_manipulability(metrics_result):
    """内部安全：最小可操作度应为正且有限。"""
    metrics, _, _ = metrics_result
    assert metrics.min_manipulability is not None
    assert metrics.min_manipulability > 0.0
    assert np.isfinite(metrics.min_manipulability)


def test_orientation_tracking(metrics_result):
    """跟踪精度：姿态被保持，姿态跟踪 RMS 非 None 且 < 0.01 rad。"""
    metrics, _, _ = metrics_result
    assert metrics.ori_tracking_rms_rad is not None
    assert metrics.ori_tracking_rms_rad < 0.01, (
        f"姿态跟踪 RMS 超限: {metrics.ori_tracking_rms_rad:.6f} rad"
    )
    assert metrics.final_orientation_error_rad is not None


def test_format_metrics_layers(metrics_result):
    """format_metrics 三段表格包含关键条目；空 orientation_errors 的 Log 不崩并显示 n/a。"""
    metrics, _, pin_model = metrics_result
    text = format_metrics(metrics)
    assert "峰值轴向力" in text
    assert "[接触安全]" in text
    assert "[内部安全]" in text
    assert "[跟踪精度]" in text

    # 容错分支：构造 orientation_errors 为空的 Log（有数据但无姿态误差记录）
    empty_ori_log = Log()
    empty_ori_log.reset_logs()
    for k in range(10):
        t = 0.001 * k
        empty_ori_log.store_data(
            t, np.zeros(7), np.zeros(7), np.zeros(3), np.zeros(3), 0.0,
            np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(7),
            np.zeros(3), np.zeros(3))
    m2 = compute_metrics(empty_ori_log, pin_model, axis=AXIS, ee_frame=_SCENE.robot.ee_frame)
    assert m2.ori_tracking_rms_rad is None
    assert m2.final_orientation_error_rad is None
    text2 = format_metrics(m2)
    assert "峰值轴向力" in text2
    assert "n/a" in text2


def test_axis_normalization_robust(metrics_result):
    """compute_metrics 对非单位向量 axis 稳健（内部归一化，含轴指标结果一致）。"""
    metrics, log, pin_model = metrics_result
    m_scaled = compute_metrics(log, pin_model, axis=STROKE * 5.0,
                               ee_frame=_SCENE.robot.ee_frame)
    assert m_scaled.peak_axial_force_N == pytest.approx(metrics.peak_axial_force_N)
    assert m_scaled.steady_axial_force_N == pytest.approx(metrics.steady_axial_force_N)
    assert m_scaled.final_lateral_error_m == pytest.approx(metrics.final_lateral_error_m)


# ---- tracking_summary：圆+8字跟踪测试分段统计（合成 Log） ----

def test_tracking_summary_synthetic():
    """合成 Log（每段恒定误差）：分段/全时程 RMS 与峰值数值正确（单位 mm）。"""
    log = Log()
    log.reset_logs()
    segments = [("过渡1", 0.0, 1.0), ("圆周", 1.0, 2.0)]
    for k in range(20):
        t = 0.1 * k
        error = 0.001 if t < 1.0 else 0.002  # 过渡1 恒 1 mm，圆周恒 2 mm
        log.store_data(
            t, np.zeros(7), np.zeros(7), np.zeros(3), np.zeros(3), error,
            np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(7),
            np.zeros(3), np.zeros(3))

    text = tracking_summary(log, segments)

    # 各段名称出现；恒定误差 → RMS = 峰值 = 段值（1.0000 / 2.0000 mm）
    assert "过渡1" in text and "圆周" in text
    assert "RMS 1.0000 mm" in text
    assert "RMS 2.0000 mm" in text
    assert "峰值 1.0000 mm" in text
    assert "峰值 2.0000 mm" in text
    assert "全时程" in text
    # 全时程：10×1mm + 10×2mm → RMS = sqrt(2.5) ≈ 1.5811 mm，峰值 2.0000 mm
    assert "1.5811" in text


def test_tracking_gate_metrics_pass_fail_and_incomplete():
    """门禁含分段/全程姿态统计、接触/限幅遥测，并明确区分 INCOMPLETE。"""
    log = Log()
    log.reset_logs()
    segments = [("圆周", 0.0, 1.0), ("8字", 1.0, 2.0)]
    for k in range(20):
        t = k * 0.1
        log.store_data(
            t, np.zeros(7), np.zeros(7), np.zeros(3), np.zeros(3), 0.001,
            np.zeros(3), np.zeros(3), np.zeros(3), np.full(7, 2.0),
            np.zeros(3), np.zeros(3), orientation_error=np.array([0.001, 0.0, 0.0]),
            torque_saturated=False, contact_count=0,
        )

    metrics = compute_tracking_metrics(log, segments)
    assert metrics.segment("圆周").position_rms_m == pytest.approx(0.001)
    assert metrics.overall.orientation_peak_rad == pytest.approx(0.001)
    assert metrics.peak_joint_torque_Nm == pytest.approx(2.0)
    assert metrics.torque_saturation_ratio == pytest.approx(0.0)
    assert metrics.max_contacts == 0

    passed = evaluate_tracking_gate(metrics, TrackingThresholds(), complete=True)
    assert passed.status == "PASS"
    assert "PASS" in format_tracking_gate(passed)

    incomplete = evaluate_tracking_gate(metrics, TrackingThresholds(), complete=False)
    assert incomplete.status == "INCOMPLETE"
    assert "未作为门禁失败" in format_tracking_gate(incomplete)

    bad_log = Log()
    bad_log.reset_logs()
    bad_log.store_data(
        0.0, np.zeros(7), np.zeros(7), np.zeros(3), np.zeros(3), 0.020,
        np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(7), np.zeros(3), np.zeros(3),
        orientation_error=np.array([0.02, 0.0, 0.0]), torque_saturated=True, contact_count=1,
    )
    bad = evaluate_tracking_gate(compute_tracking_metrics(bad_log, [("圆周", 0.0, 1.0), ("8字", 0.0, 1.0)]),
                                TrackingThresholds(), complete=True)
    assert bad.status == "FAIL"
    assert bad.failures


@pytest.mark.parametrize("nonfinite", [float("nan"), float("inf")])
def test_tracking_gate_fails_closed_for_nonfinite_metrics_and_thresholds(nonfinite):
    """NaN/inf 不能通过数值比较的空洞绕过完整跟踪门禁。"""
    log = Log()
    log.reset_logs()
    segments = [("圆周", 0.0, 1.0), ("8字", 1.0, 2.0)]
    for k in range(20):
        log.store_data(
            k * 0.1, np.zeros(7), np.zeros(7), np.zeros(3), np.zeros(3), 0.001,
            np.zeros(3), np.zeros(3), np.zeros(3), np.ones(7),
            np.zeros(3), np.zeros(3), orientation_error=np.zeros(3),
            torque_saturated=False, contact_count=0,
        )
    metrics = compute_tracking_metrics(log, segments)

    invalid_metric = replace(metrics, peak_joint_torque_Nm=nonfinite)
    metric_result = evaluate_tracking_gate(invalid_metric, TrackingThresholds(), complete=True)
    assert metric_result.status == "FAIL"
    assert any("峰值关节力矩" in failure and "非有限" in failure
               for failure in metric_result.failures)

    invalid_threshold = replace(TrackingThresholds(), circle_position_rms_m=nonfinite)
    threshold_result = evaluate_tracking_gate(metrics, invalid_threshold, complete=True)
    assert threshold_result.status == "FAIL"
    assert any("圆周位置 RMS" in failure and "阈值" in failure
               for failure in threshold_result.failures)


def test_tracking_metrics_exclude_post_trajectory_hold_window():
    """轨迹结束后的低误差保持段不得稀释窗口内误差、限幅或接触指标。"""
    segments = [("圆周", 0.0, 1.0), ("8字", 1.0, 2.0)]

    def add_sample(log: Log, t: float, error: float, torque: float,
                   saturated: bool, contacts: int) -> None:
        log.store_data(
            t, np.zeros(7), np.zeros(7), np.zeros(3), np.zeros(3), error,
            np.zeros(3), np.zeros(3), np.zeros(3), np.full(7, torque),
            np.zeros(3), np.zeros(3), orientation_error=np.zeros(3),
            torque_saturated=saturated, contact_count=contacts,
        )

    in_window = Log()
    in_window.reset_logs()
    for k in range(20):
        # 单个测试窗内异常，后续大量零误差样本不应改变其统计结果。
        add_sample(in_window, k * 0.1, 0.020 if k == 0 else 0.001,
                   10.0 if k == 0 else 2.0, k == 0, 3 if k == 0 else 0)
    reference = compute_tracking_metrics(in_window, segments)

    with_hold = Log()
    with_hold.reset_logs()
    for k in range(20):
        add_sample(with_hold, k * 0.1, 0.020 if k == 0 else 0.001,
                   10.0 if k == 0 else 2.0, k == 0, 3 if k == 0 else 0)
    for k in range(1000):
        add_sample(with_hold, 2.0 + k * 0.1, 0.0, 0.0, False, 0)
    held = compute_tracking_metrics(with_hold, segments)

    expected_rms = np.sqrt((0.020 ** 2 + 19 * 0.001 ** 2) / 20)
    assert reference.overall.position_rms_m == pytest.approx(expected_rms)
    assert reference.overall.position_peak_m == pytest.approx(0.020)
    assert reference.peak_joint_torque_Nm == pytest.approx(10.0)
    assert reference.torque_saturation_ratio == pytest.approx(1.0 / 20.0)
    assert reference.max_contacts == 3
    assert held == reference
