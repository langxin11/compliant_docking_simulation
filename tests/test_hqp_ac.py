"""HQP-AC 控制器套件测试：单元（自适应刚度/参考阻尼/姿态误差）+ 短仿真不变量。

3 s 仿真（默认场景 iiwa14_docking.yaml，回路写法参照 tests/test_metrics.py 的
fixture）验证 QP 硬约束生效、求解稳健与跟踪精度；12 s 仿真（含接触段）对关节
角度占比做软校验——只打印实际值，不作为失败依据（基线阻抗控制器为 100.76%）。
运行方式：pytest -m "not slow"（默认套件）。
"""
import numpy as np
import pinocchio as pin
import pytest

from compliant_docking.config import HQPConfig
from compliant_docking.control.hqp_ac import HQPAdaptiveController
from compliant_docking.metrics import compute_metrics
from compliant_docking.models import load_pin_model
from compliant_docking.planning.kinematics import compute_ik
from compliant_docking.planning.trajectory import DecoupledQuinticTrajectory
from compliant_docking.scene import DEFAULT_SCENE_PATH, load_scene
from compliant_docking.simulation.mujoco_env import MujRobot
from compliant_docking.telemetry import Log

# 场景驱动：与 test_metrics.py 同一初始条件来源
_SCENE = load_scene(DEFAULT_SCENE_PATH)
INIT_POS = _SCENE.task.init_pos
INIT_ORI = _SCENE.task.init_ori
IK_GUESS = _SCENE.task.ik_guess
STROKE = _SCENE.task.stroke
AXIS = STROKE / np.linalg.norm(STROKE)

DT = 0.001
TORQUE_LIMIT = 10.0  # 与 DockingConfig.max_torque 同值，QP 硬约束与 clip 同幅值
DURATION_3S = 3.0    # 3 s 时尚未接触（接触约发生在 9-11 s）
DURATION_12S = 12.0  # 12 s 覆盖接触段，用于关节角度占比软校验


def _run_hqp(duration: float) -> tuple:
    """场景驱动的 HQP-AC 对接回路（无渲染/无录帧/无逐行打印）。

    返回 (log, controller, q_init, pin_model, raw_taus)；raw_taus 为 clip 前
    的控制器原始输出（用于断言 QP 力矩约束本身生效）。
    """
    pin_model = load_pin_model(_SCENE.robot.pin_model)
    pin_data = pin_model.createData()
    init_pose = pin.SE3(INIT_ORI, INIT_POS)
    q_init, success = compute_ik(pin_model, pin_data, init_pose, initial_q=IK_GUESS,
                                 max_iters=5000, ee_frame=_SCENE.robot.ee_frame)
    assert success, "标准初始位姿的 IK 应收敛"

    controller = HQPAdaptiveController(
        pin_model, DT, HQPConfig(torque_limit=TORQUE_LIMIT),
        ee_frame=_SCENE.robot.ee_frame, r_des=INIT_ORI)
    traj = DecoupledQuinticTrajectory(INIT_POS, INIT_POS + STROKE, 15.0)
    muj_robot = MujRobot(model=_SCENE.build_mjmodel(), render=False, record=False,
                         dt=DT, target_pos=INIT_POS + STROKE, eef_body=_SCENE.eef_body)
    muj_robot.init_simulators(q_init)

    log = Log()
    log.reset_logs()
    q = q_init
    v = np.zeros(7)
    current_pos, current_vel, _ = controller.get_task_space_state(q, v)
    force_external = np.zeros(3)
    torque_external = np.zeros(3)
    raw_taus: list[np.ndarray] = []

    while muj_robot.data.time < duration:
        t = muj_robot.data.time
        pos_des, vel_des, acc_des = traj.get_state(t)
        tau_raw = controller.compute_control_task_space_with_orientation_and_imp(
            q, v, pos_des, vel_des, acc_des, current_pos, current_vel,
            force_external, torque_external)
        raw_taus.append(np.array(tau_raw))
        tau = np.clip(tau_raw, -TORQUE_LIMIT, TORQUE_LIMIT)
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

    return log, controller, q_init, pin_model, raw_taus


@pytest.fixture(scope="module")
def hqp_3s() -> tuple:
    """3 s 短仿真（未接触），模块内共享一次仿真。"""
    return _run_hqp(DURATION_3S)


@pytest.fixture(scope="module")
def hqp_12s() -> tuple:
    """12 s 仿真（覆盖接触段），产出 (max_joint_pos_pct, pin_model)。"""
    log, _, _, pin_model, _ = _run_hqp(DURATION_12S)
    metrics = compute_metrics(log, pin_model, axis=AXIS, ee_frame=_SCENE.robot.ee_frame)
    return metrics.max_joint_pos_pct, pin_model


# ----------------------------------------------------------------------
# 单元测试（不需要仿真）
# ----------------------------------------------------------------------
def test_adaptive_stiffness_monotone(hqp_3s):
    """自适应刚度单调：F=50 时各分量 ≤ F=1 时对应分量，且落在 [K_min, K0] 内。"""
    _, controller, _, _, _ = hqp_3s
    cfg = controller.config
    K_r_low_F = controller._adaptive_stiffness(1.0)
    K_r_high_F = controller._adaptive_stiffness(50.0)
    assert np.all(K_r_high_F <= K_r_low_F + 1e-12), (
        f"刚度应随接触力增大而减小: F=1 -> {K_r_low_F}, F=50 -> {K_r_high_F}")
    K_min = cfg.K_min_ratio * np.asarray(cfg.K0)
    assert np.all(K_r_low_F <= np.asarray(cfg.K0) + 1e-12)
    assert np.all(K_r_low_F >= K_min - 1e-12)
    assert np.all(K_r_high_F >= K_min - 1e-12)


def test_reference_damping_symmetric_psd(hqp_3s):
    """Eq.(29) 参考阻尼 D_r 在工作构型处对称且半正定。

    注：D_r = Λ^(1/2)K_r^(1/2) + K_r^(1/2)Λ^(1/2) 对任意不交换因子并不普遍
    保证半正定；此处按控制器实际运行的工作点（场景初始位姿的 Λ 与接触力
    F∈{0, 1, 50} N 的 K_r）验证，与运行期行为一致。
    """
    _, controller, q_init, _, _ = hqp_3s
    pin_data = controller.data
    pin.forwardKinematics(controller.model, pin_data, q_init)
    M = np.array(pin.crba(controller.model, pin_data, q_init))
    M = np.triu(M) + np.triu(M, 1).T
    J = np.array(pin.computeFrameJacobian(
        controller.model, pin_data, q_init, controller.frame_id,
        pin.ReferenceFrame.WORLD))
    A_task = J @ np.linalg.pinv(M) @ J.T
    Lambda = 0.5 * (np.linalg.pinv(A_task) + np.linalg.pinv(A_task).T)
    for F in (0.0, 1.0, 50.0):
        D_r = controller._reference_damping(Lambda, controller._adaptive_stiffness(F))
        assert np.allclose(D_r, D_r.T), f"F={F} N 时 D_r 不对称"
        min_eig = float(np.linalg.eigvalsh(0.5 * (D_r + D_r.T)).min())
        assert min_eig >= -1e-8, f"F={F} N 时 D_r 最小特征值 {min_eig:.6f} < 0"


def test_orientation_error_zero_at_desired(hqp_3s):
    """场景初始位姿的 R_cur 与 r_des 同口径，log3(R_d R_curᵀ) 应接近 0。"""
    _, controller, q_init, _, _ = hqp_3s
    _, _, R_cur = controller.get_task_space_state(q_init, np.zeros(7))
    e_ori = pin.log3(controller.r_des @ R_cur.T)
    assert np.linalg.norm(e_ori) < 1e-6, f"初始位姿姿态误差应为 0: {e_ori}"


# ----------------------------------------------------------------------
# 3 s 仿真不变量（QP 约束生效 / 求解稳健 / 跟踪精度）
# ----------------------------------------------------------------------
def test_sim_3s_invariants(hqp_3s):
    """3 s 仿真：无 NaN；原始力矩全程满足 ±max_torque；求解零失败；
    平均 QP 耗时 < 5 ms；末端位置误差 < 5 mm。"""
    log, controller, _, _, raw_taus = hqp_3s
    taus = np.array(raw_taus)
    assert np.all(np.isfinite(taus)), "控制器输出存在 NaN/Inf"

    max_abs_tau = float(np.abs(taus).max())
    assert max_abs_tau <= TORQUE_LIMIT + 1e-6, (
        f"QP 力矩硬约束未生效: max|tau|={max_abs_tau:.6f} > {TORQUE_LIMIT}")

    assert controller.n_solver_failures == 0, (
        f"ProxQP 求解失败 {controller.n_solver_failures} 次")

    qp_ms = controller.last_solve_time_ms
    print(f"\n[HQP-AC 3s] 平均 QP 求解耗时 = {qp_ms:.4f} ms/call "
          f"({len(raw_taus)} 次调用, {controller.n_solver_failures} 次失败)")
    assert qp_ms < 5.0, f"平均 QP 耗时超限: {qp_ms:.4f} ms"

    final_err = float(log.error[-1])
    print(f"[HQP-AC 3s] 末端位置误差 = {final_err * 1e3:.4f} mm")
    assert final_err < 5e-3, f"末端位置误差超限: {final_err:.6f} m"


# ----------------------------------------------------------------------
# 12 s 仿真软校验（不作为失败依据）
# ----------------------------------------------------------------------
def test_sim_12s_joint_limit_soft(hqp_12s):
    """12 s（含接触段）关节角度占比软校验：打印实际值供对比基线 100.76%。"""
    pct, _ = hqp_12s
    assert pct is not None and np.isfinite(pct)
    print(f"\n[HQP-AC 12s] 最大关节角度占比 = {pct:.2f}%（基线 impedance 单段 100.76%，"
          f"软目标 ≤ 102%，不作为失败依据）")
