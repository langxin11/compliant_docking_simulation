"""
se3_free_space.py — SE(3) Lie 阻抗控制器自由空间验证实验（任务书 §23.2-23.6）

在无接触跟踪场景（无母头、零重力）中按论文 Section IV-A 的实验设计
验证 SE3LieImpedanceController 的动力学渲染能力：

1. static   静态位姿调节（小平移+小旋转误差 → λ→0，无 NaN/力矩爆炸）
2. trans_step 平移阶跃（D=K=20 固定，A ∈ {0.5, 5, 100}；论文 Eq. 68-69，
   过阻尼/临界/欠阻尼三档——若响应不随 A 明显变化则说明退化为普通 PD）
3. rot_step  旋转阶跃（D=K=10 固定，A_rot ∈ {0.5, 2.5, 25}；论文 Eq. 70-71）
4. large_rot 179° 大角度姿态调节（主 log 分支内收敛，无 Euler 奇异）
5. zero_K    零刚度方向（body-y 平动 K=0）：外力撤除后不回位，
   高刚度方向（z）保持恢复

用法：uv run python experiments/se3_free_space.py [--quick]
退出码非零 = 任一实验失败。参数取论文 Eq. 68-73，非本仓库对接基线值。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from compliant_docking.control.se3_impedance import SE3LieImpedanceController
from compliant_docking.models import load_pin_model
from compliant_docking.planning.kinematics import compute_ik
from compliant_docking.scene import load_scene
from compliant_docking.simulation.mujoco_env import MujRobot

SCENE_PATH = Path(__file__).resolve().parents[1] / "scenes" / "iiwa14_tracking.yaml"
MAX_TORQUE = 10.0
DT = 0.001


def _make_env():
    scene = load_scene(SCENE_PATH)
    pin_model = load_pin_model(scene.robot.pin_model)
    controller = SE3LieImpedanceController(
        pin_model, DT, ee_frame=scene.robot.ee_frame)
    init_pose = pin.SE3(scene.task.init_ori, scene.task.init_pos)
    q_init, ok = compute_ik(pin_model, pin_model.createData(), init_pose,
                            initial_q=scene.task.ik_guess, max_iters=5000,
                            ee_frame=scene.robot.ee_frame)
    assert ok, "初始 IK 未收敛"
    muj = MujRobot(model=scene.build_mjmodel(), render=False, record=False,
                   dt=DT, target_pos=np.asarray(scene.task.init_pos),
                   eef_body=scene.eef_body)
    return scene, controller, muj, q_init, init_pose


def _run(controller: SE3LieImpedanceController, muj: MujRobot, q_init,
         T_d, duration: float, xfrc_world=None, xfrc_window=(0.5, None)):
    """固定 T_d 的自由空间闭环，返回 λ 与 EE body 系位移时序。"""
    muj.init_simulators(q_init)
    q, v = q_init.copy(), np.zeros(controller.nv)
    lam_t, lam_r, y_body, tau_max = [], [], [], 0.0
    n_steps = int(duration / DT)
    body_id = muj.eef_id
    for k in range(n_steps):
        if xfrc_world is not None and xfrc_window[0] <= k * DT and (
                xfrc_window[1] is None or k * DT < xfrc_window[1]):
            muj.data.xfrc_applied[body_id, :3] = xfrc_world
        else:
            muj.data.xfrc_applied[body_id, :] = 0.0
        tau = controller.compute_control(
            q, v, T_d, np.zeros(6), np.zeros(6), np.zeros(6))
        tau = np.clip(tau, -MAX_TORQUE, MAX_TORQUE)
        tau_max = max(tau_max, float(np.max(np.abs(tau))))
        q, v, _ = muj.step(tau)
        d = controller.latest_diagnostics
        lam_t.append(d["lam_translation_norm"])
        lam_r.append(d["lam_rotation_norm"])
        # EE 相对 T_d 的 body-y/z 位移（物理量，符号与 λ 反号）
        T = controller.get_body_state(q, v)[0]
        rel = T.inverse() * T_d
        y_body.append(np.array(pin.log6(rel).vector))
    return (np.array(lam_t), np.array(lam_r),
            np.array(y_body), tau_max, n_steps * DT)


def _overshoot_pct(resp, target):
    """阶跃响应超调（resp 为趋近 target 的位移曲线，起点 0）。"""
    peak = np.max(np.abs(resp))
    return max(0.0, (peak / abs(target) - 1.0)) * 100.0


# ----------------------------------------------------------------------
def exp_static(cfg_reports):
    """§23.2 静态位姿调节：小误差 → λ→0。"""
    scene, ctrl, muj, q_init, T0 = _make_env()
    T_d = T0 * pin.exp6(pin.Motion(np.array([0.01, -0.008, 0.006,
                                              0.02, -0.015, 0.01])))
    lam_t, lam_r, _, tau_max, _ = _run(ctrl, muj, q_init, T_d, 8.0)
    ok = (lam_t[-1] < 1e-3 and lam_r[-1] < 1e-2
          and np.all(np.isfinite(lam_t)) and tau_max < 1e6)
    cfg_reports.append(("static", ok, {
        "final |λ_t|": f"{lam_t[-1]:.2e} m", "final |λ_r|": f"{lam_r[-1]:.2e} rad",
        "tau_max": f"{tau_max:.2f} N·m"}))
    return ok


def exp_translation_step(cfg_reports, quick=False):
    """§23.3 平移阶跃：固定 D=K=20，改变 A（论文 Eq. 68-69）。

    时长需覆盖 A=100 的欠阻尼峰值（ω_n=√(K/A)≈0.45 rad/s，t_peak≈7s）。
    """
    dur = 9.0 if quick else 20.0
    results = {}
    for A in (0.5, 5.0, 100.0):
        scene, ctrl, muj, q_init, T0 = _make_env()
        ctrl.A = np.diag([A, A, A, ctrl.A[3, 3], ctrl.A[4, 4], ctrl.A[5, 5]])
        ctrl.D = np.diag([20.0, 20.0, 20.0, ctrl.D[3, 3], ctrl.D[4, 4], ctrl.D[5, 5]])
        ctrl.K = np.diag([20.0, 20.0, 20.0, ctrl.K[3, 3], ctrl.K[4, 4], ctrl.K[5, 5]])
        T_d = T0 * pin.SE3(np.eye(3), np.array([0.10, 0.0, 0.0]))
        lam_t, _, y_body, tau_max, _ = _run(ctrl, muj, q_init, T_d, dur)
        # 阶跃响应位移：起点 0、目标 0.10（λ_x 为误差坐标，起始 +0.10 → 0，
        # 超调体现为越过 0 到负值，故位移 = 0.10 − λ_x）
        results[A] = (lam_t, 0.10 - y_body[:, 0], tau_max)
        assert np.all(np.isfinite(lam_t)), f"A={A} 出现 NaN"
    # 期望动态应随 A 显著变化（过阻尼 → 临界 → 欠阻尼）：欠阻尼出现超调且
    # 随 A 增大；过阻尼无超调。若三者几乎相同则退化成简单 PD。
    overshoot = {A: _overshoot_pct(y, 0.10) for A, (_, y, _) in results.items()}
    reached = {A: float(np.max(np.abs(y))) for A, (_, y, _) in results.items()}
    ok = (overshoot[0.5] < 2.0 and overshoot[5.0] < 5.0
          and overshoot[100.0] > 10.0
          and all(r > 0.08 for r in reached.values()))
    cfg_reports.append(("trans_step", ok, {
        "overshoot% (A=0.5/5/100)": "/".join(f"{overshoot[A]:.1f}" for A in results),
        "peak travel m (0.5/5/100)": "/".join(f"{reached[A]:.3f}" for A in results),
        "tau_max": "/".join(f"{results[A][2]:.1f}" for A in results)}))
    return ok


def exp_rotation_step(cfg_reports, quick=False):
    """§23.4 旋转阶跃：固定 D=K=10，改变 A_rot（论文 Eq. 70-71）。

    A=25 欠阻尼（ζ=0.32、ω_n=0.63 rad/s）需 ~25s 包络收敛。
    """
    dur = 9.0 if quick else 26.0
    results = {}
    for A in (0.5, 2.5, 25.0):
        scene, ctrl, muj, q_init, T0 = _make_env()
        ctrl.A = np.diag([ctrl.A[0, 0], ctrl.A[1, 1], ctrl.A[2, 2], A, A, A])
        ctrl.D = np.diag([ctrl.D[0, 0], ctrl.D[1, 1], ctrl.D[2, 2], 10.0, 10.0, 10.0])
        ctrl.K = np.diag([ctrl.K[0, 0], ctrl.K[1, 1], ctrl.K[2, 2], 10.0, 10.0, 10.0])
        theta = np.deg2rad(60.0)
        T_d = T0 * pin.exp6(pin.Motion(
            np.concatenate([np.zeros(3), theta * np.array([0.0, 0.0, 1.0])])))
        # _run 返回 (λ_t, λ_r, λ6, τ_max, T)——旋转范数在第 2 位
        _, lam_r, _, tau_max, _ = _run(ctrl, muj, q_init, T_d, dur)
        results[A] = lam_r
        assert np.all(np.isfinite(lam_r)), f"A_rot={A} 出现 NaN"
    final = {A: float(response[-1]) for A, response in results.items()}
    # 欠阻尼（A=25）应出现明显振荡（相邻极值差 > 5% θ），过阻尼（A=0.5）单调
    def oscillation(response):
        peaks = []
        for i in range(1, len(response) - 1):
            if (response[i - 1] > response[i] < response[i + 1]
                    or response[i - 1] < response[i] > response[i + 1]):
                peaks.append(response[i])
        return float(np.ptp(peaks)) if len(peaks) >= 2 else 0.0
    osc = {A: oscillation(response) for A, response in results.items()}
    ok = (final[0.5] < 0.02 and osc[25.0] > 0.30 and osc[0.5] < 0.05
          and np.isfinite(osc[2.5]))
    cfg_reports.append(("rot_step", ok, {
        "final |λ_r| rad": "/".join(f"{final[A]:.3f}" for A in results),
        "oscillation rad (0.5/2.5/25)": "/".join(f"{osc[A]:.3f}" for A in results)}))
    return ok


def exp_large_rotation(cfg_reports, quick=False):
    """§23.5 179° 姿态调节：主 log 分支内收敛、无表示奇异。"""
    dur = 8.0 if quick else 20.0
    scene, ctrl, muj, q_init, T0 = _make_env()
    theta = np.deg2rad(179.0)
    axis = np.array([0.0, 0.0, 1.0])
    T_d = T0 * pin.exp6(pin.Motion(np.concatenate([np.zeros(3), theta * axis])))
    lam_r, _, _, tau_max, _ = _run(ctrl, muj, q_init, T_d, dur)
    finite = np.all(np.isfinite(lam_r))
    converged = lam_r[-1] < 0.05
    cfg_reports.append(("large_rot_179deg", finite and converged, {
        "final |λ_r|": f"{lam_r[-1]:.4f} rad", "tau_max": f"{tau_max:.2f} N·m",
        "min |λ_r|": f"{lam_r.min():.4f} rad"}))
    return finite and converged


def exp_zero_stiffness(cfg_reports, quick=False):
    """§23.6 零刚度方向（论文 Eq. 73 的 y 轴设计）。"""
    push_s = 1.0
    dur = 4.0 if quick else 8.0
    scene, ctrl, muj, q_init, T0 = _make_env()
    # K = diag(100, 0, 2000, 0, 20, 60)：body-y 零平动刚度（论文 Eq. 73 结构）
    ctrl.K = np.diag([100.0, 0.0, 2000.0, 0.0, 20.0, 60.0])
    ctrl.A = np.diag([4.0, 4.0, 4.0, 1.0, 1.0, 1.0])
    ctrl.D = np.diag([40.0, 40.0, 40.0, 10.0, 10.0, 10.0])
    # 沿 EE body-y 推力（世界系表达）
    R0 = np.asarray(T0.rotation)
    f_body_y = 8.0
    f_world = R0 @ np.array([0.0, f_body_y, 0.0])
    lam_t, _, y_body, _, _ = _run(ctrl, muj, q_init, T0, dur,
                                  xfrc_world=f_world,
                                  xfrc_window=(push_s, push_s + 2.0))
    # 力窗结束后的残余位移：body-y 不回位（零刚度），body-z 保持
    idx_push_end = int((push_s + 2.0) / DT)
    y_final = y_body[-1]
    y_at_release = y_body[min(idx_push_end + 10, len(y_body) - 1)]
    residual_y = abs(y_final[1])
    residual_z = abs(y_final[2])
    drift_ok = residual_y > 0.02  # ≥2 cm 残余漂移（不回位）
    stiff_ok = residual_z < 0.005  # 高刚度方向恢复
    cfg_reports.append(("zero_stiffness", drift_ok and stiff_ok, {
        "residual body-y": f"{residual_y:.3f} m（应 >0.02 不回位）",
        "residual body-z": f"{residual_z:.4f} m（应 <0.005 恢复）",
        "y at release": f"{y_at_release[1]:.3f} m"}))
    return drift_ok and stiff_ok


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="缩短时长快速验证")
    args = parser.parse_args()

    reports: list = []
    results = {
        "static": exp_static(reports),
        "trans_step": exp_translation_step(reports, args.quick),
        "rot_step": exp_rotation_step(reports, args.quick),
        "large_rot": exp_large_rotation(reports, args.quick),
        "zero_stiffness": exp_zero_stiffness(reports, args.quick),
    }
    print("\n" + "=" * 64)
    print("SE(3) Lie 阻抗自由空间验证（Kim et al. 2025 §IV-A 设计）")
    print("=" * 64)
    for name, ok, detail in reports:
        status = "PASS" if ok else "FAIL"
        print(f"[{status}] {name}")
        for k, v in detail.items():
            print(f"        {k}: {v}")
    n_fail = sum(1 for ok in results.values() if not ok)
    print("=" * 64)
    print(f"结果: {len(results) - n_fail}/{len(results)} PASS")
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
