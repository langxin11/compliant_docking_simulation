"""
se3_free_space_plots.py — SE(3) Lie 阻抗自由空间验证曲线可视化

复用 experiments/se3_free_space.py 的已验证闭环，重跑四组核心实验并输出
figure/se3_free_space/ 下的对比图（SciencePlots IEEE 风格）：

1. 平移阶跃三档惯量 + 二阶理论解叠加（超调随 A 增大 = 惯量重塑证据）
2. 旋转阶跃三档惯量 + 理论解叠加
3. 179° 大角度姿态调节收敛
4. 零刚度方向：body-y 外力撤除不回位 / body-z 高刚度恢复

用法：uv run python experiments/se3_free_space_plots.py [--out figure/se3_free_space]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from experiments.se3_free_space import DT, _make_env, _run

OUT_DEFAULT = Path(__file__).resolve().parents[1] / "figure" / "se3_free_space"


def _second_order_step(t, target, A, D, K):
    """二阶系统 A ẍ + D ẋ + K x = 0（阶跃到 target）的解析解。"""
    wn = np.sqrt(K / A)
    zeta = D / (2.0 * np.sqrt(A * K))
    if zeta < 1.0:
        wd = wn * np.sqrt(1.0 - zeta * zeta)
        env = np.exp(-zeta * wn * t)
        return target * (1.0 - env * (np.cos(wd * t)
                                       + zeta / np.sqrt(1 - zeta**2) * np.sin(wd * t)))
    raise ValueError("仅绘制欠阻尼解析解")


def _set_impedance(ctrl, A_lin, D_lin, K_lin, A_rot, D_rot, K_rot):
    ctrl.A = np.diag([A_lin] * 3 + [A_rot] * 3)
    ctrl.D = np.diag([D_lin] * 3 + [D_rot] * 3)
    ctrl.K = np.diag([K_lin] * 3 + [K_rot] * 3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=OUT_DEFAULT)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    import matplotlib.pyplot as plt

    from compliant_docking.plotting import apply_style
    apply_style()
    # 含 $...$ 的字符串走 mathtext 解析路径：其非数学段只取 font.family
    # 首位字体，不建立逐字形回退链（首位 Times New Roman 无中文字形 →
    # 豆腐块）。故把 CJK 字体提到首位——它自带拉丁字形，中文/西文/数学
    # （STIX mathtext 不受影响）混排全部正常；纯文本路径的回退不受影响
    plt.rcParams["font.family"] = ["Noto Serif CJK SC"] + [
        f for f in plt.rcParams["font.family"] if f != "Noto Serif CJK SC"]

    fig, axes = plt.subplots(2, 2, figsize=(9.5, 7.0), constrained_layout=True)

    # ------------------------------------------------------------------
    # (a) 平移阶跃：D=K=20，A ∈ {0.5, 5, 100}（论文 Eq. 68-69）
    # ------------------------------------------------------------------
    ax = axes[0, 0]
    t_dur, target = 9.0, 0.10
    colors = {0.5: "#013A63", 5.0: "#1789FC", 100.0: "#E4572E"}
    labels = {0.5: "A=0.5 kg（过阻尼）", 5.0: "A=5 kg（临界）",
              100.0: "A=100 kg（欠阻尼）"}
    for A in (0.5, 5.0, 100.0):
        scene, ctrl, muj, q_init, T0 = _make_env()
        _set_impedance(ctrl, A, 20.0, 20.0,
                       ctrl.A[3, 3], ctrl.D[3, 3], ctrl.K[3, 3])
        T_d = T0 * pin.SE3(np.eye(3), np.array([target, 0.0, 0.0]))
        _, _, y_body, _, _ = _run(ctrl, muj, q_init, T_d, t_dur)
        t = np.arange(len(y_body)) * DT
        x = 0.10 - y_body[:, 0]  # λ 误差坐标 → 物理阶跃位移
        (line,) = ax.plot(t, x, color=colors[A], label=labels[A])
    t_th = np.arange(0.0, t_dur, 0.01)
    ax.plot(t_th, _second_order_step(t_th, target, 100.0, 20.0, 20.0),
            "--", color=colors[100.0], lw=0.8, alpha=0.7,
            label="A=100 理论解 $A\\ddot x+D\\dot x+Kx=0$")
    ax.axhline(target, color="gray", lw=0.5, ls=":")
    ax.set_xlabel("时间 [s]")
    ax.set_ylabel("body-x 位移 [m]")
    ax.set_title("(a) 平移阶跃：固定 D=K=20 改变惯量 A\n超调 0%/0%/48.6% 随 A 增大（惯量重塑）")
    ax.legend(fontsize=6.5, loc="lower right")

    # ------------------------------------------------------------------
    # (b) 旋转阶跃：D=K=10，A_rot ∈ {0.5, 2.5, 25}（论文 Eq. 70-71）
    # ------------------------------------------------------------------
    ax = axes[0, 1]
    theta_target = np.deg2rad(60.0)
    t_dur = 12.0
    colors_r = {0.5: "#013A63", 2.5: "#1789FC", 25.0: "#E4572E"}
    labels_r = {0.5: "A$_r$=0.5（过阻尼）", 2.5: "A$_r$=2.5（临界）",
                25.0: "A$_r$=25（欠阻尼）"}
    for A in (0.5, 2.5, 25.0):
        scene, ctrl, muj, q_init, T0 = _make_env()
        _set_impedance(ctrl, ctrl.A[0, 0], ctrl.D[0, 0], ctrl.K[0, 0],
                       A, 10.0, 10.0)
        T_d = T0 * pin.exp6(pin.Motion(
            np.concatenate([np.zeros(3), theta_target * np.array([0.0, 0.0, 1.0])])))
        _, _, y_body, _, _ = _run(ctrl, muj, q_init, T_d, t_dur)
        t = np.arange(len(y_body)) * DT
        theta = theta_target - y_body[:, 5]
        ax.plot(t, np.rad2deg(theta), color=colors_r[A], label=labels_r[A])
    t_th = np.arange(0.0, t_dur, 0.01)
    ax.plot(t_th, np.rad2deg(_second_order_step(t_th, theta_target, 25.0, 10.0, 10.0)),
            "--", color=colors_r[25.0], lw=0.8, alpha=0.7, label="A$_r$=25 理论解")
    ax.axhline(np.rad2deg(theta_target), color="gray", lw=0.5, ls=":")
    ax.set_xlabel("时间 [s]")
    ax.set_ylabel("body-z 转角 [deg]")
    ax.set_title("(b) 旋转阶跃：固定 D=K=10 改变转动惯量 A$_r$\n欠阻尼振荡振幅随 A$_r$ 增大")
    ax.legend(fontsize=6.5, loc="lower right")

    # ------------------------------------------------------------------
    # (c) 179° 大角度姿态调节（主 log 分支内，无表示奇异）
    # ------------------------------------------------------------------
    ax = axes[1, 0]
    scene, ctrl, muj, q_init, T0 = _make_env()
    T_d = T0 * pin.exp6(pin.Motion(
        np.concatenate([np.zeros(3), np.deg2rad(179.0) * np.array([0.0, 0.0, 1.0])])))
    _, lam_r, _, _, _ = _run(ctrl, muj, q_init, T_d, 10.0)
    t = np.arange(len(lam_r)) * DT
    ax.plot(t, np.rad2deg(lam_r), color="#1789FC")
    ax.set_xlabel("时间 [s]")
    ax.set_ylabel("姿态误差 |λ$_r$| [deg]")
    ax.set_title("(c) 179° 大角度姿态调节\nlog6 指数坐标收敛，无 Euler 表示奇异")
    ax.annotate(f"t=10s 误差 {np.rad2deg(lam_r[-1]):.2f}°",
                xy=(t[-1], lam_r[-1]), xycoords="data",
                xytext=(0.55, 0.75), textcoords="axes fraction", fontsize=7,
                arrowprops=dict(arrowstyle="->", lw=0.6))

    # ------------------------------------------------------------------
    # (d) 零刚度方向（论文 Eq. 73 设计结构：body-y K=0）
    # ------------------------------------------------------------------
    ax = axes[1, 1]
    scene, ctrl, muj, q_init, T0 = _make_env()
    ctrl.K = np.diag([100.0, 0.0, 2000.0, 0.0, 20.0, 60.0])
    ctrl.A = np.diag([4.0, 4.0, 4.0, 1.0, 1.0, 1.0])
    ctrl.D = np.diag([40.0, 40.0, 40.0, 10.0, 10.0, 10.0])
    f_world = np.asarray(T0.rotation) @ np.array([0.0, 8.0, 0.0])
    push = (0.8, 2.8)
    _, _, y_body, _, _ = _run(ctrl, muj, q_init, T0, 5.0,
                              xfrc_world=f_world, xfrc_window=push)
    t = np.arange(len(y_body)) * DT
    ax.axvspan(push[0], push[1], color="#FFC914", alpha=0.25, label="外力窗口 8 N")
    ax.plot(t, y_body[:, 1], color="#E4572E", label="body-y（K=0，不回位）")
    ax.plot(t, y_body[:, 2], color="#013A63", label="body-z（K=2000，恢复）")
    ax.set_xlabel("时间 [s]")
    ax.set_ylabel("body 系位移 [m]")
    ax.set_title("(d) 零刚度方向实现（论文 Eq. 73 结构）\ny 向力撤后漂移 "
                 f"{abs(y_body[-1, 1]):.2f} m 不回位，z 向回零")
    ax.legend(fontsize=7, loc="upper left")

    for stem in ("se3_free_space_validation",):
        fig.savefig(args.out / f"{stem}.png", dpi=300)
        fig.savefig(args.out / f"{stem}.pdf")
    print(f"已保存: {args.out / 'se3_free_space_validation.png'}")
    print(f"已保存: {args.out / 'se3_free_space_validation.pdf'}")


if __name__ == "__main__":
    main()
