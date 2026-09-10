"""
compare_frameworks.py - 框架配置对比研究（复现 Ren & Shan 2026 §4.2.3 的对比设计）

2×2 配置矩阵：{单段解耦五次(=论文 Decoupled Planner), 两段式对接(限速)} ×
{CIC 经典阻抗(=论文 Classical Impedance Controller), HQP-AC}。论文的第三格
"SE(3)-TOPP + HQP-AC" 以两段式规划器作为其保守替代（本仓库未实现 SE(3)-TOPP）。

用法：
    MUJOCO_GL=egl uv run python experiments/compare_frameworks.py [--scene 场景YAML ...]

指标取自 compute_metrics（Table 10 三层：接触安全/内部安全/跟踪精度），
结果打印为对比表并写入 results/framework_comparison_<场景>.md。
"""
import argparse
import contextlib
import io
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from compliant_docking.metrics import DockingMetrics, compute_metrics  # noqa: E402
from compliant_docking.models import load_pin_model  # noqa: E402
from compliant_docking.scene import Scene, TrajectorySpec, load_scene  # noqa: E402
from compliant_docking.telemetry import Log  # noqa: E402
from experiments.run_docking import main  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
RESULTS = REPO / "results"
_FIG_WIDTH = 3.5  # IEEE 单栏图宽（英寸）

# 四配置的绘图样式（线图颜色/线型，柱图颜色同源）
_CONFIG_STYLES = [
    {"color": "0.35"},                      # 基线：灰
    {"color": "#1f77b4"},                   # 中间：蓝
    {"color": "#ff7f0e", "ls": "--"},       # 规划：橙虚线
    {"color": "#d62728"},                   # 完整：红
]
_CONFIG_SHORT = ["基线", "中间", "规划", "完整"]

# 指标呈现顺序与论文 Table 10 三层一致：(标签, 字段名, 格式)
_ROWS = [
    ("—— 接触安全 ——", None, None),
    ("峰值轴向力 [N]", "peak_axial_force_N", ".2f"),
    ("峰值广义力范数 [N]", "peak_wrench_norm_N", ".2f"),
    ("稳态轴向力 [N]", "steady_axial_force_N", ".2f"),
    ("稳态广义力范数 [N]", "steady_wrench_norm_N", ".2f"),
    ("—— 内部安全 ——", None, None),
    ("最大关节角度占比 [%]", "max_joint_pos_pct", ".2f"),
    ("最大关节速度占比 [%]", "max_joint_vel_pct", ".2f"),
    ("最小可操作度", "min_manipulability", ".4f"),
    ("—— 跟踪精度 ——", None, None),
    ("位置跟踪 RMS [mm]", "pos_tracking_rms_m", ".2f"),
    ("姿态跟踪 RMS [°]", "ori_tracking_rms_rad", ".3f"),
    ("稳态横向误差 [mm]", "final_lateral_error_m", ".2f"),
    ("稳态姿态误差 [°]", "final_orientation_error_rad", ".3f"),
]

# （配置名, 规划器, 控制器）——顺序对齐论文 §4.2.3 的渐进式三配置，
# 第四格（两段式+CIC）补全 2×2 以隔离规划器单独的贡献
CONFIGS = [
    ("基线: 单段五次+CIC", "single", "impedance"),
    ("中间: 单段五次+HQP-AC", "single", "hqp"),
    ("规划: 两段式+CIC", "twophase", "impedance"),
    ("完整: 两段式+HQP-AC", "twophase", "hqp"),
]


def _pin_model_for(scene: Scene):
    """按 run_docking.main 的同款规则构建 Pinocchio 模型（含工具惯量）。"""
    kwargs = {}
    if scene.tool.pin_inertia is not None:
        inertia = scene.tool.pin_inertia
        kwargs = dict(
            tool_frame=scene.robot.ee_frame,
            tool_mount_pos=scene.tool.pose_pos,
            tool_mount_quat=scene.tool.pose_quat,
            tool_mass=inertia.mass,
            tool_com=inertia.com,
            tool_diaginertia=inertia.diaginertia,
        )
    return load_pin_model(scene.robot.pin_model, **kwargs)


def run_config(scene: Scene, controller: str) -> tuple[DockingMetrics, float, Log]:
    """跑一个配置并返回（Table10 指标, 仿真终态误差 mm, 日志）。"""
    with contextlib.redirect_stdout(io.StringIO()):
        log = main(render=False, record=False, plot=False, scene=scene,
                   controller=controller)
    pin_model = _pin_model_for(scene)
    axis = scene.task.stroke / np.linalg.norm(scene.task.stroke)
    metrics = compute_metrics(log, pin_model, axis=axis, ee_frame=scene.robot.ee_frame)
    return metrics, float(log.error[-1]) * 1000.0, log


def format_table(title: str, results: dict[str, DockingMetrics],
                 finals: dict[str, float], config_names: list[str]) -> str:
    """对齐的 Markdown 对比表。"""
    header = "| 指标 | " + " | ".join(config_names) + " |"
    sep = "|" + "---|" * (len(config_names) + 1)
    lines = [f"### {title}", "", header, sep]
    for label, field, fmt in _ROWS:
        if field is None:
            lines.append(f"| **{label}** |" + " |" * len(config_names))
            continue
        cells = []
        for name in config_names:
            val = getattr(results[name], field)
            cells.append(f"{val:{fmt}}" if val is not None else "n/a")
        lines.append(f"| {label} | " + " | ".join(cells) + " |")
    lines.append("| 终态位置误差 [mm] | "
                 + " | ".join(f"{finals[n]:.1f}" for n in config_names) + " |")
    return "\n".join(lines)


def plot_comparison(scene: Scene, logs: dict[str, Log],
                    results: dict[str, DockingMetrics]) -> list[Path]:
    """四配置对比图：接触力时序（轴向分量 + 合力范数）与关键指标柱状。"""
    import matplotlib
    matplotlib.use("Agg")  # 无显示环境出图，必须在 import pyplot 之前
    import matplotlib.pyplot as plt

    from compliant_docking.plotting import apply_style

    apply_style()
    out = REPO / "figure" / "framework_comparison"
    out.mkdir(parents=True, exist_ok=True)
    saved: list[Path] = []

    def _save(fig, stem: str) -> None:
        for suffix in (".png", ".pdf"):
            path = out / f"{scene.name}_{stem}{suffix}"
            fig.savefig(path, dpi=600)
            saved.append(path)
        plt.close(fig)

    axis_idx = int(np.argmax(np.abs(scene.task.stroke)))

    # 1) 接触力时序：四配置叠加（轴向分量 / 合力范数）
    fig, axes = plt.subplots(2, 1, sharex=True,
                             figsize=(_FIG_WIDTH, 3.8), constrained_layout=True)
    for (name, _, _), style, short in zip(CONFIGS, _CONFIG_STYLES, _CONFIG_SHORT, strict=True):
        log = logs[name]
        t = np.asarray(log.t_list, dtype=float)
        f = np.asarray(log.force_externals, dtype=float).reshape(-1, 3)
        axes[0].plot(t, f[:, axis_idx], label=short, **style)
        axes[1].plot(t, np.linalg.norm(f, axis=1), label=short, **style)
    axes[0].set_ylabel("对接轴向力 [N]")
    axes[1].set_ylabel("接触力范数 [N]")
    axes[1].set_xlabel("时间 [s]")
    axes[0].legend(ncols=4, framealpha=0.9, fontsize=6,
                   columnspacing=1.0, handlelength=1.6)
    _save(fig, "contact_force")

    # 2) 关键接触指标柱状（三层指标中的接触安全层）
    bars = [("峰值轴向力 [N]", "peak_axial_force_N"),
            ("|稳态轴向力| [N]", "steady_axial_force_N"),
            ("峰值广义力范数 [N]", "peak_wrench_norm_N")]
    fig, axes = plt.subplots(1, 3, figsize=(_FIG_WIDTH, 2.4), constrained_layout=True)
    colors = [s["color"] for s in _CONFIG_STYLES]
    for ax, (label, field) in zip(axes, bars, strict=True):
        vals = [abs(getattr(results[n], field) or 0.0) for n in results]
        ax.bar(range(len(vals)), vals, color=colors)
        ax.set_xticks(range(len(vals)))
        ax.set_xticklabels(_CONFIG_SHORT, fontsize=6)
        ax.set_ylabel(label)
    _save(fig, "contact_metrics")
    return saved


def main_script() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scene", action="append", default=[
        "scenes/iiwa14_docking.yaml", "scenes/fr3_docking.yaml"])
    args = parser.parse_args()

    RESULTS.mkdir(exist_ok=True)
    for scene_path in args.scene:
        base = load_scene(REPO / scene_path)
        # 行程短于默认 standoff 时预对接点会退回起点（接近段长度为零），
        # 此时取 standoff = 行程一半，保证两段都为正长度
        stroke_len = float(np.linalg.norm(base.task.stroke))
        standoff = min(TrajectorySpec().standoff, 0.5 * stroke_len)
        twophase = replace(base, trajectory=TrajectorySpec(standoff=standoff))
        variants = {"single": base, "twophase": twophase}

        results, finals, names, logs = {}, {}, [], {}
        for name, planner, controller in CONFIGS:
            print(f"[{base.name}] 运行 {name} ...", flush=True)
            metrics, final_mm, log = run_config(variants[planner], controller)
            results[name], finals[name], logs[name] = metrics, final_mm, log
            names.append(name)

        table = format_table(f"{base.name} 框架对比（2×2 配置矩阵）",
                             results, finals, names)
        out = RESULTS / f"framework_comparison_{base.name}.md"
        out.write_text(f"# 框架对比研究：{base.name}\n\n"
                       f"配置：{CONFIGS}\n\n{table}\n", encoding="utf-8")
        print(table, "\n", flush=True)
        figures = plot_comparison(base, logs, results)
        print("图件：", ", ".join(str(p.relative_to(REPO)) for p in figures), flush=True)
        print(f"已写入 {out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main_script())
