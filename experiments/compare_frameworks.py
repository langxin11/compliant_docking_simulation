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
from experiments.run_docking import main  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
RESULTS = REPO / "results"

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


def run_config(scene: Scene, controller: str) -> tuple[DockingMetrics, float]:
    """跑一个配置并返回（Table10 指标, 仿真终态误差 mm）。"""
    with contextlib.redirect_stdout(io.StringIO()):
        log = main(render=False, record=False, plot=False, scene=scene,
                   controller=controller)
    pin_model = _pin_model_for(scene)
    axis = scene.task.stroke / np.linalg.norm(scene.task.stroke)
    metrics = compute_metrics(log, pin_model, axis=axis, ee_frame=scene.robot.ee_frame)
    return metrics, float(log.error[-1]) * 1000.0


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

        results, finals, names = {}, {}, []
        for name, planner, controller in CONFIGS:
            print(f"[{base.name}] 运行 {name} ...", flush=True)
            metrics, final_mm = run_config(variants[planner], controller)
            results[name], finals[name] = metrics, final_mm
            names.append(name)

        table = format_table(f"{base.name} 框架对比（2×2 配置矩阵）",
                             results, finals, names)
        out = RESULTS / f"framework_comparison_{base.name}.md"
        out.write_text(f"# 框架对比研究：{base.name}\n\n"
                       f"配置：{CONFIGS}\n\n{table}\n", encoding="utf-8")
        print(table, "\n", flush=True)
        print(f"已写入 {out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main_script())
