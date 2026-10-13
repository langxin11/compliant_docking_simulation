"""Figures and a Chinese report from the selected design's audited saved data."""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib.pyplot as plt
import numpy as np

import experiments.prepare_petal_guidance as geometry
from compliant_docking.plotting import apply_style
from experiments.models_interfaces import selected_candidate as selected


def save(fig, out, name):
    for ext in ("png", "pdf"):
        fig.savefig(out / f"{name}.{ext}", dpi=180)
    plt.close(fig)


def profiles(out):
    apply_style("report", cjk_first=True)
    theta = np.linspace(-45., 45., 2001)
    fig, axes = plt.subplots(1, 2, figsize=(11.6, 4.4), constrained_layout=True)
    for directory, label, color in ((selected.DIRECTORIES["narrow"], "历史窄平顶：4.21875° / 0.1", "#718896"),
                                     (selected.MODEL, "用户候选：1° / 0.3", "#B07828")):
        p = json.loads((directory / "model_info.json").read_text())["parameters"]
        h = geometry.surface_height(np.full_like(theta, 41.), theta, p)
        slope = np.rad2deg(np.arctan(abs(np.gradient(h, np.deg2rad(theta))) / 41.))
        axes[0].plot(theta, h, label=label, color=color, lw=1.9)
        axes[1].plot(theta, slope, label=label, color=color, lw=1.9)
    axes[0].set(xlabel="花瓣中心角 [°]", ylabel="高度 [mm]", xlim=(-45, 45), ylim=(-.5, 19))
    axes[1].set(xlabel="花瓣中心角 [°]", ylabel="局部角向坡度 [°]", xlim=(-45, 45), ylim=(-1, 45))
    for axis in axes:
        axis.legend(fontsize=9)
    fig.suptitle("半径41 mm的解析工作面；不含边缘圆滑", fontsize=12)
    save(fig, out, "selected_profiles")


def traces(out):
    apply_style("report", cjk_first=True)
    fig, axes = plt.subplots(2, 3, figsize=(12.6, 6.6), constrained_layout=True)
    for column, case in enumerate(selected.CASES):
        for directory, group, label, color in ((selected.BASELINE, "narrow", "历史窄平顶", "#718896"),
                                              (out, "selected", "1° / 0.3", "#B07828")):
            with np.load(directory / f"{selected.study.name(group, case)}.npz") as a:
                t = a["t"]
                force = np.linalg.norm(a["diagnostic_interface_world"][:, :3], axis=1)
                visible = t >= 9.
                axes[0, column].plot(t[visible], a["lateral_mm"][visible], label=label, color=color, lw=1.)
                axes[1, column].plot(t[visible], force[visible], label=label, color=color, lw=1.)
                for row in range(2):
                    axes[row, column].set(xlim=(9., t[-1]), xlabel="时间 [s]")
                    axes[row, column].axvspan(t[-1]-1., t[-1], color=".94", zorder=-1)
        point = selected.POINTS[case]
        axes[0, column].set_title(f"({point[0]:g}, {point[1]:g}) mm / {point[2]:g}°")
        axes[0, column].axhline(.5, color="#A44D36", ls=":", lw=1., label="横向门槛0.5 mm")
        axes[0, column].set(ylabel="横向残差 [mm]", ylim=(-.15, 6.5))
        axes[1, column].set_ylabel("接触合力 [N]")
        axes[0, column].legend(fontsize=8)
        axes[1, column].legend(fontsize=8)
    fig.suptitle("三个代表工况；灰色历史参考，金色本次新仿真；末1秒为验收区间", fontsize=12)
    save(fig, out, "selected_traces")


def build(out):
    summary = json.loads((out / "summary.json").read_text())
    audit = json.loads((out / "partial_audit.json").read_text())
    assert audit["status"] == "PASS" and audit["records"] == 4
    assert len(json.loads((out / "archived_reference_checks.json").read_text())) == 4
    profiles(out)
    traces(out)
    header = (f"# 1°平顶半角、0.3斜坡过渡比例\n\n"
        f"新候选三个代表工况中 **{summary['passes']}/3** 通过原落座与载荷门槛。"
        "步长复查虽也通过落座门槛，但横向与相位差超过原数值一致性限值，标记为 `SENSITIVE`。"
        "因此先保留为候选，不将本次结果视为整体改进或稳定收敛证明。"
        "这是小范围代表验证，不等于完整捕获范围或制造贴合已经验证。\n\n"
        "只改变平顶半角（4.21875°→1°）与斜坡过渡比例（0.1→0.3）。"
        "导向高度18 mm、边缘圆滑1 mm、轴向余量0.4 mm、法兰间距46.4 mm，"
        "安装结构和止挡保持原值；整体惯量随导面重新计算。"
        "平顶总宽2°，每段43°斜坡两端各30%圆滑、中间40%直线。\n\n"
        "采用原零重力实验配置，摩擦系数0.15，进给速度10 mm/s；接触后逐步释放横向与绕轴定位刚度。"
        "主工况物理步长0.25 ms，控制周期与F/T延迟均为0.5 ms。"
        "旧窄平顶列来自冻结历史数据，本次没有重跑基线；其文件哈希、运行源码、"
        "同工况的参考位置和时间戳已核对。\n\n"
        "| XY / 绕轴误差 | 方案 | 横向残差 mm | 相位误差 ° | 轴向间隙 mm | 止挡接触 % | 峰值合力 N | 峰值轴向力矩 Nm | 结果 |\n"
        "|---|---|---:|---:|---:|---:|---:|---:|---|\n")
    rows = []
    for comparison in summary["comparisons"].values():
        p = comparison["error"]
        point = f"({p[0]:g}, {p[1]:g}) mm / {p[2]:g}°"
        for key, label in (("archived_narrow", "历史窄平顶"), ("selected", "1° / 0.3")):
            m = comparison[key]
            rows.append(f"| {point} | {label} | {m['lateral_mm']:.4f} | {m['phase_deg']:.4f} | "
                        f"{m['gap_mm']:.6f} | {100*m['stop_fraction']:.1f} | {m['force_N']:.2f} | "
                        f"{m['moment_Nm']:.4f} | {'通过' if m['status']=='CANDIDATE_PASS' else '未通过'} |")
    num = summary["nx6_timestep_check"]
    quarter = selected.metrics(json.loads((out / "g_selected_nx6_lateral_released_dt_quarter.json").read_text()))
    delta = num["differences"]
    tail = ("\n\n表中几何指标为最后1秒的最大残差，载荷为全程峰值。"
        "原门槛：横向≤0.5 mm、倾斜≤0.5°、相位≤2°、轴向间隙≤0.75 mm、止挡接触≥95%，"
        "合力≤40 N、轴向力矩≤2 Nm。\n\n"
        f"固定选择(-6,0) mm / 0°以0.125 ms复查：`{num['status']}`。"
        f"小步长结果也通过原落座与载荷门槛，横向残差{quarter['lateral_mm']:.4f} mm、"
        f"相位误差{quarter['phase_deg']:.4f}°、合力峰值{quarter['force_N']:.2f} N。"
        f"两步长横向差{delta['lateral_mm']:.6f} mm、相位差{delta['phase_deg']:.6f}°、"
        f"合力峰值差{delta['peak_force_N']:.4f} N。"
        "原一致性限值为横向/间隙差≤0.1 mm、相位差≤0.1°；此次横向与相位均略超限。"
        "建议后续先确认接触离散与步长敏感性，再扩展工况；本次没有继续改变几何参数或验收门槛。\n\n"
        "原生配合探针、11组MuJoCo/Pinocchio位姿与质量矩阵核对通过。"
        "4条新仿真的逐物理步数据已重新计算几何/载荷门槛并核对反馈延迟、力矩坐标变换、"
        f"接触前刚度与关节限制；异常记录：`{json.dumps(audit['simulation_issues'],ensure_ascii=False)}`。\n\n"
        "![解析轮廓与坡度](selected_profiles.png)\n\n"
        "![横向残差与载荷](selected_traces.png)\n\n"
        "数据见 `summary.json`、`partial_audit.json`、`archived_reference_checks.json`。"
        "模型为独立仿真候选，尚未生成新的制造STEP文件；连续捕获、摩擦/倾斜鲁棒性和锁定尚未验证。\n")
    text = header + "\n".join(rows) + tail
    (out / "report.md").write_text(text)
    snapshot = out / "analysis_snapshot" / Path(__file__).name
    snapshot.parent.mkdir(exist_ok=True)
    snapshot.write_bytes(Path(__file__).read_bytes())
    (out / "analysis_manifest.json").write_text(json.dumps({"source_sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest()}, indent=2)+"\n")
    print(out / "report.md")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path)
    build(parser.parse_args().out)
