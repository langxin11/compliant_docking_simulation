"""Compare saved, matched insertion rollouts; do not rerun or retune control."""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from compliant_docking.plotting import COLORS, apply_style
from compliant_docking.scene import REPO_ROOT


def paired_config(metadata):
    config = copy.deepcopy(metadata["scene"])
    config["tool"].pop("mjcf")
    config["target"].pop("mjcf")
    return config


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdf", type=Path, default=REPO_ROOT / "runs/contact_diagnostics_20261002")
    parser.add_argument("--convex", type=Path, default=REPO_ROOT / "runs/convex_contact_20261002")
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "runs/collision_comparison_20261002")
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    sdf_manifest = json.loads((args.sdf / "manifest.json").read_text())
    convex_manifest = json.loads((args.convex / "manifest.json").read_text())
    if sdf_manifest["packages"] != convex_manifest["packages"]:
        raise ValueError("Runtime package versions differ; these are not a matched comparison")
    controlled_sources = [path for path in sdf_manifest["sources"]
                          if path.startswith("src/compliant_docking/control/")
                          or path.startswith("src/compliant_docking/planning/")
                          or path in ["experiments/run_docking.py", "src/compliant_docking/docking_task.py",
                                      "src/compliant_docking/simulation/mujoco_env.py",
                                      "src/compliant_docking/scene.py"]]
    if any(sdf_manifest["sources"][p] != convex_manifest["sources"].get(p) for p in controlled_sources):
        raise ValueError("Control, planning or solver-loop source differs from the recorded SDF run")
    analysis_manifest_path = args.sdf / "analysis_manifest.json"
    sdf_analysis = (json.loads(analysis_manifest_path.read_text())
                    if analysis_manifest_path.exists() else sdf_manifest)
    analysis_sources = ["src/compliant_docking/contact_diagnostics.py"]
    if any(sdf_analysis["sources"][p] != convex_manifest["sources"].get(p) for p in analysis_sources):
        raise ValueError("Saved contact-analysis source differs between models")
    apply_style("report", cjk_first=True)
    summary = []
    rows = []
    fig, axes = plt.subplots(6, 3, figsize=(13, 16), constrained_layout=True)
    for row, name in enumerate([f"{case}_{profile}" for case in ["nominal", "xy", "combined"]
                                for profile in ["stiff", "compliant"]]):
        sdf = json.loads((args.sdf / f"{name}.json").read_text())
        convex = json.loads((args.convex / f"{name}.json").read_text())
        if paired_config(sdf) != paired_config(convex):
            raise ValueError(f"{name}: control, task or numerical settings differ")
        for label, directory, record, color in [("SDF", args.sdf, sdf, COLORS["stiff"]),
                                                 ("凸碰撞模型", args.convex, convex, COLORS["planned"])]:
            diagnostics = record["contact_diagnostics"]
            gate = record["gate"]
            summary.append(dict(case=name, backend=label, benchmark=gate["status"],
                                geometry=record["geometry_evaluation"]["status"],
                                contact_load=record["contact_load_gate"]["status"],
                                assessment=record["assessment"]["status"],
                                final_depth_mm=gate["final_depth_mm"],
                                final_yaw_deg=gate["final_yaw_deg"],
                                peak_sensor_moment_Nm=diagnostics["peak_sensor_axial_moment_Nm"],
                                peak_contact_moment_Nm=diagnostics["peak_contact_axial_moment_Nm"],
                                balance_residual_Nm=diagnostics["peak_balance_moment_residual_Nm"],
                                reasons=record["assessment"]["reasons"]))
            with np.load(directory / f"{name}.npz") as data:
                t = data["diagnostic_t"]
                # This crown scene's insertion axis is world Z. Load curves are
                # every-step; state curves alone may be decimated for plotting.
                signals = [data["depth_mm"], data["yaw_deg"], data["diagnostic_interface_world"][:, 5]]
                for col, signal in enumerate(signals):
                    stride = 1 if col == 2 else 10
                    axes[row, col].plot(t[::stride], signal[::stride], color=color,
                                        linewidth=.9, label=label)
            rows.append(f"| {name} | {label} | {gate['status']} | {record['geometry_evaluation']['status']} | "
                        f"{record['contact_load_gate']['status']} | {record['assessment']['status']} | "
                        f"{gate['final_depth_mm']:.2f} | {gate['final_yaw_deg']:.2f} | "
                        f"{diagnostics['peak_sensor_axial_moment_Nm']:.3f} | "
                        f"{diagnostics['peak_contact_axial_moment_Nm']:.3f} |")
        for col, label in enumerate(["入口进给 [mm]", "实际绕轴角 [°]", "真实接口轴矩 [N·m]"]):
            axes[row, col].set(xlabel="时间 [s]", ylabel=label, title=name)
            axes[row, col].grid(alpha=.7)
            axes[row, col].legend(frameon=False)
        axes[row, 2].axhline(2., color="#888888", linestyle="--", linewidth=.7)
        axes[row, 2].axhline(-2., color="#888888", linestyle="--", linewidth=.7)
    for suffix in ["png", "pdf"]:
        fig.savefig(args.out / f"collision_comparison.{suffix}", dpi=160)
    plt.close(fig)
    result = dict(matched_runtime_packages=sdf_manifest["packages"], matched_configuration=True, matched_control_sources=controlled_sources, matched_contact_analysis_sources=analysis_sources,
                  sdf_directory=str(args.sdf), convex_directory=str(args.convex), rows=summary,
                  geometry_manifest=str(REPO_ROOT / "assets/interfaces/convex_crown/manifest.json"),
                  limitations=["sampled geometry and declared research load limits",
                               "net interface force/moment limits do not certify local contact stress",
                               "candidate seating does not certify locking"])
    (args.out / "comparison.json").write_text(json.dumps(result, indent=2)+"\n")
    report = ["# SDF / 凸碰撞模型的插入对照", "",
              "复用原有六组 SDF 数据；与新模型逐组核对完整配置（只允许两个接口 MJCF 路径不同）、控制/轨迹/仿真回路源文件指纹和运行库版本。",
              "原始视觉网格、安装相位、质量、惯量、轨迹、增益、误差和验收阈值保持一致。", "",
              "| 工况 | 碰撞模型 | 原门禁 | 几何 | 接触载荷 | 综合 | 入口进给 mm | 最终 yaw ° | F/T 峰矩 Nm | 接触峰矩 Nm |",
              "|---|---|---|---|---|---|---:|---:|---:|---:|", *rows, "",
              "![对照曲线](collision_comparison.png)", "",
              "凸模型目录同时保留 nominal 的半步长数据，用于检查稳定性。",
              "几何通过和净载荷通过均不等于机械锁定或局部接触应力通过。"]
    (args.out / "report.md").write_text("\n".join(report)+"\n")
    print(args.out / "report.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
