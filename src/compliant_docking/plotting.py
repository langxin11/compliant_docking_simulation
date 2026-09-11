"""
plotting.py - SciencePlots IEEE 中文绘图 / SciencePlots IEEE plotting with CJK support

项目统一绘图入口：基于 SciencePlots 的 ["science", "ieee", "no-latex"] 风格
（IEEE 单栏、不依赖 LaTeX），叠加中文字体回退（Noto CJK）与
``axes.unicode_minus=False``（中文字体缺 U+2212 负号），每张图同时输出
PNG（位图）与 PDF（矢量）。

Unified plotting entry point for the project. Built on SciencePlots'
``["science", "ieee", "no-latex"]`` style (IEEE single column, no LaTeX) with
a CJK font fallback (Noto) and ``axes.unicode_minus=False`` (CJK fonts lack
the U+2212 minus sign). Every figure is saved as both PNG (raster) and
PDF (vector).

Author: langxin11
Date: 2025
"""

from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

if TYPE_CHECKING:  # 仅类型检查用，避免运行时环导入 / type-checking only, avoids runtime cycle
    from compliant_docking.telemetry import Log

# IEEE 单栏图宽（英寸）/ IEEE single-column figure width (inches)
_FIG_WIDTH = 3.5


def apply_style() -> None:
    """应用 SciencePlots IEEE + 中文字体回退样式（幂等，可重复调用）/
    Apply the SciencePlots IEEE style with CJK font fallback (idempotent)."""
    # 导入 scienceplots 即向 matplotlib 注册样式表（此处无需直接引用）/
    # Importing scienceplots registers its stylesheets with matplotlib
    import scienceplots  # noqa: F401

    # 顺序固定：science → ieee → no-latex，确保 text.usetex 关闭 /
    # Order matters: science → ieee → no-latex, guaranteeing text.usetex = False
    plt.style.use(["science", "ieee", "no-latex"])

    # 中文字体回退（覆盖在 style.use 之后）：具体字体列表必须放在 font.family——
    # matplotlib 仅对 font.family 列表构建逐字形回退链；font.serif 这类泛型别名
    # 只会解析出单个最优字体（本机装有真实 Times New Roman，中文将无回退而变方框）。
    # Times New Roman 渲染西文，CJK 字形落到 Noto；mathtext 用 STIX 与衬线体一致；
    # 中文字体普遍缺 U+2212，必须改用 ASCII 负号 /
    # CJK fallback applied after style.use: the concrete list must live in
    # font.family -- matplotlib only builds a per-glyph fallback chain from
    # font.family entries; a generic alias like "serif" resolves to a single
    # best-match font (real Times New Roman here), leaving CJK glyphs without
    # fallback (tofu). Times New Roman renders Latin, CJK falls to Noto; STIX
    # mathtext matches the serif look; CJK fonts lack U+2212 so the ASCII
    # minus is required
    mpl.rcParams["font.family"] = ["Times New Roman", "Noto Serif CJK SC",
                                   "Noto Sans CJK SC", "DejaVu Serif"]
    mpl.rcParams["font.serif"] = ["Times New Roman", "Noto Serif CJK SC",
                                  "Noto Sans CJK SC", "DejaVu Serif"]
    mpl.rcParams["mathtext.fontset"] = "stix"
    mpl.rcParams["axes.unicode_minus"] = False


def plot_docking_log(log: "Log", out_dir: str | Path, *,
                     scene_name: str | None = None,
                     docking_axis: int = 2,
                     dpi: int = 600) -> list[Path]:
    """
    绘制对接仿真结果图（SciencePlots IEEE 中文风格，PNG + PDF 双格式）/
    Plot docking simulation results (SciencePlots IEEE style, PNG + PDF).

    Args:
        log: 已灌入时序数据的 telemetry.Log（须先 reset_logs + store_data）/
            Populated telemetry.Log (reset_logs + store_data first)
        out_dir: 图件输出目录（不存在则创建）/ Output directory (created if missing)
        scene_name: 文件名前缀；None 时用 "docking_" / Filename prefix; "docking_" if None
        docking_axis: 对接轴索引（默认 2 = 世界 Z）/ Docking axis index (default 2 = world Z)
        dpi: PNG 输出分辨率 / PNG resolution

    Returns:
        生成的全部文件路径列表 / List of all generated file paths
    """
    apply_style()

    if len(log.t_list) == 0:
        raise ValueError("日志为空，请先仿真并记录数据 / empty log: run the simulation first")

    prefix = f"{scene_name}_" if scene_name is not None else "docking_"
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    # 时间轴与各时序列表统一转 ndarray / Convert all logged series to ndarrays
    t = np.asarray(log.t_list, dtype=float)
    pos_des = np.asarray(log.pos_desired, dtype=float).reshape(-1, 3)
    pos_act = np.asarray(log.pos_actual, dtype=float).reshape(-1, 3)
    forces = np.asarray(log.force_externals, dtype=float).reshape(-1, 3)
    tau = np.atleast_2d(np.asarray(log.tau_hist, dtype=float))

    saved: list[Path] = []

    def _save(fig: plt.Figure, stem: str) -> None:
        # 每张图同时落 PNG（dpi）与 PDF（矢量）/ Save each figure as PNG (dpi) and PDF (vector)
        for suffix in (".png", ".pdf"):
            path = out / f"{stem}{suffix}"
            fig.savefig(path, dpi=dpi)
            saved.append(path)
        plt.close(fig)

    # 1. 末端三轴位置跟踪（期望虚线 vs 实际实线）/
    # 1. EE per-axis position tracking (desired dashed vs actual solid)
    fig, axes = plt.subplots(3, 1, sharex=True,
                             figsize=(_FIG_WIDTH, 4.6), constrained_layout=True)
    for i, ax in enumerate(axes):
        ax.plot(t, pos_des[:, i], "--", label="期望")
        ax.plot(t, pos_act[:, i], "-", label="实际")
        ax.set_ylabel(f"{'XYZ'[i]} 位置 [m]")
    axes[0].legend(ncols=2)
    axes[-1].set_xlabel("时间 [s]")
    _save(fig, f"{prefix}ee_tracking")

    # 2. 跟踪误差范数（对数轴；clip 防 log(0)）/
    # 2. Tracking error norm (log axis; clip guards log(0))
    err_mm = np.clip(np.asarray(log.error, dtype=float) * 1000.0, 1e-9, None)
    fig, ax = plt.subplots(figsize=(_FIG_WIDTH, 2.2), constrained_layout=True)
    ax.semilogy(t, err_mm)
    ax.set_ylabel("跟踪误差范数 [mm]")
    ax.set_xlabel("时间 [s]")
    _save(fig, f"{prefix}tracking_error")

    # 3. 接触力：合力范数与对接轴向分量 /
    # 3. Contact force: resultant norm and docking-axis component
    fig, ax = plt.subplots(figsize=(_FIG_WIDTH, 2.2), constrained_layout=True)
    ax.plot(t, np.linalg.norm(forces, axis=1), label="合力范数")
    ax.plot(t, forces[:, docking_axis], label="对接轴向分量")
    ax.set_ylabel("接触力 [N]")
    ax.set_xlabel("时间 [s]")
    ax.legend(ncols=2)
    _save(fig, f"{prefix}contact_force")

    # 4. 各关节力矩（4x2 栅格，第 8 格置空）/
    # 4. Joint torques (4x2 grid, 8th cell hidden)
    fig, axes = plt.subplots(4, 2, sharex=True,
                             figsize=(_FIG_WIDTH, 4.6), constrained_layout=True)
    for i, ax in enumerate(axes.flat):
        if i < tau.shape[1]:
            ax.plot(t, tau[:, i])
            ax.set_ylabel(f"J{i + 1} [N·m]")
        else:
            ax.set_visible(False)
    for ax in axes[-1]:
        if ax.get_visible():
            ax.set_xlabel("时间 [s]")
    _save(fig, f"{prefix}joint_torques")

    # 5. 姿态误差范数（弧度→度；仅在记录了姿态误差时生成，
    #    列表可能短于时间轴，按其自身长度取时间切片）/
    # 5. Orientation error norm (rad→deg); only when logged. The list may be
    #    shorter than the time axis, so slice time to its own length.
    if len(log.orientation_errors) > 0:
        ori = np.asarray(log.orientation_errors, dtype=float).reshape(-1, 3)
        ori_deg = np.linalg.norm(ori, axis=1) * 180.0 / np.pi
        fig, ax = plt.subplots(figsize=(_FIG_WIDTH, 2.2), constrained_layout=True)
        ax.plot(t[: len(ori_deg)], ori_deg)
        ax.set_ylabel("姿态误差范数 [°]")
        ax.set_xlabel("时间 [s]")
        _save(fig, f"{prefix}orientation_error")

    # 6. 规划轨迹本体：上图为 3D 末端路径（规划虚线 vs 实际实线），
    #    下图为速度剖面（两段式轨迹的对接段限速平台在此直接可见）。
    #    Axes3D 的刻度/轴标签不参与 constrained_layout 的占位计算，会裁切进
    #    下方子图，故本图关闭 constrained layout，改用手动边距 /
    # 6. The planned trajectory itself: top panel is the 3D EE path
    #    (planned dashed vs actual solid), bottom panel the speed profile,
    #    where the slow docking-phase plateau is directly visible. Axes3D
    #    tick/axis labels are not measured by constrained_layout (they would
    #    be clipped under the lower panel), so manual margins are used here.
    vel_des = np.asarray(log.vel_desired, dtype=float).reshape(-1, 3)
    vel_act = np.asarray(log.vel_actual, dtype=float).reshape(-1, 3)
    fig = plt.figure(figsize=(_FIG_WIDTH, 5.0))
    ax3d = fig.add_subplot(2, 1, 1, projection="3d")
    ax3d.plot(pos_des[:, 0], pos_des[:, 1], pos_des[:, 2], "--", label="规划")
    ax3d.plot(pos_act[:, 0], pos_act[:, 1], pos_act[:, 2], "-", label="实际")
    for axis in (ax3d.xaxis, ax3d.yaxis, ax3d.zaxis):
        axis.set_major_locator(mpl.ticker.MaxNLocator(3))
    ax3d.set_xlabel("X [m]")
    ax3d.set_ylabel("Y [m]")
    ax3d.set_zlabel("Z [m]")
    ax3d.legend(ncols=2)
    ax_sp = fig.add_subplot(2, 1, 2)
    ax_sp.plot(t, np.linalg.norm(vel_des, axis=1), "--", label="规划")
    ax_sp.plot(t, np.linalg.norm(vel_act, axis=1), "-", label="实际")
    ax_sp.set_ylabel("末端速度 [m/s]")
    ax_sp.set_xlabel("时间 [s]")
    ax_sp.legend(ncols=2)
    fig.subplots_adjust(left=0.15, right=0.84, top=0.97, bottom=0.10, hspace=0.42)
    _save(fig, f"{prefix}planned_trajectory")

    return saved
