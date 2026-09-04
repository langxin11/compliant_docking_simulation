"""
test_plotting.py - SciencePlots 中文绘图单元测试 / Unit tests for CJK plotting
"""

import matplotlib

matplotlib.use("Agg")  # 无显示环境下出图，必须在 import pyplot 之前 / must precede pyplot import

import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
import pytest

from compliant_docking.plotting import apply_style, plot_docking_log
from compliant_docking.telemetry import Log


def _cjk_font_available() -> bool:
    """判断 Noto CJK 是否可用（findfont 回退到 DejaVu 即视为缺失）/
    Whether a Noto CJK font resolves (a DejaVu fallback means missing)."""
    path = fm.findfont(fm.FontProperties(family=["Noto Serif CJK SC"]))
    return "DejaVu" not in path


def test_apply_style_sets_chinese_no_latex():
    """apply_style 后：LaTeX 关闭、ASCII 负号、中文字体在回退列表 /
    After apply_style: LaTeX off, ASCII minus, CJK font in the fallback list."""
    apply_style()
    assert plt.rcParams["text.usetex"] is False
    assert plt.rcParams["axes.unicode_minus"] is False
    assert "Noto Serif CJK SC" in plt.rcParams["font.serif"]


@pytest.mark.skipif(not _cjk_font_available(),
                    reason="系统缺 Noto CJK 字体 / Noto CJK font not installed")
def test_cjk_font_resolves():
    """中文字体应能解析到 Noto 字体文件 / The CJK family must resolve to a Noto file."""
    path = fm.findfont(fm.FontProperties(family=["Noto Serif CJK SC"]))
    assert "Noto" in path


def test_plot_docking_log_smoke(tmp_path):
    """灌合成数据出图：文件齐全非空；scene_name=None 时前缀为 docking_ /
    Smoke-plot synthetic data: all files exist non-empty; default prefix is docking_."""
    log = Log()
    log.reset_logs()

    n = 60
    t = np.linspace(0.0, 1.0, n)
    rng = np.random.default_rng(0)
    for k in range(n):
        pos_des = np.array([0.1 * t[k], -0.2 * t[k], 0.5 - 0.18 * t[k]])
        noise = rng.normal(0.0, 1e-3, 3)  # 1mm 级噪声 / ~1 mm noise
        pos_act = pos_des + noise
        force = np.zeros(3)
        if k >= n - 10:
            force[2] = 5.0  # 末段 5N 对接轴阶跃 / 5 N step on docking axis at the tail
        log.store_data(
            t[k], np.zeros(7), np.zeros(7), pos_act, np.zeros(3),
            float(np.linalg.norm(noise)), pos_des, np.zeros(3), np.zeros(3),
            rng.uniform(-1.0, 1.0, 7), force, np.zeros(3),
            orientation_error=np.array([0.01, -0.02, 0.005]) * t[k])

    files = plot_docking_log(log, tmp_path, scene_name="unit")
    names = {f.name for f in files}
    for expected in ("unit_ee_tracking.png", "unit_ee_tracking.pdf",
                     "unit_tracking_error.png", "unit_contact_force.png",
                     "unit_joint_torques.png", "unit_orientation_error.png",
                     "unit_planned_trajectory.png"):
        assert expected in names, f"missing figure: {expected}"
    for f in files:
        assert f.stat().st_size > 0, f"empty file: {f}"

    # 不给 scene_name 时前缀应为 docking_ / default prefix when scene_name is None
    files_default = plot_docking_log(log, tmp_path, scene_name=None)
    assert any(f.name == "docking_tracking_error.png" for f in files_default)
