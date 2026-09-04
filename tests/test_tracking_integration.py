"""自由空间轨迹跟踪门禁的完整闭环回归。"""
from pathlib import Path

import pytest

from compliant_docking import cli
from compliant_docking.telemetry import Log

REPO_ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.slow
@pytest.mark.parametrize("scene_name", ["iiwa14_tracking.yaml", "fr3_tracking.yaml"])
def test_full_circle_figure8_gate_passes(monkeypatch, scene_name):
    """两种机械臂均完整覆盖圆形与 8 字并通过对接前门禁。"""
    monkeypatch.setattr(Log, "plot_results", lambda self, *args, **kwargs: [])
    run_docking = cli._load_run_docking()

    log = run_docking.main(
        render=False,
        record=False,
        duration=13.1,
        scene_path=REPO_ROOT / "scenes" / scene_name,
    )

    assert log.tracking_gate.status == "PASS"
    assert log.tracking_metrics.max_contacts == 0
    assert log.tracking_metrics.torque_saturation_ratio == pytest.approx(0.0)
    assert log.tracking_metrics.segment("圆周").position_rms_m <= 0.005
    assert log.tracking_metrics.segment("8字").position_rms_m <= 0.005
