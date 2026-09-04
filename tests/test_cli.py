"""命令行跟踪门禁退出码测试（不启动物理仿真）。"""
from types import SimpleNamespace

from compliant_docking import cli
from compliant_docking.telemetry import Log


def _log_with_gate(status: str | None) -> Log:
    log = Log()
    log.reset_logs()
    if status is not None:
        log.tracking_gate = SimpleNamespace(status=status)
    return log


def test_tracking_gate_failure_returns_2(monkeypatch):
    """完整 tracking 门禁 FAIL 必须向自动化调用者返回非零。"""
    monkeypatch.setattr(
        cli,
        "_load_run_docking",
        lambda: SimpleNamespace(main=lambda **_: _log_with_gate("FAIL")),
    )
    assert cli.main(["--quick"]) == 2


def test_incomplete_and_docking_keep_success_exit_code(monkeypatch):
    """--quick 的 INCOMPLETE 与普通 docking 都不是门禁失败。"""
    monkeypatch.setattr(
        cli,
        "_load_run_docking",
        lambda: SimpleNamespace(main=lambda **_: _log_with_gate("INCOMPLETE")),
    )
    assert cli.main(["--quick"]) == 0

    monkeypatch.setattr(
        cli,
        "_load_run_docking",
        lambda: SimpleNamespace(main=lambda **_: _log_with_gate(None)),
    )
    assert cli.main([]) == 0
