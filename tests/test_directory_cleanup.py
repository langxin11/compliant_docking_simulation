"""Boundaries of archive forwarding and writable generated-output locations."""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

from experiments import _archive_compat  # noqa: E402


def test_cold_legacy_dispatch_creates_workspace(tmp_path):
    archive = tmp_path / "archive/orbital_showcase"
    archive.mkdir(parents=True)
    (archive / "outputs").symlink_to("../../runs/archive_workspaces/orbital_showcase/outputs")
    (archive / "crown_assembly.py").write_text(
        'from pathlib import Path\n'
        'OUT = Path(__file__).resolve().parent / "outputs/smoke"\n'
        'OUT.mkdir(parents=True, exist_ok=True)\n'
        '(OUT / "created.txt").write_text("ok")\n'
    )
    wrapper = tmp_path / "experiments/orbital_showcase/crown_assembly.py"
    wrapper.parent.mkdir(parents=True)
    shutil.copy2(ROOT / "experiments/orbital_showcase/crown_assembly.py", wrapper)
    shutil.copy2(ROOT / "experiments/_archive_compat.py", tmp_path / "experiments/_archive_compat.py")
    assert not (tmp_path / "runs").exists()
    subprocess.run([sys.executable, str(wrapper)], cwd=tmp_path, check=True)
    assert (tmp_path / "runs/archive_workspaces/orbital_showcase/outputs/smoke/created.txt").read_text() == "ok"
    assert not (tmp_path / "results").exists()


def test_legacy_module_uses_only_archived_implementation():
    from experiments.orbital_showcase import hexframe_integration

    assert Path(hexframe_integration.__file__) == ROOT / "archive/orbital_showcase/hexframe_integration.py"
    assert hexframe_integration.RESOURCE.resolve() == ROOT / "assets/modules/hexframe"
    assert (ROOT / "experiments/orbital_showcase/assets/hexframe_module").resolve() == hexframe_integration.RESOURCE.resolve()


def test_archived_audit_refuses_frozen_results(monkeypatch):
    monkeypatch.setattr("sys.argv", ["hexframe_assembly_audit.py", "--out", str(ROOT / "results/historical")])
    with pytest.raises(SystemExit) as raised:
        _archive_compat.dispatch("__main__", "hexframe_assembly_audit.py")
    assert raised.value.code == 2
