"""Generate real outputs in an isolated clone-shaped workspace, preserving frozen inputs."""
import hashlib
import importlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))


def hashes(directory):
    return {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in directory.rglob("*") if p.is_file() and "__pycache__" not in p.parts}


@pytest.fixture(scope="module")
def cold_package(tmp_path_factory):
    workspace = tmp_path_factory.mktemp("generated-cold")
    resource = workspace / "assets/modules/hexframe"
    shutil.copytree(ROOT / "assets/modules/hexframe", resource, ignore=shutil.ignore_patterns("__pycache__"))
    assert not (workspace / "runs").exists()
    return workspace, resource


def run_tool(path, *args):
    env = dict(os.environ, MUJOCO_GL="egl", MPLBACKEND="Agg")
    return subprocess.run([sys.executable, str(path), *args], capture_output=True, text=True, env=env)


def test_demo_validation_and_static_preview_generate_in_cold_runs(cold_package):
    workspace, resource = cold_package
    before = hashes(resource)
    for name, args in (("demo.py", ("--headless", "--seconds", "0.025")),
                       ("validate_model.py", ()), ("create_preview.py", ())):
        result = run_tool(resource / name, *args)
        assert result.returncode == 0, result.stderr
    runtime = workspace / "runs/hexframe_module"
    summary = json.loads((runtime / "results/demo/summary.json").read_text())
    assert summary["final"]["time_s"] >= 0.025
    assert json.loads((runtime / "results/demo/trajectory.json").read_text())
    assert (runtime / "results/geometry_validation.json").is_file()
    assert (runtime / "preview/hexframe_module_preview.png").is_file()
    assert not (runtime / "preview/module_docking.gif").exists()  # No implicit saved input.
    assert hashes(resource) == before


def test_preview_rejects_missing_explicit_trajectory_before_writing(cold_package):
    workspace, resource = cold_package
    out = workspace / "runs/missing_preview"
    result = run_tool(resource / "create_preview.py", "--out", str(out), "--trajectory-dir", str(workspace / "missing"))
    assert result.returncode == 2
    assert "Trajectory input requires" in result.stderr
    assert not out.exists()


def test_generator_refuses_existing_frozen_model_before_optional_cad_import(cold_package):
    _, resource = cold_package
    before = hashes(resource)
    result = run_tool(resource / "generate.py", "--out", str(resource))
    assert result.returncode == 2
    assert "Output directory exists" in result.stderr
    assert hashes(resource) == before


def test_convex_validate_only_preserves_frozen_manifest(monkeypatch, tmp_path):
    tool = importlib.import_module("prepare_convex_interface")
    frozen = tool.CONVEX_DIRECTORY / "manifest.json"
    before = frozen.read_bytes()
    def validation(scene, candidate, components, report):
        report.mkdir(parents=True)
        return {"status": "PASS"}
    monkeypatch.setattr(tool, "validate", validation)
    assert tool.main(["--validate-only", "--report", str(tmp_path / "report")]) == 0
    assert (tmp_path / "report/validation_manifest.json").is_file()
    assert frozen.read_bytes() == before


def test_selected_postprocessor_writes_report_without_publishing_frozen_results(monkeypatch, tmp_path):
    tool = importlib.import_module("petal_selected_report")
    source = ROOT / "results/petal_angle1_blend030_20261003"
    for name in ("summary.json", "partial_audit.json", "archived_reference_checks.json",
                 "g_selected_nx6_lateral_released_dt_quarter.json"):
        shutil.copy2(source / name, tmp_path / name)
    monkeypatch.setattr(tool, "profiles", lambda out: None)
    monkeypatch.setattr(tool, "traces", lambda out: None)
    tool.build(tmp_path)
    assert (tmp_path / "report.md").is_file()
    assert (tmp_path / "analysis_manifest.json").is_file()
