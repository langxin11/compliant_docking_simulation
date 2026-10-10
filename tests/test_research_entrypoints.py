"""Layer isolation, keyword binding, strict provenance and spawned imports."""
import ast
import importlib
import multiprocessing
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from compliant_docking.research import protocols
from compliant_docking.research.petal_trials import variant
from compliant_docking.scene import load_scene
from experiments.control import lateral_release, yaw_release


def spawned_legacy_variant():
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))
    legacy = importlib.import_module("petal_insertion_suite")
    scene = legacy.variant(load_scene("scenes/iiwa14_petal_insertion.yaml"), "nominal", "released")
    return scene.name, scene.se3_impedance.k_diag, legacy.variant.__module__


def test_control_entry_rejects_interface_and_other_control_factors():
    for args in (["--geometry-study"], ["--profile", "lateral_released"], ["--scene", "candidate.yaml"]):
        with pytest.raises(SystemExit) as error:
            yaw_release.build_parser().parse_args(args)
        assert error.value.code == 2


def test_paired_lateral_release_never_schedules_numerics_or_speed(monkeypatch, tmp_path):
    scheduled = []
    monkeypatch.setattr(lateral_release, "execute", lambda out, base, jobs, stage, workers: scheduled.extend((job, stage) for job in jobs))
    monkeypatch.setattr(lateral_release, "compare_pairs", lambda out: None)
    monkeypatch.setattr(lateral_release, "summarize", lambda out: {})
    from compliant_docking.research import petal_trials
    monkeypatch.setattr(petal_trials, "preflight", lambda base: {"status": "PASS"})
    monkeypatch.setattr(protocols, "source_manifest", lambda base: {"sources": {}})
    lateral_release.run_study(SimpleNamespace(stage="paired", out=tmp_path, jobs=1))
    assert len(scheduled) == 2 * len(lateral_release.POINTS)
    assert {job[1] for job, _ in scheduled} == {"released", "lateral_released"}
    assert {job[2] for job, _ in scheduled} == {"baseline"}
    assert {stage for _, stage in scheduled} == {"pilot", "extension"}


def test_default_matrix_passes_telemetry_by_keyword_without_becoming_error(monkeypatch, tmp_path):
    calls = []
    def capture(out, base, case, profile, setting, preview=False, error=None, telemetry="auto"):
        calls.append((error, telemetry))
    monkeypatch.setattr(yaw_release, "run_case", capture)
    monkeypatch.setattr(yaw_release, "preflight", lambda base: {"status": "PASS"})
    monkeypatch.setattr(yaw_release, "summarize", lambda out: None)
    monkeypatch.setattr(yaw_release, "source_manifest", lambda base: {"sources": {}})
    args = yaw_release.build_parser().parse_args(["--case", "nominal", "--profile", "released", "--telemetry", "core", "--out", str(tmp_path)])
    yaw_release.run_matrix(args)
    assert calls == [(None, "core")]


def test_new_manifest_covers_dependencies_and_canonical_implementations():
    manifest = protocols.source_manifest(load_scene("scenes/iiwa14_petal_insertion.yaml"))
    required = {"uv.lock", "pyproject.toml", "src/compliant_docking/research/petal_trials.py",
                "src/compliant_docking/research/rollout.py", "experiments/control/yaw_release.py",
                "experiments/models_interfaces/petal_guidance.py"}
    assert required <= manifest["sources"].keys()


def test_missing_migrated_snapshot_or_dependency_mismatch_rejects_reuse(tmp_path):
    manifest = protocols.source_manifest(load_scene("scenes/iiwa14_petal_insertion.yaml"))
    assert not protocols.reusable_sources(manifest, manifest, tmp_path)
    changed = dict(manifest, sources=dict(manifest["sources"], **{"uv.lock": "changed"}))
    assert not protocols.reusable_sources(manifest, changed, tmp_path)


def test_shared_research_library_never_imports_experiment_or_cli_entries():
    root = Path(protocols.__file__).parent
    for path in root.glob("*.py"):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                assert not (node.module or "").startswith(("experiments", "compliant_docking.cli")), path
            elif isinstance(node, ast.Import):
                assert all(not alias.name.startswith("experiments") for alias in node.names), path


def test_legacy_function_survives_spawn_and_has_single_canonical_source():
    context = multiprocessing.get_context("spawn")
    with ProcessPoolExecutor(max_workers=1, mp_context=context) as executor:
        name, stiffness, module = executor.submit(spawned_legacy_variant).result(timeout=30)
    expected = variant(load_scene("scenes/iiwa14_petal_insertion.yaml"), "nominal", "released")
    assert name == expected.name
    assert stiffness == expected.se3_impedance.k_diag
    assert module == "compliant_docking.research.petal_trials"


def test_system_formal_entry_rejects_protocol_overrides():
    from experiments.system.hexframe import build_parser
    for override in (["--scene", "other.yaml"], ["--dt", "0.0005"], ["--duration", "2"]):
        with pytest.raises(SystemExit) as error:
            build_parser().parse_args(["accept", "--out", "runs/test"] + override)
        assert error.value.code == 2


def test_lateral_release_speed_factor_rejects_paired_protocol_directory(tmp_path):
    import json
    (tmp_path / "study_plan.json").write_text(json.dumps({"protocol": "paired lateral release"}))
    args = SimpleNamespace(stage="speed", out=tmp_path, jobs=1, speed_case="yaw_n15", setting=["baseline"])
    with pytest.raises(ValueError, match="separate new directory"):
        lateral_release.run_study(args)
