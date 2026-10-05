"""Representative tests of the explicitly saved 1 degree / 0.3 guide design.

The old narrow model is an archived reference, not a newly rerun baseline.
Physics, controller, reference commands and acceptance gates use the existing
reviewed implementation. Three primary cases and one fixed timestep check do
not establish a continuous capture region or manufacturing fit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from concurrent.futures import ProcessPoolExecutor, as_completed
from importlib.metadata import version
from pathlib import Path

import numpy as np

import experiments.petal_guidance_report as report
from compliant_docking.research import protocols as grid
from compliant_docking.research.cases import POINTS
from compliant_docking.research.petal_trials import preflight
from compliant_docking.scene import REPO_ROOT, load_scene
from experiments.models_interfaces import petal_guidance as study
from experiments.petal_guidance_geometry import DIRECTORIES, validate

CASES = ("nx6", "combo_ny6_p15", "combo_py6_n15")
MODEL = REPO_ROOT / "assets/interfaces/petal_guidance/angle1_blend030"
BASELINE = REPO_ROOT / "runs/petal_guidance_geometry_20261003"


def configure():
    study.GEOMETRIES = ("selected",)
    DIRECTORIES["selected"] = MODEL


def run_one(out, case, setting):
    configure()
    return study.run_one(out, load_scene("scenes/iiwa14_petal_original_insertion.yaml"),
                         "selected", case, setting, "representative_user_design")


def archived_references(out):
    """Verify the archived inputs and compare commands at each matching timestep."""
    baseline = json.loads((BASELINE / "source_manifest.json").read_text())
    current = json.loads((out / "source_manifest.json").read_text())
    original = grid.source_manifest(load_scene("scenes/iiwa14_petal_original_insertion.yaml"))
    if any(baseline["sources"].get(p) != digest for p, digest in original["sources"].items()):
        raise ValueError("Historical reference sources differ after migration; use archived source_snapshot or a separately reviewed fresh-reference protocol")
    assert all(current["sources"][p] == digest for p, digest in original["sources"].items())
    for filename, digest in baseline["geometry_assets"]["narrow"]["imported_files"].items():
        assert report.sha(DIRECTORIES["narrow"] / filename) == digest
    frozen = json.loads((BASELINE / "artifact_manifest.json").read_text())
    checks = {}
    for _, case, setting in study.read_records(out):
        current_name = study.name("selected", case, setting)
        previous_name = study.name("narrow", case, setting)
        paths = [BASELINE / f"{previous_name}.{ext}" for ext in ("json", "npz")]
        for path in paths:
            assert report.sha(path) == frozen["artifacts_sha256"][path.name]
        with np.load(out / f"{current_name}.npz") as a, np.load(paths[1]) as b:
            for field in ("t", "desired_position"):
                np.testing.assert_array_equal(a[field], b[field])
        checks[current_name] = dict(status="PASS", baseline_policy="archived narrow rollout, not rerun",
            baseline_files_sha256={str(p.relative_to(REPO_ROOT)): report.sha(p) for p in paths},
            positions_and_timestamps_identical=True, runtime_sources_identical=True)
    grid.write_json(out / "archived_reference_checks.json", checks)
    return checks


def metrics(record):
    geometry, loads = record["geometry_evaluation"], record["contact_load_gate"]
    return dict(status=record["assessment"]["status"],
        lateral_mm=geometry["last_second_max_lateral_mm"],
        phase_deg=geometry["last_second_max_phase_error_deg"],
        gap_mm=geometry["last_second_max_abs_axial_gap_mm"],
        stop_fraction=geometry["last_second_stop_contact_fraction"],
        force_N=loads["peak_contact_force_N"], moment_Nm=loads["peak_contact_axial_moment_Nm"])


def finish(out):
    configure()
    audit = report.audit(out, complete_matrix=False)
    # With one configured geometry the generic cross-geometry reference check is
    # vacuous. Only the independent comparison with the archived baseline counts.
    audit["matched_references"] = 0
    audit["reference_policy"] = "separate archived_reference_checks.json compares to the historical narrow baseline"
    audit["archived_reference_checks"] = len(archived_references(out))
    grid.write_json(out / "partial_audit.json", audit)
    (out / "reference_checks.json").unlink()
    records = study.read_records(out)
    comparison = {}
    for case in CASES:
        previous = json.loads((BASELINE / f"{study.name('narrow', case)}.json").read_text())
        current = records[("selected", case, study.PRIMARY)]
        comparison[case] = dict(error=POINTS[case], archived_narrow=metrics(previous), selected=metrics(current))
    numerics = grid.sensitivity(records[("selected", "nx6", "dt_half")],
                               records[("selected", "nx6", "dt_quarter")])
    result = dict(primary_cases=len(CASES),
        passes=sum(v["selected"]["status"] == "CANDIDATE_PASS" for v in comparison.values()),
        comparisons=comparison, nx6_timestep_check=numerics,
        primary_physics_dt_s=.00025, check_physics_dt_s=.000125,
        control_period_s=.0005, feedback_delay_s=.0005,
        baseline_policy="archived narrow rollouts; verified runtime and reference commands; not freshly rerun",
        continuous_capture_verified=False, locking_verified=False, manufacturing_fit_verified=False)
    grid.write_json(out / "summary.json", result)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)


def run(out, workers):
    configure()
    base = load_scene("scenes/iiwa14_petal_original_insertion.yaml")
    previous = json.loads((BASELINE / "source_manifest.json").read_text())
    current = grid.source_manifest(base)
    if any(previous["sources"].get(p) != digest for p, digest in current["sources"].items()):
        raise ValueError("Historical candidate references cannot be safely reused after migration; run the archived source_snapshot or design a fresh-reference study")
    out.mkdir(parents=True, exist_ok=True)
    manifest_path = out / "source_manifest.json"
    if manifest_path.exists() or study.read_records(out):
        raise ValueError("Study already started; use --audit to finish or choose a new directory")
    manifest = grid.source_manifest(base)
    for filename in ("petal_guidance_study.py", "prepare_petal_guidance.py", "petal_guidance_geometry.py",
                     "petal_guidance_report.py", "petal_lateral_study.py", "prepare_petal_design.py",
                     "petal_designer.py", "petal_selected_study.py"):
        path = REPO_ROOT / "experiments" / filename
        manifest["sources"][str(path.relative_to(REPO_ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest["geometry_assets"] = {"selected": json.loads((MODEL / "manifest.json").read_text())}
    assert manifest["geometry_assets"]["selected"]["generator_sha256"] == manifest["sources"]["experiments/prepare_petal_design.py"]
    grid.write_json(manifest_path, manifest)
    for filename in manifest["sources"]:
        snapshot = out / "source_snapshot" / filename
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPO_ROOT / filename, snapshot)
    for filename in ("tests/test_petal_design_build.py", "pyproject.toml", "uv.lock"):
        snapshot = out / "source_snapshot" / filename
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPO_ROOT / filename, snapshot)
    robot_manifest = json.loads((BASELINE / "robot_assets.json").read_text())
    for filename, digest in robot_manifest["files_sha256"].items():
        assert report.sha(REPO_ROOT / filename) == digest
    grid.write_json(out / "robot_assets.json", robot_manifest)
    grid.write_json(out / "environment.json", {p: version(p) for p in ("mujoco", "pin", "numpy", "scipy", "matplotlib")})
    for filename, digest in manifest["geometry_assets"]["selected"]["imported_files"].items():
        assert report.sha(MODEL / filename) == digest
    native = out / "selected_geometry_validation.json"
    if native.exists():
        checked = json.loads(native.read_text())
        assert checked["assets_manifest_sha256"] == report.sha(MODEL / "manifest.json")
        assert checked["validator_sha256"] == manifest["sources"]["experiments/petal_guidance_geometry.py"]
    else:
        grid.write_json(native, validate(MODEL, {case: POINTS[case] for case in CASES}))
    assert json.loads(native.read_text())["status"] == "PASS"
    limits = out / "selected_preflight.json"
    grid.write_json(limits, preflight(study.geometry_scene(base, "selected")))
    # Start the longer fixed refinement first; this does not adapt to outcomes.
    jobs = [("nx6", "dt_quarter")] + [(case, study.PRIMARY) for case in CASES]
    grid.write_json(out / "study_plan.json", dict(jobs=jobs, controls_and_gates_unchanged=True,
        selected_angle_deg=1., selected_blend=.3, baseline_policy="archived, verified; not rerun",
        numerical_selection="fixed nx6 check, selected before observing outcomes"))
    with ProcessPoolExecutor(max_workers=workers) as executor:
        futures = [executor.submit(run_one, out, case, setting) for case, setting in jobs]
        for future in as_completed(futures):
            future.result()
    finish(out)


def main():
    global BASELINE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "runs/petal_angle1_blend030_20261003")
    parser.add_argument("--jobs", type=int, default=3)
    parser.add_argument("--audit", action="store_true")
    parser.add_argument("--reference", type=Path, default=BASELINE,
                        help="Recorded geometry baseline with exact current provenance and artifact manifest")
    args = parser.parse_args()
    BASELINE = args.reference.resolve()
    if args.audit:
        finish(args.out)
    else:
        run(args.out, args.jobs)


if __name__ == "__main__":
    main()
