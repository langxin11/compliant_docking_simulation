"""Three geometry candidates using the existing lateral-release robot rollout."""
from __future__ import annotations

import hashlib
import json
import shutil
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import replace
from importlib.metadata import version

import numpy as np

from compliant_docking.research.cases import POINTS
from compliant_docking.scene import REPO_ROOT, load_scene

GEOMETRIES = ("original","narrow","radial")
PILOT = ("nominal","combo_ny6_p15","combo_py6_n15")
PRIMARY = "dt_half"  # 0.25 ms physical; 0.5 ms control/sensor delay
QUARTER_BUDGET = 4


def geometry_scene(base,geometry):
    from experiments.petal_guidance_geometry import DIRECTORIES
    if geometry not in GEOMETRIES:
        raise ValueError("Unknown geometry")
    directory = DIRECTORIES[geometry]
    return replace(base,name=f"iiwa14_petal_guidance_{geometry}",
        tool=replace(base.tool,mjcf=directory/"active.xml"),
        target=replace(base.target,mjcf=directory/"passive.xml"))


def name(geometry,case,setting=PRIMARY):
    return f"g_{geometry}_{case}_lateral_released_{setting}"


def read_records(out):
    records = {}
    for path in out.glob("g_*.json"):
        r = json.loads(path.read_text())
        if "guidance_geometry" in r:
            records[(r["guidance_geometry"],r["guidance_case"],r["numerics"])] = r
    return records


def run_one(out,base,geometry,case,setting,stage):
    from compliant_docking.research.petal_trials import run_case
    from compliant_docking.research.protocols import error_tuple, write_json
    scene = geometry_scene(base,geometry)
    r = run_case(out,scene,f"g_{geometry}_{case}","lateral_released",setting,error=error_tuple(POINTS[case]))
    r.update(guidance_geometry=geometry,guidance_case=case,guidance_error=POINTS[case],guidance_stage=stage)
    write_json(out/f"{name(geometry,case,setting)}.json",r)
    print("GEOMETRY",geometry,case,setting,r["assessment"],flush=True)
    return r


def execute(out,base,jobs,stage,workers):
    records = read_records(out)
    pending = [job for job in jobs if job not in records]
    if not pending:
        return
    with ProcessPoolExecutor(max_workers=workers) as executor:
        futures = [executor.submit(run_one,out,base,*job,stage) for job in pending]
        for future in as_completed(futures):
            future.result()


def summarize(out):
    from compliant_docking.research.protocols import sensitivity, write_json
    records = read_records(out)
    passes = {g:sum(r["assessment"]["status"] == "CANDIDATE_PASS"
        for (geometry,_,setting),r in records.items() if geometry == g and setting == PRIMARY) for g in GEOMETRIES}
    comparisons = {name(g,c):sensitivity(r,records[(g,c,"dt_quarter")])
        for (g,c,setting),r in records.items() if setting == PRIMARY and (g,c,"dt_quarter") in records}
    summary = dict(records=len(records),primary_passes=passes,primary_physics_dt_s=.00025,
        control_period_s=.0005,feedback_delay_s=.0005,numerical_comparisons=comparisons,
        continuous_capture_verified=False,locking_verified=False)
    write_json(out/"summary.json",summary)
    write_json(out/"numerical_comparison.json",comparisons)
    return summary


def select_checks(records):
    def quality(geometry):
        selected = [r for (g,_,s),r in records.items() if g == geometry and s == PRIMARY]
        return (-sum(r["assessment"]["status"] == "CANDIDATE_PASS" for r in selected),
                max(r["contact_load_gate"]["peak_contact_force_N"] for r in selected),geometry)
    best = min(("narrow","radial"),key=quality)
    return best,[("original","combo_ny6_p15","dt_quarter"),
                 (best,"combo_ny6_p15","dt_quarter"),
                 (best,"combo_py6_n15","dt_quarter"),(best,"nx6","dt_quarter")]


def reference_audit(out):
    """Reference trajectories are shared; inertias and trigger times can change."""
    from compliant_docking.research.protocols import write_json
    records,checks = read_records(out),{}
    for case in POINTS:
        keys = [(g,case,PRIMARY) for g in GEOMETRIES]
        if not all(k in records for k in keys):
            continue
        arrays = []
        for key in keys:
            with np.load(out/f"{name(*key)}.npz") as archive:
                arrays.append({k:archive[k] for k in ("t","desired_position")})
        for item in arrays[1:]:
            np.testing.assert_array_equal(arrays[0]["t"],item["t"])
            np.testing.assert_array_equal(arrays[0]["desired_position"],item["desired_position"])
        checks[case] = dict(status="PASS",reference_positions_and_timestamps_identical=True,
            triggers_s={g:records[(g,case,PRIMARY)]["feedback_audit"]["yaw_trigger_s"] for g in GEOMETRIES})
    write_json(out/"reference_checks.json",checks)


def run_study(args):
    from compliant_docking.research import protocols as grid
    from compliant_docking.research.petal_trials import preflight
    from experiments.petal_guidance_geometry import DIRECTORIES, validate
    out = args.out
    out.mkdir(parents=True,exist_ok=True)
    base = load_scene("scenes/iiwa14_petal_insertion.yaml")
    manifest = grid.source_manifest(base)
    for filename in ("petal_guidance_study.py","prepare_petal_guidance.py","petal_guidance_geometry.py",
                     "petal_lateral_study.py"):
        path = REPO_ROOT/"experiments"/filename
        manifest["sources"][str(path.relative_to(REPO_ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest["geometry_assets"] = {g:json.loads((DIRECTORIES[g]/"manifest.json").read_text()) for g in GEOMETRIES}
    plan = dict(geometries=GEOMETRIES,points=POINTS,pilot=PILOT,primary_setting=PRIMARY,
        primary_physics_dt_s=.00025,control_period_s=.0005,feedback_delay_s=.0005,
        profile="lateral_released",all_new_rollouts=True,numerical_budget=QUARTER_BUDGET,
        numerical_selection="best new pass count, then lower peak force; both combos and nx6; original combo",
        friction=.15,outer_diameter_mm=100.,guide_height_mm=18.,separation_mm=46.4,
        angular_candidate="flat half-angle 4.21875 deg plus rounded linear ramp (two declared changes)",
        radial_candidate="same angular shape plus complementary 6 mm wide / 3 mm crest-drop radial lead")
    for filename,value in (("source_manifest.json",manifest),("study_plan.json",plan)):
        path = out/filename
        if path.exists() and json.loads(path.read_text()) != json.loads(json.dumps(value,default=grid._json_default)):
            raise ValueError("Geometry study sources/assets/plan changed; select a new output directory")
        grid.write_json(path,value)
    for filename in manifest["sources"]:
        snapshot = out/"source_snapshot"/filename
        snapshot.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(REPO_ROOT/filename,snapshot)
    for filename in ("tests/test_petal_guidance.py","tests/test_contact_yaw.py","tests/test_petal_grid.py",
                     "pyproject.toml","uv.lock"):
        snapshot = out/"source_snapshot"/filename
        snapshot.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(REPO_ROOT/filename,snapshot)
    grid.write_json(out/"environment.json",{p:version(p) for p in ("mujoco","pin","numpy","scipy","matplotlib")})
    for g in GEOMETRIES:
        path = out/f"{g}_geometry_validation.json"
        cached = json.loads(path.read_text()) if path.exists() else {}
        if (cached.get("assets_manifest_sha256") != hashlib.sha256((DIRECTORIES[g]/"manifest.json").read_bytes()).hexdigest()
                or cached.get("validator_sha256") != manifest["sources"]["experiments/petal_guidance_geometry.py"]):
            grid.write_json(path,validate(DIRECTORIES[g],POINTS))
        assert json.loads(path.read_text())["status"] == "PASS"
        grid.write_json(out/f"{g}_preflight.json",preflight(geometry_scene(base,g)))
        info = json.loads((DIRECTORIES[g]/"model_info.json").read_text())
        if g != "original":
            assert manifest["geometry_assets"][g]["generator_sha256"] == manifest["sources"]["experiments/prepare_petal_guidance.py"]
            assert info["manufacturing_cad_generated"] is False
        for filename,digest in manifest["geometry_assets"][g]["imported_files"].items():
            assert hashlib.sha256((DIRECTORIES[g]/filename).read_bytes()).hexdigest() == digest
    pilot = [(g,c,PRIMARY) for c in PILOT for g in GEOMETRIES]
    execute(out,base,pilot,"pilot",args.jobs)
    summarize(out)
    remaining = [(g,c,PRIMARY) for c in POINTS if c not in PILOT for g in GEOMETRIES]
    execute(out,base,remaining,"extension",args.jobs)
    reference_audit(out)
    summarize(out)
    best,checks = select_checks(read_records(out))
    assert len(checks) == QUARTER_BUDGET
    path = out/"numerical_plan.json"
    plan = dict(selected_geometry=best,jobs=checks)
    if path.exists():
        assert json.loads(path.read_text()) == json.loads(json.dumps(plan))
    grid.write_json(path,plan)
    execute(out,base,checks,"quarter_check",args.jobs)
    summarize(out)
    from experiments.petal_guidance_report import build_report
    build_report(out)


def main(argv=None):
    import argparse
    from pathlib import Path
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--jobs", type=int, choices=(1, 2, 3), default=1)
    run_study(parser.parse_args(argv))

if __name__ == "__main__":
    main()
