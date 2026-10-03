"""Experiment setup, provenance guards and boundary selection semantics."""
import importlib
import sys
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import pytest

from compliant_docking.docking_task import build_docking_trajectory
from compliant_docking.scene import load_scene

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"experiments"))
grid = importlib.import_module("petal_capture_grid")
suite = importlib.import_module("petal_insertion_suite")
legacy = importlib.import_module("insertion_suite")


def record(ok, lateral=.1, stop=1.):
    return dict(assessment=dict(status="CANDIDATE_PASS" if ok else "FAIL", reasons=[]),
        geometry_evaluation=dict(last_second_max_lateral_mm=lateral,
            last_second_max_phase_error_deg=.01, last_second_max_abs_axial_gap_mm=.001,
            last_second_max_tilt_deg=.01, last_second_stop_contact_fraction=stop),
        contact_load_gate=dict(peak_contact_force_N=4.,peak_contact_axial_moment_Nm=.1))


@pytest.mark.parametrize("case", list(legacy.CASES))
@pytest.mark.parametrize("profile", ["stiff", "compliant", "released"])
def test_presets_retain_exact_effective_configuration(case, profile):
    base=load_scene("scenes/iiwa14_petal_insertion.yaml")
    before=legacy.variant(base,case,"compliant" if profile=="released" else profile)
    before=replace(before,name=f"{base.name}_{case}_{profile}",
                   se3_impedance=replace(before.se3_impedance,
                       contact_yaw=base.se3_impedance.contact_yaw if profile=="released" else None))
    after=suite.variant(base,case,profile)
    # Dataclasses contain arrays: compare exact numeric storage, not ambiguous ==.
    def normal(v):
        if isinstance(v, dict):
            return {k:normal(x) for k,x in v.items()}
        return v.tolist() if isinstance(v,np.ndarray) else v
    assert normal(asdict(before))==normal(asdict(after))


def test_grid_errors_do_not_change_physics_or_leak_truth_to_planner():
    base=load_scene("scenes/iiwa14_petal_insertion.yaml")
    scene=suite.variant(base,"arbitrary","released",(-.006,.006,-15.))
    np.testing.assert_allclose(scene.docking.estimate_pos,[-.006,.506,.35],atol=1e-15)
    assert scene.docking.estimate_yaw_deg==-15.
    assert scene.physics==base.physics
    assert scene.se3_impedance==base.se3_impedance
    original=build_docking_trajectory(scene.task,scene.docking,scene.trajectory)
    truth_changed=replace(scene,target=replace(scene.target,pos=scene.target.pos+[.01,0,0]))
    unchanged=build_docking_trajectory(truth_changed.task,truth_changed.docking,truth_changed.trajectory)
    for a,b in zip(original.poses,unchanged.poses,strict=True):
        np.testing.assert_array_equal(a.homogeneous,b.homogeneous)


def test_names_preserve_sign_and_fraction_and_reject_nonfinite_axes():
    assert grid.point_key((-3.,6.,-7.5)) != grid.point_key((3.,6.,-7.5))
    assert grid.point_key((0.,0.,0.)) == grid.point_key((-0.,0.,-0.))
    assert grid.axes_checked([6.,0.,-6.,6.]) == [-6.,0.,6.]
    with pytest.raises(ValueError):
        grid.axes_checked([np.nan])


def test_midpoints_require_actual_adjacent_status_transition():
    axes=[[-6.,0.,6.],[-6.,0.,6.],[-15.,0.,15.]]
    pairs=grid.adjacent_pairs(axes)
    records={(-6.,0.,0.):record(False,lateral=.7), (0.,0.,0.):record(True),
             (6.,0.,0.):record(True), (6.,6.,0.):record(False,lateral=.8)}
    selected=grid.midpoint_candidates(pairs,records,4)
    assert {v[0] for v in selected}=={(-3.,0.,0.), (6.,3.,0.)}
    assert grid.midpoint_candidates(pairs,records,0)==[]
    checks=grid.boundary_points(records,pairs,2)
    assert len(checks)==2 and {grid.passed(records[p]) for p in checks}=={True,False}


def test_half_step_requires_equal_status_and_original_peak_limits():
    a,b=record(True),record(True)
    assert grid.sensitivity(a,b)["status"]=="STABLE_IN_TWO_STEPS"
    b["contact_load_gate"]["peak_contact_force_N"]=4.6
    assert grid.sensitivity(a,b)["status"]=="SENSITIVE"
    b=record(True)
    b["assessment"]["status"]="INCOMPLETE"
    assert grid.sensitivity(a,b)["status"]=="SENSITIVE"


def test_reuse_checks_controller_sources_and_entire_assessment_body(tmp_path):
    base=load_scene("scenes/iiwa14_petal_insertion.yaml")
    current=grid.source_manifest(base)
    snapshot=tmp_path/'source_snapshot/experiments'
    snapshot.mkdir(parents=True)
    import shutil
    shutil.copy2(grid.REPO_ROOT/'experiments/petal_insertion_suite.py',snapshot/'petal_insertion_suite.py')
    assert grid.reusable_sources(current,current,tmp_path)
    changed=dict(current,sources=dict(current["sources"]))
    changed["sources"]["src/compliant_docking/control/contact_yaw.py"]="changed"
    assert not grid.reusable_sources(current,changed,tmp_path)
    import ast
    source=snapshot/'petal_insertion_suite.py'
    tree=ast.parse(source.read_text())
    run=next(f for f in tree.body if isinstance(f,ast.FunctionDef) and f.name=='run_case')
    run.body.append(ast.Pass())
    source.write_text(ast.unparse(tree))
    assert not grid.reusable_sources(current,current,tmp_path)


def test_lateral_profiles_only_change_declared_axes_and_optional_speed():
    base=load_scene('scenes/iiwa14_petal_insertion.yaml')
    for profile,after in [('lateral_released',0.),('lateral_soft',20.)]:
        scene=suite.variant(base,'combined',profile)
        assert scene.se3_impedance.contact_yaw.lateral_stiffness_after==after
        assert scene.se3_impedance.k_diag==base.se3_impedance.k_diag
        assert scene.se3_impedance.a_diag==base.se3_impedance.a_diag
        assert scene.se3_impedance.d_diag==base.se3_impedance.d_diag
        assert scene.docking.insertion_speed==base.docking.insertion_speed
    slow=suite.variant(base,'combined','lateral_released_slow')
    assert slow.docking.insertion_speed==base.docking.insertion_speed/2
    np.testing.assert_array_equal(slow.docking.estimate_pos,base.docking.estimate_pos)
    assert suite.variant(base,'combined','released').se3_impedance.contact_yaw.lateral_stiffness_after is None


@pytest.mark.skipif(
    not (grid.REPO_ROOT / "runs/petal_contact_control_20261002_v2/source_manifest.json").exists(),
    reason="requires the ignored local frozen archive",
)
def test_current_controller_cannot_reuse_historical_archive():
    import json

    current = grid.source_manifest(load_scene("scenes/iiwa14_petal_insertion.yaml"))
    previous_dir = grid.REPO_ROOT / "runs/petal_contact_control_20261002_v2"
    previous = json.loads((previous_dir / "source_manifest.json").read_text())
    assert not grid.reusable_sources(previous, current, previous_dir)
