"""Research naming, numerical criteria, source fingerprints and strict reuse."""
from __future__ import annotations

import ast
import hashlib
import itertools
import json

import mujoco
import numpy as np
import pinocchio as pin

from compliant_docking.scene import REPO_ROOT

from .rollout import _json_default


def point_key(point):
    def token(value):
        return f"{0. if value == 0 else value:+g}".replace("+", "p").replace("-", "m").replace(".", "d")
    return "grid_x"+token(point[0])+"_y"+token(point[1])+"_yaw"+token(point[2])

def record_name(point, setting="baseline"):
    return point_key(point)+"_released"+("_dt_half" if setting == "dt_half" else "")

def error_tuple(point):
    return point[0]/1000., point[1]/1000., point[2]

def axes_checked(values):
    if not values or not np.isfinite(values).all():
        raise ValueError("Grid axes must contain finite values")
    return sorted(set(float(v) for v in values))

def passed(record):
    return record["assessment"]["status"] == "CANDIDATE_PASS"

def adjacent_pairs(axes):
    """Only actual neighboring grid nodes, never diagonal interpolation."""
    result = []
    for point in itertools.product(*axes):
        for axis in range(3):
            index = axes[axis].index(point[axis])
            if index+1 < len(axes[axis]):
                other = list(point)
                other[axis] = axes[axis][index+1]
                result.append((tuple(point), tuple(other)))
    return result

def midpoint_candidates(pairs, records, limit):
    candidates = []
    for a, b in pairs:
        if a not in records or b not in records or passed(records[a]) == passed(records[b]):
            continue
        point = tuple(round((x+y)/2, 10) for x, y in zip(a, b, strict=True))
        if point not in records:
            # Prefer the nearest observed transition, spread across different
            # failure endpoints, and do not infer sign symmetry.
            failure = b if passed(records[a]) else a
            candidates.append((sum(abs(v) for v in point), point, failure, a, b))
    selected, used_failures = [], set()
    for _, point, failure, a, b in sorted(candidates):
        if failure not in used_failures and point not in [s[0] for s in selected]:
            selected.append((point, a, b))
            used_failures.add(failure)
        if len(selected) >= limit:
            return selected[:limit]
    for _, point, _, a, b in sorted(candidates):
        if point not in [s[0] for s in selected]:
            selected.append((point, a, b))
        if len(selected) >= limit:
            break
    return selected[:limit]

def margin(record):
    g, c = record.get("geometry_evaluation"), record.get("contact_load_gate")
    if g is None or c is None:
        return float("inf")
    return max(g["last_second_max_lateral_mm"]/.5,
               g["last_second_max_phase_error_deg"]/2.,
               g["last_second_max_abs_axial_gap_mm"]/.75,
               g["last_second_max_tilt_deg"]/.5,
               c["peak_contact_force_N"]/40., c["peak_contact_axial_moment_Nm"]/2.,
               1.01 if g["last_second_stop_contact_fraction"] < .95 else 0.)

def boundary_points(records, pairs, limit):
    pool = {p for a, b in pairs if a in records and b in records
            and passed(records[a]) != passed(records[b]) for p in (a, b)}
    pool |= {p for p, r in records.items() if r.get("grid_stage") == "refinement"}
    if not pool:
        pool = set(records)
    good = sorted((p for p in pool if passed(records[p])), key=lambda p: (-margin(records[p]), p))
    bad = sorted((p for p in pool if not passed(records[p]) and records[p].get("geometry_evaluation")),
                 key=lambda p: (margin(records[p]), p))
    selected = []
    for index in range(max(len(good), len(bad))):
        for group in (good, bad):
            if index < len(group) and group[index] not in selected:
                selected.append(group[index])
            if len(selected) >= limit:
                return selected[:limit]
    return selected[:limit]

def source_manifest(base):
    files = sorted((REPO_ROOT/"src").rglob("*.py")) + [
        REPO_ROOT/"experiments/petal_insertion_suite.py", REPO_ROOT/"experiments/petal_capture_grid.py",
        REPO_ROOT/"experiments/run_docking.py", REPO_ROOT/"experiments/insertion_suite.py",
        REPO_ROOT/"pyproject.toml", REPO_ROOT/"uv.lock", base.path]
    files += sorted(p for layer in ("control", "models_interfaces", "system") for p in (REPO_ROOT/"experiments"/layer).rglob("*.py"))
    return dict(mujoco_version=mujoco.__version__, pinocchio_version=pin.__version__,
                sources={str(p.relative_to(REPO_ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
                assets=json.loads((base.tool.mjcf.parent/"manifest.json").read_text()))

def write_json(path, value):
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, default=_json_default)+"\n")
    temporary.replace(path)

def reusable_sources(previous, current, directory):
    """Reject missing or changed provenance; migrated archives need fresh runs.

    Exact manifests are mandatory after the shared implementation migration.
    The saved assessment is also compared as AST, rather than trusting wrappers.
    """
    if previous != current:
        return False
    source = "src/compliant_docking/research/petal_trials.py"
    old_path = directory / "source_snapshot" / source
    if not old_path.is_file():
        return False
    old_tree = ast.parse(old_path.read_text())
    new_tree = ast.parse((REPO_ROOT / source).read_text())
    old = next((f for f in old_tree.body if isinstance(f, ast.FunctionDef) and f.name == "run_case"), None)
    new = next((f for f in new_tree.body if isinstance(f, ast.FunctionDef) and f.name == "run_case"), None)
    return old is not None and new is not None and ast.dump(old) == ast.dump(new)


def sensitivity(a, b):
    if not a.get("geometry_evaluation") or not b.get("geometry_evaluation"):
        return dict(status="NOT_COMPARABLE", differences={})
    g1, g2 = a["geometry_evaluation"], b["geometry_evaluation"]
    c1, c2 = a["contact_load_gate"], b["contact_load_gate"]
    differences = dict(lateral_mm=abs(g1["last_second_max_lateral_mm"]-g2["last_second_max_lateral_mm"]),
        axial_gap_mm=abs(g1["last_second_max_abs_axial_gap_mm"]-g2["last_second_max_abs_axial_gap_mm"]),
        phase_deg=abs(g1["last_second_max_phase_error_deg"]-g2["last_second_max_phase_error_deg"]),
        peak_force_N=abs(c1["peak_contact_force_N"]-c2["peak_contact_force_N"]),
        peak_axial_moment_Nm=abs(c1["peak_contact_axial_moment_Nm"]-c2["peak_contact_axial_moment_Nm"]))
    stable = (a["assessment"]["status"] == b["assessment"]["status"] and differences["lateral_mm"] <= .1
              and differences["axial_gap_mm"] <= .1 and differences["phase_deg"] <= .1
              and differences["peak_force_N"] <= max(.5, .1*c1["peak_contact_force_N"])
              and differences["peak_axial_moment_Nm"] <= max(.05, .1*c1["peak_contact_axial_moment_Nm"]))
    return dict(status="STABLE_IN_TWO_STEPS" if stable else "SENSITIVE", differences=differences,
                baseline=a["assessment"], half_dt=b["assessment"])
