"""Read-only migration checks against a Git baseline and optional fresh runs.

uv run python scripts/check_restructuring.py --baseline 16a54d4 --out runs/.../migration_check.json
"""
from __future__ import annotations

import argparse
import ast
import json
import subprocess
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MOVED = {
    "experiments/petal_insertion_suite.py": ("src/compliant_docking/research/petal_trials.py",
        ("variant", "preflight", "feedback_audit", "run_case", "summarize")),
    "experiments/insertion_suite.py": ("src/compliant_docking/research/rollout.py",
        ("_json_default", "save_rollout", "preview_rollout")),
}


def functions(text):
    return {node.name: ast.dump(node) for node in ast.parse(text).body if isinstance(node, ast.FunctionDef)}


def check_sources(baseline):
    checks = {}
    for old, (new, names) in MOVED.items():
        before = functions(subprocess.check_output(["git", "show", f"{baseline}:{old}"], cwd=ROOT, text=True))
        after = functions((ROOT / new).read_text())
        for name in names:
            if before[name] != after[name]:
                raise AssertionError(f"Migrated function changed: {name}")
            checks[name] = "IDENTICAL_AST"
    physical_paths = ["src/compliant_docking/control", "src/compliant_docking/assembly",
                      "src/compliant_docking/orchestration", "scenes", "assets"]
    subprocess.run(["git", "diff", "--quiet", baseline, "--", *physical_paths], cwd=ROOT, check=True)
    return dict(functions=checks, unchanged_physical_paths=physical_paths)


def check_petal(old, new):
    left = json.loads((old / "nominal_released.json").read_text())
    right = json.loads((new / "nominal_released.json").read_text())
    fields = ("scene", "assessment", "geometry_evaluation", "contact_load_gate", "feedback_audit",
              "gate", "simulation_warnings", "waypoints", "waypoint_times")
    for field in fields:
        if left[field] != right[field]:
            raise AssertionError(f"Petal metadata differs: {field}")
    channels = 0
    for suffix in ("npz", "contacts.npz"):
        with np.load(old / f"nominal_released.{suffix}") as a, np.load(new / f"nominal_released.{suffix}") as b:
            assert a.files == b.files
            for field in a.files:
                np.testing.assert_array_equal(a[field], b[field])
                channels += 1
    return dict(status="IDENTICAL", case="nominal", profile="released", setting="baseline",
                metadata_fields=list(fields), identical_array_channels=channels,
                old=str(old), new=str(new))


def check_system(old, new):
    left = json.loads((old / "validation.json").read_text())
    right = json.loads((new / "validation.json").read_text())
    fields = ("status", "faults", "first_contact_s", "lock_time_s", "peak_axial_force_n",
              "max_penetration_mm", "final_module_error_mm", "events")
    for field in fields:
        if left[field] != right[field]:
            raise AssertionError(f"System acceptance differs: {field}")
    assert json.loads((new / "audit.json").read_text())["status"] == "PASS"
    return dict(status="IDENTICAL", fields=list(fields), metrics={k: right[k] for k in fields if k != "events"},
                events_identical=True, independent_audit="PASS", old=str(old), new=str(new))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", default="16a54d4")
    parser.add_argument("--petal-old", type=Path)
    parser.add_argument("--petal-new", type=Path)
    parser.add_argument("--system-old", type=Path)
    parser.add_argument("--system-new", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = dict(baseline=args.baseline, source_checks=check_sources(args.baseline))
    if args.petal_old or args.petal_new:
        if not args.petal_old or not args.petal_new:
            parser.error("Provide both Petal directories")
        result["petal"] = check_petal(args.petal_old, args.petal_new)
    if args.system_old or args.system_new:
        if not args.system_old or not args.system_new:
            parser.error("Provide both system directories")
        result["system"] = check_system(args.system_old, args.system_new)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
