"""Read-only migration checks against a Git baseline and optional fresh runs.

uv run python scripts/check_restructuring.py --baseline 16a54d4 --out runs/.../migration_check.json
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
MOVED = {
    "experiments/petal_insertion_suite.py": ("src/compliant_docking/research/petal_trials.py",
        ("variant", "preflight", "feedback_audit", "run_case", "summarize")),
    "experiments/insertion_suite.py": ("src/compliant_docking/research/rollout.py",
        ("_json_default", "save_rollout", "preview_rollout")),
}


def functions(text):
    return {node.name: ast.dump(node) for node in ast.parse(text).body if isinstance(node, ast.FunctionDef)}


def git_bytes(ref, path):
    return subprocess.check_output(["git", "show", f"{ref}:{path}"], cwd=ROOT)


def git_files(ref, path):
    return subprocess.check_output(
        ["git", "ls-tree", "-r", "--name-only", ref, "--", path], cwd=ROOT, text=True
    ).splitlines()


def check_directory_mapping():
    manifest = json.loads((ROOT / "docs/evidence/directory_cleanup_20261004.json").read_text())
    checked = 0
    for item in manifest["files"]:
        if item["operation"] == "move_local_output":
            continue  # Local ignored channels are deliberately not required in a clone.
        destination = ROOT / item["new"]
        if hashlib.sha256(destination.read_bytes()).hexdigest() != item["sha256"]:
            raise AssertionError(f"Migration bytes changed: {item['new']}")
        if item["previously_tracked"]:
            if hashlib.sha256(git_bytes(manifest["baseline"], item["old"])).hexdigest() != item["sha256"]:
                raise AssertionError(f"Migration baseline differs: {item['old']}")
        checked += 1
    for path, target in manifest["compatibility_symlinks"].items():
        link = ROOT / path
        if not link.is_symlink() or str(link.readlink()) != target:
            raise AssertionError(f"Compatibility link differs: {path}")
        if "hexframe_module" in path and not link.is_dir():
            raise AssertionError(f"Production resource alias is dangling: {path}")
    return dict(baseline=manifest["baseline"], identical_files=checked,
                local_ignored_channels_required=False)


def check_sources(baseline):
    checks = {}
    for old, (new, names) in MOVED.items():
        before = functions(git_bytes(baseline, old).decode())
        after = functions((ROOT / new).read_text())
        for name in names:
            if before[name] != after[name]:
                raise AssertionError(f"Migrated function changed: {name}")
            checks[name] = "IDENTICAL_AST"
    physical_paths = ["src/compliant_docking/control", "src/compliant_docking/assembly",
                      "src/compliant_docking/planning", "src/compliant_docking/simulation"]
    subprocess.run(["git", "diff", "--quiet", baseline, "--", *physical_paths], cwd=ROOT, check=True)
    # Permit exact generated-output destination edits; every other orchestration byte remains fixed.
    for path in git_files(baseline, "src/compliant_docking/orchestration"):
        before = git_bytes(baseline, path).decode()
        after = (ROOT / path).read_text()
        if path.endswith("/run_docking.py"):
            after = after.replace('save_path="runs/figures/"', 'save_path="figure/"')
            after = after.replace('os.path.join(current_dir, "runs", "videos")', 'os.path.join(current_dir, "video")')
            after = after.replace('# 生成视频与其它运行产物一起写入 runs/videos/ /', '# 视频固定落在仓库根 video/，与编排器位于 experiments/ 时期一致 /')
            after = after.replace('# Generated videos land in <repo>/runs/videos/', '# Videos land in <repo>/video exactly as when this module lived in experiments/')
        if before != after:
            raise AssertionError(f"Orchestration changed beyond output path: {path}")
    # Only the declared production-resource location may change in the scene configuration.
    for path in git_files(baseline, "scenes"):
        before = git_bytes(baseline, path)
        after = (ROOT / path).read_bytes()
        if path == "scenes/hexframe_assembly.yaml":
            left, right = yaml.safe_load(before), yaml.safe_load(after)
            assert left["assembly"]["resource"] == "experiments/orbital_showcase/assets/hexframe_module"
            assert right["assembly"]["resource"] == "assets/modules/hexframe"
            right["assembly"]["resource"] = left["assembly"]["resource"]
            if left != right:
                raise AssertionError(f"Scene changed beyond resource path: {path}")
        elif before != after:
            raise AssertionError(f"Scene bytes changed: {path}")
    # Existing assets and every migrated resource are checked against original Git blobs.
    for path in git_files(baseline, "assets"):
        if git_bytes(baseline, path) != (ROOT / path).read_bytes():
            raise AssertionError(f"Existing asset bytes changed: {path}")
    mapping = check_directory_mapping()
    manifest = json.loads((ROOT / "docs/evidence/directory_cleanup_20261004.json").read_text())
    resources = [item for item in manifest["files"] if item["operation"] == "move_production_asset"]
    expected_resources = {item["new"] for item in resources}
    actual_resources = {str(path.relative_to(ROOT)) for path in (ROOT / "assets/modules/hexframe").rglob("*")
                        if path.is_file() and "__pycache__" not in path.parts}
    if actual_resources != expected_resources:
        raise AssertionError("Production resource inventory changed beyond migration mapping")
    for item in resources:
        if git_bytes(baseline, item["old"]) != (ROOT / item["new"]).read_bytes():
            raise AssertionError(f"Production asset changed: {item['new']}")
    return dict(functions=checks, unchanged_physical_paths=physical_paths,
                permitted_path_edits=["assembly.resource", "orchestration plot_results save_path", "orchestration video_dir"],
                directory_mapping=mapping, migrated_production_assets=len(resources))


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
