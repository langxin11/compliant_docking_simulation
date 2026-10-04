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


IMPORTED_MAINTAINED = {
    "assets/modules/hexframe/demo.py", "assets/modules/hexframe/validate_model.py",
    "assets/modules/hexframe/create_preview.py", "assets/modules/hexframe/generate.py",
    "assets/modules/hexframe/blender/build_scene.py", "assets/modules/hexframe/README.md",
    "assets/modules/hexframe/interface/source/generate.py",
}
GENERATED_EDIT_PATHS = IMPORTED_MAINTAINED | {
    "experiments/prepare_petal_interface.py", "experiments/prepare_petal_guidance.py",
    "experiments/prepare_convex_interface.py", "experiments/petal_selected_report.py",
    "src/compliant_docking/simulation/mujoco_env.py",
    "src/compliant_docking/simulation/consistency.py",
}


def apply_recorded_edits(before, entry):
    text = before.decode()
    for edit in entry["edits"]:
        if edit["before"] not in text:
            raise AssertionError(f"Recorded source edit no longer applies: {entry['path']}")
        text = text.replace(edit["before"], edit["after"])
    return text.encode()


def check_generated_policy():
    policy = json.loads((ROOT / "docs/evidence/generated_artifacts_20261004.json").read_text())
    artifacts = {item["old"]: item for item in policy["artifacts"]}
    expected_data = set(git_files(policy["baseline"], "assets/modules/hexframe/results"))
    expected_data |= set(git_files(policy["baseline"], "assets/modules/hexframe/preview"))
    expected_data.add("assets/modules/hexframe/blender/module.glb")
    if len(expected_data) != 20 or set(artifacts) != expected_data | {"demo/torque.png", "demo/tracking_error.png"}:
        raise AssertionError("Generated-artifact policy must cover exactly the declared 20 import and 2 demo files")
    for path, item in artifacts.items():
        if hashlib.sha256(git_bytes(policy["baseline"], path)).hexdigest() != item["sha256"]:
            raise AssertionError(f"Retired artifact Git baseline differs: {path}")
        if (ROOT / path).exists() or (ROOT / path).is_symlink():
            raise AssertionError(f"Generated artifact remains in source assets: {path}")
        local = ROOT / item["new"]
        if local.exists() and hashlib.sha256(local.read_bytes()).hexdigest() != item["sha256"]:
            raise AssertionError(f"Preserved local artifact bytes differ: {item['new']}")
        if not item["new"].startswith("runs/"):
            raise AssertionError(f"Retired data must remain under ignored runs: {path}")
    edits = {item["path"]: item for item in policy["source_edits"]}
    if set(edits) != GENERATED_EDIT_PATHS or len(edits) != len(policy["source_edits"]):
        raise AssertionError("Unexpected or missing maintained output source in policy")
    for path, entry in edits.items():
        before = git_bytes(policy["baseline"], path)
        if hashlib.sha256(before).hexdigest() != entry["before_sha256"]:
            raise AssertionError(f"Maintained source baseline differs: {path}")
        after = (ROOT / path).read_bytes()
        if after != apply_recorded_edits(before, entry) or hashlib.sha256(after).hexdigest() != entry["after_sha256"]:
            raise AssertionError(f"Source changed beyond recorded output edits: {path}")
    # All class/controller methods and generator geometry helpers retain their original AST.
    stable_classes = {"assets/modules/hexframe/demo.py": "ModuleDemo",
                      "src/compliant_docking/simulation/mujoco_env.py": "MujRobot",
                      "src/compliant_docking/simulation/consistency.py": "RobotController"}
    for path, name in stable_classes.items():
        old = {node.name: ast.dump(node) for node in ast.parse(git_bytes(policy["baseline"], path)).body if isinstance(node, ast.ClassDef)}
        new = {node.name: ast.dump(node) for node in ast.parse((ROOT / path).read_text()).body if isinstance(node, ast.ClassDef)}
        if old[name] != new[name]:
            raise AssertionError(f"Demo/controller class changed: {name}")
    for path in ("assets/modules/hexframe/generate.py", "assets/modules/hexframe/interface/source/generate.py"):
        before, after = functions(git_bytes(policy["baseline"], path)), functions((ROOT / path).read_text())
        for name in before.keys() - {"main"}:
            if before[name] != after[name]:
                raise AssertionError(f"Imported generator geometry changed: {path}:{name}")
    return policy, artifacts, edits


def check_directory_mapping():
    manifest = json.loads((ROOT / "docs/evidence/directory_cleanup_20261004.json").read_text())
    policy, retired, edits = check_generated_policy()
    checked, maintained, removed = 0, 0, 0
    for item in manifest["files"]:
        if item["operation"] == "move_local_output":
            continue  # Local ignored channels are deliberately not required in a clone.
        destination = ROOT / item["new"]
        if item["new"] in retired:
            if retired[item["new"]]["sha256"] != item["sha256"]:
                raise AssertionError(f"Retired data differs from preceding migration: {item['new']}")
            removed += 1
        elif item["new"] in edits:
            if edits[item["new"]]["before_sha256"] != item["sha256"]:
                raise AssertionError(f"Maintained import differs from original migration: {item['new']}")
            maintained += 1
        else:
            if hashlib.sha256(destination.read_bytes()).hexdigest() != item["sha256"]:
                raise AssertionError(f"Migration bytes changed: {item['new']}")
            checked += 1
        if item["previously_tracked"]:
            if hashlib.sha256(git_bytes(manifest["baseline"], item["old"])).hexdigest() != item["sha256"]:
                raise AssertionError(f"Migration baseline differs: {item['old']}")
    for path, target in manifest["compatibility_symlinks"].items():
        link = ROOT / path
        if not link.is_symlink() or str(link.readlink()) != target:
            raise AssertionError(f"Compatibility link differs: {path}")
        if "hexframe_module" in path and not link.is_dir():
            raise AssertionError(f"Production resource alias is dangling: {path}")
    return dict(baseline=manifest["baseline"], identical_files=checked,
                maintained_import_files=maintained, retired_generated_import_files=removed,
                generated_policy_baseline=policy["baseline"], local_ignored_channels_required=False)


def check_sources(baseline):
    checks = {}
    for old, (new, names) in MOVED.items():
        before = functions(git_bytes(baseline, old).decode())
        after = functions((ROOT / new).read_text())
        for name in names:
            if before[name] != after[name]:
                raise AssertionError(f"Migrated function changed: {name}")
            checks[name] = "IDENTICAL_AST"
    policy, retired, edits = check_generated_policy()
    physical_paths = ["src/compliant_docking/control", "src/compliant_docking/assembly",
                      "src/compliant_docking/planning"]
    subprocess.run(["git", "diff", "--quiet", baseline, "--", *physical_paths], cwd=ROOT, check=True)
    for path in git_files(baseline, "src/compliant_docking/simulation"):
        before = git_bytes(baseline, path)
        expected = apply_recorded_edits(before, edits[path]) if path in edits else before
        if expected != (ROOT / path).read_bytes():
            raise AssertionError(f"Simulation changed beyond exact manual output edits: {path}")
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
    expected_resources = {item["new"] for item in resources} - retired.keys()
    actual_resources = {str(path.relative_to(ROOT)) for path in (ROOT / "assets/modules/hexframe").rglob("*")
                        if path.is_file() and "__pycache__" not in path.parts}
    if actual_resources != expected_resources:
        raise AssertionError("Production resource inventory changed beyond migration mapping")
    for item in resources:
        before = git_bytes(baseline, item["old"])
        if item["new"] in retired:
            if hashlib.sha256(before).hexdigest() != retired[item["new"]]["sha256"]:
                raise AssertionError(f"Original excluded import data differs: {item['new']}")
            continue
        expected = apply_recorded_edits(before, edits[item["new"]]) if item["new"] in edits else before
        if expected != (ROOT / item["new"]).read_bytes():
            raise AssertionError(f"Production asset changed beyond exact policy: {item['new']}")
    return dict(functions=checks, unchanged_physical_paths=physical_paths,
                permitted_path_edits=["assembly.resource", "orchestration plot_results save_path", "orchestration video_dir"],
                directory_mapping=mapping, original_migrated_production_assets=len(resources),
                retained_production_inventory=len(expected_resources),
                unchanged_import_resources=len(expected_resources) - len(IMPORTED_MAINTAINED),
                maintained_import_files=len(IMPORTED_MAINTAINED), retired_generated_import_files=20,
                simulation_manual_output_edits="EXACT_RECORDED_EDITS_WITH_IDENTICAL_CLASSES")


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
