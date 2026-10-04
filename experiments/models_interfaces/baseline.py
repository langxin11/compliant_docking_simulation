"""Qualify the fixed original Petal model; never change interface or controller."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from compliant_docking.research.petal_trials import preflight
from compliant_docking.scene import REPO_ROOT, load_scene


def check_baseline():
    scene = load_scene("scenes/iiwa14_petal_insertion.yaml")
    result = preflight(scene)
    manifest_path = scene.tool.mjcf.parent / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    fingerprints = {}
    for name, digest in manifest["imported_files"].items():
        path = manifest_path.parent / name
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != digest:
            raise ValueError(f"Asset fingerprint mismatch: {path}")
        fingerprints[str(path.relative_to(REPO_ROOT))] = actual
    for path in (manifest_path, scene.path, scene.robot.mjcf):
        fingerprints[str(path.relative_to(REPO_ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(status="PASS", scope="declared fixed-base zero-gravity rigid-body simulation only",
                model_preflight=result, inputs_sha256=fingerprints)

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.out.exists() and any(args.out.iterdir()):
        raise ValueError("Use a new baseline-check output directory")
    result = check_baseline()
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "baseline_check.json").write_text(json.dumps(result, indent=2) + "\n")
    print(result["status"])

if __name__ == "__main__":
    main()
