"""当前默认 Petal 固定模型的资格检查入口。

读取正式场景及资产清单，检查双引擎一致性、初值和哈希；向新目录写检查报告。
PASS 仅覆盖声明的固定基座、零重力刚体模型检查，不代表完成对接或实机标定。
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from compliant_docking.research.petal_trials import preflight
from compliant_docking.scene import REPO_ROOT, load_scene


def check_baseline():
    """核对默认模型与资产 SHA-256，返回检查数据；资产不匹配时抛出 ValueError。"""
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
    """运行模型检查并写 baseline_check.json；拒绝向非空目录写入。"""
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
