"""Freeze a completed geometry study and its reviewed project documentation."""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path

from compliant_docking.scene import REPO_ROOT


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while data := stream.read(1024*1024):
            digest.update(data)
    return digest.hexdigest()


def freeze(out):
    out = out.resolve()
    manifest_path = out/"artifact_manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        assert manifest["status"] == "FROZEN"
        for name,digest in manifest["artifacts_sha256"].items():
            assert sha(out/name) == digest,name
        print("Frozen artifacts already verified; no files changed.")
        return
    audit = json.loads((out/"validation_audit.json").read_text())
    assert audit["status"] == "PASS" and audit["complete_matrix"] and audit["records"] == 31
    assert json.loads((out/"scene_equivalence.json").read_text())["status"] == "PASS"
    previous = REPO_ROOT/"runs/petal_lateral_control_20261003"
    old = json.loads((previous/"artifact_manifest.json").read_text())
    for name,digest in old["artifacts_sha256"].items():
        assert sha(previous/name) == digest,name
    for name,digest in old["documentation_sha256"].items():
        assert sha(out/"history_documents"/name) == digest,name
    launch = REPO_ROOT/"runs"/(out.name+"_launch.log")
    if launch.exists():
        shutil.copy2(launch,out/"launch.log")
    documentation = ["README.md","docs/development_plan.md","docs/experiments.md",
        "results/petal_guidance_geometry_validation.md","scenes/iiwa14_petal_guided_insertion.yaml",
        "assets/interfaces/petal_guidance/README.md"]
    for name in documentation:
        snapshot = out/"final_documents"/name
        snapshot.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(REPO_ROOT/name,snapshot)
    for report in (REPO_ROOT/documentation[3],out/"report.md"):
        for target in re.findall(r"\]\(([^)]+)\)",report.read_text()):
            if target.startswith(("http:","https:","#")):
                continue
            assert (report.parent/target).resolve().exists(),(report,target)
    snapshot = out/"analysis_snapshot"/Path(__file__).name
    snapshot.parent.mkdir(exist_ok=True)
    shutil.copy2(Path(__file__),snapshot)
    source = json.loads((out/"source_manifest.json").read_text())
    assets = {}
    robot_assets = out/"robot_assets.json"
    if robot_assets.exists():
        for name,digest in json.loads(robot_assets.read_text())["files_sha256"].items():
            assert sha(REPO_ROOT/name) == digest,name
            assets[name] = digest
    for g,asset_manifest in source["geometry_assets"].items():
        from petal_guidance_geometry import DIRECTORIES
        for name,digest in asset_manifest["imported_files"].items():
            path = DIRECTORIES[g]/name
            assert sha(path) == digest,(g,name)
            assets[str(path.relative_to(REPO_ROOT))] = digest
        path = DIRECTORIES[g]/"manifest.json"
        assets[str(path.relative_to(REPO_ROOT))] = sha(path)
    # The runtime's existing equality guard now rejects this completed directory
    # before it can replace source snapshots or regenerate timestamped PDFs.
    source["dataset_state"] = "FROZEN"
    (out/"source_manifest.json").write_text(json.dumps(source,indent=2)+"\n")
    paths = sorted(p for p in out.rglob("*") if p.is_file() and "__pycache__" not in p.parts
        and p != manifest_path and not p.name.endswith(".tmp"))
    manifest = dict(status="FROZEN",records=31,
        artifacts_sha256={str(p.relative_to(out)):sha(p) for p in paths},
        documentation_sha256={name:sha(REPO_ROOT/name) for name in documentation},
        model_files_sha256=assets,history_artifacts_preserved=len(old["artifacts_sha256"]))
    manifest_path.write_text(json.dumps(manifest,indent=2)+"\n")
    for group,directory in (("artifacts_sha256",out),("documentation_sha256",REPO_ROOT),
                            ("model_files_sha256",REPO_ROOT)):
        for name,digest in manifest[group].items():
            assert sha(directory/name) == digest,(group,name)
    print(f"Verified {len(paths)} study artifacts, {len(documentation)} project files, "
          f"{len(assets)} model files, and preserved {manifest['history_artifacts_preserved']} historical artifacts.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out",type=Path)
    freeze(parser.parse_args().out)
