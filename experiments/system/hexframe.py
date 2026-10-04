"""Fixed formal HexFrame precheck, full acceptance and recorded-state replay."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MPLBACKEND", "Agg")

from compliant_docking.assembly.runner import run
from compliant_docking.scene import REPO_ROOT, load_scene

SCENE = REPO_ROOT / "scenes/hexframe_assembly.yaml"

def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("precheck", "accept", "replay"):
        sub = commands.add_parser(command)
        sub.add_argument("--out", type=Path, required=True)
        if command in ("accept", "replay"):
            sub.add_argument("--record", action="store_true")
    return parser

def main(argv=None):
    args = build_parser().parse_args(argv)
    status = run(load_scene(SCENE), output=args.out, record=getattr(args, "record", False),
                 preview_only=args.command == "precheck", replay=args.command == "replay")
    if args.command != "replay":
        entry = Path(__file__).resolve()
        manifest = dict(command=args.command, entrypoint=str(entry.relative_to(REPO_ROOT)),
                        entrypoint_sha256=hashlib.sha256(entry.read_bytes()).hexdigest(),
                        source_manifest="source_manifest.json", validation="validation.json")
        (args.out / "entrypoint_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return status

if __name__ == "__main__":
    raise SystemExit(main())
