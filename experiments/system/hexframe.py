"""HexFrame 正式流程入口：预检、完整验收和同源记录回放。

固定场景采用关节伺服与接触导纳。precheck 结果为 INCOMPLETE；accept 执行完整流程；
replay 读取既有验收记录。输出与入口指纹写入指定目录，返回底层运行器退出码。
"""
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
    """建立 precheck/accept/replay 子命令与必需输出目录参数。"""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("precheck", "accept", "replay"):
        sub = commands.add_parser(command)
        sub.add_argument("--out", type=Path, required=True)
        if command in ("accept", "replay"):
            sub.add_argument("--record", action="store_true")
    return parser

def main(argv=None):
    """执行选定系统阶段，非回放时保存入口指纹，返回运行器状态码。"""
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
