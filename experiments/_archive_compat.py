"""Dispatch legacy imports and commands to the single archived implementation."""
from __future__ import annotations

import argparse
import importlib.util
import runpy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def dispatch(name: str, relative: str):
    source = ROOT / "archive/orbital_showcase" / relative
    for relative_output in ("outputs", "contact_force_study/outputs", "crown_seating/outputs"):
        (ROOT / "runs/archive_workspaces/orbital_showcase" / relative_output).mkdir(
            parents=True, exist_ok=True)
    # Historical modules import their siblings by their original bare names.
    for directory in (source.parent, ROOT / "archive/orbital_showcase"):
        if str(directory) not in sys.path:
            sys.path.insert(0, str(directory))
    if name == "__main__":
        if relative in {"crown_assembly_audit.py", "hexframe_assembly_audit.py", "crown_seating/audit.py"}:
            parser = argparse.ArgumentParser(description="Archived audit; source hashes remain strict")
            parser.add_argument("--out", type=Path, help="Writable copy of a complete historical run")
            args = parser.parse_args()
            module = dispatch("_archived_audit", relative)
            if args.out:
                destination = args.out.resolve()
                if destination.is_relative_to(ROOT / "results"):
                    parser.error("Frozen results are read-only evidence; use a writable run copy")
                module.OUT = destination
            module.main()
            return
        runpy.run_path(str(source), run_name=name)
        return
    spec = importlib.util.spec_from_file_location(name, source)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module
