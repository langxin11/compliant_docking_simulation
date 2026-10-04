"""Compatibility entry point; implementation lives in archive/orbital_showcase/build_assets.py."""
# ruff: noqa: E402
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from experiments._archive_compat import dispatch

dispatch(__name__, "build_assets.py")
