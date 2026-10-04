"""Compatibility import for the layered experiment implementation."""
import sys
from importlib import import_module
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
_module = import_module("experiments.control.rq2_lateral")
sys.modules[__name__] = _module
