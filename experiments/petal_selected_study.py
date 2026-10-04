"""Compatibility import for the layered experiment implementation."""
import sys
from importlib import import_module
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
_module = import_module("experiments.models_interfaces.selected_candidate")
sys.modules[__name__] = _module
if __name__ == "__main__":
    _module.main()
