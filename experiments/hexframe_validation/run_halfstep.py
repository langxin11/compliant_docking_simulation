"""Compatibility entry for system validation; historical protocol unchanged."""
import sys
from importlib import import_module
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
_module = import_module("experiments.system.validation.run_halfstep")
sys.modules[__name__] = _module
if __name__ == "__main__":
    _module.main()
