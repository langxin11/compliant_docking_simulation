"""User-supplied HexFrame geometry, mass and ports in the vertical robot scene."""
from __future__ import annotations

import json
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path

import assembly_sequence as seq
import numpy as np
import vertical_assembly as vertical
from scipy.spatial.transform import Rotation

from compliant_docking.assembly.config import ModuleResource
from compliant_docking.assembly.geometry import HexFrameGeometry

HERE = Path(__file__).resolve().parent
RESOURCE = HERE / "assets/hexframe_module"
INFO = json.loads((RESOURCE / "model_info.json").read_text())
A = INFO["ports"][0]["position_m"][0]
GAP = 2*INFO["port_mating_site_offset_m"]
PAIR_ROTATION = Rotation.from_euler("z", 45, degrees=True).as_matrix()@np.diag([1., -1., -1.])


@dataclass(frozen=True)
class HexFrameLayout:
    pick: tuple[float, float, float] = (.34, .35, .195)
    seed: tuple[float, float, float] = (-.34, .35, .195)
    lift_height: float = .78
    retract_distance: float = .075
    contact_duration: float = 18.
    contact_force_n: float = .4


@dataclass(frozen=True)
class SceneHooks:
    build_model: Callable
    phases: Callable
    wrench: Callable
    seating_contact: Callable
    storage_wrench: Callable
    contact_force_n: float


@contextmanager
def configure(engine, layout: HexFrameLayout | None = None) -> Iterator[SceneHooks]:
    """Scope legacy globals to one run; explicit hooks select module behavior.

    Compatibility helpers use shared state internally, so a process runs one
    scenario at a time. All state is restored even if planning/rendering raises.
    """
    layout = layout or HexFrameLayout()
    settings = [(vertical, ["PICK", "SEED", "LIFT_HEIGHT", "APOTHEM"]),
                (seq, ["APOTHEM", "ENGAGEMENT", "OUT", "R_W_F", "R_MODULE", "PICK", "SEED",
                       "PICK_TIP", "INSTALLED", "INSTALL_TIP", "phases", "LOCK_SITES"]),
                (engine, ["APOTHEM", "HEIGHT", "OUT"])]
    saved = [(module, {name: getattr(module, name) for name in names}) for module, names in settings]

    def adjusted_phases():
        result = engine.phases()
        result[9].seconds = layout.contact_duration
        # The larger module raises the installed tool pose; 75 mm retract is
        # enough to clear the head while keeping this arm within its workspace.
        for index in [11, 12]:
            result[index].end = seq.INSTALL_TIP+[0, 0, layout.retract_distance]
        return result

    try:
        vertical.PICK = np.array(layout.pick)
        vertical.SEED = np.array(layout.seed)
        vertical.LIFT_HEIGHT = layout.lift_height
        vertical.APOTHEM = seq.APOTHEM = A
        seq.ENGAGEMENT = GAP
        engine.APOTHEM, engine.HEIGHT = A, GAP
        engine.OUT = HERE / "outputs/hexframe_assembly"
        engine.configure()
        seq.LOCK_SITES = [("storage_anchor", "module1_port_4_mating"), *seq.LOCK_SITES[1:]]
        result_titles = {
            0: "01  左侧基座锁定 · 中央机械臂 · 右侧标准存储接口",
            4: "05  机械臂锁定确认后，存储接口解锁",
            5: "06  竖直提起模块 1，脱离存储接口",
        }
        def socket_phases():
            result = adjusted_phases()
            for index, title in result_titles.items():
                result[index].title = title
            return result
        yield SceneHooks(lambda: build_model(engine), socket_phases, wrench, stop_loaded, storage_wrench,
                         layout.contact_force_n)
    finally:
        for module, values in reversed(saved):
            for name, value in values.items():
                setattr(module, name, value)


# Compatibility only: formal runs use AssemblyRuntime and never enter configure().


class _LegacyRuntime:
    resource = ModuleResource(RESOURCE)
    baseline = seq.ASSETS / "vertical_assembly.xml"
    root = seq.ROOT

    def __init__(self, engine=None):
        self.output = engine.OUT if engine is not None else seq.OUT
        self.pick, self.seed = vertical.PICK, vertical.SEED
        self.module_rotation, self.tip_rotation = seq.R_MODULE, vertical.TIP_ROTATION

    def lock_error(self, model, data, index):
        return seq.lock_error(model, data, index)


def _geometry(engine=None):
    return HexFrameGeometry(_LegacyRuntime(engine))


def build_model(engine):
    return _geometry(engine).build_model()


def check_base_dock(model, data):
    return _geometry().check_base_dock(model, data)


def check_storage_docks(model, data):
    return _geometry().check_storage_docks(model, data)


def is_storage_contact(names):
    return _geometry().is_storage_contact(names)


def storage_wrench(model, data):
    return _geometry().storage_wrench(model, data)


def wrench(model, data):
    return _geometry().wrench(model, data)


def stop_loaded(model, data):
    return _geometry().stop_loaded(model, data)
