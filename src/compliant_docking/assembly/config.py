"""Validated assembly configuration; imported CAD remains the authority."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]

@dataclass(frozen=True)
class ModuleResource:
    path: Path
    mapping: tuple[int, ...] = (3, 4, 5, 0, 1, 2)

    @property
    def info(self):
        return json.loads((self.path / "model_info.json").read_text())

    @property
    def apothem(self):
        return float(self.info["ports"][0]["position_m"][0])

    @property
    def gap(self):
        return 2 * self.info["port_mating_site_offset_m"]

    @property
    def pair_rotation(self):
        return Rotation.from_quat(self.info["port_pair_rotation_wxyz"], scalar_first=True).as_matrix()

    def port(self, number):
        if number not in range(1, 7):
            raise ValueError("HexFrame port number must be 1–6")
        return self.info["ports"][self.mapping[number-1]]

@dataclass(frozen=True)
class AssemblyScene:
    source: Path
    name: str
    resource: ModuleResource
    baseline: Path
    pick: tuple[float, float, float]
    seed: tuple[float, float, float]
    lift_height: float
    retract_distance: float
    contact_duration: float
    contact_force_n: float
    timestep: float

    @property
    def docking(self):
        return None

    def build_mujoco_model(self, output: Path):
        from .runtime import AssemblyRuntime
        runtime = AssemblyRuntime(self, output)
        return runtime.geometry.build_model()


def load_assembly_scene(path: Path, raw=None):
    raw = raw or yaml.safe_load(path.read_text())
    if raw["scene"].get("kind") != "hexframe_assembly":
        raise ValueError("Unsupported assembly scene kind")
    cfg = raw["assembly"]
    def asset(value):
        p = Path(value)
        return p if p.is_absolute() else ROOT / p
    physics = raw["physics"]
    if physics != {"timestep": .001, "gravity": [0, 0, 0]}:
        raise ValueError("Validated assembly requires 1 ms zero-gravity physics")
    mapping = tuple(cfg["port_mapping"])
    if mapping != (3, 4, 5, 0, 1, 2):
        raise ValueError("HexFrame side-port mapping must be 3,4,5,0,1,2")
    layout = cfg["layout"]
    for key in ["pick", "seed"]:
        v = np.asarray(layout[key], dtype=float)
        if v.shape != (3,) or not np.isfinite(v).all():
            raise ValueError(f"Invalid {key} position")
    for key in ["lift_height", "retract_distance", "contact_duration", "contact_force_n"]:
        if not np.isfinite(layout[key]) or layout[key] <= 0:
            raise ValueError(f"Invalid {key}")
    if not .15 <= layout["contact_force_n"] <= .6:
        raise ValueError("Contact force target must lie within the unchanged seating gate")
    return AssemblyScene(path, raw["scene"]["name"], ModuleResource(asset(cfg["resource"]), mapping),
                         asset(cfg["workcell"]), tuple(layout["pick"]), tuple(layout["seed"]),
                         layout["lift_height"], layout["retract_distance"], layout["contact_duration"],
                         layout["contact_force_n"], physics["timestep"])
