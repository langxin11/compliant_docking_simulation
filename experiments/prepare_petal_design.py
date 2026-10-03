"""Build a saved guide-only design through the immutable reviewed generator.

The private generator instance receives an explicit parameter policy. The wrapper
and delegated generator hashes are both recorded. Mounting geometry, guide height
and stop clearance must remain unchanged; other designs need a full CAD rebuild.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import petal_designer as designer

ALLOWED_CHANGES = {"guide_tip_half_angle_deg", "guide_blend_fraction", "guide_edge_round_mm",
                   "radial_lead_width_mm", "radial_crest_drop_mm"}


def checked_parameters(record):
    if record.get("schema") != "petal-guidance-design/v1":
        raise ValueError("Use a design saved by the interactive designer")
    values = designer.validate_parameters(record["design_parameters"])
    for key, value in values.items():
        if key not in ALLOWED_CHANGES and value != designer.NARROW[key]:
            raise ValueError(f"{key} changed: rebuild the mounting/stop CAD before generating")
    return designer.full_parameters(values)


def build(design_path, destination):
    design_path, destination = Path(design_path), Path(destination)
    if destination.exists():
        raise ValueError("Design output exists; select a new version directory")
    record = json.loads(design_path.read_text())
    parameters = checked_parameters(record)
    delegated_path = Path(designer.geometry.__file__).resolve()
    spec = importlib.util.spec_from_file_location("_private_petal_design_builder", delegated_path)
    private = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(private)
    # This fresh module instance is local to this build; the imported generator
    # used by the designer, archived studies and validators is never changed.
    private.candidate_parameters = lambda original, variant: dict(parameters)
    meta = private.generate("narrow", destination=destination)
    label = destination.name
    for side in ("active", "passive"):
        tree = ET.parse(destination/f"{side}.xml")
        tree.getroot().set("model", f"PetalDock100_design_{label}_{side}")
        ET.indent(tree)
        tree.write(destination/f"{side}.xml", encoding="utf-8", xml_declaration=True)
    meta.update(version=f"3.1-simulation-design-{label}", design_status="UNVALIDATED_DESIGN",
                design_source=str(design_path.resolve()))
    (destination/"model_info.json").write_text(json.dumps(meta, indent=2)+"\n")
    (destination/"selected_design.json").write_bytes(design_path.read_bytes())
    path = destination/"manifest.json"
    manifest = json.loads(path.read_text())
    manifest.update(
        interface=f"PetalDock100 saved design {label}",
        generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        base_generator_sha256=hashlib.sha256(delegated_path.read_bytes()).hexdigest(),
        parameter_policy="private module instance receives the declared saved design parameters",
        parameter_overrides={k: v for k, v in parameters.items() if v != designer.NARROW[k]},
        design_sha256=hashlib.sha256(design_path.read_bytes()).hexdigest(),
        imported_files={str(f.relative_to(destination)): hashlib.sha256(f.read_bytes()).hexdigest()
                        for f in sorted(destination.rglob("*")) if f.is_file() and f != path},
        changes=["explicit user-selected guide shape", "unchanged mounting plate and stop reference",
                 "full assembly inertia updated by reviewed guide-mesh delta"],
    )
    path.write_text(json.dumps(manifest, indent=2)+"\n")
    return meta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--design", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    info = build(args.design, args.out)
    print(args.out, info["parameters"], info["inertia_convergence"])


if __name__ == "__main__":
    main()
