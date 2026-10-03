"""Import reviewed PetalDock100 XML/meshes as ordinary Scene fragments.

Does not execute attachment scripts. Geometry and full inertia are preserved;
only body/site/visual names are adapted to the repository's scene contract.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from compliant_docking.scene import REPO_ROOT

DIRECTORY = REPO_ROOT / "assets/interfaces/petal_dock100"


def prepare(source, destination=DIRECTORY):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    assets = ET.parse(source / "mjcf/dock_assets.xml").getroot()
    inputs = {source / "mjcf/dock_assets.xml", source / "model_info.json"}
    destination.mkdir(parents=True, exist_ok=True)
    for mesh in assets.findall("./asset/mesh"):
        mesh_source = (source / "mjcf" / mesh.get("file")).resolve()
        if not mesh_source.is_relative_to(source):
            raise ValueError("Mesh path escapes attachment root")
        relative = mesh_source.relative_to(source)
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(mesh_source, target)
        mesh.set("file", relative.as_posix())
        inputs.add(mesh_source)
    for side in ("active", "passive"):
        path = source / f"mjcf/{side}_contents.xml"
        inputs.add(path)
        root = ET.Element("mujoco", model=f"PetalDock100_{side}_fragment")
        ET.SubElement(root, "compiler", angle="radian")
        for child in assets:
            root.append(copy.deepcopy(child))
        body = ET.SubElement(ET.SubElement(root, "worldbody"), "body", name="dock")
        for child in ET.parse(path).getroot():
            child = copy.deepcopy(child)
            if child.get("name"):
                name = child.get("name").removeprefix(side+"_")
                if name.startswith("visual_"):
                    name = name.removeprefix("visual_")+"_visual"
                elif name == "flange":
                    name = "sensor_site"
                child.set("name", name)
            body.append(child)
        ET.indent(root)
        ET.ElementTree(root).write(destination/f"{side}.xml", encoding="utf-8", xml_declaration=True)
    shutil.copyfile(source / "model_info.json", destination / "model_info.json")
    generated = sorted(p for p in destination.rglob("*") if p.is_file() and p.name != "manifest.json")
    manifest = dict(
        interface="PetalDock100 V2", source_files={str(p.relative_to(source)):
            hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(inputs)},
        imported_files={str(p.relative_to(destination)):
            hashlib.sha256(p.read_bytes()).hexdigest() for p in generated},
        changes=["root body renamed dock", "flange site renamed sensor_site",
                 "side prefixes removed; visual names end in _visual",
                 "mesh file paths relocated; geometry and inertia unchanged"],
        bundled_manifest_used=False)
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    args = parser.parse_args()
    manifest = prepare(args.source)
    print(f"Imported {len(manifest['imported_files'])} files to {DIRECTORY}")
