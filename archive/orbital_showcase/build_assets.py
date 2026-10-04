"""Create original, visual-only orbital assembly assets beside this script."""
from __future__ import annotations

import itertools
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ASSETS = HERE / "assets"


def vector(values):
    return " ".join(f"{x:.16g}" for x in values)


def geom(parent, kind, *, material="orb_metal", **attrs):
    values = {k: vector(v) if isinstance(v, (list, tuple, np.ndarray)) else str(v)
              for k, v in attrs.items()}
    return ET.SubElement(parent, "geom", type=kind, material=material,
                         contype="0", conaffinity="0", mass="0", group="4", **values)


def beam(parent, start, end, radius=.003, material="orb_metal"):
    geom(parent, "capsule", fromto=[*start, *end], size=[radius], material=material)


def palette(asset):
    for name, rgba, specular in [
        ("metal", [.78, .83, .86, 1], .65),
        ("white", [.91, .94, .95, 1], .35),
        ("blue", [.12, .32, .88, 1], .55),
        ("dark", [.035, .06, .12, 1], .22),
        ("gold", [.72, .49, .19, 1], .55),
        ("cyan", [.12, .78, .90, 1], .40),
        ("cell", [.025, .08, .24, 1], .70),
    ]:
        ET.SubElement(asset, "material", name="orb_"+name, rgba=vector(rgba),
                      specular=str(specular), shininess="0.5")


def write_tree(tree, name):
    ET.indent(tree)
    tree.write(ASSETS/name, encoding="unicode", xml_declaration=False)


def build(sky_color="blue", with_pedestal=True, filename="arm.xml"):
    ASSETS.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(ROOT/"assets/iiwa14/iiwa14_arm.xml")
    robot = tree.getroot()
    robot.find("compiler").set("meshdir", str(ROOT/"assets/iiwa14/assets"))
    asset = robot.find("asset")
    palette(asset)
    sky = asset.find("texture[@type='skybox']")
    colors = {"blue": (".34 .47 .60", ".12 .22 .34"),
              "black": (".018 .035 .075", ".002 .006 .018")}
    rgb1, rgb2 = colors[sky_color]
    sky.attrib.update(builtin="gradient", rgb1=rgb1, rgb2=rgb2,
                      mark="random", markrgb=".90 .94 1", random=".0015",
                      width="1024", height="6144")
    for name in ["gray", "light_gray"]:
        asset.find(f"material[@name='{name}']").set("rgba", ".87 .91 .94 1")
    asset.find("material[@name='orange']").set("rgba", ".22 .35 .48 1")
    asset.find("material[@name='groundplane']").set("rgba", "0 0 0 0")
    visual = robot.find("visual")
    ET.SubElement(visual, "headlight", ambient=".28 .30 .36", diffuse=".65 .68 .75",
                  specular=".18 .18 .18")
    world = robot.find("worldbody")
    platform = ET.SubElement(world, "body", name="orbital_platform")
    # A 4.8 x 3.8 m deck deliberately dwarfs the roughly one-metre arm.
    geom(platform, "box", pos=[0, .25, -.075], size=[2.4, 1.9, .07], material="orb_white")
    geom(platform, "box", pos=[0, .25, -.20], size=[2.2, 1.7, .06], material="orb_dark")
    for x in np.linspace(-2.3, 2.3, 24):
        beam(platform, [x, -1.58, .001], [x, 2.08, .001], .003, "orb_metal")
    for x in [-2.4, 2.4]:
        beam(platform, [x, -1.65, .015], [x, 2.15, .015], .022, "orb_white")
    for y in [-1.65, 2.15]:
        beam(platform, [-2.4, y, .015], [2.4, y, .015], .022, "orb_white")
    for x in np.linspace(-2.2, 2.2, 12):
        beam(platform, [x, -1.45, -.265], [x+.20, 1.95, -.265], .012, "orb_metal")
    # Outboard solar wings: original geometry, no third-party mesh downloads.
    for sign in [-1, 1]:
        beam(platform, [sign*2.2, .25, -.15], [sign*2.95, .25, -.15], .025, "orb_gold")
        # Long, slender photovoltaic wings: 0.8 x 6.0 m (aspect ratio 7.5).
        geom(platform, "box", pos=[sign*2.95, .25, -.15], size=[.4, 3.0, .016],
             material="orb_gold")
        for x in range(3):
            for y in range(22):
                cx, cy = sign*(2.69+x*.26), -2.60+y*.27
                geom(platform, "box", pos=[cx, cy, -.130],
                     size=[.119, .125, .003], material="orb_cell")
                beam(platform, [cx-.12, cy, -.126],
                     [cx+.12, cy, -.126], .0012, "orb_metal")
    if with_pedestal:
        # A support pedestal ends below the existing socket.
        geom(platform, "box", pos=[0, 1.10, .11], size=[.14, .14, .018])
        for x, y in itertools.product([-.12, .12], [.98, 1.22]):
            beam(platform, [x, y, .12], [x, y, .495], .003)
        geom(platform, "box", pos=[0, 1.10, .507], size=[.14, .14, .012], material="orb_dark")
        for x in [-.12, .12]:
            beam(platform, [x, .98, .12], [x, 1.22, .495], .002)
            beam(platform, [x, 1.22, .12], [x, .98, .495], .002)
    write_tree(tree, filename)
    return ASSETS/filename


if __name__ == "__main__":
    print(build())
