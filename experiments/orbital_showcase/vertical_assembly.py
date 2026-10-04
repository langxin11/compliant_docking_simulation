"""Compact spacecraft bus, bilateral slender solar wings and vertical side-port assembly.

The hexagonal end faces stand vertically. Rotating the complete module makes
side port 1 point upward and opposite side port 4 downward; no end-face port is added.
"""
# ruff: noqa: E402
from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET

import assembly_sequence as seq
import mujoco
import numpy as np
from build_assets import beam, geom, vector, write_tree
from PIL import Image, ImageDraw, ImageFont
from side_docking import APOTHEM, R_F_M, port_pose

TIP_ROTATION = np.diag([1., -1., -1.])
PICK = np.array([.42, .40, .18])
SEED = np.array([-.42, .40, .18])
LIFT_HEIGHT = .68
INITIAL_Q = np.array([1.20122253, .55704019, -.68567805, -1.35044302,
                      .34976560, 1.35536911, -2.62515672])


def configure():
    seq.OUT = seq.HERE/"outputs/vertical"
    seq.R_W_F = TIP_ROTATION
    seq.R_MODULE = TIP_ROTATION @ R_F_M
    seq.PICK, seq.SEED = PICK.copy(), SEED.copy()
    seq.PICK_TIP = PICK+[0., 0., APOTHEM+seq.ENGAGEMENT]
    seq.INSTALLED = SEED+[0., 0., 2*APOTHEM+seq.ENGAGEMENT]
    seq.INSTALL_TIP = seq.INSTALLED+[0., 0., APOTHEM+seq.ENGAGEMENT]
    seq.phases = phases


def phases():
    pick, installed = seq.PICK_TIP, seq.INSTALL_TIP
    ready = pick+[0., 0., .16]
    above_pick = np.r_[pick[:2], LIFT_HEIGHT]
    above_seed = np.r_[installed[:2], LIFT_HEIGHT]
    return [
        seq.Phase("stowed", "01  左侧组装 · 中央机械臂 · 右侧模块存储", 1.5, ready),
        seq.Phase("approach", "02  从上方接近模块 1 的接口 1", 4., pick+[0., 0., .04]),
        seq.Phase("capture", "03  竖直下降，对准并进入捕获位置", 4., pick),
        seq.Phase("grip_check", "04  锁定机械臂与模块 1，确认抓取", 1.5, pick, "grip_on"),
        seq.Phase("rack_release", "05  连接确认后，解除存放架锁", 1., pick, "rack_off"),
        seq.Phase("lift", "06  竖直提起模块 1，离开存储区", 3., above_pick),
        seq.Phase("transfer", "07  越过中央区域，转运到左侧组装位", 5., above_seed),
        seq.Phase("align", "08  模块 1 接口 4 朝下，对准模块 2 接口 1", 3., installed+[0., 0., .075]),
        seq.Phase("mate", "09  竖直向下对接，模块 2 固定在基座上", 6., installed),
        seq.Phase("assembly_check", "10  确认就位，锁定模块间连接", 2., installed, "assembly_on"),
        seq.Phase("gripper_release", "11  安装确认后，机械臂解锁", 1., installed, "grip_off"),
        seq.Phase("retreat", "12  机械臂竖直抬升撤离", 3., installed+[0., 0., .12]),
        seq.Phase("complete", "完成：两个模块竖直连接，机械臂已释放", 2., installed+[0., 0., .12]),
    ]


def compact_bus(world):
    body = ET.SubElement(world, "body", name="compact_bus")
    top = geom(body, "box", pos=[0, .25, -.02], size=[1.30, .90, .02], material="orb_metal")
    top.set("contype", "1")
    top.set("conaffinity", "1")
    geom(body, "box", pos=[0, .25, -.25], size=[1.24, .84, .21], material="orb_white")
    geom(body, "box", pos=[0, .25, -.48], size=[1.18, .78, .02], material="orb_dark")
    for x in [-1.24, 1.24]:
        for y in [-.53, 1.03]:
            beam(body, [x, y, -.44], [x, y, -.045], .014, "orb_metal")
    # Service-bus side panels give the base visible depth and structure.
    for x in np.linspace(-1.03, 1.03, 5):
        geom(body, "box", pos=[x, -.596, -.25], size=[.185, .006, .145], material="orb_dark")
        geom(body, "box", pos=[x, -.604, -.25], size=[.160, .005, .120], material="orb_white")
    for y in [-.65, 1.15]:
        beam(body, [-1.30, y, .012], [1.30, y, .012], .011, "orb_white")
    for x in [-1.30, 1.30]:
        beam(body, [x, -.65, .012], [x, 1.15, .012], .011, "orb_white")
    # Two equal wings extend outboard along X, below the working deck.
    for sign in [-1, 1]:
        beam(body, [sign*1.24, .25, -.23], [sign*1.55, .25, -.23], .025, "orb_metal")
        geom(body, "cylinder", pos=[sign*1.43, .25, -.23], zaxis=[1, 0, 0],
             size=[.054, .06], material="orb_gold")
        wing = ET.SubElement(body, "body", name=f"solar_wing_{'left' if sign < 0 else 'right'}",
                             pos=vector([sign*1.55, .25, -.23]), euler="12 0 0")
        for panel in range(3):
            geom(wing, "box", pos=[sign*(.4+.8*panel), 0, 0], size=[.391, .21, .012], material="orb_gold")
            for i in range(5):
                for j in [-1, 1]:
                    geom(wing, "box", pos=[sign*(.08+.16*i+.8*panel), j*.10, .015],
                         size=[.073, .092, .003], material="orb_cell")
    return body


def cradle(world, center, name, material="orb_dark"):
    x, y, z = center
    body = ET.SubElement(world, "body", name=name)
    bottom = z-APOTHEM-.0045
    geom(body, "box", pos=[x, y, .018], size=[.175, .145, .018], material=material)
    # Open central slot accommodates the downward-facing side interface.
    for dy in [-.089, .089]:
        rail = geom(body, "box", pos=[x, y+dy, bottom-.008], size=[.142, .007, .008], material="orb_metal")
        rail.set("contype", "1")
        rail.set("conaffinity", "1")
        for dx in [-.125, .125]:
            geom(body, "box", pos=[x+dx, y+dy, (bottom+.028)/2],
                 size=[.010, .014, (bottom-.028)/2], material="orb_white")
            geom(body, "box", pos=[x+dx, y+dy, bottom+.018], size=[.012, .012, .018], material="orb_gold")
    geom(body, "sphere", name=name+"_led", pos=[x, y-.148, .027], size=[.009], material="orb_cyan")


def build_model():
    # Reuse the tested arm, free module and three latch definitions; replace only
    # this experiment's environment and apply the configured rigid-body frames.
    seq.build_model(filename="vertical_core.xml", base_filename="vertical_arm_base.xml")
    tree = ET.parse(seq.ASSETS/"vertical_core.xml")
    root, world = tree.getroot(), tree.getroot().find("worldbody")
    root.set("model", "compact_vertical_assembly")
    for name in ["orbital_platform", "workcell", "storage", "spare_slot", "seed_station"]:
        world.remove(world.find(f"body[@name='{name}']"))
    compact_bus(world)
    for center, name in [(PICK, "storage"), (PICK+[.40, 0, 0], "spare_slot"), (SEED, "seed_station")]:
        cradle(world, center, name)
    # Mark the two functional zones without filling the deck with equipment.
    for center, width, material in [(SEED, .40, "orb_cyan"), (PICK+[.20, 0, 0], .78, "orb_gold")]:
        x, y, _ = center
        for dy in [-.185, .185]:
            beam(world, [x-width/2, y+dy, .006], [x+width/2, y+dy, .006], .002, material)
    write_tree(tree, "vertical_assembly.xml")
    return mujoco.MjModel.from_xml_path(str(seq.ASSETS/"vertical_assembly.xml"))


def check_geometry(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for module_name in ["module1", "module2"]:
        module = data.body(module_name)
        rotation = module.xmat.reshape(3, 3)
        for number in range(1, 7):
            body = data.body(f"{module_name}_port_{number}")
            local = rotation.T@(body.xpos-module.xpos)
            np.testing.assert_allclose(local, port_pose(number).translation, atol=1e-12)
            assert abs(local[2]) < 1e-12
        np.testing.assert_allclose(data.body(f"{module_name}_port_1").xmat.reshape(3, 3)[:, 2], [0, 0, 1], atol=1e-12)
        np.testing.assert_allclose(data.body(f"{module_name}_port_4").xmat.reshape(3, 3)[:, 2], [0, 0, -1], atol=1e-12)
    return dict(status="PASS", all_six_ports_on_original_side_faces=True,
                port1_normal_world=[0, 0, 1], port4_normal_world=[0, 0, -1],
                bus_dimensions_m=[2.6, 1.8, .5], wing_dimensions_m=[2.4, .42],
                wing_inner_edge_abs_x_m=1.55, wing_height_m=-.23,
                solar_note="Fixed visual tilt, no Sun-pointing or power analysis")


def preview_card():
    out = seq.OUT
    font = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 26)
    small = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 20)
    image = Image.new("RGB", (1600, 1100), "#0a1424")
    draw = ImageDraw.Draw(image)
    draw.text((36, 20), "竖直组装 / 紧凑厚基座 / 对称细长太阳翼", font=font, fill="#edf5ff")
    draw.text((36, 61), "左侧组装 · 中央机械臂 · 右侧存储    |    基座 2.6 × 1.8 × 0.5 m", font=small, fill="#a9bdd6")
    # A panoramic crop retains both complete wings while removing empty sky.
    image.paste(Image.open(out/"platform.png").crop((0, 160, 1280, 600)).resize((1536, 528)), (32, 111))
    image.paste(Image.open(out/"initial_workcell.png").resize((752, 423)), (32, 661))
    image.paste(Image.open(out/"installed.png").resize((752, 423)), (816, 661))
    image.save(out/"preview.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--replay", action="store_true")
    parser.add_argument("--no-render", action="store_true")
    args = parser.parse_args()
    configure()
    seq.OUT.mkdir(parents=True, exist_ok=True)
    model = build_model()
    geometry = check_geometry(model)
    (seq.OUT/"geometry_check.json").write_text(json.dumps(geometry, indent=2)+"\n")
    if args.replay:
        with np.load(seq.OUT/"rollout.npz") as saved:
            records = {key: saved[key] for key in saved.files}
        report = json.loads((seq.OUT/"validation.json").read_text())
    else:
        spline, phase_list, planning = seq.plan(model, initial_q=INITIAL_Q)
        records, report = seq.simulate(model, spline, phase_list, planning)
        report["collision_coverage"] = "robot proxies, module rails, cradle support rails, bus deck; solar wings visual only"
        lower, upper = np.asarray(planning["joint_limits"])
        report["minimum_joint_limit_margin_rad"] = float(np.min(np.concatenate([
            records["qpos"][:, :7]-lower, upper-records["qpos"][:, :7]], axis=1)))
        (seq.OUT/"validation.json").write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n")
        paths = [seq.HERE/"vertical_assembly.py", seq.HERE/"assembly_sequence.py",
                 seq.HERE/"build_assets.py", seq.HERE/"side_docking.py", seq.ASSETS/"vertical_assembly.xml"]
        (seq.OUT/"source_manifest.json").write_text(json.dumps({str(p.relative_to(seq.ROOT)):
            hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}, indent=2)+"\n")
    if not args.no_render:
        seq.render(model, records, phases(), args.video,
                   work_camera=seq.camera([0, .3, .22], 2.8, 85, -23),
                   wide_camera=seq.camera([0, .25, -.02], 6.5, 75, -24),
                   detail_azimuth=90, detail_elevation=-12)
        preview_card()
        print(seq.OUT/"preview.png", flush=True)
    if report["status"] != "PASS":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
