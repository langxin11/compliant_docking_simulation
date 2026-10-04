"""Independent orbital scene generator, physics check, rollout and replay."""
# ruff: noqa: E402 -- set the graphics backend before importing MuJoCo.
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from contextlib import redirect_stdout
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MPLBACKEND", "Agg")
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT/"src"))

import imageio.v2 as imageio
import mujoco
import numpy as np
from build_assets import build
from PIL import Image, ImageDraw, ImageFont
from side_docking import build_side_assets, geometry_manifest, make_scene, simulate, validate


def camera(lookat, distance, azimuth=135, elevation=-23):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    return cam


def render(scene, out, q, times, video):
    model = scene.build_mjmodel()
    model.vis.map.znear = .003
    model.vis.map.zfar = 30
    data = mujoco.MjData(model)
    options = mujoco.MjvOption()
    options.geomgroup[3] = False
    options.geomgroup[4] = True
    options.sitegroup[:] = False
    overview = camera([0, .25, -.22], 8.8, 125, -30)
    detail = camera([0, .5, .46], .70, 135, -15)
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(font_path, 24)
    small = ImageFont.truetype(font_path, 15)
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        def frame(index, cam):
            data.qpos[:] = q[index]
            mujoco.mj_forward(model, data)
            if cam is detail:
                center = data.body("tool_module").xpos
                target_center = data.body("target_module").xpos
                cam.lookat[:] = (center + target_center)/2
                cam.distance = max(.66, float(np.linalg.norm(center-target_center))*1.8+.25)
            renderer.update_scene(data, camera=cam, scene_option=options)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            return renderer.render().copy()
        index = int(len(q)*.28)
        wide = frame(index, overview)
        close = frame(index, detail)
        imageio.imwrite(out/"overview.png", wide)
        imageio.imwrite(out/"detail.png", close)
        imageio.imwrite(out/"assembly.png", frame(index, camera([0, .65, .52], 1.75, 135, -18)))
        poster = Image.new("RGB", (1600, 1040), "#080f20")
        draw = ImageDraw.Draw(poster)
        draw.text((46, 30), "ORBITAL ASSEMBLY / COMPLIANT DOCKING", font=font, fill="#e5edf7")
        draw.text((46, 67), "ARM > MODULE 1 : PORT 1   /   MODULE 1 : PORT 4 > MODULE 2 : PORT 1", font=small, fill="#8eacc7")
        poster.paste(Image.fromarray(wide).resize((1508, 848)), (46, 112))
        poster.paste(Image.fromarray(close).resize((540, 304)), (1014, 618))
        draw.rectangle((1013, 617, 1555, 923), outline="#526b86", width=2)
        draw.text((1034, 633), "SIDE PORTS / M1:P4 > M2:P1", font=small, fill="#c0d3ec")
        draw.text((46, 984), "Fixed-base contact experiment  /  orbital setting for visualization", font=small, fill="#8eacc7")
        poster.save(out/"preview.png")
        # Isolate the fixed module to make its six-side-port layout inspectable.
        groups = model.geom_group.copy()
        for g in range(model.ngeom):
            body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g]) or ""
            if body.startswith("target_") and groups[g] == 4:
                model.geom_group[g] = 5
        module_options = mujoco.MjvOption()
        module_options.geomgroup[:] = False
        module_options.geomgroup[5] = True
        module_options.sitegroup[:] = False
        module_camera = camera(data.body("target_module").xpos.copy(), .53, 155, -30)
        renderer.update_scene(data, camera=module_camera, scene_option=module_options)
        imageio.imwrite(out/"hexagonal_module.png", renderer.render().copy())
        model.geom_group[:] = groups
        if video:
            with imageio.get_writer(out/"orbital_docking.mp4", fps=24, codec="libx264", quality=8,
                                    macro_block_size=1) as writer:
                for t in np.arange(times[0], times[-1], 1/24):
                    i = min(int(np.searchsorted(times, t)), len(q)-1)
                    wide = frame(i, overview)
                    close = frame(i, detail)
                    composed = Image.fromarray(wide)
                    composed.paste(Image.fromarray(close).resize((400, 225)), (855, 465))
                    draw = ImageDraw.Draw(composed)
                    draw.rectangle((0, 0, 1280, 79), fill="#080f20")
                    draw.text((24, 12), "ORBITAL ASSEMBLY / COMPLIANT DOCKING", font=font, fill="#e5edf7")
                    draw.text((24, 49), "ARM > M1:P1   /   M1:P4 > M2:P1   /   SIX SIDE PORTS", font=small, fill="#92b3d5")
                    draw.text((1090, 23), f"t = {t:05.2f} s", font=small, fill="#92b3d5")
                    writer.append_data(np.asarray(composed))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preview-only", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--replay", action="store_true", help="Render the saved rollout without rerunning control")
    parser.add_argument("--sky", choices=["blue", "black"], default="blue")
    args = parser.parse_args()
    build(args.sky)
    build_side_assets()
    out = HERE/"outputs"
    out.mkdir(exist_ok=True)
    scene = make_scene()
    verification, initial_q = validate(scene)
    (out/"physics_check.json").write_text(json.dumps(verification, indent=2)+"\n")
    geometry_manifest(scene, out)
    print("Side-docking consistency:", verification, flush=True)
    if args.preview_only:
        render(scene, out, [initial_q], np.array([0.]), False)
    else:
        if args.replay:
            with np.load(out/"rollout.npz") as saved:
                if "scenario" not in saved or str(saved["scenario"]) != "side_ports_v1":
                    raise ValueError("Saved rollout predates side-port docking; run without --replay first")
                q, times = saved["q"], saved["t"]
        else:
            sources = (list((ROOT/"src").rglob("*.py"))
                       + list(HERE.glob("*.py")) + [ROOT/"experiments/run_docking.py"])
            manifest = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
            with (out/"simulation.log").open("w") as stream, redirect_stdout(stream):
                log = simulate(scene)
            q, times = np.asarray(log.joint_angles), np.asarray(log.t_list)
            np.savez_compressed(out/"rollout.npz", q=q, t=times,
                                scenario="side_ports_v1", force=np.asarray(log.force_externals),
                                position=np.asarray(log.pos_actual),
                                qd=np.asarray(log.joint_velocities))
            (out/"docking_samples.json").write_text(json.dumps(log.docking_samples)+"\n")
            (out/"insertion_gate.json").write_text(json.dumps(log.docking_gate, indent=2)+"\n")
            (out/"source_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
            print("Insertion:", log.docking_gate, flush=True)
        render(scene, out, q, times, args.video)
    print(out/"preview.png", flush=True)


if __name__ == "__main__":
    main()
