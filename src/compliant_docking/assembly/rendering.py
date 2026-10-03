"""Render saved MuJoCo state; captions stay outside the docking interfaces."""
from __future__ import annotations

import os
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def caption_font(size):
    font_path = os.environ.get("HEXFRAME_FONT", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if not Path(font_path).is_file():
        raise FileNotFoundError("HexFrame captions require Noto Sans CJK or a CJK font file supplied through HEXFRAME_FONT")
    return ImageFont.truetype(font_path, size)


def camera(lookat, distance, azimuth=135, elevation=-27):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    return cam

def render(model, records, phase_list, video, *, output, work_camera=None, wide_camera=None,
           detail_azimuth=175, detail_elevation=-18,
           base_prelocked=False,
           footer="刚体动力学流程演示 · 理想锁定约束 · 花冠微接触未模拟"):
    data = mujoco.MjData(model)
    model.vis.map.znear = .003
    options = mujoco.MjvOption()
    options.geomgroup[3] = False
    options.geomgroup[4] = True
    options.sitegroup[:] = False
    work = work_camera if work_camera is not None else camera([-.10, .67, .42], 2.15, 155, -25)
    wide = wide_camera if wide_camera is not None else camera([0, .25, -.22], 8.8, 125, -30)
    font = caption_font(23)
    small = caption_font(17)
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        def frame(index, cam=work):
            data.qpos[:] = records["qpos"][index]
            data.qvel[:] = records["qvel"][index]
            data.eq_active[:] = records["locks"][index]
            for name, locked in [("gripper_led", data.eq_active[1]), ("storage_led", data.eq_active[0]),
                                 ("seed_station_led", base_prelocked or data.eq_active[2])]:
                g = model.geom(name)
                g.matid[:] = -1
                g.rgba[:] = [.13, .85, .60, 1] if locked else [.96, .57, .13, 1]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=cam, scene_option=options)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            return Image.fromarray(renderer.render().copy())

        def annotated(index):
            result = frame(index)
            draw = ImageDraw.Draw(result)
            k = int(records["phase"][index])
            # Module identities live in the header, outside all docking faces.
            # A side view makes the end-effector and two mating faces visible.
            focus = (data.body("module1").xpos+data.body("gripper").xpos)/2 if k <= 5 else (
                data.body("module1").xpos+data.body("module2").xpos)/2
            detail = frame(index, camera(focus.copy(), .75, detail_azimuth, detail_elevation)).resize((384, 216))
            result.paste(detail, (875, 458))
            draw = ImageDraw.Draw(result)
            draw.rectangle((874, 457, 1259, 674), outline="#8bb6ce", width=2)
            draw.rectangle((874, 429, 1259, 456), fill="#11283e")
            draw.text((888, 431), "末端接口 → 模块 1 接口 1" if k <= 5 else "模块 1 接口 4 → 模块 2 接口 1",
                      font=small, fill="#edf5ff", stroke_width=1, stroke_fill="#11283e")
            draw.rectangle((0, 0, 1280, 114), fill="#0a1424")
            draw.text((24, 11), phase_list[k].title, font=font, fill="#e8f0fa")
            locks = records["locks"][index]
            state = "   |   ".join(f"{label}：{'锁定' if lock else '解除'}" for label, lock in
                                 zip(["存储接口" if base_prelocked else "存放架", "机械臂", "模块间"], locks, strict=True))
            draw.text((24, 51), state, font=small, fill="#8ce1cb")
            module2_note = "接口 4 已与基座接口锁定" if base_prelocked else "左侧固定的基准模块"
            draw.text((24, 81), f"模块 1：从右侧存储位搬运的模块    |    模块 2：{module2_note}",
                      font=small, fill="#c6e9ff")
            draw.text((1120, 56), f"{records['t'][index]:05.2f} s", font=small, fill="#a9bdd6")
            draw.rectangle((0, 688, 1280, 720), fill="#0a1424")
            draw.text((24, 691), footer, font=small, fill="#a9bdd6")
            return result

        # Six key views emphasize assembly logic instead of a distant platform.
        chosen = [0, 3, 5, 8, 10, 12]
        indices = [int(np.flatnonzero(records["phase"] == k)[-1]) for k in chosen]
        storyboard = Image.new("RGB", (1600, 1480), "#0a1424")
        draw = ImageDraw.Draw(storyboard)
        draw.text((28, 17), "空间模块组装：存放 → 抓取 → 转运 → 安装 → 撤离", font=font, fill="#e8f0fa")
        for tile, index in enumerate(indices):
            im = annotated(index).resize((768, 432))
            storyboard.paste(im, (24+(tile % 2)*784, 70+(tile//2)*456))
        storyboard.save(output/"storyboard.png")
        annotated(0).save(output/"initial_workcell.png")
        annotated(len(records["t"])-1).save(output/"installed.png")
        frame(0, wide).save(output/"platform.png")
        if video:
            with imageio.get_writer(output/"assembly_sequence.mp4", fps=24, codec="libx264",
                                    quality=8, macro_block_size=1) as writer:
                for t in np.arange(0., records["t"][-1], 1/24):
                    index = min(int(np.searchsorted(records["t"], t)), len(records["t"])-1)
                    writer.append_data(np.asarray(annotated(index)))

def preview_card(out):
    font = caption_font(26)
    small = caption_font(20)
    image = Image.new("RGB", (1600, 1100), "#0a1424")
    draw = ImageDraw.Draw(image)
    draw.text((36, 20), "竖直组装 / 紧凑厚基座 / 对称细长太阳翼", font=font, fill="#edf5ff")
    draw.text((36, 61), "左侧组装 · 中央机械臂 · 右侧存储    |    基座 2.6 × 1.8 × 0.5 m", font=small, fill="#a9bdd6")
    # A panoramic crop retains both complete wings while removing empty sky.
    image.paste(Image.open(out/"platform.png").crop((0, 160, 1280, 600)).resize((1536, 528)), (32, 111))
    image.paste(Image.open(out/"initial_workcell.png").resize((752, 423)), (32, 661))
    image.paste(Image.open(out/"installed.png").resize((752, 423)), (816, 661))
    image.save(out/"preview.png")

def render_connection(model, records, *, output, sites, title, footer, filename, distance=.34, time=0.):
    """Show sockets without labels on their geometry."""
    data = mujoco.MjData(model)
    index = int(np.argmin(abs(records["t"]-time)))
    data.qpos[:] = records["qpos"][index]
    data.qvel[:] = records["qvel"][index]
    data.eq_active[:] = records["locks"][index]
    model.geom("storage_led").matid[:] = -1
    model.geom("storage_led").rgba[:] = [.13, .85, .60, 1] if data.eq_active[0] else [.96, .57, .13, 1]
    mujoco.mj_forward(model, data)
    options = mujoco.MjvOption()
    options.geomgroup[3] = False
    options.geomgroup[4] = True
    options.sitegroup[:] = False
    focus = np.mean([data.site(site).xpos for site in sites], axis=0)
    with mujoco.Renderer(model, height=600, width=900) as renderer:
        renderer.update_scene(data, camera=camera(focus, distance, 125, -16), scene_option=options)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        result = Image.fromarray(renderer.render().copy())
    draw = ImageDraw.Draw(result)
    draw.rectangle((0, 0, 900, 62), fill="#0a1424")
    draw.text((20, 14), title, font=caption_font(24), fill="#e8f0fa")
    draw.rectangle((0, 566, 900, 600), fill="#0a1424")
    draw.text((20, 570), footer,
              font=caption_font(17), fill="#a9bdd6")
    result.save(output/filename)
