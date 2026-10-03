"""Render saved default-HexFrame dynamics without rerunning the simulation."""
from __future__ import annotations

import argparse
import json

import assembly_sequence as seq
import crown_assembly as engine
import mujoco
import numpy as np
import vertical_assembly as vertical
from hexframe_integration import configure
from PIL import Image, ImageDraw, ImageFont


def render_connection(model, records, *, sites, title, footer, filename, distance=.34, time=0.):
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
        renderer.update_scene(data, camera=seq.camera(focus, distance, 125, -16), scene_option=options)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        result = Image.fromarray(renderer.render().copy())
    font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    draw = ImageDraw.Draw(result)
    draw.rectangle((0, 0, 900, 62), fill="#0a1424")
    draw.text((20, 14), title, font=ImageFont.truetype(font_path, 24), fill="#e8f0fa")
    draw.rectangle((0, 566, 900, 600), fill="#0a1424")
    draw.text((20, 570), footer,
              font=ImageFont.truetype(font_path, 17), fill="#a9bdd6")
    result.save(engine.OUT/filename)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    with configure(engine) as hooks:
        model = mujoco.MjModel.from_xml_path(str(engine.OUT/"model.xml"))
        anchor = json.loads((engine.OUT/"accepted_anchor.json").read_text())
        model.site("assembly_anchor").pos[:] = anchor["pos"]
        model.site("assembly_anchor").quat[:] = anchor["quat"]
        with np.load(engine.OUT/"rollout.npz") as saved:
            records = {k: saved[k] for k in saved.files}
        seq.render(model, records, hooks.phases(), args.video,
                   work_camera=seq.camera([0, .30, .28], 2.8, 85, -23),
                   wide_camera=seq.camera([0, .25, -.02], 6.5, 75, -24),
                   detail_azimuth=90, detail_elevation=-12,
                   base_prelocked=True,
                   footer="默认 HexFrame · 六侧面接口 · 导向与止挡接触 · 理想锁定后卸力释放")
        vertical.preview_card()
        render_connection(model, records, sites=["base_dock_mating"],
                          title="基座标准接口 ↔ 模块 2 侧面接口 4",
                          footer="初始已锁定 · PetalDock100 V2 · 基座捕获与锁紧机构未模拟",
                          filename="base_connection.png")
        for time, filename, title in [(0., "storage_connection.png", "标准存储接口锁定模块 1 · 备用标准接口空闲"),
                                      (15., "storage_released.png", "机械臂已提起模块 1 · 存储接口解锁并脱离")]:
            render_connection(model, records, sites=["storage_dock_mating", "spare_dock_mating"],
                              title=title, footer="PetalDock100 V2 · 接口 4 朝下 · 理想锁定与受保护的解锁交接",
                              filename=filename, distance=.78, time=time)
        print(engine.OUT/"preview.png")


if __name__ == "__main__":
    main()
