"""Shared presentation cameras and visual-only research rendering theme.

These settings never change collision geometry, dynamics or recorded states.
"""
from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass(frozen=True)
class CameraPreset:
    lookat: tuple[float, float, float]
    distance: float
    azimuth: float
    elevation: float

    def camera(self):
        camera = mujoco.MjvCamera()
        camera.lookat[:] = self.lookat
        camera.distance = self.distance
        camera.azimuth = self.azimuth
        camera.elevation = self.elevation
        return camera


CAMERAS = {
    "overview": CameraPreset((0.0, 0.24, 0.40), 1.40, 135.0, -18.0),
    "approach": CameraPreset((0.0, 0.50, 0.435), 0.40, 135.0, -18.0),
    "contact": CameraPreset((0.0, 0.50, 0.385), 0.23, 135.0, -5.0),
}


def apply_render_theme(model, *, tool_prefix="tool_", target_prefix="target_"):
    """Light neutral sky, quiet floor, soft lighting; mutate visual fields only."""
    buffer = model.tex_data if hasattr(model, "tex_data") else model.tex_rgb
    for index in range(model.ntex):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TEXTURE, index)
        channels = int(model.tex_nchannel[index]) if hasattr(model, "tex_nchannel") else 3
        height, width = int(model.tex_height[index]), int(model.tex_width[index])
        offset = int(model.tex_adr[index])
        pixels = buffer[offset:offset + height*width*channels].reshape(height, width, channels)
        if int(model.tex_type[index]) == int(mujoco.mjtTexture.mjTEXTURE_SKYBOX):
            pixels[:, :, :3] = [239, 242, 246]
        elif name == "groundplane":
            brightness = pixels[:, :, :3].mean(axis=2)
            light = brightness > np.median(brightness)
            pixels[:, :, :3] = np.where(light[:, :, None], [225, 229, 234], [217, 222, 228])
    model.vis.headlight.ambient[:] = [.35, .35, .35]
    model.vis.headlight.diffuse[:] = [.55, .55, .55]
    model.vis.headlight.specular[:] = [.12, .12, .12]
    model.vis.rgba.haze[:] = [.94, .95, .97, 1.0]
    for i in range(model.nmat):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL, i)
        if name == "groundplane":
            model.mat_reflectance[i] = 0.0
        model.mat_specular[i] = min(float(model.mat_specular[i]), 0.22)
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if name.endswith("_visual"):
            if name.startswith(tool_prefix):
                model.geom_rgba[i] = [.88, .67, .37, 1.0]
            elif name.startswith(target_prefix):
                model.geom_rgba[i] = [.42, .59, .72, 1.0]


def scene_options(model, *, interface_only=False, prefixes=("tool_", "target_")):
    """Hide collision proxies and marker sites; show crown meshes in closeups."""
    option = mujoco.MjvOption()
    option.geomgroup[3] = False
    option.sitegroup[:] = False
    if interface_only:
        option.geomgroup[:] = False
        option.geomgroup[5] = True
        # The supplied model is a rendering copy, never the running simulation.
        for geom_id in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
            if name.startswith(prefixes) and name.endswith("_visual"):
                model.geom_group[geom_id] = 5
    return option
