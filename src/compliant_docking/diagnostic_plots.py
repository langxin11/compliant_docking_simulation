"""Shared-style figures for the insertion suite's passive diagnostic channel."""
from __future__ import annotations

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pinocchio as pin

from .collision_geometry import CONVEX_DIRECTORY
from .docking_task import target_rotation
from .plotting import COLORS, apply_style
from .rendering import CAMERAS, apply_render_theme, scene_options
from .simulation.mujoco_env import MujRobot

CASE_NAMES = {"nominal": "零新增误差", "xy": "XY 偏差", "combined": "XY + yaw 偏差"}
PROFILE_NAMES = {"stiff": "高绕轴刚度", "compliant": "低绕轴刚度"}


def _save(fig, out, name):
    for suffix in ("png", "pdf"):
        fig.savefig(out / f"{name}.{suffix}", dpi=180)
    plt.close(fig)


def _axial(record, key, moment=True):
    R = target_rotation(record["scene"].target)
    values = record["data"]["diagnostic_"+key]
    return values[:, 3:] @ R[:, 2] if moment else values[:, :3] @ R[:, 2]


def _project(scene, point, width, height):
    # MuJoCo's monocular renderer averages its two OpenGL cameras. Project the
    # recorded vectors into that same camera; 2D overlays intentionally ignore
    # occlusion so buried contacts remain visible. No force clipping is applied.
    left, right_camera = scene.camera
    position = (left.pos+right_camera.pos)/2
    forward, up = (left.forward+right_camera.forward)/2, (left.up+right_camera.up)/2
    right = np.cross(forward, up)
    delta = np.asarray(point)-position
    depth = delta @ forward
    near = (left.frustum_near+right_camera.frustum_near)/2
    bottom, top = left.frustum_bottom, left.frustum_top
    frustum_width = left.frustum_width or (top-bottom)*width/height
    u = (delta @ right)*near/depth
    v = (delta @ up)*near/depth
    return np.array([width*((u-left.frustum_center)/frustum_width+.5),
                     height*(1-(v-bottom)/(top-bottom))])


def plot_diagnostic_suite(out, records, calibration):
    apply_style("report", cjk_first=True)
    baseline = [r for r in records if r["numerics"] == "baseline"]
    fig, ax = plt.subplots(figsize=(9, 3.8), constrained_layout=True)
    ax.plot(calibration["coarse_yaw_deg"], 1000*np.asarray(calibration["coarse_height_m"]),
            color=COLORS["planned"], label="采样网格无重叠高度")
    for candidate in calibration["candidates"]:
        x, y = candidate["yaw_deg"], 1000*candidate["root_height_m"]
        ax.scatter(x, y, color=COLORS["actual"], zorder=3)
        ax.annotate(f"{x:.2f}° / {y:.2f} mm", (x, y), xytext=(7, 12),
                    textcoords="offset points", fontsize=9)
    ax.set(xlabel="公头根相对母头根的 yaw [°]", ylabel="根坐标系间高度 [mm]",
           title="冠形接口的同轴紧凑参考（包含原有 40° 安装角）", xlim=(-5, 360))
    ax.grid(alpha=.7)
    ax.legend(loc="upper right", frameon=False)
    _save(fig, out, "geometry_reference")

    fig, axes = plt.subplots(len(baseline), 2, squeeze=False, figsize=(12, 2.8*len(baseline)),
                             constrained_layout=True)
    for row, record in enumerate(baseline):
        t = record["data"]["diagnostic_t"]
        title = CASE_NAMES[record["case"]]+" · "+PROFILE_NAMES[record["profile"]]
        for column, moment in enumerate([False, True]):
            axis = axes[row, column]
            for key, label, color, style in [
                ("sensor_world", "F/T 传感器", COLORS["text"], "-"),
                ("interface_world", "接口接触合力", COLORS["planned"], "-"),
                ("inertial_world", "有效惯性载荷", COLORS["actual"], "--")]:
                axis.plot(t, _axial(record, key, moment), label=label, color=color, linestyle=style,
                          linewidth=.9)
            axis.set(xlabel="求解时间 [s]", ylabel="轴矩 [N·m]" if moment else "轴向力 [N]", title=title)
            axis.grid(alpha=.7)
            axis.legend(loc="upper left", frameon=False, ncols=3, fontsize=8)
    _save(fig, out, "load_diagnostics")

    numerical = [r for r in records if r["numerics"] != "baseline"]
    if numerical:
        groups = list(dict.fromkeys((r["case"], r["profile"]) for r in numerical))
        fig, axes = plt.subplots(len(groups), 2, squeeze=False, figsize=(13, 3.1*len(groups)),
                                 constrained_layout=True)
        settings = {"baseline": COLORS["stiff"], "dt_half": "#AA4499",
                    "sdf_refined": COLORS["compliant"]}
        for (case, profile), pair in zip(groups, axes, strict=True):
            for record in records:
                if (record["case"], record["profile"]) != (case, profile):
                    continue
                color = settings[record["numerics"]]
                physics = record["scene"].physics
                backend = ("凸碰撞" if record["scene"].tool.mjcf.parent == CONVEX_DIRECTORY
                           else f"SDF {physics.sdf_iterations}/{physics.sdf_initpoints}")
                label = f"{record['numerics']} · {1000*physics.timestep:g} ms · {backend}"
                for axis, key in zip(pair, ["sensor_world", "interface_world"], strict=True):
                    axis.plot(record["data"]["diagnostic_t"], _axial(record, key),
                              color=color, label=label, linewidth=.9)
            limit = next(r["scene"].docking.max_axial_moment for r in records if r["case"] == case)
            for axis, signal in zip(pair, ["F/T 轴矩", "真实接口轴矩"], strict=True):
                axis.axhline(limit, color="#888888", linewidth=.7, linestyle="--")
                axis.axhline(-limit, color="#888888", linewidth=.7, linestyle="--")
                axis.set(xlabel="求解时间 [s]", ylabel=signal+" [N·m]",
                         title=CASE_NAMES[case]+" · "+PROFILE_NAMES[profile])
                axis.grid(alpha=.7)
                axis.legend(loc="upper left", frameon=False, fontsize=8)
        _save(fig, out, "sensitivity")
    _render_peaks(out, baseline)


def _render_peaks(out, records):
    columns = min(3, len(records))
    rows = int(np.ceil(len(records)/columns))
    fig, axes = plt.subplots(rows, columns, squeeze=False, figsize=(15, 4.1*rows))
    fig.subplots_adjust(left=.01, right=.99, bottom=.08, top=.91, wspace=.025, hspace=.25)
    max_force = max((np.linalg.norm(c["force_world"]) for r in records
                     for c in r["diagnostics"]["peak_contacts"]), default=1.)
    force_scale = .045/max(max_force, 1.)
    for axis, record in zip(axes.flat, records, strict=False):
        scene, data, summary = record["scene"], record["data"], record["diagnostics"]
        index = int(np.argmax(np.abs(_axial(record, "sensor_world"))))
        robot = MujRobot(scene.build_mjmodel(), render=False, record=False, eef_body=scene.eef_body)
        apply_render_theme(robot.model, tool_prefix=scene.tool.prefix, target_prefix=scene.target.prefix)
        for geom_id in range(robot.model.ngeom):
            name = mujoco.mj_id2name(robot.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
            if name.startswith(scene.tool.prefix) and name.endswith("_visual"):
                robot.model.geom_rgba[geom_id, 3] = .55
        robot.data.qpos[:] = data["diagnostic_q"][index]
        robot.data.qvel[:] = data["diagnostic_v"][index]
        mujoco.mj_forward(robot.model, robot.data)
        robot.coordinate_frame_labels = False
        robot.set_coordinate_frames({"actual": pin.SE3(data["diagnostic_rotation"][index], data["diagnostic_position"][index])})
        camera = CAMERAS["contact"].camera()
        camera.distance = .30
        camera.lookat[:] = (scene.target.pos+data["diagnostic_position"][index])/2
        option = scene_options(robot.model, interface_only=True,
                               prefixes=(scene.tool.prefix, scene.target.prefix))
        with mujoco.Renderer(robot.model, height=480, width=640) as renderer:
            renderer.update_scene(robot.data, camera=camera, scene_option=option)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            robot._add_coordinate_frames(renderer.scene)
            projected = []
            for contact in summary["peak_contacts"]:
                p, force = np.asarray(contact["position_world"]), np.asarray(contact["force_world"])
                projected.append((_project(renderer.scene, p, 640, 480),
                                  _project(renderer.scene, p+force_scale*force, 640, 480),
                                  np.linalg.norm(force)))
            axis.imshow(renderer.render())
            for start, end, magnitude in projected:
                axis.scatter(*start, color="#CC3344", s=12, zorder=5)
                if magnitude > .05:
                    axis.annotate("", xy=end, xytext=start, arrowprops=dict(arrowstyle="-|>",
                                  color=COLORS["planned"], lw=1.2, shrinkA=0, shrinkB=0), zorder=6)
        title = CASE_NAMES[record["case"]]+" · "+PROFILE_NAMES[record["profile"]]
        axis.set_title(f"{title}\nt = {summary['peak_sensor_time_s']:.3f} s · F/T 峰矩 {summary['peak_sensor_axial_moment_Nm']:.3f} N·m", fontsize=11)
        axis.axis("off")
    for axis in list(axes.flat)[len(records):]:
        axis.axis("off")
    fig.text(.015, .025, f"金色公头半透明 / 蓝色母头；红点/青色力箭头投影显示，忽略遮挡（统一比例，1 N = {1000*force_scale:.3f} mm）；隐藏机械臂", fontsize=10)
    _save(fig, out, "contact_peaks")
