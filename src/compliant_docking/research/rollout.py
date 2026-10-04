"""Single-source persisted trial telemetry and recorded-state previews."""
from __future__ import annotations

import json
import os
from dataclasses import asdict
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pinocchio as pin
from matplotlib.colors import to_rgba

from compliant_docking.docking_task import estimated_target_pose, target_rotation
from compliant_docking.plotting import COLORS, apply_style
from compliant_docking.rendering import CAMERAS, apply_render_theme, scene_options
from compliant_docking.simulation.mujoco_env import MujRobot

CASES = {"nominal": (0.0, 0.0, 0.0), "xy": (0.002, -0.002, 0.0),
         "combined": (0.002, -0.002, 5.0)}

def _json_default(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(type(value).__name__)

def save_rollout(out, name, scene, log, telemetry="full"):
    """Persist one rollout. telemetry="core" omits per-step diagnostic_* channels
    (and the raw contacts.npz events) from the file while still returning the full
    array dict, so in-memory gate assessment keeps working; intended for physical
    timestep refinement reruns whose conclusions live in the JSON gates/peaks."""
    if telemetry not in ("full", "core"):
        raise ValueError(f"unknown telemetry mode: {telemetry}")
    samples = log.docking_samples
    arrays = {key: np.asarray([s[key] for s in samples]) for key in samples[0]}
    arrays.update(q=np.asarray(log.joint_angles), qd=np.asarray(log.joint_velocities),
                  desired_position=np.asarray(log.pos_desired), position=np.asarray(log.pos_actual),
                  torque=np.asarray(log.tau_hist), torque_saturated=np.asarray(log.torque_saturated))
    metadata = dict(scene=asdict(scene), gate=log.docking_gate,
                    waypoint_names=log.docking_trajectory.names,
                    waypoint_times=log.docking_trajectory.times.tolist(),
                    waypoints=[pose.homogeneous.tolist() for pose in log.docking_trajectory.poses])
    if log.se3_diagnostics:
        arrays.update({"control_"+key: np.asarray([sample[key] for sample in log.se3_diagnostics])
                       for key in log.se3_diagnostics[0]})
    if hasattr(log, "feedback_convention"):
        metadata["feedback_convention"] = log.feedback_convention
    if log.contact_diagnostics:
        arrays.update({"diagnostic_"+key: np.asarray([sample[key] for sample in log.contact_diagnostics])
                       for key in log.contact_diagnostics[0]})
        metadata["contact_diagnostics"] = log.contact_summary
        metadata["geometry_evaluation"] = log.geometry_evaluation
        if telemetry == "full":
            # Re-save with synchronized poses, solve-time F/T, inertia and contact sum.
            events = {key: np.asarray([row[key] for row in log.contact_events])
                      for key in log.contact_events[0]} if log.contact_events else {"t": np.array([])}
            np.savez_compressed(out / f"{name}.contacts.npz", **events)
    metadata["telemetry"] = telemetry
    written = arrays if telemetry == "full" else {
        key: value for key, value in arrays.items() if not key.startswith("diagnostic_")}
    np.savez_compressed(out / f"{name}.npz", **written)
    (out / f"{name}.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False, default=_json_default)+"\n")
    return arrays

def preview_rollout(out, scene, log):
    """Three phases from the actual recorded joint states, with waypoint frames."""
    apply_style("report", cjk_first=True)
    robot = MujRobot(scene.build_mjmodel(), render=False, record=False, eef_body=scene.eef_body)
    apply_render_theme(robot.model, tool_prefix=scene.tool.prefix, target_prefix=scene.target.prefix)
    trajectory = log.docking_trajectory
    path = np.asarray([trajectory.get_state(t)[0] for t in np.linspace(0, trajectory.total_duration, 100)])
    frames = dict(zip(trajectory.names, trajectory.poses, strict=True))
    frames["truth"] = pin.SE3(target_rotation(scene.target), scene.target.pos)
    frames["estimate"] = estimated_target_pose(scene.docking)
    robot.coordinate_frame_labels = False
    times = np.asarray([s["t"] for s in log.docking_samples])
    above_t = trajectory.times[trajectory.names.index("approach")]
    indices = [0, int(np.argmin(abs(times-above_t))), len(times)-1]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.1))
    fig.subplots_adjust(left=.01, right=.99, bottom=.16, top=.87, wspace=.035)
    labels = ["(a) 侧方起点 · 全景", "(b) 估计目标上方", "(c) 插入后的保持状态"]
    with mujoco.Renderer(robot.model, height=600, width=800) as renderer:
        for panel, (axis, index, label, camera_name) in enumerate(
                zip(axes, indices, labels, ["overview", "approach", "contact"], strict=True)):
            robot.data.qpos[:] = log.joint_angles[index]
            mujoco.mj_forward(robot.model, robot.data)
            body = robot.data.body(scene.eef_body)
            actual = pin.SE3(body.xmat.reshape(3, 3), body.xpos)
            robot.set_coordinate_frames(frames if panel == 0 else
                                        {"actual": actual, "truth": frames["truth"]})
            camera = CAMERAS[camera_name].camera()
            # Presets follow target translation; all panels retain one azimuth.
            camera.lookat[:] += scene.target.pos - np.array([0, .5, .35])
            option = scene_options(robot.model, interface_only=panel > 0,
                                   prefixes=(scene.tool.prefix, scene.target.prefix))
            renderer.update_scene(robot.data, camera=camera, scene_option=option)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            robot._add_coordinate_frames(renderer.scene)
            if panel == 0:
                for point in path:
                    robot._append_marker(renderer.scene, point, to_rgba(COLORS["planned"]), .002)
            axis.imshow(renderer.render())
            axis.set_title(f"{label}\nt = {times[index]:.1f} s", fontsize=12, linespacing=1.5)
            axis.axis("off")
    fig.text(.015, .075, "青色：规划路径    坐标轴：X 红 / Y 绿 / Z 蓝", fontsize=10)
    fig.text(.505, .09, "局部图：金色公头 / 蓝色母头（隐藏机械臂）", fontsize=10)
    fig.text(.505, .045, "坐标系：实际末端与真实目标", fontsize=10)
    for suffix in ("png", "pdf"):
        fig.savefig(out / f"scene_preview.{suffix}", dpi=200)
    plt.close(fig)
