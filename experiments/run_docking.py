"""
run_docking.py - Robotic Task Space Control Simulation (docking experiment)

This script implements a simulation environment for task space control of a KUKA iiwa14 robot
using MuJoCo for physics simulation and Pinocchio for dynamics calculations. It demonstrates
operational space control with impedance for smooth interaction with the environment.

The simulation includes:
- Force/torque sensor integration
- Task space trajectory tracking
- External force compensation
- Visualization of robot motion and performance metrics

Author: langxin11
Date: 2025
"""
import os
from dataclasses import replace
from pathlib import Path

# 在无显示环境（如服务器/CI）下，指定 MuJoCo 使用 EGL 离屏渲染后端 /
# In headless environments (e.g., server/CI), set MuJoCo to use EGL offscreen backend
# 若本机有显示并安装了驱动，也可改为 'glfw'；某些环境需使用 'osmesa' /
# If you have a display and drivers, 'glfw' may work; some setups need 'osmesa'
if "DISPLAY" not in os.environ or not os.environ["DISPLAY"]:
    os.environ["MUJOCO_GL"] = "egl"  # 试试 osmesa，也可以改成 egl


import mujoco
import numpy as np
import pinocchio as pin

from compliant_docking.config import DockingConfig, HQPConfig, ImpedanceConfig, SE3ImpedanceConfig
from compliant_docking.contact_diagnostics import ContactDiagnostics, summarize_diagnostics
from compliant_docking.control.contact_yaw import ContactYawSchedule, control_stride
from compliant_docking.control.hqp_ac import HQPAdaptiveController
from compliant_docking.control.se3_impedance import SE3LieImpedanceController
from compliant_docking.control.task_space import TaskSpaceController
from compliant_docking.docking_task import (
    build_docking_trajectory,
    docking_sample,
    estimated_target_pose,
    evaluate_docking,
    target_rotation,
)
from compliant_docking.metrics import (
    TrackingThresholds,
    compute_metrics,
    compute_tracking_metrics,
    evaluate_tracking_gate,
    format_metrics,
    format_tracking_gate,
)
from compliant_docking.models import load_assembled_pin_model, load_pin_model
from compliant_docking.planning.kinematics import compute_ik
from compliant_docking.planning.motion_reference import get_motion_reference
from compliant_docking.planning.trajectory import (
    CircleFigure8Trajectory,
    DecoupledQuinticTrajectory,
    TwoPhaseDockingTrajectory,
)
from compliant_docking.planning.waypoints import WaypointPoseTrajectory
from compliant_docking.scene import DEFAULT_SCENE_PATH, Scene, load_scene
from compliant_docking.simulation.mujoco_env import MujRobot
from compliant_docking.telemetry import Log
from compliant_docking.wrench import WrenchSample


def run_simulation(muj_robot:MujRobot,
                   task_dynamics:TaskSpaceController | HQPAdaptiveController | SE3LieImpedanceController,
                   trajector_planner: DecoupledQuinticTrajectory
                   | TwoPhaseDockingTrajectory
                   | CircleFigure8Trajectory | WaypointPoseTrajectory,
                   log:Log,
                   cfg:DockingConfig,
                   q_init:np.ndarray,
                   scene:Scene | None = None,
                   r_des:np.ndarray | None = None,
                   plot:bool = True, diagnostics:bool | str = False):
    """
    执行主仿真循环：读取轨迹 → 计算任务空间阻抗控制力矩 → MuJoCo 步进 → 记录/绘图 /
    Run the main simulation loop: sample trajectory → compute task-space impedance torque → MuJoCo step → log/plot

    参数说明 / Args:
    - muj_robot: MuJoCo 封装（写入力矩、推进仿真、可视化/录帧） / MuJoCo wrapper (apply torque, step sim, render/record)
    - task_dynamics: 任务空间控制器（Pinocchio 提供雅可比/动力学项） / Task-space controller (uses Pinocchio for Jacobians/dynamics)
    - trajector_planner: 轨迹规划器（解耦 5 次多项式，输出 pos/vel/acc） / Trajectory planner (decoupled quintic; pos/vel/acc)
    - log: 日志记录与绘图类 / Logger for data capture and plotting
    - cfg: 任务/仿真参数（时长、步长、力矩限幅等） / Task & simulation config (duration, dt, torque clamp, ...)
    - q_init: 初始关节位置（由 IK 求得） / Initial joint configuration (from IK)
    - scene: 场景配置（仅用于视频文件名取 scene.name；未传时回落为历史名 docking_update） /
      Scene config (only used for the video filename via scene.name; falls back to "docking_update" if omitted)
    - r_des: 期望末端姿态旋转矩阵（世界系，通常为 scene.task.init_ori；可选）。提供时
      每步记录世界系姿态误差向量 log(R_d R^T) 供 metrics 姿态指标使用 /
      Desired EE orientation (world frame, typically scene.task.init_ori; optional).
      When given, the per-step world-frame orientation error log(R_d R^T) is logged
      for the orientation metrics.

    返回 / Returns:
    - log: 记录了完整时序数据的日志对象 / The populated Log object
    """

    log.reset_logs()

    muj_robot.init_simulators(q_init)

    # 可视化用路径与控制采样相互独立：预采样完整规划路径，只进入渲染场景，
    # 不参与物理或控制。规划轨迹为青色，实际轨迹为橙色，当前期望点为绿色。
    path_duration = float(getattr(
        trajector_planner, "total_duration", cfg.traj_duration,
    ))
    path_times = np.linspace(0.0, path_duration, 160)
    planned_path = np.asarray([
        trajector_planner.get_state(sample_t)[0] for sample_t in path_times
    ])
    muj_robot.set_trajectory_visualization(planned_path)
    docking = scene.docking if scene is not None else None
    contact_logger = (ContactDiagnostics(muj_robot.model, scene, store_events=diagnostics != "summary")
                      if docking and diagnostics else None)
    if docking is not None:
        frames = {name: pose for name, pose in zip(trajector_planner.names, trajector_planner.poses, strict=True)}
        frames["target_truth"] = pin.SE3(target_rotation(scene.target), scene.target.pos)
        frames["target_estimate"] = estimated_target_pose(docking)
        muj_robot.set_coordinate_frames(frames)
        tool_geoms = {i for i in range(muj_robot.model.ngeom)
                      if (mujoco.mj_id2name(muj_robot.model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith(scene.tool.prefix)}
        target_geoms = {i for i in range(muj_robot.model.ngeom)
                        if (mujoco.mj_id2name(muj_robot.model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith(scene.target.prefix)}

    q = q_init
    v = np.zeros(task_dynamics.model.nv)

    current_pos, current_vel, current_ori = task_dynamics.get_task_space_state(q, v)

    force_external = np.zeros(3)
    torque_external = np.zeros(3)
    is_tracking = scene is not None and scene.trajectory is not None and scene.trajectory.type == "tracking"
    # PI 动量观测器随步更新（HQP force_source=observer 时非 no-op）/
    # Per-step PI momentum-observer update (no-op unless HQP observer mode)
    update_observer = getattr(task_dynamics, "update_momentum_observer", None)

    # SE(3) Lie 控制器专用通道：期望 body 运动参考 + F/T body wrench。
    # 旧控制器（impedance/hqp）保持原调用方式不受影响；wrench 变换为
    # sensor site 系 → EE body 系（含参考点平移矩），与 body Jacobian 配对。
    use_se3 = isinstance(task_dynamics, SE3LieImpedanceController)
    sensor_site_id = -1
    F_body = np.zeros(6)
    stride = 1
    yaw_schedule = None
    if use_se3:
        if scene is None:
            raise ValueError("se3_lie 控制器需要 scene（提供 sensor_site 名）")
        if r_des is None:
            raise ValueError("se3_lie 控制器需要 r_des（固定期望姿态）")
        sensor_site_id = mujoco.mj_name2id(
            muj_robot.model, mujoco.mjtObj.mjOBJ_SITE, scene.sensor_site)
        if sensor_site_id < 0:
            raise ValueError(f"组装模型缺少 F/T 传感器 site: {scene.sensor_site!r}")
        override = scene.se3_impedance
        stride = control_stride(override.control_period if override else None,
                                float(muj_robot.model.opt.timestep))
        if override is not None and override.contact_yaw is not None:
            if docking is None or not np.allclose(task_dynamics.K, np.diag(np.diag(task_dynamics.K))):
                raise ValueError("contact_yaw requires waypoint docking and diagonal stiffness")
            yaw_schedule = ContactYawSchedule(override.contact_yaw, task_dynamics.K[5, 5],
                                              np.diag(task_dynamics.K)[:2])
        # A sensor is sampled at the FIRST solve of each control interval and
        # consumed at the NEXT update. Its causal delay is one control period,
        # independent of the number of physics substeps. No extra mj_forward
        # or constraint solve is introduced into the dynamics pipeline.
        sensor_sample = WrenchSample.from_site(
            0., np.zeros(3), np.zeros(3),
            muj_robot.data.site_xpos[sensor_site_id],
            muj_robot.data.site_xmat[sensor_site_id])
        log.feedback_convention = dict(
            control_period_s=stride*float(muj_robot.model.opt.timestep),
            delay_s=stride*float(muj_robot.model.opt.timestep),
            sensor_sample="first solve of previous control interval; bootstrap zero at t=0",
            wrench="world at saved sensor origin, transported to current control body pose",
            telemetry="pose, F/T and contacts at pre-integration solve t")

    step_index = 0
    while muj_robot.data.time < cfg.duration:
        # while True:
        t = muj_robot.data.time
        # 1) 根据当前仿真时间采样期望的末端位置/速度/加速度 /
        # 1) Sample desired end-effector pos/vel/acc at current sim time
        pos_des, vel_des, acc_des = trajector_planner.get_state(t)
        muj_robot.set_desired_position(pos_des)
        #print(f"Current time: {t}, Desired position: {pos_des}, Desired position: {pos_des}, Desired velocity: {vel_des}, Desired acceleration: {acc_des}")

        # 2) 任务空间控制（含平动/姿态阻抗与外力补偿）→ 得到关节力矩 tau /
        # 2) Task-space control (translation/rotation impedance + external force) → joint torques tau
        control_update = step_index % stride == 0
        if use_se3 and control_update:
            # SE(3) Lie 阻抗（Kim et al. 2025）：期望 body 运动参考 + body wrench
            T_d, V_d, Vdot_d = get_motion_reference(trajector_planner, t, r_des)
            control_pose = pin.SE3(current_ori, np.asarray(current_pos))
            feedback_sample = sensor_sample
            if feedback_sample.t > t+1e-12:
                raise ValueError("Future sensor sample cannot be used as feedback")
            F_body = np.zeros(6) if is_tracking else feedback_sample.at_body(control_pose)
            if yaw_schedule is not None:
                task_dynamics.K[5, 5] = yaw_schedule.update(
                    t, trajector_planner.phase(t), float(F_body[2]))
                if yaw_schedule.lateral_stiffness is not None:
                    task_dynamics.K[0, 0], task_dynamics.K[1, 1] = yaw_schedule.lateral_stiffness
            control_t = float(t)
            tau_unclipped = task_dynamics.compute_control(
                q, v, T_d, V_d, Vdot_d, F_body)
        elif not use_se3:
            tau_unclipped = task_dynamics.compute_control_task_space_with_orientation_and_imp(
                q, v, pos_des, vel_des, acc_des, current_pos, current_vel, force_external, torque_external)

        # 力矩限幅以防止过大的控制输入导致碰撞检测失败 /
        # Clamp torques to prevent excessive control inputs that cause collision detection failure
        torque_saturated = bool(np.any(np.abs(tau_unclipped) > cfg.max_torque))
        tau = np.clip(tau_unclipped, -cfg.max_torque, cfg.max_torque)

        # 3) 将 tau 写入 MuJoCo，并推进一步物理仿真 /
        # 3) Apply tau to MuJoCo and advance one simulation step
        if contact_logger is not None or use_se3:
            q_solve, v_solve = muj_robot.data.qpos.copy(), muj_robot.data.qvel.copy()
        if use_se3:
            solve_pos, solve_vel, solve_ori = (np.asarray(current_pos).copy(),
                                              np.asarray(current_vel).copy(),
                                              np.asarray(current_ori).copy())
        try:
            q, v, eef_pos = muj_robot.step(tau)
        except Exception as e:
            print(f"\nSimulation error at t={t:.3f}s: {e}")
            print(f"Torques: {tau}")
            print(f"Joint positions: {q}")
            raise RuntimeError(f"仿真在 t={t:.3f}s 异常终止") from e
        if contact_logger is not None:
            sample, events = contact_logger.capture(muj_robot.data, t, q_solve, v_solve)
            log.contact_diagnostics.append(sample)
            log.contact_events.extend(events)
        # 观测器以（步进后状态, 实际施加力矩）推进
        if update_observer is not None:
            update_observer(q, v, tau)
        #print(f"Current time: {t}, End-effector position: {q},tau: {v}",)

        # 4) 使用 Pinocchio 更新当前末端状态（位置/速度/姿态） /
        # 4) Update current end-effector state (pos/vel/orientation) via Pinocchio
        current_pos, current_vel, current_ori = task_dynamics.get_task_space_state(q, v)

        current_pos = np.array(current_pos)
        tau = np.array(tau)

        # 5) 读取力/力矩传感器（根据模型定义：此处取负号与力方向约定相关） /
        # 5) Read force/torque sensors (sign depends on model’s convention)
        force_sensor = -muj_robot.data.sensor("force_sensor").data

        torque_sensor = -muj_robot.data.sensor("torque_sensor").data


        # 6) 将传感器数据通过当前末端旋转矩阵变换到控制参考系下 /
        # 6) Rotate sensor data with current EE rotation to controller’s reference frame
        # 注意：变换方向取决于 `current_ori` 的参考系定义，需与传感器坐标系一致 /
        # Note: direction depends on `current_ori` definition and sensor frame
        measured_force = current_ori @ force_sensor
        measured_torque = current_ori @ torque_sensor
        if use_se3:
            solved_sample = WrenchSample.from_site(
                t, force_sensor, torque_sensor,
                muj_robot.data.site_xpos[sensor_site_id],
                muj_robot.data.site_xmat[sensor_site_id])
            solved_body = solved_sample.at_body(pin.SE3(solve_ori, solve_pos))
            measured_force = solve_ori @ solved_body[:3]
            measured_torque = solve_ori @ solved_body[3:]
            if control_update:
                sensor_sample = solved_sample
        # 自由空间跟踪是柔顺接触前置门禁：保留传感器遥测，但绝不把 F/T
        # 反馈回控制器，避免偶发噪声或虚假接触污染基础跟踪性能。
        if is_tracking:
            force_external = np.zeros(3)
            torque_external = np.zeros(3)
        else:
            force_external = measured_force
            torque_external = measured_torque

        #print('current:',current_pos,"current_ori:",current_ori)

        # print('far:',np.linalg.norm(eef_pos-current_pos))

        #log.store_data(t, pos_des, q, v, tau, np.linalg.norm(current_pos - pos_des))

        # 7) 记录关节/末端/控制量/外力等数据，便于后续绘图分析 /
        # 7) Log joint/EE/control/external data for plotting/analysis
        #    提供期望姿态 r_des 时同步记录世界系姿态误差 log(R_d R^T)（供 metrics 使用）/
        #    When r_des is given, also log the world-frame orientation error log(R_d R^T)
        reference_rotation = (trajector_planner.get_pose(t).rotation
                              if use_se3 and hasattr(trajector_planner, "get_pose") else r_des)
        log_q, log_v = (q_solve, v_solve) if use_se3 else (q, v)
        log_pos, log_vel, log_ori = ((solve_pos, solve_vel, solve_ori) if use_se3
                                     else (current_pos, current_vel, current_ori))
        ori_err_vec = pin.log3(reference_rotation @ log_ori.T) if reference_rotation is not None else None
        log.store_data(
            t, log_q, log_v, log_pos, log_vel,
            np.linalg.norm(log_pos - pos_des),
            pos_des, vel_des, acc_des, tau,
            measured_force, measured_torque,
            orientation_error=ori_err_vec,
            torque_saturated=torque_saturated,
            contact_count=muj_robot.data.ncon)
        if docking is not None:
            # Pose, speed, sensor and contact data all refer to solve t.
            # Truth enters only evaluation and rendering, never the schedule.
            J = pin.computeFrameJacobian(task_dynamics.model, task_dynamics.data, q_solve,
                                         task_dynamics.end_effector_id,
                                         pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            interface_contacts = sum(
                (int(contact.geom1) in tool_geoms and int(contact.geom2) in target_geoms)
                or (int(contact.geom2) in tool_geoms and int(contact.geom1) in target_geoms)
                for contact in muj_robot.data.contact)
            log.docking_samples.append(docking_sample(
                docking, scene.target, t=t, phase=trajector_planner.phase(t),
                position=solve_pos, rotation=solve_ori, velocity=solve_vel,
                angular_velocity=J[3:] @ v_solve, force=measured_force,
                moment=measured_torque, interface_contacts=interface_contacts,
                other_contacts=muj_robot.data.ncon-interface_contacts))
            log.docking_joint_limit_violation |= bool(
                np.any(q < task_dynamics.model.lowerPositionLimit-1e-6)
                or np.any(q > task_dynamics.model.upperPositionLimit+1e-6))
            muj_robot.set_coordinate_frames({
                "desired": T_d, "actual": pin.SE3(current_ori, current_pos)}, replace=False)
        # SE(3) Lie 控制器诊断（标量子集；完整 latest_diagnostics 留在控制器内）
        if use_se3:
            d = task_dynamics.latest_diagnostics
            log.se3_diagnostics.append({
                "t": t,
                "lam_translation_norm": d["lam_translation_norm"],
                "lam_rotation_norm": d["lam_rotation_norm"],
                "lam_dot_norm": d["lam_dot_norm"],
                "cond_dexp": d["cond_dexp"],
                "cond_task": d["cond_task"],
                "F_body_norm": d["F_body_norm"],
                "tau_norm": d["tau_norm"],
                "torque_saturated": torque_saturated,
                "control_update": control_update,
                "control_t": control_t,
                "feedback_t": feedback_sample.t,
                "feedback_age_s": control_t-feedback_sample.t,
                "feedback_body": F_body.copy(),
                "feedback_origin": feedback_sample.origin.copy(),
                "feedback_world_at_origin": feedback_sample.world.copy(),
                "control_position": control_pose.translation.copy(),
                "control_rotation": control_pose.rotation.copy(),
                "yaw_stiffness": float(task_dynamics.K[5, 5]),
                "lateral_stiffness": np.diag(task_dynamics.K)[:2].copy(),
                "filtered_axial_force_N": yaw_schedule.filtered_force_N if yaw_schedule else 0.,
                "yaw_trigger_t": yaw_schedule.trigger_t if yaw_schedule and yaw_schedule.trigger_t is not None else -1.,
            })
        step_index += 1
    if plot:
        log.plot_results(save_path="figure/",
                         scene_name=scene.name if scene is not None else None)

    if muj_robot.record:
        # 创建绝对路径以确保视频保存在正确位置（仓库根目录 video/，与迁移前一致）/
        # Use an absolute path so the video lands in <repo>/video as before the refactor
        current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        video_dir = os.path.join(current_dir, "video")
        if not os.path.exists(video_dir):
            try:
                os.makedirs(video_dir)
                print(f"Created video directory: {video_dir}")
            except Exception as e:
                print(f"Error creating video directory: {e}")

        video_name = scene.name if scene is not None else "docking_update"
        video_path = os.path.join(video_dir, f"{video_name}.mp4")
        print(f"Saving video to absolute path: {video_path}")
        muj_robot.to_mp4(video_path)



def main(render=True, record=True, dt=0.001, traj_duration=15.0, duration=None,
         scene_path: str | Path = DEFAULT_SCENE_PATH, controller: str | None = None,
         scene: Scene | None = None, plot: bool = True, diagnostics: bool | str = False):
    """
    Main function that sets up and runs the robot control simulation.

    Args:
        render (bool): Whether to render the simulation visually
        record (bool): Whether to record the simulation to a video file
        dt (float): Simulation time step in seconds
        traj_duration (float): Duration of the trajectory execution in seconds
        duration (float): Total simulation duration in seconds
        scene_path (str | Path): 场景 YAML 路径（默认 DEFAULT_SCENE_PATH，即 iiwa14 对接场景） /
            Scene YAML path (default: the built-in iiwa14 docking scene)
        controller (str): 控制器选择："impedance"（默认，固定增益任务空间阻抗 +
            软限幅）、"se3_lie"（SE(3) Lie 群阻抗，Kim et al. 2025 T-RO §III-A，
            T̃/λ/dexp/γ 全链路 + body wrench 反馈）或 "hqp"（HQP-AC 约束自适应
            控制，Ren & Shan 2026 §3.2，关节位置/速度/力矩极限为 QP 硬约束，
            刚度按接触力自适应）

    Returns:
        Log: 记录了完整仿真时序数据的日志对象 / populated telemetry log
    """
    # 加载场景：机械臂/工具/目标/物理参数与任务初始条件来自场景 YAML；
    # 允许直接注入 Scene 对象（批量实验用 dataclasses.replace 做配置变体）/
    # Load scene from YAML, or accept a pre-built Scene (batch experiments
    # build config variants via dataclasses.replace)
    scene = scene if scene is not None else load_scene(scene_path)
    controller = controller or scene.controller
    docking_trajectory = None
    if scene.docking is not None:
        if controller != "se3_lie":
            raise ValueError("waypoint docking requires se3_lie for moving attitude and axial compliance")
        docking_trajectory = build_docking_trajectory(scene.task, scene.docking, scene.trajectory)
    if duration is None:
        duration = (docking_trajectory.total_duration + scene.docking.hold_s
                    if docking_trajectory is not None else 20.0)

    # 参数集中管理：函数入参覆盖 DockingConfig 默认值 /
    # Centralized params: function args override DockingConfig defaults
    cfg = replace(DockingConfig(), dt=dt, traj_duration=traj_duration, duration=duration)

    log = Log()

    # 1) 构建 Pinocchio 模型/数据（用于雅可比/动力学计算；重力由 load_pin_model 置零） /
    # 1) Build Pinocchio model/data (for Jacobians and dynamics; gravity zeroed by load_pin_model)
    pin_kwargs = {}
    if scene.tool.pin_inertia is not None:
        inertia = scene.tool.pin_inertia
        pin_kwargs = {
            "tool_frame": scene.robot.ee_frame,
            "tool_mount_pos": scene.tool.pose_pos,
            "tool_mount_quat": scene.tool.pose_quat,
            "tool_mass": inertia.mass,
            "tool_com": inertia.com,
            "tool_diaginertia": inertia.diaginertia,
        }
    pin_model = (load_assembled_pin_model(scene) if scene.robot.pin_model is None
                 else load_pin_model(scene.robot.pin_model, **pin_kwargs))
    pin_data = pin_model.createData()

    # 任务初始条件（先于控制器构建提取，供 IK 与 HQP 期望姿态使用） /
    # Task init conditions (extracted before controller construction, for IK and HQP r_des)
    init_pos = scene.task.init_pos
    init_ori = scene.task.init_ori

    # 组装 MuJoCo 模型（控制器摩擦前馈与仿真环境同源，提前到控制器构建之前）/
    # Assemble the MuJoCo model up front: its dof_frictionloss feeds the
    # controllers' friction feedforward so compensation matches simulation
    mj_model = scene.build_mjmodel()
    frictionloss = mj_model.dof_frictionloss.copy()
    damping = mj_model.dof_damping.copy()

    # 控制器装配：impedance = 固定增益任务空间阻抗（历史主路径，行为不变）；
    # hqp = HQP-AC（同签名鸭子类型替换，力矩硬约束取 ±cfg.max_torque，
    # 与 impedance 路径 run 循环里的 clip 限幅同幅值，两条路径公平对比）。
    # frictionloss 为零（如 iiwa14）时前馈项恒为零，行为不变 /
    # Controller assembly: "impedance" keeps the legacy fixed-gain task-space
    # impedance path unchanged; "hqp" swaps in the HQP-AC controller via the
    # same-signature duck-typed interface. Zero frictionloss (e.g. iiwa14)
    # makes the friction feedforward a no-op
    imp_cfg = ImpedanceConfig()
    if scene.impedance is not None:
        ov = scene.impedance
        imp_cfg = ImpedanceConfig(
            k=ov.k if ov.k is not None else imp_cfg.k,
            d=ov.d if ov.d is not None else imp_cfg.d,
            k_rot=ov.k_rot if ov.k_rot is not None else imp_cfg.k_rot,
            d_rot=ov.d_rot if ov.d_rot is not None else imp_cfg.d_rot)
        print(f"阻抗覆盖: k={imp_cfg.k} d={imp_cfg.d} k_rot={imp_cfg.k_rot} d_rot={imp_cfg.d_rot}")
    if controller == "impedance":
        task_dynamics = TaskSpaceController(pin_model, cfg.dt, imp_cfg, ee_frame=scene.robot.ee_frame,
                                            frictionloss=frictionloss, damping=damping,
                                            friction_mode=scene.friction_comp)
    elif controller == "se3_lie":
        # SE(3) Lie 群阻抗（Kim et al. 2025 T-RO §III-A）：场景可选
        # se3_impedance 段覆盖 A/D/K 对角与零空间阻尼，缺省取基线映射值
        se3_cfg = SE3ImpedanceConfig()
        ov = scene.se3_impedance
        if ov is not None:
            se3_kwargs = {}
            if ov.a_diag is not None:
                se3_kwargs["A_diag"] = np.asarray(ov.a_diag, dtype=float)
            if ov.d_diag is not None:
                se3_kwargs["D_diag"] = np.asarray(ov.d_diag, dtype=float)
            if ov.k_diag is not None:
                se3_kwargs["K_diag"] = np.asarray(ov.k_diag, dtype=float)
            if ov.null_damping is not None:
                se3_kwargs["null_damping"] = float(ov.null_damping)
            if se3_kwargs:
                se3_cfg = replace(se3_cfg, **se3_kwargs)
                print(f"SE(3) 阻抗覆盖: {se3_kwargs}")
        task_dynamics = SE3LieImpedanceController(
            pin_model, ov.control_period if ov and ov.control_period else cfg.dt,
            se3_cfg, ee_frame=scene.robot.ee_frame,
            frictionloss=frictionloss, damping=damping,
            friction_mode=scene.friction_comp)
        print("控制器: SE(3) Lie 群阻抗（Kim et al. 2025 T-RO，T̃/λ/dexp/γ 全链路；"
              "未含论文 §III-B NRIC 鲁棒内环）")
    elif controller == "hqp":
        hqp_ov = scene.hqp
        preload_axis = None
        if hqp_ov is not None and hqp_ov.preload_force:
            stroke_norm = np.linalg.norm(scene.task.stroke)
            preload_axis = scene.task.stroke / stroke_norm if stroke_norm > 0 else None
        task_dynamics = HQPAdaptiveController(
            pin_model, cfg.dt, HQPConfig(torque_limit=cfg.max_torque),
            ee_frame=scene.robot.ee_frame, r_des=init_ori, frictionloss=frictionloss,
            damping=damping, impedance=imp_cfg, friction_mode=scene.friction_comp,
            **(dict(force_source=hqp_ov.force_source,
                    observer_kp=hqp_ov.observer_kp, observer_ki=hqp_ov.observer_ki,
                    preload_force=hqp_ov.preload_force,
                    preload_ramp_s=hqp_ov.preload_ramp_s,
                    preload_axis=preload_axis,
                    contact_deadband=hqp_ov.contact_deadband) if hqp_ov is not None else {}))
        print(f"控制器: HQP-AC（Ren & Shan 2026 §3.2，关节位置/速度/力矩 QP 硬约束 + "
              f"接触力自适应刚度；力矩约束 ±{cfg.max_torque} N·m）")
    else:
        raise ValueError(f"未知控制器: {controller!r}（可选 'impedance'、'se3_lie' 或 'hqp'）")
    if np.any(frictionloss > 0):
        print(f"摩擦前馈: frictionloss={np.round(frictionloss, 3)} N·m（取自组装模型 dof_frictionloss）")
    if np.any(damping > 0):
        print(f"阻尼前馈: dof_damping={np.round(damping, 3)} N·m·s/rad（取自组装模型）")

    # 计算初始位姿的逆运动学，得到初始关节位置 /
    # Compute IK for initial pose to get initial joint configuration
    init_pose = pin.SE3(init_ori, init_pos)

    # 使用合适的初始猜测值以帮助IK收敛 /
    # Use a good initial guess to help IK converge
    initial_guess = scene.task.ik_guess
    q_init, success = compute_ik(pin_model, pin_data, init_pose, initial_q=initial_guess, max_iters=5000,
                                 ee_frame=scene.robot.ee_frame)

    if not success:
        if scene.docking is not None:
            raise ValueError("Initial docking pose is unreachable; refusing to start from failed IK")
        print("Warning: IK did not converge perfectly, but continuing with best solution found.")
    if docking_trajectory is not None:
        # Check a continuous IK branch through the commanded path. Collision
        # acceptance remains based on the actual rollout, including link contacts.
        from contextlib import redirect_stdout
        from io import StringIO
        q_check = q_init.copy()
        with redirect_stdout(StringIO()):
            for sample_t in np.linspace(0, docking_trajectory.total_duration, 80):
                q_check, reachable = compute_ik(
                    pin_model, pin_data, docking_trajectory.get_pose(sample_t),
                    initial_q=q_check, max_iters=200, ee_frame=scene.robot.ee_frame)
                if not reachable:
                    raise ValueError(f"Docking waypoint path is unreachable at t={sample_t:.3f}s")

    #步进 Pinocchio 以更新数据 /
    # Step Pinocchio to update data
    pin.forwardKinematics(pin_model, pin_data, q_init)
    # 更新所有坐标系位姿
    # Update frame placements
    pin.updateFramePlacements(pin_model, pin_data)
    #获取末端在世界坐标系的SE(3)位姿
    H_init = pin_data.oMf[pin_model.getFrameId(scene.robot.ee_frame)]
    print(f"Initial end-effector position: {H_init.translation}")
    print(f"Initial joint positions: {q_init}", success)

    # 跟踪测试模式（trajectory.type == "tracking"）：圆+8字跟踪测试不对接 /
    # Tracking-test mode: circle + figure-8 tracking, no docking target involved
    is_tracking = scene.trajectory is not None and scene.trajectory.type == "tracking"

    # Set target position (relative motion): 目标 = 初始位置 + 对接行程 /
    # Target = initial position + docking stroke (from scene)
    # 跟踪测试分支下目标位置即初始位置（MujRobot 构造仍需要 target_pos） /
    # In tracking mode target_pos = init_pos (MujRobot construction still needs it)
    target_pos = (docking_trajectory.poses[-1].translation.copy() if docking_trajectory is not None
                  else init_pos if is_tracking else init_pos + scene.task.stroke)
    print(f"Target position: {target_pos}")

    # Initialize robot simulation with parameters
    # 2) 初始化 MuJoCo 机器人（写 tau、推进仿真、渲染/录帧） /
    # 2) Initialize MuJoCo robot (apply tau, step sim, render/record)
    muj_robot = MujRobot(
        model=mj_model,
        render=render,
        record=record,
        dt=cfg.dt,
        target_pos=target_pos,
        eef_body=scene.eef_body,
        camera=scene.camera,
    )

    # Create trajectory planner
    # 3) 构建任务空间轨迹规划器，三路分发：trajectory.type == "tracking" 时用
    #    圆+8字跟踪测试轨迹（过渡→竖直圆→过渡→平面8字→保持，用于测试控制器
    #    轨迹跟踪能力，cfg.traj_duration 被忽略）；trajectory 段（缺省 twophase）
    #    用两段式对接轨迹（接近段宽松限速 + 对接段严格限速，Ren & Shan 2026
    #    任务结构的简化版），此时 cfg.traj_duration 被忽略；无 trajectory 段则
    #    维持历史单段解耦五次轨迹 /
    #    Three-way planner dispatch: "tracking" → circle + figure-8 tracking-test
    #    trajectory (cfg.traj_duration ignored); trajectory section (default
    #    twophase) → two-phase docking trajectory; else the legacy quintic
    # SE(3)-TOPP 分发（置于最前；论文 §3.1 的实现）
    if docking_trajectory is not None:
        trajector_planner = docking_trajectory
        print(f"轨迹: 侧方接近/上方路点/下降/柔顺插入，运动 {trajector_planner.total_duration:.3f}s，"
              f"保持 {scene.docking.hold_s:.3f}s")
    elif scene.trajectory is not None and scene.trajectory.type == "se3topp":
        from compliant_docking.planning.se3_topp import SE3ToppTrajectory
        traj_spec = scene.trajectory
        # 当前内置对接场景姿态恒定；规划器仍按步输出完整 T_d/V_d/Vdot_d，
        # 时变姿态接口已接入控制链路，尚缺的是专门的场景级验证基线。
        final_ori = init_ori
        trajector_planner = SE3ToppTrajectory(
            init_pos, init_ori, target_pos, final_ori,
            standoff=traj_spec.standoff,
            v_max_approach=traj_spec.v_max_approach,
            a_max_approach=traj_spec.a_max_approach,
            omega_max_approach=traj_spec.omega_max_approach,
            alpha_max_approach=traj_spec.alpha_max_approach,
            v_max_docking=traj_spec.v_max_docking,
            a_max_docking=traj_spec.a_max_docking,
            omega_max_docking=traj_spec.omega_max_docking,
            alpha_max_docking=traj_spec.alpha_max_docking)
        t1, t2 = trajector_planner.durations
        print(f"轨迹: SE(3)-TOPP（接近 {t1:.3f}s + 对接 {t2:.3f}s，总时长 "
              f"{trajector_planner.total_duration:.3f}s，时间最优剖面）")
    elif is_tracking:
        traj_spec = scene.trajectory
        trajector_planner = CircleFigure8Trajectory(
            init_pos,
            transition_duration=traj_spec.transition_duration,
            circle_duration=traj_spec.circle_duration,
            circle_radius=traj_spec.circle_radius,
            circle_frequency=traj_spec.circle_frequency,
            circle_center_offset=traj_spec.circle_center_offset,
            figure8_duration=traj_spec.figure8_duration,
            figure8_radius_x=traj_spec.figure8_radius_x,
            figure8_radius_y=traj_spec.figure8_radius_y,
            figure8_frequency=traj_spec.figure8_frequency,
        )
        t_tr, t_c, t_8 = trajector_planner.durations
        print(f"轨迹: 圆+8字跟踪测试（过渡 {t_tr:.3f} + 圆 {t_c:.3f} + "
              f"过渡 {t_tr:.3f} + 8字 {t_8:.3f}，总时长 "
              f"{trajector_planner.total_duration:.3f} s）")
    elif scene.trajectory is not None:
        traj_spec = scene.trajectory
        trajector_planner = TwoPhaseDockingTrajectory(
            init_pos, target_pos,
            standoff=traj_spec.standoff,
            v_max_approach=traj_spec.v_max_approach,
            a_max_approach=traj_spec.a_max_approach,
            v_max_docking=traj_spec.v_max_docking,
            a_max_docking=traj_spec.a_max_docking,
        )
        t1, t2 = trajector_planner.durations
        print(f"轨迹: 两段式对接（接近段 {t1:.3f}s + 对接段 {t2:.3f}s，总时长 "
              f"{trajector_planner.total_duration:.3f}s）")
        print(f"提示: 场景提供 trajectory 段，cfg.traj_duration={cfg.traj_duration}s 被忽略")
    else:
        trajector_planner = DecoupledQuinticTrajectory(init_pos, target_pos, cfg.traj_duration)
        print(f"轨迹: 单段解耦五次多项式（时长 {cfg.traj_duration}s）")

    # Run simulation with specified duration
    run_simulation(
        muj_robot,
        task_dynamics,
        trajector_planner,
        log,
        cfg=cfg,
        q_init=q_init,
        scene=scene,
        r_des=init_ori,
        plot=plot,
        diagnostics=diagnostics,
    )

    # 4) 性能指标输出：
    #    - 跟踪测试模式：跳过对接指标（无接触/无对接轴），改为按轨迹段打印
    #      位置/姿态误差、力矩限幅与接触数，并执行结构化 PASS/FAIL 门禁；
    #    - 对接模式：对接性能指标（Ren & Shan 2026, Acta Astronautica,
    #      Table 10 三层指标），复用前已构建的 pin_model（不重复加载）；
    #      对接轴 = 轨迹推进方向（stroke）归一化。
    #    本阶段指标仅打印到 stdout（不落盘），打印必须发生在 CLI 汇总行之前 /
    #    Print metrics to stdout only (no persistence), before the CLI summary line
    if is_tracking:
        tracking_metrics = compute_tracking_metrics(log, trajector_planner.segments)
        last_sample_time = log.t_list[-1] if log.t_list else float("-inf")
        tracking_gate = evaluate_tracking_gate(
            tracking_metrics,
            scene.tracking_thresholds or TrackingThresholds(),
            # 日志记录的是步进前时刻，因此容许一个控制周期；按实际日志覆盖范围
            # 判定，不能只相信请求的 duration（仿真若提前退出不得误报完整）。
            complete=last_sample_time + cfg.dt >= trajector_planner.total_duration,
        )
        # 附加在既有 Log 返回值上，保持 main() 的历史调用接口不变。
        log.tracking_metrics = tracking_metrics
        log.tracking_gate = tracking_gate
        print(format_tracking_gate(tracking_gate))
    else:
        axis = (np.array([0., 0., -1.]) if docking_trajectory is not None
                else scene.task.stroke / np.linalg.norm(scene.task.stroke))
        metrics = compute_metrics(log, pin_model, axis=axis, ee_frame=scene.robot.ee_frame)
        print(format_metrics(metrics))
    if docking_trajectory is not None:
        log.docking_gate = evaluate_docking(log, scene.docking, docking_trajectory, cfg.dt)
        log.docking_trajectory = docking_trajectory
        if diagnostics:
            log.contact_summary = summarize_diagnostics(log.contact_diagnostics, log.contact_events,
                                                        target_rotation(scene.target))
        print(f"[insertion gate] {log.docking_gate}")

    log.simulation_warnings = {str(mujoco.mjtWarning(i)).split(".")[-1]: int(w.number)
                               for i, w in enumerate(muj_robot.data.warning) if w.number}
    return log

if __name__ == '__main__':
    main(render=False, record=True, dt=0.001, traj_duration=15.0, duration=18.0)
