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


import numpy as np
import pinocchio as pin

from compliant_docking.config import DockingConfig, HQPConfig, ImpedanceConfig
from compliant_docking.control.hqp_ac import HQPAdaptiveController
from compliant_docking.control.task_space import TaskSpaceController
from compliant_docking.metrics import (
    TrackingThresholds,
    compute_metrics,
    compute_tracking_metrics,
    evaluate_tracking_gate,
    format_metrics,
    format_tracking_gate,
)
from compliant_docking.models import load_pin_model
from compliant_docking.planning.kinematics import compute_ik
from compliant_docking.planning.trajectory import (
    CircleFigure8Trajectory,
    DecoupledQuinticTrajectory,
    TwoPhaseDockingTrajectory,
)
from compliant_docking.scene import DEFAULT_SCENE_PATH, Scene, load_scene
from compliant_docking.simulation.mujoco_env import MujRobot
from compliant_docking.telemetry import Log


def run_simulation(muj_robot:MujRobot,
                   task_dynamics:TaskSpaceController | HQPAdaptiveController,
                   trajector_planner: DecoupledQuinticTrajectory
                   | TwoPhaseDockingTrajectory
                   | CircleFigure8Trajectory,
                   log:Log,
                   cfg:DockingConfig,
                   q_init:np.ndarray,
                   scene:Scene | None = None,
                   r_des:np.ndarray | None = None,
                   plot:bool = True):
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



    q = q_init
    v = np.zeros(task_dynamics.model.nv)

    current_pos, current_vel, _ = task_dynamics.get_task_space_state(q, v)

    force_external = np.zeros(3)
    torque_external = np.zeros(3)
    is_tracking = scene is not None and scene.trajectory is not None and scene.trajectory.type == "tracking"
    # PI 动量观测器随步更新（HQP force_source=observer 时非 no-op）/
    # Per-step PI momentum-observer update (no-op unless HQP observer mode)
    update_observer = getattr(task_dynamics, "update_momentum_observer", None)


    while muj_robot.data.time < cfg.duration:
        # while True:
        t = muj_robot.data.time
        # 1) 根据当前仿真时间采样期望的末端位置/速度/加速度 /
        # 1) Sample desired end-effector pos/vel/acc at current sim time
        pos_des, vel_des, acc_des = trajector_planner.get_state(t)
        #print(f"Current time: {t}, Desired position: {pos_des}, Desired velocity: {vel_des}, Desired acceleration: {acc_des}")

        # 2) 任务空间控制（含平动/姿态阻抗与外力补偿）→ 得到关节力矩 tau /
        # 2) Task-space control (translation/rotation impedance + external force) → joint torques tau
        tau_unclipped = task_dynamics.compute_control_task_space_with_orientation_and_imp(
            q, v, pos_des, vel_des, acc_des, current_pos, current_vel, force_external, torque_external)

        # 力矩限幅以防止过大的控制输入导致碰撞检测失败 /
        # Clamp torques to prevent excessive control inputs that cause collision detection failure
        torque_saturated = bool(np.any(np.abs(tau_unclipped) > cfg.max_torque))
        tau = np.clip(tau_unclipped, -cfg.max_torque, cfg.max_torque)

        # 3) 将 tau 写入 MuJoCo，并推进一步物理仿真 /
        # 3) Apply tau to MuJoCo and advance one simulation step
        try:
            q, v, eef_pos = muj_robot.step(tau)
        except Exception as e:
            print(f"\nSimulation error at t={t:.3f}s: {e}")
            print(f"Torques: {tau}")
            print(f"Joint positions: {q}")
            raise RuntimeError(f"仿真在 t={t:.3f}s 异常终止") from e
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
        ori_err_vec = pin.log3(r_des @ current_ori.T) if r_des is not None else None
        log.store_data(
            t, q, v, current_pos, current_vel,
            np.linalg.norm(current_pos - pos_des),
            pos_des, vel_des, acc_des, tau,
            measured_force, measured_torque,
            orientation_error=ori_err_vec,
            torque_saturated=torque_saturated,
            contact_count=muj_robot.data.ncon)


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



def main(render=True, record=True, dt=0.001, traj_duration=15.0, duration=20.0,
         scene_path: str | Path = DEFAULT_SCENE_PATH, controller: str = "impedance",
         scene: Scene | None = None, plot: bool = True):
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
            软限幅）或 "hqp"（HQP-AC 约束自适应控制，Ren & Shan 2026 §3.2，
            关节位置/速度/力矩极限为 QP 硬约束，刚度按接触力自适应）

    Returns:
        Log: 记录了完整仿真时序数据的日志对象 / populated telemetry log
    """
    # 加载场景：机械臂/工具/目标/物理参数与任务初始条件来自场景 YAML；
    # 允许直接注入 Scene 对象（批量实验用 dataclasses.replace 做配置变体）/
    # Load scene from YAML, or accept a pre-built Scene (batch experiments
    # build config variants via dataclasses.replace)
    scene = scene if scene is not None else load_scene(scene_path)

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
    pin_model = load_pin_model(scene.robot.pin_model, **pin_kwargs)
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
        raise ValueError(f"未知控制器: {controller!r}（可选 'impedance' 或 'hqp'）")
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
        print("Warning: IK did not converge perfectly, but continuing with best solution found.")

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
    target_pos = init_pos if is_tracking else init_pos + scene.task.stroke
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
    if scene.trajectory is not None and scene.trajectory.type == "se3topp":
        from compliant_docking.planning.se3_topp import SE3ToppTrajectory
        traj_spec = scene.trajectory
        final_ori = init_ori  # 对接场景姿态恒定（SE(3) 机制就绪，按步姿态参考接口待接入）
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
        axis = scene.task.stroke / np.linalg.norm(scene.task.stroke)
        metrics = compute_metrics(log, pin_model, axis=axis, ee_frame=scene.robot.ee_frame)
        print(format_metrics(metrics))

    return log

if __name__ == '__main__':
    main(render=False, record=True, dt=0.001, traj_duration=15.0, duration=18.0)
