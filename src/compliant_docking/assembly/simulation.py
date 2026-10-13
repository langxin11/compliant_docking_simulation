"""接触导纳装配主循环与理想 weld 交接；落座门限保持原验收协议不变。"""
from __future__ import annotations

import json

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import quaternion
from .sequence import reference_ik


def control_every_steps(dt):
    """接触 IK 每 10 ms 更新一次、记录通道 100 Hz，这里换算成物理步数。"""
    return max(1, round(.01/dt))


def simulate(r, model, spline, phase_list, planning):
    contact_wrench = r.geometry.wrench
    contact_seated = r.geometry.stop_loaded
    storage_contact = r.geometry.storage_wrench
    contact_force_n = r.scene.contact_force_n
    data, kin = mujoco.MjData(model), mujoco.MjData(model)
    ends = np.cumsum([p.seconds for p in phase_list])
    starts = np.r_[0, ends[:-1]]
    data.qpos[:7] = spline(0)
    mujoco.mj_forward(model, data)
    position = r.install_tip+[0, 0, .006]
    correction = np.zeros(3)
    qref = data.qpos[:7].copy()
    qdot, filtered, zvelocity = np.zeros(7), 0., -.001
    first_contact, locked, dwell, unload_dwell = None, None, 0., 0.
    invalid_run = 0.
    events, done, faults = [], set(), set()
    records = {key: [] for key in ["t", "qpos", "qvel", "phase", "locks"]}
    telemetry = []
    storage_telemetry = []
    gripper_id = model.equality("gripper_lock").id
    low, high = np.asarray(planning["joint_limits"])
    control_every = control_every_steps(r.dt)
    # 测量链变体只供 P0/P1/P2 验证入口使用。默认值（无噪声、raw_strict 判定）
    # 保持正式 1 ms 验收逐位不变；只有场景显式声明时才启用噪声或滤波判定。
    gate = getattr(r, "seating_gate", "raw_strict")
    noise_sigma = float(getattr(r, "force_noise_sigma", 0.) or 0.)
    rng = np.random.default_rng(getattr(r, "noise_seed", 0)) if noise_sigma else None
    start_step = 0
    for step in range(start_step, round(ends[-1]/r.dt)+1):
        t = step*r.dt
        k = min(int(np.searchsorted(ends, t, side="right")), len(phase_list)-1)
        mujoco.mj_forward(model, data)
        f, moment, count, unexpected, depth = contact_wrench(model, data)
        seating_condition = contact_seated(model, data)
        physical_error = r.lock_error(model, data, 2)
        ready = False
        rows = np.flatnonzero((data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
                              & (data.efc_id == gripper_id))
        grip_load = float(np.linalg.norm(data.efc_force[rows[:3]])) if len(rows) else 0.
        if locked is not None and grip_load <= .2 and np.linalg.norm(data.qvel[:7]) <= .002:
            unload_dwell += r.dt
        else:
            unload_dwell = 0.
        if unexpected:
            faults.add("structural contact")
        if np.any(data.qpos[:7] < low) or np.any(data.qpos[:7] > high):
            faults.add("joint limit")
        contact_mode = k == 9 and locked is None
        if contact_mode:
            # 轴向力是测量信号：真实接触力叠加可选的传感器高斯噪声（单位 N）。
            axial = f[2] if rng is None else f[2]+rng.normal(0., noise_sigma)
            # 一阶低通，时间常数 20 ms；raw_strict 与 filtered_debounce 都用它积分导纳。
            filtered += (1-np.exp(-r.dt/.02))*(axial-filtered)
            if gate == "filtered_debounce":
                if first_contact is None and filtered >= .1:
                    first_contact = t
                in_window = .15 <= filtered <= .6
            else:
                if first_contact is None and axial >= .1:
                    first_contact = t
                in_window = .15 <= axial <= .6
            if first_contact is not None:
                # 接触导纳：目标轴向力从 0 在 1 s 内线性升到 contact_force_n（0.4 N），
                # 进给速度按（滤波力 − 目标力）积分，积分时间 0.5 s、阻尼系数 200，
                # 幅值限制在 ±1 mm/s，避免接触瞬间的大力阶跃把模块顶开。
                target = contact_force_n*min(1., (t-first_contact)/1.)
                zvelocity += r.dt*(filtered-target-200*zvelocity)/.5
                zvelocity = float(np.clip(zvelocity, -.001, .001))
            else:
                zvelocity = -.001
            position[2] = max(r.install_tip[2]-.002, position[2]+zvelocity*r.dt)
            if step % control_every == 0:
                tip = data.site("gripper_tip")
                actual = tip.xmat.reshape(3, 3)
                position[:2] += .03*(r.install_tip[:2]-tip.xpos[:2])
                error = Rotation.from_matrix(r.tip_rotation@actual.T).as_rotvec()
                correction[:2] += .03*error[:2]
                correction[:2] = np.clip(correction[:2], -.052, .052)
                yaw = .8*np.arctan2(actual[1, 0], actual[0, 0])
                desired_rotation = Rotation.from_rotvec(correction).as_matrix()@Rotation.from_euler("z", yaw).as_matrix()@r.tip_rotation
                qref = reference_ik(model, kin, qref, position, desired_rotation)
                # 接触阶段保持真实伺服阻尼：参考速度置零，让 PD 的速度项起阻尼作用。
                # 若把实测关节速度直接当作参考速度反馈，会抵消该项的速度阻尼，
                # 仿真中可能出现接触后的慢漂。
                qdot = np.zeros(7)
            desired, velocity = qref, qdot
            error = physical_error
            ready = (first_contact is not None and t-first_contact >= 1. and in_window
                     and error[0] <= .00075 and error[1] <= np.deg2rad(.5)
                     and error[2] <= .0005 and error[3] <= np.deg2rad(.5)
                     and depth <= .0003 and np.linalg.norm(moment) <= .5
                     and seating_condition)
            if ready:
                dwell += r.dt
                invalid_run = 0.
            elif first_contact is not None and t-first_contact >= 1.:
                # filtered_debounce 允许测量毛刺持续 10 ms 以内不重置 dwell；
                # 历史 raw_strict 门限则要求每一步都满足条件，任一步失败即清零。
                invalid_run += r.dt
                if gate != "filtered_debounce" or invalid_run > .01:
                    dwell = 0.
            else:
                dwell, invalid_run = 0., 0.
            if dwell >= .5:
                # 持续就位满 0.5 s 后锁定：weld 目标 site 取模块 1 当前实测位姿
                # （换算到模块 2 本体系），因此接通 weld 的瞬间不产生位置跳变。
                anchor = model.site("assembly_anchor").id
                module2 = data.body("module2")
                model.site_pos[anchor] = module2.xmat.reshape(3, 3).T@(data.body("module1").xpos-module2.xpos)
                model.site_quat[anchor] = quaternion(module2.xmat.reshape(3, 3).T@data.body("module1").xmat.reshape(3, 3))
                mujoco.mj_forward(model, data)
                data.eq_active[2] = True
                locked = t
                qref, qdot = data.qpos[:7].copy(), np.zeros(7)
                events.append(dict(t=t, event="assembly_on", force_n=float(f[2]), dwell_s=dwell))
                print(events[-1], flush=True)
        elif k == 9 or k == 10:
            if locked is None:
                faults.add("contact seating timeout")
                break
            desired, velocity = qref, np.zeros(7)
        else:
            desired, velocity = spline(t), spline(t, 1)
            # 接触段的 IK 初值取转运段末端的参考关节角，而不是初始存储姿态，
            # 避免从大偏差起步迭代。
            qref = desired.copy()
        phase = phase_list[k]
        if phase.event and phase.event not in done and t-starts[k] >= phase.seconds*.6:
            if phase.event != "grip_off" or unload_dwell >= .5:
                error = r.handover(model, data, phase.event)
                events.append(dict(t=t, event=phase.event, error_m=error[0],
                                   gripper_constraint_force_n=grip_load, unloaded_dwell_s=unload_dwell))
                done.add(phase.event)
                print(events[-1], flush=True)
        data.ctrl[:] = r.kp*desired+r.kd*velocity+data.qfrc_bias[:7]
        torque = r.kp*(desired-data.qpos[:7])+r.kd*(velocity-data.qvel[:7])+data.qfrc_bias[:7]
        if np.any(abs(torque) > r.torque_limits):
            faults.add("torque saturation")
        row = [t, axial if contact_mode else f[2], np.linalg.norm(f[:2]), np.linalg.norm(moment), count, depth, dwell,
               *physical_error, int(data.eq_active[2]), grip_load, unload_dwell,
               int(seating_condition), int(ready), float(np.linalg.norm(data.qvel[:7]))]
        if noise_sigma or gate != "raw_strict":
            # 修改过测量链的运行把真实力和滤波通道与测量轴向力（第 1 列）并列记录，
            # 便于事后区分传感器效应与物理过程。
            row += [f[2], filtered]
        telemetry.append(row)
        if storage_contact is not None:
            storage_force, storage_depth, storage_count = storage_contact(model, data)
            separation = float(np.linalg.norm(data.site("module1_port_4_mating").xpos-data.site("storage_dock_mating").xpos))
            storage_telemetry.append([t, *storage_force, storage_depth, storage_count, separation,
                                      *data.eq_active.astype(int)])
        if step % control_every == 0:
            for key, value in [("t", t), ("qpos", data.qpos.copy()), ("qvel", data.qvel.copy()),
                               ("phase", k), ("locks", data.eq_active.copy())]:
                records[key].append(value)
        if not np.isfinite(data.qpos).all():
            raise RuntimeError("Nonfinite state")
        if step < round(ends[-1]/r.dt):
            mujoco.mj_step(model, data)
    a = np.asarray(telemetry)
    final_error = float(np.linalg.norm(data.body("module1").xpos-r.installed))
    if list(data.eq_active) != [False, False, True]:
        faults.add("incomplete handover")
    if final_error > .001:
        faults.add("final module position")
    if sum(w.number for w in data.warning):
        faults.add("simulation warning")
    if a[:, 1].max() > 10 or a[:, 5].max() > .0003:
        faults.add("contact load or penetration limit")
    if storage_telemetry:
        stored = np.asarray(storage_telemetry)
        if np.linalg.norm(stored[:, 1:4], axis=1).max() > 10 or stored[:, 4].max() > .0003:
            faults.add("storage contact load or penetration limit")
    report = dict(status="FAIL" if faults else "PASS", faults=sorted(faults), events=events,
                  peak_axial_force_n=float(a[:, 1].max()), max_penetration_mm=float(a[:, 5].max()*1000),
                  final_module_error_mm=final_error*1000, first_contact_s=first_contact,
                  lock_time_s=locked, module_mass_kg=float(model.body("module1").mass[0]),
                  module_inertia_kg_m2=model.body("module1").inertia.tolist(),
                  duration_s=float(ends[-1]),
                  contact_force_target_n=contact_force_n,
                  seating_gate=gate, force_noise_sigma_n=noise_sigma,
                  noise_seed=int(getattr(r, "noise_seed", 0)),
                  scope="fixed-base zero-gravity arm/module dynamics; real guide/stop contact; ideal weld locks; MuJoCo bias-compensated joint servo")
    if storage_telemetry:
        report["storage_interface"] = dict(
            module_port="module1_port_4", storage_port="storage_dock_port",
            peak_contact_force_n=float(np.linalg.norm(stored[:, 1:4], axis=1).max()),
            max_penetration_mm=float(stored[:, 4].max()*1000),
            final_clearance_m=float(stored[-1, 6]))
        np.savez_compressed(r.output/"storage_trace.npz", values=stored,
                           columns=np.array(["time", "force_x", "force_y", "force_z", "penetration",
                                             "loaded_contacts", "mating_separation", "storage_locked",
                                             "gripper_locked", "assembly_locked"]))
    columns = ["time", "axial_force", "lateral_force", "moment", "contacts",
               "penetration", "seating_dwell", "position_error", "angle_error",
               "relative_speed", "relative_spin", "assembly_locked",
               "gripper_constraint_force", "unloaded_dwell",
               "seating_contact_condition", "seating_eligible", "arm_speed_norm"]
    if noise_sigma or gate != "raw_strict":
        columns += ["true_axial_force", "filtered_axial_force"]
    np.savez_compressed(r.output/"contact_trace.npz", values=a,
                       columns=np.array(columns))
    records = {key: np.asarray(value) for key, value in records.items()}
    np.savez_compressed(r.output/"rollout.npz", **records)
    (r.output/"validation.json").write_text(json.dumps(report, indent=2)+"\n")
    # 保存锁定时接受的 anchor 位姿，回放时从同一坐标系渲染，不出现姿态跳变。
    (r.output/"accepted_anchor.json").write_text(json.dumps(dict(pos=model.site("assembly_anchor").pos.tolist(),
                                                           quat=model.site("assembly_anchor").quat.tolist())))
    print(json.dumps(report), flush=True)
    report["module_asset"] = "hexframe"
    return records, report
