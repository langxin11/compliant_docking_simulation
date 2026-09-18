# 规划器 API

所有规划器共享接口 `get_state(t) -> (pos, vel, acc)`（世界系线量前馈）。
`SE3ToppTrajectory` 还提供 `get_motion_state(t) -> (T_d, V_d, Vdot_d)`，其中
twist 与其导数在期望 body frame 表达。`get_motion_reference` 将两类接口统一成
SE(3) Lie 控制器所需的 body 运动参考。

## SE(3)-TOPP

::: compliant_docking.planning.se3_topp.SE3ToppTrajectory
    options:
      members:
        - __init__
        - get_state
        - get_pose
        - get_motion_state
        - durations
        - total_duration

## SE(3) 运动参考适配

```python
get_motion_reference(planner, t, r_des) -> tuple[pin.SE3, np.ndarray, np.ndarray]
```

返回 `(T_d, V_d, Vdot_d)`。若规划器实现 `get_motion_state` 则直接透传；否则
调用 `get_state`，以固定 `r_des` 构造位姿并把世界系线速度/加速度旋转到
desired body frame，角速度与角加速度置零。

## 解耦五次 / 两段式对接 / 圆+8字

::: compliant_docking.planning.trajectory.DecoupledQuinticTrajectory
    options:
      members:
        - __init__
        - get_state
        - verify_boundary_conditions

::: compliant_docking.planning.trajectory.TwoPhaseDockingTrajectory
    options:
      members:
        - __init__
        - get_state
        - durations
        - total_duration
        - pre_dock_pos

::: compliant_docking.planning.trajectory.CircleFigure8Trajectory
    options:
      members:
        - __init__
        - get_state
        - durations
        - total_duration
        - segments

## 逆运动学

::: compliant_docking.planning.kinematics.compute_ik
