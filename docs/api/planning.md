# 规划器 API

所有规划器共享接口 `get_state(t) -> (pos, vel, acc)`（世界系线量前馈）。

## SE(3)-TOPP

::: compliant_docking.planning.se3_topp.SE3ToppTrajectory
    options:
      members:
        - __init__
        - get_state
        - get_pose
        - durations
        - total_duration

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
