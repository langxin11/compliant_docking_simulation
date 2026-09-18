# 控制器 API

三类控制器共享 `get_task_space_state(q, v)` 遥测接口，但主控制律的数据契约不同：
经典阻抗和 HQP-AC 接收世界轴对齐的任务空间线量；SE(3) Lie 控制器接收 body
运动参考与 EE body wrench。坐标约定与推导见
[SE(3) Lie 群阻抗控制器](../theory/se3_lie_impedance.md)。

## 任务空间阻抗（CIC）

::: compliant_docking.control.task_space.TaskSpaceController
    options:
      members:
        - __init__
        - get_task_space_state
        - get_task_space_state_with_orientation
        - compute_control_task_space_with_orientation_and_imp

## HQP-AC

::: compliant_docking.control.hqp_ac.HQPAdaptiveController
    options:
      members:
        - __init__
        - compute_control_task_space_with_orientation_and_imp
        - update_momentum_observer
        - observed_wrench
        - last_solve_time_ms

## SE(3) Lie 群阻抗

```python
SE3LieImpedanceController(
    robot_model,
    dt,
    config=None,
    ee_frame="cylinder_link",
    frictionloss=None,
    damping=None,
    friction_mode="torque",
    friction_tau_scale=2.0,
    A=None,
    D=None,
    K=None,
)
```

- `get_task_space_state(q, v)`：返回世界系位置、线速度与姿态，供统一遥测使用；
- `get_body_state(q, v)`：返回 EE 位姿、LOCAL Jacobian 与其时间导数；
- `compute_control(q, v, T_d, V_d, Vdot_d, F_body, F_d=None)`：执行 Eq. 44–66
  标称控制律并返回关节力矩；运动量和 wrench 均为线量在前；
- `latest_diagnostics`：最近一步的 `lam`、`lam_dot`、任务矩阵条件数、wrench
  范数与力矩范数等诊断，不承载控制状态。

`A`、`D`、`K` 可直接注入一般 6×6 矩阵；未提供时由
`SE3ImpedanceConfig` 的对角参数构造。

## Lie 群数学工具

公开函数均采用 Pinocchio 的线量在前排列：twist `[v; ω]`、wrench `[f; n]`。

::: compliant_docking.control.lie_se3
    options:
      members:
        - skew3
        - vee3
        - hat4
        - vee4
        - ad6
        - adjoint
        - adjoint_wrench
        - dexp_so3
        - dexp_inv_so3
        - dexp_dot_so3
        - dexp_inv_dot_so3
        - dexp_se3
        - dexp_inv_se3
        - dexp_dot_se3
        - dexp_inv_dot_se3

## Wrench 坐标与参考点变换

```python
transform_wrench(
    force, torque, p_source, R_source, p_target, R_target
) -> tuple[np.ndarray, np.ndarray]

wrench_to_body(
    force, torque, p_source, R_source, T_target
) -> np.ndarray
```

`transform_wrench` 返回 target frame 表达、target 原点参考的 `(force, torque)`；
`wrench_to_body` 是面向控制器的便利封装，返回 `[f_E; n_E]`。两者都会保留
参考点变更产生的 `(p_source-p_target)×f` 力矩项，不负责更改传感器读数符号。

## 摩擦与关节阻尼前馈

::: compliant_docking.control.friction
    options:
      members:
        - FRICTION_MODES
        - validate_friction_mode
        - friction_feedforward

## PI 动量观测器

::: compliant_docking.control.momentum_observer.MomentumObserver
