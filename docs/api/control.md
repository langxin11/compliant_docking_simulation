# 控制器 API

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

## PI 动量观测器

::: compliant_docking.control.momentum_observer.MomentumObserver
