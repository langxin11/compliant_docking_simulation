# 核心模块 API

## 场景

::: compliant_docking.scene
    options:
      members:
        - load_scene
        - Scene
        - RobotSpec
        - ToolSpec
        - TargetSpec
        - PhysicsSpec
        - TaskSpec
        - ImpedanceOverride
        - HQPOverride
        - TrajectorySpec

## 指标与门禁

::: compliant_docking.metrics
    options:
      members:
        - compute_metrics
        - format_metrics
        - tracking_summary
        - compute_tracking_metrics
        - evaluate_tracking_gate
        - format_tracking_gate
        - DockingMetrics
        - TrackingThresholds

## 遥测

::: compliant_docking.telemetry.Log
    options:
      members:
        - reset_logs
        - store_data
        - plot_results

## 模型加载

::: compliant_docking.models.load_pin_model

## 绘图

::: compliant_docking.plotting
    options:
      members:
        - apply_style
        - plot_docking_log
