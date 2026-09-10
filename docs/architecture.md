# 架构总览

## 分层与数据流

一次对接仿真按以下管线推进（每控制周期 1 ms）：

```text
场景 YAML ──load_scene──▶ Scene（装配 MjSpec + Pinocchio 模型）
                              │
        ┌─────────────────────┼──────────────────────────┐
        ▼                     ▼                          ▼
  规划器 Planner        控制器 Controller           MuJoCo 环境
  (get_state(t))        (compute(...))             (MujRobot.step)
        │                     ▲                          │
        │   pos/vel/acc       │ tau (限幅后)              │ q, v, 传感器
        └────────▶ 主循环 ◀───┘          ▲───────────────┘
                  (experiments/run_docking.run_simulation)
                              │
                              ▼
                    telemetry.Log（时序记录）
                              │
                    ┌─────────┴─────────┐
                    ▼                   ▼
             plotting 出图          metrics 指标/门禁
         figure/<场景>/*.png|pdf   stdout + results/*.md
```

## 模块契约

| 模块 | 职责 | 关键契约 |
|---|---|---|
| `scene` | YAML → 可仿真场景 | `load_scene(path) -> Scene`；`Scene.build_mjmodel()` 组装 MjSpec（公头挂 `ee_site`，母头可选挂 worldbody）；`target` 可选（跟踪场景无母头） |
| `planning.trajectory` | 三类规划器 | 统一接口 `get_state(t) -> (pos, vel, acc)`，世界系线量；`DecoupledQuinticTrajectory`（单段五次）、`TwoPhaseDockingTrajectory`（两段限速）、`CircleFigure8Trajectory`（跟踪测试） |
| `planning.se3_topp` | SE(3)-TOPP 规划器 | 同上接口 + `get_pose(t)`（完整位姿，供按步姿态参考）；测地线 + 解析梯形/三角形剖面 |
| `control.task_space` | 经典阻抗（CIC） | 雅可比 LOCAL_WORLD_ALIGNED；Khatib 操作空间映射 `τ = JᵀΛ(u−J̇v)+C`；摩擦/阻尼前馈 + I 项 |
| `control.hqp_ac` | HQP-AC | 主 QP（关节 pos/vel/τ 硬约束 + 自适应刚度）+ 零空间 QP（奇异性规避 + 位姿阻抗）；`force_source` 切换传感器/观测器；`preload_force` 接触预紧 |
| `control.momentum_observer` | PI 动量观测器 | `update(q, v, τ_applied)`；残差 = 未建模广义力；`force_contact` 扣除已知耗散模型 |
| `simulation.mujoco_env` | MuJoCo 封装 | `step(τ) -> (q, v, eef_pos)`（**副本语义由 telemetry 保证**）；离屏渲染/录帧 |
| `telemetry` | 时序记录 | `store_data(...)` 存副本；`plot_results()` 委托 plotting |
| `plotting` | 出图 | SciencePlots IEEE 中文，PNG+PDF，`figure/<场景>/` 归档 |
| `metrics` | Table 10 指标 | `compute_metrics`（对接三层指标）、`tracking_summary`（分段跟踪）、`evaluate_tracking_gate`（PASS/FAIL 门禁） |
| `experiments.run_docking` | 主入口 | `main(scene=…, controller=…, plot=…)`；观测器按步更新；tracking 模式隔离 F/T 反馈 |

## 关键设计决策

1. **双模型同源**：控制器用 Pinocchio，物理用 MuJoCo。两者的一致性由三层保障：
   - 质量矩阵：多位形下相对误差 < 1e-10（`tests/test_scene.py::test_fr3_pin_tool_inertia_matches_assembled_mujoco_mass_matrix`）；
   - 关节耗散（frictionloss/damping）与工具惯量：由场景 YAML 声明、`run_docking` 从组装模型读取后前馈；
   - 重力：双方置零（空间场景语义）。
2. **行为锚点**：慢速接触回归（12 s，`tests/test_regression.py`）锁定确定性数值（终态误差/峰值/稳态接触力），重构改变行为必须显式重建基线；`--quick` CLI 锚点行用于快速回归。
3. **锚点与功能的隔离**：iiwa14 场景零摩擦（摩擦前馈两模式等价），是所有数值锚点的载体；FR3 场景承载摩擦相关的行为验证。
4. **遥测副本语义**：MuJoCo 的 `qpos/qvel` 切片是视图，`Log.store_data` 必须存副本（历史 bug，见 commit 3932fe5）。
5. **规划器鸭子类型**：三类规划器共享 `get_state` 签名，主循环零分支切换；`get_pose`/`segments` 等扩展接口按需提供。
