# 架构总览

## 分层与数据流

一次对接仿真按以下管线推进（默认控制周期 1 ms）：

```text
场景 YAML ──load_scene──▶ Scene（装配 MjSpec + Pinocchio 模型）
                              │
        ┌─────────────────────┼──────────────────────────┐
        ▼                     ▼                          ▼
  规划器 Planner        控制器 Controller           MuJoCo 环境
  get_state(t)          impedance / se3_lie / hqp  MujRobot.step(τ)
        │                     ▲                          │
        │  运动参考            │ τ（限幅后）                 │ q, v, F/T
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
| `scene` | YAML → 可仿真场景 | `load_scene(path) -> Scene`；`Scene.build_mjmodel()` 组装 MjSpec；`target` 可选；`impedance`、`hqp`、`se3_impedance` 与 `friction_comp` 提供控制配置覆盖 |
| `planning.trajectory` | 位置轨迹 | 统一接口 `get_state(t) -> (pos, vel, acc)`，世界系线量；单段五次、两段式五次与圆+8字轨迹 |
| `planning.se3_topp` | SE(3)-TOPP 规划器 | `get_state(t)` 保持通用兼容；`get_motion_state(t)` 输出完整 body 运动参考 \((T_d,V_d,\dot V_d)\)；测地线 + 解析梯形/三角形剖面 |
| `planning.motion_reference` | 运动参考适配 | SE(3)-TOPP 直接透传 body 参考；纯位置轨迹结合固定 \(r_d\) 转为 \((T_d,V_d,\dot V_d)\) |
| `control.task_space` | 经典阻抗（CIC） | 雅可比 LOCAL_WORLD_ALIGNED；Khatib 操作空间映射 \(\tau=J^T\Lambda(u-\dot Jv)+C\)；摩擦/阻尼前馈 + I 项 |
| `control.se3_impedance` | SE(3) Lie 群阻抗 | LOCAL body Jacobian；\(\tilde T\)/\(\operatorname{log}_6\)/\(\operatorname{dexp}\)/有效 wrench/惯量重塑全链路；`compute_control(q, v, T_d, V_d, Vdot_d, F_body)` |
| `control.lie_se3` | Lie 群数学工具 | SO(3)/SE(3) 的 `dexp`、逆与解析时间导数，Adjoint、wrench Adjoint 和 `ad`；小角度 Taylor 稳定分支 |
| `control.hqp_ac` | HQP-AC | 主 QP（关节 pos/vel/τ 硬约束 + 自适应刚度）+ 零空间 QP（奇异性规避 + 位姿阻抗）；`force_source` 切换传感器/观测器；`preload_force` 接触预紧 |
| `control.friction` | 共享耗散前馈 | `torque` / `velocity` 两种摩擦方向模型 + 线性关节阻尼；CIC 与 SE(3) 控制器共享 helper，HQP 保持同等语义 |
| `control.momentum_observer` | PI 动量观测器 | `update(q, v, τ_applied)`；残差 = 未建模广义力；`force_contact` 扣除已知耗散模型 |
| `wrench` | F/T 坐标变换 | sensor site 原点/坐标系 → EE body 原点/坐标系；保留参考点平移产生的力矩项，确保与 LOCAL Jacobian 配对 |
| `simulation.mujoco_env` | MuJoCo 封装 | `step(τ) -> (q, v, eef_pos)`（**副本语义由 telemetry 保证**）；离屏渲染/录帧 |
| `telemetry` | 时序记录 | `store_data(...)` 存副本；`plot_results()` 委托 plotting |
| `plotting` | 出图 | SciencePlots IEEE 中文，PNG+PDF，`figure/<场景>/` 归档 |
| `metrics` | Table 10 指标 | `compute_metrics`（对接三层指标）、`tracking_summary`（分段跟踪）、`evaluate_tracking_gate`（PASS/FAIL 门禁） |
| `experiments.run_docking` | 主入口 | `main(scene=…, controller=…, plot=…)`；观测器按步更新；tracking 模式隔离 F/T 反馈 |

## SE(3) body-frame 数据流

`se3_lie` 不复用经典控制器的“世界系位置阻抗 + 姿态 PD”路径，而是保持运动
参考、状态、外力与雅可比的 frame/参考点一致：

```text
SE3ToppTrajectory.get_motion_state(t)
          │  T_d, V_d, Vdot_d（desired body）
          ├────────────────────────────┐
          │                            ▼
纯位置轨迹.get_state(t)       get_motion_reference
          │ + 固定 r_des               │
          └────────────────────────────┘
                                       │
MuJoCo F/T（sensor site）               │
          │ wrench_to_body             │
          ▼                            ▼
F_body（EE 原点/EE body 轴） ──▶ SE3LieImpedanceController
                                      │ LOCAL J, Jdot；Pinocchio M, h
                                      ▼
                      逆动力学 τ + 摩擦/阻尼前馈 → 限幅 → MuJoCo
```

SE(3)-TOPP 的时变姿态接口已经接入主循环；当前仓库内置对接场景的起止姿态
相同，因此端到端基线仍只覆盖恒定姿态。带时变姿态的场景验证是后续实验项，
不是接口缺失。

## 关键设计决策

1. **双模型同源**：控制器用 Pinocchio，物理用 MuJoCo。两者的一致性由三层保障：
   - 质量矩阵：多位形下相对误差 < 1e-10（`tests/test_scene.py::test_fr3_pin_tool_inertia_matches_assembled_mujoco_mass_matrix`）；
   - 关节耗散（frictionloss/damping）与工具惯量：由场景 YAML 声明、`run_docking` 从组装模型读取后前馈；
   - 重力：双方置零（空间场景语义）。
2. **行为锚点**：慢速接触回归（12 s，`tests/test_regression.py`）锁定确定性数值（终态误差/峰值/稳态接触力），重构改变行为必须显式重建基线；`--quick` CLI 锚点行用于快速回归。
3. **锚点与功能的隔离**：iiwa14 场景零摩擦（摩擦前馈两模式等价），是所有数值锚点的载体；FR3 场景承载摩擦相关的行为验证。
4. **遥测副本语义**：MuJoCo 的 `qpos/qvel` 切片是视图，`Log.store_data` 必须存副本（历史 bug，见 commit 3932fe5）。
5. **双层规划器契约**：所有规划器共享 `get_state`，保证旧控制器零改动；支持
   `get_motion_state` 的规划器可额外给出完整 SE(3) body 运动参考，适配器对旧规划器
   用固定姿态补齐该契约。
6. **wrench 与雅可比同 frame、同参考点**：经典/HQP 路径沿用世界轴对齐遥测；
   SE(3) 路径将 sensor-site wrench 旋转并移矩到 EE body 原点，再与 LOCAL
   Jacobian 配对，避免丢失 `(p_S-p_E)×f` 耦合项。
