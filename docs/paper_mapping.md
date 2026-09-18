# 论文-代码对照表

复现对象：**Ren, Q. & Shan, J. (2026). A unified framework for compliant control and
trajectory planning in robotic in-orbit assembly. _Acta Astronautica_ 243, 32–45.**

图例：✅ 已复现 · ⚠️ 部分复现/等效替代 · ❌ 未复现 · ➕ 本仓库扩展（论文之外）。

## 总览：论文三大贡献

| 贡献 | 状态 | 落点 |
|---|---|---|
| (1) SE(3)-TOPP 规划器 | ✅ | `planning/se3_topp.py` |
| (2) HQP-AC 控制器 | ✅ | `control/hqp_ac.py` |
| (3) 统一安全框架（规划×控制器协同） | ✅ | `experiments/compare_frameworks.py` + `results/` |

## §2 系统模型

| 论文内容 | 状态 | 说明 |
|---|---|---|
| 自由漂浮航天器基座 + 7-DOF 机械臂 | ❌ | 本仓库为固定基座；浮基涉及动力学耦合重构，未立项 |
| SE(3) 数学预备（hat/vee、Exp/Log） | ✅ | 经 Pinocchio `exp6`/`log6`/`log3` 使用 |
| 莲花型接口 + CoACD 凸分解（196 块） | ⚠️ 等效替代 | 使用自研花冠锥面公/母头（原生 SDF 接触，`assets/interfaces/`） |

## §3.1 SE(3)-TOPP 规划器（贡献 1）

| 论文内容 | 代码 | 测试 |
|---|---|---|
| §3.1.2 分段常螺旋测地线 T(s)=T·Exp(sξ̂)（Eq.15） | `SE3ToppTrajectory._make_segment` / `_segment_state`（`pin.log6`/`exp6`） | `test_rotating_segment_geodesic_and_omega_limit` |
| Theorem 1：ν_b = ξṡ，ν̇_b = ξs̈（线性映射） | 元素级限幅 → 标量界：`_make_segment` 的 `min(V/|ξ|)` | `test_durations_match_hand_computed_topp` |
| §3.1.3 TOPP（Eq.20 速度界、Eq.21 加速度系数） | `_topp_duration` / `_topp_profile_exact`（解析梯形/三角形） | `test_topp_duration_analytic_cases` |
| §3.1.4 参考轨迹生成（位姿/扭转/加速度一致） | `get_state` / `get_pose` / `get_motion_state`；`planning.motion_reference` 接入主循环 | `test_rest_to_rest_boundaries_and_limits`、`test_get_motion_state_*` |
| Table 8 分段限速（接近 0.10 m/s·0.20 rad/s，对接 0.02/0.05） | `TrajectorySpec` 默认值（`scenes/*.yaml` 的 `trajectory:` 段） | — |
| overshoot 路径点（T3 含 6 cm 过盈） | ⚠️ 语义等价：场景 `stroke` 已含过盈量 | — |

**实测**：iiwa14+HQP，同限速下总时长 10.9→6.7 s（-39%），接触峰值力 7.83→7.92 N（不变）。

**姿态参考状态**：按步 `(T_d, V_d, Vdot_d)` body 运动参考接口已由
`SE3ToppTrajectory.get_motion_state()` 生成，并经 `planning.motion_reference` 接入
`se3_lie` 主循环。当前内置对接场景的起止姿态相同，因此已有端到端数值基线
只覆盖恒定姿态；尚缺一个时变姿态对接场景及对应验收基线。

**toppra 评估**：数值 TOPP 包（TOPP-RA）已试验——论文的 TOPP 是纯运动学且常螺旋段限幅为常数，解析解即精确最优，数值包无增益，未采用（依赖已移除）。

## §3.2 HQP-AC 控制器（贡献 2）

| 论文内容 | 代码 | 测试 |
|---|---|---|
| Eq.23–25 PI 广义动量观测器（无传感器外力估计） | `control/momentum_observer.py`（精确 Pinocchio 偏置；误差符号取 e=p−p̂ 保证稳定） | `tests/test_momentum_observer.py`（4 例） |
| Eq.26 阻抗方程 M_dΔν̇+D_dΔν+K_d e_p = F̂_ext | `hqp_ac.compute`：`target = ν̇_r + Λ⁻¹F_r`（F_r 含接触力/预紧） | `tests/test_hqp_ac.py` |
| Eq.27 6D 位姿误差 e_p=[e_pos; e_ori] | `hqp_ac.compute`（`e_ori = log3(R_cur·R_desᵀ)`） | `test_orientation_error_zero_at_desired` |
| Eq.29 参考阻尼 D_r = Λ^½K_r^½ + K_r^½Λ^½ | `HQPAdaptiveController._reference_damping` | `test_reference_damping_symmetric_psd` |
| Eq.30–31 主任务 QP | `hqp_ac.compute` 主 QP（ProxQP，seidel/默认求解） | `tests/test_hqp_ac.py` |
| Eq.32b–36 关节位置/速度/力矩硬约束（ZOH dt_p） | `HQPAdaptiveController._constraint_matrices` | `test_sim_12s_joint_limit_soft` |
| Eq.37–38 自适应刚度 α=σ(k_α‖F‖)，K_r=clip((1−α)K0, K_min, K0) | `_adaptive_stiffness`；➕ 扩展 `contact_deadband`（静摩擦估计地板） | `test_adaptive_stiffness_monotone` |
| Eq.39–40 可操作度 ω=√det(JJᵀ) 及梯度 | `_manipulability` / `_manipulability_gradient` | `test_hqp_ac.py` |
| Eq.41–42 关节位姿阻抗 q̈_ji 与奇异性规避加权 | `q_ddot_ji` / `q_ddot_sa` / `a_null` | `tests/test_hqp_ac.py` |
| Eq.43 动力学一致广义逆 J#=M⁻¹JᵀΛ 与零空间投影 N | `hqp_ac.compute` 第 7 步 | `tests/test_hqp_ac.py` |

**无传感器验证（FR3 对接）**：观测器+8 N 检测死区与传感器模式行为重合（首触 17.1 s / 峰值 5.25 N / 终态 21.0 mm）；观测器增益论文未公开，本文 K_p=20、K_i=40 为实测整定。场景样例 `scenes/fr3_docking_sensorless.yaml`。

**接触预紧**（➕ 扩展，修复纯阻抗"轻触即停"）：`preload_force` 沿 stroke 方向斜坡保持期望接触力；实测稳态 5.22 N（目标 5.0）。

## §4 仿真与结果

| 论文内容 | 状态 | 落点 |
|---|---|---|
| Table 10 三层指标（接触安全/内部安全/跟踪精度） | ✅ | `metrics.DockingMetrics` + `format_metrics` |
| §4.2.3 对比研究：解耦+CIC / 解耦+HQP-AC / SE(3)+HQP-AC | ✅ 2×2 矩阵 | `experiments/compare_frameworks.py`；`results/framework_comparison_*.md` |
| 49% 峰值力 / 30% 稳态力降低 | ⚠️ 方向一致、幅度场景相关 | FR3：峰值 -81%（27.2→5.2 N，基线含摩擦瞬态）；稳态力 HQP≈0（欠插入语义，预紧功能补齐） |
| 灾难性关节限位违反的预防 | ✅ | HQP 硬约束 + 指标层 `max_joint_pos_pct`（无 >100% 记录） |

**重要差异**：论文规划器基线（Decoupled Planner）比对接限速更激进，规划器贡献显著；本仓库单段五次基线本身慢于对接限速，故规划器贡献在时长上不显（SE(3)-TOPP 对两段式五次为 -39% 时长，见上）。

## ➕ 论文之外的扩展

| 功能 | 位置 |
|---|---|
| SE(3) Lie 群标称阻抗（Kim et al. 2025 §III-A） | `control/se3_impedance.py`、`control/lie_se3.py`；完整 `log6`/`dexp`/惯量重塑链路 |
| SE(3) body 运动参考适配 | `planning/motion_reference.py`；SE(3)-TOPP 透传，纯位置轨迹结合固定姿态补齐 |
| sensor-site → EE-body wrench 变换 | `wrench.py`；同时处理坐标旋转和参考点平移矩 |
| 力矩方向摩擦前馈（零速死区补偿，`friction_comp: torque`） | `control/task_space.py`、`control/hqp_ac.py`；FR3 对接横向偏差 5.19→2.10 mm |
| 关节耗散（frictionloss/damping）前馈 + 工具惯量对齐 | `load_pin_model`、场景 `pin_inertia`；质量矩阵一致性 <1e-10 |
| 跟踪测试轨迹（圆+8字，C2 平滑）与自由空间门禁 | `CircleFigure8Trajectory`、`TrackingThresholds`；iiwa14 0.23 mm / FR3 1.31 mm |
| 框架对比研究自动化 | `experiments/compare_frameworks.py` |
| SciencePlots IEEE 中文绘图体系 | `plotting.py`；`figure/<场景>/` 归档 |

## 尚未复现或尚缺端到端验证

1. **自由漂浮基座**（§2 核心建模差异）——浮基动力学耦合、动量守恒、基座扰动补偿均需重构；
2. **莲花型接口几何**——以自研花冠 SDF 接口等效替代；
3. **时变姿态场景的端到端验证**——规划器、运动参考适配器与 `se3_lie`
   控制接口已经接线；内置场景仍为恒定姿态，尚未建立旋转对接的数值基线。
