# 主仿真理论与数据流

本文档梳理柔顺对接主仿真（`experiments/run_docking.py`）的控制理论基础与
端到端数据流。整体架构见 [架构总览](../architecture.md)；与论文公式的逐条
对应见 [论文-代码对照](../paper_mapping.md)。

## 端到端数据流

每个控制周期（1 ms）依次执行：

1. **轨迹采样**：规划器输出期望末端位置/速度/加速度 `(pos_des, vel_des, acc_des)`；
2. **任务空间状态估计**：Pinocchio 计算末端位姿、LOCAL_WORLD_ALIGNED 雅可比、线速度；
3. **控制律**：期望任务加速度 `u` → 动力学一致映射 → 关节力矩 `τ`（限幅 ±10 N·m）；
4. **物理步进**：MuJoCo `step(τ)` 返回 `(q, v, eef_pos)`；
5. **传感**：F/T 传感器取负号后经当前姿态旋转到世界系；
6. **记录**：`telemetry.Log` 存储时序（q/v 为副本，规避 MuJoCo 视图陷阱）。

## 理论基础

### 1) 动力学与运动学

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau + J^T F_{ext},
\qquad
\ddot{x} = J\ddot{q} + \dot{J}\dot{q}
$$

本仓库为空间场景，**重力双方置零**（Pinocchio 侧由 `load_pin_model` 统一处理）。

### 2) 平动阻抗（u_pos）

$$
u_{pos} = \ddot{x}_d + M_d^{-1}\left(F_{ext} - D_d(\dot{x}-\dot{x}_d) - K_d(x-x_d)\right)
$$

- 参数（`ImpedanceConfig`，场景 `impedance:` 段可覆盖）：默认 `m=10, d=80, k=50`；
  FR3 摩擦场景覆盖为 `k=300/500, d=100/140`；
- 摩擦 I 项：摩擦非零时自动启用（`k_i=150`，±10 N 限幅，接触后冻结）；
- 雅可比参考系为 **LOCAL_WORLD_ALIGNED**（末端 frame 原点的世界轴对齐量）。

### 3) 姿态阻抗（u_rot）

- 姿态误差李代数映射：$e_R = \mathrm{log}(R_d R^T)$；
- 角速度误差 $e_\omega = -\omega$（参考角速度为零）；
- $u_{rot} = M_R^{-1}(K_R e_R + D_R e_\omega)$，参数 `m_rot=1, d_rot=10, k_rot=25`。

### 4) 操作空间映射与力矩合成（Khatib 形式）

$$
\Lambda = (J M^{-1} J^T)^{-1},\qquad
\tau = J^T\Lambda\,(u - \dot{J}\dot{q}) + C\dot{q} + \tau_{null} + \tau_{ff}
$$

- 主任务项使任务加速度精确等于指令 $u$；
- 零空间阻尼 $-N^T D_{null}\dot{q}$，$N = I - J\#J$，`null_damping=10`；
- $\tau_{ff}$：摩擦前馈 + 关节阻尼前馈（见下）。

### 5) 关节耗散前馈

- **velocity 模式**：$\tau_{ff} = f\cdot\tanh(\dot q / 0.01)$——运动中精确，零速补偿消失；
- **torque 模式**（默认）：$\tau_{ff} = f\cdot\tanh(\tau_{pre}/(f/2))$——以补偿前力矩方向
  决定摩擦方向，治零速死区粘滑；
- 阻尼前馈 $\tau_{damp} = d\cdot\dot q$（精确线性补偿）；
- `frictionloss`/`damping` 由组装 MjModel 读取，与仿真严格同源。

### 6) 逆运动学（初始化）

阻尼最小二乘迭代 $\Delta q = J^T(JJ^T + \lambda I)^{-1}e$，位置误差叠加姿态
log 误差；`damp=1e-8`，`eps=1e-7`。

### 7) 轨迹规划

| 规划器 | 说明 |
|---|---|
| `DecoupledQuinticTrajectory` | 三轴解耦五次多项式，rest-to-rest，边界速度/加速度为零 |
| `TwoPhaseDockingTrajectory` | 两段五次：接近段宽松限速 + 预对接点 + 对接段严格限速（论文的保守简化） |
| `SE3ToppTrajectory` | SE(3) 测地线 + 解析时间最优剖面（论文 §3.1 本体） |
| `CircleFigure8Trajectory` | 圆+8字跟踪测试（C2 平滑，无接触） |

## 坐标系与单位约定

- **传感器变换**：$F^{ctrl} = R F_{sensor}$（读取时取负号，约定见
  `run_docking.run_simulation` 注释）；
- **雅可比参考系**：`LOCAL_WORLD_ALIGNED`（frame 原点的世界轴对齐量——WORLD
  参考系的线速度块语义为绕世界原点的空间运动，不适合末端点）；
- **末端 frame**：iiwa14 为 `cylinder_link`（URDF 组合模型，含工具惯量）、
  FR3 为 `attachment_site`（MJCF 直读 + 场景声明 `pin_inertia` 工具惯量）；
- **单位**：SI（m/s/rad/N/N·m），门禁阈值以 m/rad 声明。

## 历史改进备忘

旧版文档（2025-11）列出的改进项现状：参数配置化 ✅（`ImpedanceConfig` + 场景
覆盖）、力矩限幅 ✅（`max_torque` + HQP 硬约束）、旋转限幅 ✅（已随 Khatib
重写移除硬编码截断）、关节限位保护 ✅（HQP QP 硬约束）、重力补偿 ✅（场景置
零语义明确）、雅可比参考系 ✅（WORLD → LOCAL_WORLD_ALIGNED 修正）。
