# 主仿真理论与数据流

本文档梳理柔顺对接主仿真（`experiments/run_docking.py`）的控制理论基础与
端到端数据流。整体架构见 [架构总览](../architecture.md)；与论文公式的逐条
对应见 [论文-代码对照](../paper_mapping.md)。

## 端到端数据流

每个控制周期（1 ms）依次执行：

1. **轨迹采样**：所有规划器输出世界系线量 `(pos_des, vel_des, acc_des)`；
   `se3_lie` 路径额外经 `get_motion_reference` 得到 body 量 `(T_d, V_d, Vdot_d)`；
2. **任务空间状态估计**：经典/HQP 路径使用 LOCAL_WORLD_ALIGNED 雅可比；
   `se3_lie` 使用 LOCAL body Jacobian 与 `Jdot`；
3. **控制律**：所选控制器计算关节力矩 `τ`（统一在主循环限幅至 ±10 N·m）；
4. **物理步进**：MuJoCo `step(τ)` 返回 `(q, v, eef_pos)`；
5. **传感**：F/T 传感器保留既有取负号约定；经典/HQP 路径旋转到世界轴，
   `se3_lie` 路径同时旋转并移矩到 EE body 原点；
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

### 3) 经典/HQP 路径的姿态阻抗（u_rot）

- 姿态误差李代数映射：$e_R = \mathrm{log}(R_d R^T)$；
- 角速度误差 $e_\omega = -\omega$（参考角速度为零）；
- $u_{rot} = M_R^{-1}(K_R e_R + D_R e_\omega)$，参数 `m_rot=1, d_rot=10, k_rot=25`。

这里的参考角速度为零是经典任务空间阻抗基线的约定。SE(3) Lie 路径不使用
该方程，而是接收按步 `T_d`、body twist `V_d` 及其导数 `Vdot_d`；完整推导见
[SE(3) Lie 群阻抗控制器](se3_lie_impedance.md)。

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

### 6) SE(3) Lie 路径：body motion 与 body wrench

规划器适配层输出：

$$
(T_d, V_d, \dot V_d) = \operatorname{get\_motion\_reference}(planner,t,R_d).
$$

- `SE3ToppTrajectory` 的常螺旋段直接满足 $V_d=\xi\dot s$、
  $\dot V_d=\xi\ddot s$；
- 纯位置轨迹使用固定 $R_d$，将世界系线速度/加速度左乘 $R_d^T$ 转入
  desired body frame，角量置零；
- 传感器 site wrench 通过 `wrench_to_body` 变换到 EE body frame 与 EE 原点，
  矩变换包含 $(p_S-p_E)\times f$，随后与 LOCAL Jacobian 配对。

### 7) 逆运动学（初始化）

阻尼最小二乘迭代 $\Delta q = J^T(JJ^T + \lambda I)^{-1}e$，位置误差叠加姿态
log 误差；`damp=1e-8`，`eps=1e-7`。

### 8) 轨迹规划

| 规划器 | 说明 |
|---|---|
| `DecoupledQuinticTrajectory` | 三轴解耦五次多项式，rest-to-rest，边界速度/加速度为零 |
| `TwoPhaseDockingTrajectory` | 两段五次：接近段宽松限速 + 预对接点 + 对接段严格限速（论文的保守简化） |
| `SE3ToppTrajectory` | SE(3) 测地线 + 解析时间最优剖面；同时提供 `get_state` 与 body `get_motion_state`（论文 §3.1 本体） |
| `CircleFigure8Trajectory` | 圆+8字跟踪测试（C2 平滑，无接触） |

## 坐标系与单位约定

- **经典/HQP 传感器变换**：$F^{ctrl} = R F_{sensor}$（读取时取负号，约定见
  `run_docking.run_simulation` 注释）；
- **SE(3) 传感器变换**：sensor-site wrench → EE-body wrench，既改变表达坐标系，
  也改变力矩参考点；
- **雅可比参考系**：经典/HQP 使用 `LOCAL_WORLD_ALIGNED`（frame 原点的世界轴
  对齐量）；SE(3) 控制律使用 `LOCAL`，并与 body twist/wrench 严格配对；
- **末端 frame**：iiwa14 为 `cylinder_link`（URDF 组合模型，含工具惯量）、
  FR3 为 `attachment_site`（MJCF 直读 + 场景声明 `pin_inertia` 工具惯量）；
- **单位**：SI（m/s/rad/N/N·m），门禁阈值以 m/rad 声明。

## 历史改进备忘

旧版文档（2025-11）列出的改进项现状：参数配置化 ✅（`ImpedanceConfig` + 场景
覆盖）、力矩限幅 ✅（`max_torque` + HQP 硬约束）、旋转限幅 ✅（已随 Khatib
重写移除硬编码截断）、关节限位保护 ✅（HQP QP 硬约束）、重力补偿 ✅（场景置
零语义明确）、雅可比参考系 ✅（WORLD → LOCAL_WORLD_ALIGNED 修正）。
