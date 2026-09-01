# run_docking.py（柔顺对接主仿真）的理论基础与数据流

本文档总结该程序的控制理论基础，并梳理从轨迹规划、控制计算到物理仿真的端到端数据流。附带关键代码锚点，便于交叉定位。

## 系统组成与数据流

- **轨迹规划**：使用解耦五次多项式在任务空间生成期望位置、速度、加速度 `(pos_des, vel_des, acc_des)`。

  - 代码：`DecoupledQuinticTrajectory` 类（compliant_docking/planning/trajectory.py）
  - 关键方法：`get_state()` (compliant_docking/planning/trajectory.py)
- **任务空间状态估计**：用 Pinocchio 计算末端位姿和雅可比，得到当前位置、速度以及姿态误差（李代数对数映射），并计算角速度。

  - 代码：`get_task_space_state_with_orientation()` (compliant_docking/control/task_space.py)
  - 姿态误差计算：`pin.log3(self.initial_orientation @ current_rot.T)` (compliant_docking/control/task_space.py)
- **任务空间阻抗 + 姿态控制**：综合轨迹、当前状态与外力，给出任务空间期望加速度向量 `u = [u_pos; u_rot]`，再经动力学一致的映射得到关节力矩 `tau`；加入零空间阻尼抑制漂移。

  - 主控制器：`compute_control_task_space_with_orientation_and_imp()` (compliant_docking/control/task_space.py)
  - 平动阻抗计算：compliant_docking/control/task_space.py
  - 姿态阻抗计算：compliant_docking/control/task_space.py
  - 动力学一致映射矩阵 `lambda_`：compliant_docking/control/task_space.py
  - 零空间阻尼：compliant_docking/control/task_space.py
  - 关节力矩合成：compliant_docking/control/task_space.py
- **物理仿真**：MuJoCo 写入 `tau` 并步进，返回最新关节位置/速度与末端位置；可视化与录帧在此阶段进行。

  - 步进方法：`MujRobot.step()` (compliant_docking/simulation/mujoco_env.py)
  - 录帧逻辑：compliant_docking/simulation/mujoco_env.py
- **外力测量**：从 MuJoCo 读取力/力矩传感器数据，并通过当前姿态矩阵变换到控制使用的参考系，回馈给控制器中的阻抗外力项。

  - 传感器读取：experiments/run_docking.py
  - 坐标变换：experiments/run_docking.py
- **记录与可视化**：按时间记录关节、末端、控制量、外力等，结束后出图与导出视频。

  - 数据记录：experiments/run_docking.py
  - 绘图输出：experiments/run_docking.py
  - 视频导出：experiments/run_docking.py

## 理论基础（含公式）

### 1) 机器人动力学与任务空间运动学

- 关节空间动力学（本文控制实现中将重力项在控制器侧置零处理，可视 XML/仿真环境实际重力）
  $$
  M(q)\,\ddot{q} + C(q,\dot{q}) + g(q) = \tau + J^T(q)\,F_{ext}
  $$
- 任务空间运动学
  $$
  x = f(q),\quad \dot{x} = J(q)\,\dot{q},\quad \ddot{x} = J(q)\,\ddot{q} + \dot{J}(q,\dot{q})\,\dot{q}
  $$

### 2) 任务空间阻抗控制（平动）

- 期望的末端阻抗关系（对三轴平动）：

  $$
  M_d\,(\ddot{x} - \ddot{x}_d) + D_d\,(\dot{x} - \dot{x}_d) + K_d\,(x - x_d) = F_{ext} - F_{des}
  $$
- 由此可得用于控制的期望加速度项（代码中的 `u_pos`）：

  $$
  u_{pos} = \ddot{x}_d + M_d^{-1}\big( F_{ext} - F_{des} - D_d(\dot{x}-\dot{x}_d) - K_d(x-x_d) \big)
  $$

  - 实现对应：compliant_docking/control/task_space.py
  - 其中参数设置：`m=10` (虚拟质量), `d=50` (虚拟阻尼), `k=100` (虚拟刚度)
  - 期望外力 `force_desired` 设为零向量

### 3) 姿态误差与角速度阻尼（旋转阻抗）

- 姿态误差采用李代数对数映射（so(3)）：

  $$
  e_R = \mathrm{log}\big(R_d R^T\big) \in \mathbb{R}^3
  $$

  - 实现：compliant_docking/control/task_space.py (`pin.log3(self.initial_orientation @ current_rot.T)`)
  - 期望姿态 `initial_orientation` 定义在 compliant_docking/control/task_space.py
- 角速度误差（本实现默认 $\dot{R}_d=0$）：

  $$
  e_\omega = \omega_d - \omega \approx -\omega
  $$

  - 实现：compliant_docking/control/task_space.py (`vel_rot_err = -vel_rot_cur`)
- 旋转阻抗的期望角加速度项（代码中的 `u_rot`）：

  $$
  u_{rot} = M_{R}^{-1}\big( K_R\,e_R + D_R\,e_\omega \big)
  $$

  - 实现对应：compliant_docking/control/task_space.py
  - 参数设置：`m2=1` (虚拟质量), `d2=10` (虚拟阻尼), `k2=25` (虚拟刚度)
  - 包含对旋转控制量的幅值限制（compliant_docking/control/task_space.py）

### 4) 任务空间到关节力矩的动力学一致映射

- 标准操作空间惯性：

  $$
  \Lambda = (J\,M^{-1}\,J^T)^{-1}
  $$
- 本实现采用加权广义逆的动力学一致映射形式：

  $$
  \lambda = W M^{-T} J^T (J M^{-1} W M^{-T} J^T)^{-1}
  $$

  - 实现：compliant_docking/control/task_space.py
  - 其中权重矩阵 `W = I_7` (compliant_docking/control/task_space.py)
  - 质量矩阵 `M` 通过 `pin.crba()` 计算 (compliant_docking/control/task_space.py)
- 关节力矩合成包含以下项：

  $$
  \tau = \lambda\,\big( u - \dot{J}\,\dot{q} + J\,M^{-1}\,C \big) + N\,\tau_{null}
  $$

  - 主任务项：$\lambda (u - \dot{J} \dot{q} + J M^{-1} C)$ (compliant_docking/control/task_space.py)
  - 其中 `u` 为6维任务空间加速度指令（平动3维+旋转3维）
  - $\dot{J}$ 为雅可比时间导数 (compliant_docking/control/task_space.py)
  - `C` 为科氏/离心项向量 (compliant_docking/control/task_space.py)
  - 零空间阻尼项：$N\tau_{null} = -N D_{null} \dot{q}$ (compliant_docking/control/task_space.py)
  - 零空间投影矩阵：$N = I - \lambda J M^{-1}$ (compliant_docking/control/task_space.py)

### 5) 雅可比时间导数项

- 耦合加速度项 $\dot{J}\,\dot{q}$ 在控制律中显式补偿：
  - 雅可比时间导数计算：compliant_docking/control/task_space.py
  - 在力矩计算中使用：compliant_docking/control/task_space.py

### 6) 零空间控制/阻尼

- 使用投影矩阵 $N$ 抑制零空间速度，提升数值稳定：
  $$
  N = I - \lambda\,J\,M^{-1}, \quad \tau_{null} = -D_{null}\,\dot{q}
  $$

  - 零空间阻尼系数：`D_null = 10 * I_7` (compliant_docking/control/task_space.py)
  - 零空间投影矩阵计算：compliant_docking/control/task_space.py
  - 零空间阻尼项：compliant_docking/control/task_space.py

### 7) 逆运动学（初始化）

- 阻尼最小二乘（damped least squares）迭代：
  $$
  \Delta q = J^T\,(J\,J^T + \lambda I)^{-1}\,e,\quad q \leftarrow \mathrm{Integrate}(q,\,\Delta q)
  $$

  - 位姿误差 `e` 叠加位置与姿态 `log` 误差
  - 函数定义：`compute_ik()` (compliant_docking/planning/kinematics.py)
  - 阻尼因子：`damp = 1e-8` (compliant_docking/planning/kinematics.py)
  - 最大迭代次数：`max_iters = 3000` (默认)
  - 收敛阈值：`eps = 1e-7` (默认)

### 8) 五次多项式轨迹（边界条件满足）

- 三轴解耦，多项式：
  $$
  p(t) = a_0 t^5 + a_1 t^4 + a_2 t^3 + a_3 t^2 + a_4 t + a_5
  $$
- 约束：$p(0)=p_0,\; p(T)=p_f,\; \dot{p}(0)=\dot{p}(T)=0,\; \ddot{p}(0)=\ddot{p}(T)=0$；线性方程组 $A a = b$ 解得系数。
  - 类定义：`DecoupledQuinticTrajectory` (compliant_docking/planning/trajectory.py)
  - 系数矩阵构建：`_solve_quintic_coefficients()` (compliant_docking/planning/trajectory.py)
  - 状态采样：`get_state()` (compliant_docking/planning/trajectory.py)

## 关键实现锚点

- **轨迹规划类**：`DecoupledQuinticTrajectory` (compliant_docking/planning/trajectory.py)

  - 系数求解：`_solve_quintic_coefficients()` (compliant_docking/planning/trajectory.py)
  - 状态采样：`get_state()` (compliant_docking/planning/trajectory.py)
- **任务空间控制器**：`TaskSpaceController` (compliant_docking/control/task_space.py)

  - 初始化：`__init__()` (compliant_docking/control/task_space.py)
  - 任务空间状态估计：`get_task_space_state_with_orientation()` (compliant_docking/control/task_space.py)
  - 主控制函数：`compute_control_task_space_with_orientation_and_imp()` (compliant_docking/control/task_space.py)
  - 平动阻抗计算：compliant_docking/control/task_space.py
  - 姿态阻抗计算：compliant_docking/control/task_space.py
  - 动力学一致映射：compliant_docking/control/task_space.py
  - 零空间阻尼：compliant_docking/control/task_space.py
  - 力矩合成：compliant_docking/control/task_space.py
- **逆运动学求解**：`compute_ik()` (compliant_docking/planning/kinematics.py)
- **MuJoCo仿真包装**：`MujRobot` (compliant_docking/simulation/mujoco_env.py)

  - 仿真步进：`step()` (compliant_docking/simulation/mujoco_env.py)
  - 离屏渲染设置：`setup_renderer()` (compliant_docking/simulation/mujoco_env.py)
- **主仿真循环**：`run_simulation()` (experiments/run_docking.py)

  - 传感器读取：experiments/run_docking.py
  - 坐标变换：experiments/run_docking.py
  - 数据记录：experiments/run_docking.py
  - 结果绘图：experiments/run_docking.py
  - 视频导出：experiments/run_docking.py

## 坐标系与单位

- **传感器坐标系变换**：传感器数据需通过当前末端旋转矩阵变换到控制参考系

  $$
  F_{ext}^{ctrl} = R\,F_{sensor},\quad T_{ext}^{ctrl} = R\,T_{sensor}
  $$

  - 实现位置：experiments/run_docking.py
  - 传感器数据读取时取负号：experiments/run_docking.py
  - `R` 为当前末端姿态矩阵（通过 `get_task_space_state()`获取）
- **重力处理**：

  - Pinocchio 模型的重力置零统一由 `load_pin_model()` 在加载时完成：`model.gravity.linear = np.array([0., 0., 0.])` (compliant_docking/models.py)
  - MuJoCo XML中的重力设置独立于控制器
  - 若需要显式重力补偿，可在控制律中添加 `g(q)` 项
- **末端执行器帧**：

  - 帧名称：`"cylinder_link"` (compliant_docking/control/task_space.py)
  - 雅可比参考系：`pin.ReferenceFrame.WORLD` (compliant_docking/control/task_space.py)

## 建议与可能改进

- **参数配置化**：

  - 将阻抗参数 `m,d,k`（平动）和 `m2,d2,k2`（旋转）暴露为配置参数或构造函数参数
  - 当前硬编码位置：compliant_docking/control/task_space.py（平动与旋转阻抗参数）
  - 建议添加配置文件或参数字典以便快速调参
- **坐标系一致性检查**：

  - 确认传感器坐标系定义与旋转变换方向的一致性
  - 验证传感器读数的符号约定（当前取负号：experiments/run_docking.py）
  - 建议添加坐标系可视化辅助调试
- **控制器增强**：

  - 考虑添加关节限位保护（当前仅在IK中使用：compliant_docking/planning/kinematics.py）
  - 可添加力矩限幅以提高安全性
  - 旋转控制量当前有简单限幅（compliant_docking/control/task_space.py），可优化为更系统的处理
- **录帧性能优化**：

  - 当前每20步录一帧（compliant_docking/simulation/mujoco_env.py）
  - 可将采样率作为参数配置
  - 考虑使用异步录帧避免阻塞仿真
- **重力补偿**：

  - 若MuJoCo启用重力但控制器未补偿，可能产生稳态误差
  - 可在控制律中添加 `g(q)` 项：`tau += pin.computeGeneralizedGravity(model, data, q)`

---

本文档对应代码版本：v0.2.0 包结构（主要文件：experiments/run_docking.py、compliant_docking/planning/、compliant_docking/control/task_space.py、compliant_docking/simulation/mujoco_env.py、compliant_docking/telemetry.py）

**文档更新日期**：2025年11月19日**主要更新内容**：

- 根据实际代码实现更新所有代码行号引用
- 补充详细的参数设置说明（阻抗参数、零空间阻尼等）
- 明确传感器坐标系变换细节
- 更新关键实现锚点，精确定位到函数和类
- 增强建议与改进部分，提供更具体的优化方向
