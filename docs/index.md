# 柔顺对接仿真

一个面向研究复现与控制算法验证的 7 自由度机械臂柔顺对接平台。项目以
**MuJoCo** 提供接触物理、以 **Pinocchio** 提供运动学与刚体动力学，在同一套
场景、遥测和评价管线中比较经典任务空间阻抗、SE(3) Lie 群阻抗与 HQP-AC。

项目复现并扩展了 Ren & Shan 2026（Acta Astronautica）的统一柔顺控制与轨迹
规划框架，并实现 Kim et al. 2025（IEEE T-RO）的标称 SE(3) Lie 群阻抗控制器。

## 核心能力

- **双机械臂场景**：KUKA iiwa14（零摩擦数值锚点）与 Franka FR3（真实减速器摩擦）；
- **三类控制器**：经典任务空间阻抗（CIC）、SE(3) Lie 群阻抗（完整
  `log6`/`dexp`/body-wrench 链路）与 HQP-AC（关节硬约束、自适应刚度、奇异性规避）；
- **SE(3)-TOPP 规划器**：常螺旋测地线 + 解析时间最优剖面，并提供按步
  `(T_d, V_d, Vdot_d)` body 运动参考；另含两段式五次与圆+8字跟踪轨迹；
- **无传感器方案**：PI 广义动量观测器（Eq.23–25）替代 F/T 传感器，含接触预紧力跟踪；
- **一致的力与耗散建模**：传感器 wrench 的坐标系/参考点变换、力矩方向摩擦
  前馈与关节阻尼前馈；
- **可复现实验**：自由空间跟踪门禁、Table 10 三层指标、框架对比研究，以及
  SE(3) 惯量重塑/大角度/零刚度验证。

## 快速开始

```bash
uv sync --dev
uv run docking --quick                          # 2s 冒烟（INCOMPLETE，仅链路检查）
uv run docking --controller se3_lie --quick     # SE(3) Lie 控制链路
uv run pytest -q                                # 快速测试套件
uv run pytest -q -m slow                        # 慢速接触回归（数值锚点）
uv run docking --scene scenes/fr3_docking.yaml  # FR3 对接
MUJOCO_GL=egl uv run python experiments/compare_frameworks.py  # 框架对比
```

## 文档导览

| 页面 | 内容 |
|---|---|
| [架构总览](architecture.md) | 模块分层、数据流与关键契约 |
| [论文-代码对照](paper_mapping.md) | Ren & Shan 2026 公式/表格 → 代码位置与复现状态 |
| [阻抗控制基础](theory/impedance_control.md) | 关节/笛卡尔阻抗控制理论 |
| [SE(3) Lie 群阻抗](theory/se3_lie_impedance.md) | 群上误差、`dexp`、body wrench 与惯量重塑 |
| [主仿真理论与数据流](theory/simulation_flow.md) | 控制律公式、端到端数据流、坐标系约定 |
| [实验复现手册](experiments.md) | 对接、跟踪、SE(3) 自由空间实验与数值基线 |
| [API 参考](api/control.md) | 控制器、Lie 工具、规划适配器与核心模块 |

## 运行环境

Python 3.12 · MuJoCo ≥ 3.2 · Pinocchio · uv 管理；无显示环境渲染走 EGL（`MUJOCO_GL=egl`）。
