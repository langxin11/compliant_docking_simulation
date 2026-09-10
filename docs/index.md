# 柔顺对接仿真

基于 **MuJoCo**（物理）与 **Pinocchio**（动力学）的 7 自由度机械臂柔顺对接仿真，复现并扩展
Ren & Shan 2026（Acta Astronautica）的统一柔顺控制与轨迹规划框架。

## 核心能力

- **双机械臂**：KUKA iiwa14（零摩擦锚点场景）与 Franka FR3（上游真实减速器摩擦）；
- **HQP-AC 控制器**：关节位置/速度/力矩硬约束 QP + 接触力自适应刚度 + 零空间奇异性规避（论文 §3.2）；
- **SE(3)-TOPP 规划器**：常螺旋测地线 + 解析时间最优剖面（论文 §3.1），以及保守版两段式五次与跟踪测试轨迹；
- **无传感器方案**：PI 广义动量观测器（Eq.23–25）替代 F/T 传感器，含接触预紧力跟踪；
- **力矩方向摩擦前馈**：零速死区补偿（FR3 真实摩擦下的粘滑抑制）；
- **工程化门禁**：自由空间跟踪门禁（PASS/FAIL/退出码）与 Table 10 三层指标、框架对比研究。

## 快速开始

```bash
uv sync --dev
uv run docking --quick                          # 2s 冒烟（INCOMPLETE，仅链路检查）
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
| [主仿真理论与数据流](theory/simulation_flow.md) | 控制律公式、端到端数据流、坐标系约定 |
| [实验复现手册](experiments.md) | 对接/跟踪/门禁/对比研究的运行方式与基线数值 |
| [API 参考](api/control.md) | 控制器、规划器与核心模块的自动生成参考 |

## 运行环境

Python 3.12 · MuJoCo ≥ 3.2 · Pinocchio · uv 管理；无显示环境渲染走 EGL（`MUJOCO_GL=egl`）。
