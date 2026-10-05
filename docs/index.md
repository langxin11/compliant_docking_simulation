# 柔顺对接仿真

七自由度机械臂的固定基座、零重力柔顺对接仿真研究仓库。MuJoCo 提供独立物理世界，
Pinocchio 提供运动学和控制侧刚体动力学。

| 从哪里开始 | 研究问题与交付 |
|---|---|
| [模型与接口](models_interfaces.md) | 固定模型、坐标/单位、接触条件、同源检查与接口候选 |
| [柔顺控制算法研究](control_research.md) | RQ1 绕轴释放、RQ2 横向释放、同点矩阵、失败与数值复核 |
| [完整对接/装配](system_validation.md) | 全流程验收、交接与同一记录的 Demo 回放 |

先阅读[当前主线与收束决策](research_focus.md)、[研究范围](research_scope.md)，再选择实验入口（见[实验复现手册](experiments.md)）。
HexFrame 当前使用独立关节伺服与接触导纳，尚未集成 SE(3) 研究策略。
模型检查、落座候选和系统通过分别使用其声明范围的证据。

## 快速开始

```bash
uv sync --frozen --dev
uv run docking --quick
uv run pytest -m 'not slow' -q
uv run mkdocs build --strict
```

`--quick` 是短时链路检查。完整 53 s 系统验收与视频回放命令见系统页。
无显示渲染使用 EGL；Python 项目最低版本 3.10，CI 使用 3.12。

## 继续阅读

[架构](architecture.md)、[实验复现](experiments.md)、[历史证据](historical_evidence.md)、
[论文对照](paper_mapping.md)、[SE(3) 理论](theory/se3_lie_impedance.md)、[API](api/control.md)。
当前完善任务见[路线图](development_plan.md)。
