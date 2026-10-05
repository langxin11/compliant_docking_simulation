# 柔顺对接仿真

七自由度机械臂的固定基座、零重力柔顺对接仿真研究仓库。MuJoCo 提供独立物理世界，
Pinocchio 提供运动学和控制侧刚体动力学。项目从论文复现起步、逐步长成柔顺对接研究
平台，这段历史见[项目演化](project_evolution.md)。

## 当前研究主线

> 在声明的定位误差与固定模型条件下，柔顺策略能否改善对接完成、最终误差和
> 任务效率，其失败边界是什么，以及如何接入完整装配？

这一主线的机制表述——接触以后各自由度的柔顺如何按接口几何分配
（geometry-informed selective compliance）——来自收束期间的讨论与文档整理，
不是项目立项时的研究目标；问题定义见[研究主线](research_focus.md)，理论依据
（RCC / 主动刚度 / 混合位置力控制 / SE(3) 阻抗）见
[选择性柔顺](theory/selective_compliance.md)。

## 三层研究

| 从哪里开始 | 研究问题与交付 |
|---|---|
| [模型与接口](models_interfaces.md) | 固定模型、坐标/单位、接触条件、同源检查与接口候选 |
| [对接方法与接触柔顺](control_research.md) | 方法库（规划×柔顺统一、SE(3) 阻抗）与 RQ1/RQ2 柔顺分配机制对照 |
| [完整对接/装配](system_validation.md) | 全流程验收、交接与同一记录的 Demo 回放 |

HexFrame 当前使用独立关节伺服与接触导纳，尚未集成 SE(3) 研究策略。
模型检查、落座候选和系统通过分别使用其声明范围的证据，三层职责见
[研究范围](research_scope.md)。

## 建议阅读顺序

```text
项目演化 → 当前研究问题 → 研究范围与三层证据
→ 阻抗控制基础 → SE(3) 阻抗 → 选择性柔顺
→ RQ1/RQ2 协议 → 控制主结果 → HexFrame 系统验证
```

按顺序读可以不看历史细节先建立主线；只查复现命令直接进
[实验复现手册](experiments.md)。哈希、日期、失败协议等二级证据集中在
[历史证据索引](historical_evidence.md)与 `results/`，不在主线页面展开。

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

[项目演化](project_evolution.md)、[架构](architecture.md)、[实验复现](experiments.md)、
[历史证据](historical_evidence.md)、[论文对照](paper_mapping.md)、
[SE(3) 理论](theory/se3_lie_impedance.md)、[选择性柔顺](theory/selective_compliance.md)、
[API](api/control.md)。当前完善任务见[路线图](development_plan.md)。
