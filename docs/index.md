# 柔顺对接仿真

七自由度机械臂的固定基座、零重力柔顺对接仿真仓库。MuJoCo 提供独立物理世界，
Pinocchio 提供运动学和控制侧刚体动力学。本项目**以论文复现为基础，并在此基础上
适当拓展**；演化历史见[项目演化](project_evolution.md)。

## 项目定位

- **复现基础**：Ren & Shan（2026, Acta Astronautica）在轨装配的规划—柔顺控制
  统一框架（SE(3)-TOPP 规划器 + HQP-AC 控制器），以及 Kim et al.（2025,
  IEEE T-RO）的 SE(3) 阻抗控制（§III-A 标称阻抗）；复现状态逐条对照见
  [论文对照](paper_mapping.md)。
- **适当拓展**：PetalDock / angle1 几何导向接口与交互调形工具
  （[接口交互调形](petal_designer.md)）、绕轴释放与横向释放两组小型接触机制实验
  （[接触阶段控制实验](control_research.md)）、HexFrame 完整装配演示
  （[系统验证](system_validation.md)）。

## 三层组织

| 从哪里开始 | 定位与内容 |
|---|---|
| [模型与接口](models_interfaces.md) | 仿真平台基线：固定模型、坐标/单位、接触条件、同源检查与接口候选 |
| [规划与柔顺控制](paper_mapping.md) | 论文复现核心（规划×柔顺统一、SE(3) 阻抗）与[接触阶段控制实验](control_research.md) |
| [完整装配系统](system_validation.md) | HexFrame 完整装配演示：全流程验收、交接与同一记录的回放 |

三层是组织与导航骨架，不是开发顺序；职责、证据使用规则与有效域见
[研究组织与范围](research_scope.md)。HexFrame 当前使用独立关节伺服与接触导纳，
尚未集成 SE(3) 研究策略；petal_designer 是接口几何的交互调形工具，不是实验入口。

## 建议阅读顺序

```text
论文对照 → 项目演化 → 研究组织与范围
→ 阻抗控制基础 → SE(3) 阻抗 → 接触阶段控制实验
→ 接触刚度实验结果 → HexFrame 系统验证
```

按顺序读可以不看历史细节先建立全貌；只查复现命令直接进
[实验复现手册](experiments.md)。哈希、日期、失败协议等二级证据集中在
[历史证据索引](historical_evidence.md)与 `results/`，不在导航页展开。

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
[SE(3) 理论](theory/se3_lie_impedance.md)、[API](api/control.md)。
当前完善任务见[路线图](development_plan.md)。
