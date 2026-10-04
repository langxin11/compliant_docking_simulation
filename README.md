# Compliant Docking Simulation

[![CI](https://github.com/langxin11/compliant_docking_simulation/actions/workflows/ci.yml/badge.svg)](https://github.com/langxin11/compliant_docking_simulation/actions/workflows/ci.yml)
![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

七自由度机械臂柔顺对接仿真，按 **模型与接口 → 柔顺控制算法研究 → 完整对接/装配** 组织。
MuJoCo 提供物理世界，Pinocchio 提供运动学与控制侧刚体动力学；内置 KUKA iiwa14 与 Franka FR3。

| 层次 | 回答的问题 | 入口与说明 |
|---|---|---|
| 模型与接口 | 研究对象、坐标、接触条件是否明确且一致？ | [模型基线](docs/models_interfaces.md)、`experiments/models_interfaces/` |
| 柔顺控制算法研究 | 固定接口下，绕轴或横向释放改变了什么？ | [RQ1/RQ2 协议](docs/control_research.md)、`experiments/control/` |
| 完整对接/装配 | 规划、控制、状态切换与交接能否完成任务？ | [系统验收](docs/system_validation.md)、`experiments/system/` |

当前主研究限定固定基座、固定目标、零重力与刚性模型。单接口研究使用原版 PetalDock100 V2，
候选 `angle1_blend030` 独立保留，尚未晋升。HexFrame 使用 **MuJoCo 偏置补偿关节伺服与接触导纳**，
尚未集成单接口研究的 SE(3) 阻抗链路；其通过记录是当前系统配置的证据。
[假设、依赖与证据边界](docs/research_scope.md)。

## 快速开始

```bash
git clone https://github.com/langxin11/compliant_docking_simulation.git
cd compliant_docking_simulation
uv sync --frozen --dev
uv run docking --quick  # 2 s 链路检查，不代表完整验收
uv run pytest -m 'not slow' -q
```

推荐 [uv](https://docs.astral.sh/uv/)。标准虚拟环境也可使用 `python -m pip install -e .`。
无显示环境设 `MUJOCO_GL=egl`；Ubuntu 需提供 `libegl1`、`libegl-dev`。

## 按问题运行

```bash
# 模型与接口：固定基线的双引擎、关节与初始接触检查
uv run python -m experiments.models_interfaces.baseline --out runs/model_baseline_my_run

# RQ1：固定接口，仅比较绕轴策略；每次使用新目录
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.control.rq1_yaw \
  --case combined --setting baseline --out runs/rq1_my_run

# RQ2：固定绕轴释放，同点比较保持/释放 XY；不混入减速试验
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.control.rq2_lateral \
  --stage paired --out runs/rq2_my_run --jobs 3

# 系统：完整 53 s 装配 + 独立审计
OPENBLAS_NUM_THREADS=1 MUJOCO_GL=egl uv run python -m experiments.system.hexframe \
  accept --out runs/hexframe_my_run

# 从同一验收记录生成视频
MUJOCO_GL=egl uv run python -m experiments.system.hexframe \
  replay --out runs/hexframe_my_run --record
```

模型检查、计算完成、落座候选、系统验收与视频生成有不同含义。研究结论读取 JSON `assessment`；
系统完整验收要求 `validation.json` 和独立 `audit.json` 均通过。预检为 `INCOMPLETE`。
[入口、退出语义与数据策略](docs/experiment_entrypoints.md)。旧命令继续兼容，历史报告按原协议解释。

## 已有能力与证据

- 控制方法库：经典任务空间阻抗、SE(3) Lie 群阻抗与 HQP-AC。
- 规划与反馈：SE(3)-TOPP、路点轨迹、F/T 坐标变换、动量观测器与固定周期采样。
- 数学验证：有限差分 oracle、功率守恒、小角度/近 π 分支和双引擎一致性。
- 控制历史：绕轴释放与横向释放的同点对照；保留卡滞与数值敏感工况。
- 系统验证：HexFrame 抓取、转运、接触就位、卸力交接和撤离；P0/P1/P2 独立扩展。

[历史证据索引](docs/historical_evidence.md)区分方法实现、复现结果与项目实验发现。
不同矩阵的通过数量不合成统一成功率曲线。SE(3) 实现为标称控制器，未包含 NRIC 鲁棒内环。

## 演示

| iiwa14 经典阻抗 | FR3 摩擦场景 | iiwa14 SE(3) Lie |
|---|---|---|
| [![iiwa14](demo/docking_preview.gif)](demo/docking.mp4) | [![FR3](demo/fr3_docking_preview.jpg)](demo/fr3_docking.mp4) | [![SE3](demo/se3_lie_docking_preview.jpg)](demo/se3_lie_docking.mp4) |

上述视频为各自历史场景演示。自由空间验证图覆盖惯量重塑、179° 姿态与零刚度方向：

![SE(3) 自由空间验证](demo/se3_free_space_validation.png)

## 代码与贡献

```text
assets/                          唯一来源的模型资产、许可与导入清单
  modules/hexframe/              HexFrame 原始模型、CAD、网格与来源记录
scenes/                          明确物理与控制配置的场景
experiments/models_interfaces/   模型资格检查、接口候选与几何编排
experiments/control/             RQ1/RQ2、误差范围与自由空间验证
experiments/system/              正式验收、预检、回放与扩展验证
src/compliant_docking/           共享模型、控制、规划、仿真、研究记录与系统实现
tests/                           数学、配置、来源保护与行为回归
docs/                            研究协议、架构、证据与开发说明
  reports/                       本地研究报告草稿（不参与文档站点发布）
results/                         历史报告与冻结证据；不续写旧结果
  historical/                    从旧实验与运行目录迁入的冻结证据
archive/                         历史实验实现与说明；不作为研究主入口
runs/                            本地运行、图件、设计草稿与站点输出（忽略）
demo/                            整理过的展示图件与视频
scripts/                         项目维护与迁移检查工具
```

正式实验从上述三个分层入口启动。旧命令仅保留兼容入口；历史实现与证据的迁移对应关系见
[目录归档记录](docs/directory_cleanup.md)。本地环境 `.venv/` 和测试缓存默认忽略。

程序运行生成的日志、轨迹、数组、图件、预览和导出文件默认写入 `runs/`，不进入 Git。
`assets/` 保留仿真必需的模型和来源记录；`results/` 只保留经审查的冻结证据，
`demo/` 只保留明确列入白名单的展示文件。新增运行数据不会自动成为项目成果。
具体默认路径、保留边界和验证方法见[生成数据策略](docs/generated_artifacts.md)。

```bash
uv run ruff check .
uv run pytest -m 'not slow' -q
uv run mkdocs build --strict
```

[贡献指南](CONTRIBUTING.md)说明检查、物理变更和证据记录要求；[变更记录](CHANGELOG.md)记录本次结构调整。
[完善路线图](docs/development_plan.md)和[迁移验证记录](docs/restructuring_validation.md)给出当前状态。

[架构](docs/architecture.md) · [复现手册](docs/experiments.md) · [论文对照](docs/paper_mapping.md) ·
[SE(3) 理论](docs/theory/se3_lie_impedance.md) · [API](docs/api/control.md)。

## 许可与参考

项目代码使用 [MIT](LICENSE)。导入资产遵循各资产目录内的许可与来源清单。
控制方法参考 Kim et al. (2025, IEEE T-RO) 与 Ren & Shan (2026, Acta Astronautica)，
对应实现与尚未实现部分见论文对照文档。项目目前仅有仿真证据，未验证实机、制造公差或真实锁紧。
