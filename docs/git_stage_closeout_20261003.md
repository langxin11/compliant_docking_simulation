# 本地 Git 阶段提交整理（2026-10-03）

仓库根目录为 `/home/xiao/workspace/maintained/compliant_docking_simulation`，
沿用 `integration/hexframe-main-20261003` 分支。接手时 HEAD 为 `d27a5be`，
暂存区为空；只有一个工作树，未发现项目或父目录适用的 AGENTS.md。

## 提交范围

1. 交互调形与保存参数构建：页面、后端、构建器、测试及操作说明；
   同时纳入此前尚未入库的必要对接/接触诊断/几何基础、原始接口以及
   窄平顶和径向历史对照模型，保证这一提交能独立加载和测试。
2. 1° / 0.3 候选：保存参数、独立模型与场景、代表验证脚本、验证报告、
   原阶段收尾记录与完整性清单，以及轻量证据副本和本次 Git 核对记录。

`cli.py` 与 `scene.py` 暂存的是本阶段归档 `source_snapshot` 中的实际运行版本。
它们和其余运行源码的提交字节与本阶段 47 项源码哈希一致。后续 HexFrame
接线保留在工作区，不覆盖当前工作文件。

`tests/test_petal_grid.py` 仅将需要本地冻结数据的检查拆为独立可跳过测试，
其余源码复用保护检查继续在干净检出中执行；控制器、参数和验收门槛未修改。

## 数据与大文件

现有 `.gitignore` 排除整个 `runs/`、缓存和虚拟环境，规则没有修改。
约 6.3 GB 原始仿真归档（包括本次四条轨迹数组、接触数组、日志、源码快照和
全部历史冻结实验）继续本地保存，不进入提交。

[轻量验证证据](../results/petal_angle1_blend030_20261003/README.md) 中的
24 个 JSON/报告/图件是按字节复制的审阅证据，另有来源哈希清单。
它们不代替逐步数据归档；完整再审计需要保留的本地 `runs/`。

模型网格、MJCF、惯量与哈希清单作为必要资产提交，单文件均小于 10 MiB。
已有 OBJ 末尾空行按原字节保留，未为消除格式提示而改变原模型。

## 完整性与验证

[本次完整性核对](../results/petal_git_integrity_audit_20261003.json) 核验了
7 份冻结实验清单的 1109 项记录、本次90项归档成果、599项模型文件和
47项运行源码快照。收尾清单中的7项文档记录，有6项哈希匹配。

接手时已存在一处文档哈希差异：`docs/stage_closeout_20261003.md`
的记录值为 `b7621223ed9f1324600fc802776ec0613654da2f22f4785e4a66b690f820c6e6`，
实际文件值为 `c9800190108311247dbca805d4578f79f6542d2c5d81366886d83864e6d26384`。
此处只记录差异，保留原文档和原清单，不重写历史记录。

首个提交的暂存内容导出到独立目录后，相关测试141项通过、1项因未包含
本地冻结归档而跳过、1项完整仿真回归未执行。Python静态检查、两个浏览器
JavaScript模块语法检查通过；源码与文档差异格式检查通过。
最终暂存副本文档严格构建通过，候选场景可由MuJoCo和Pinocchio编译加载，
未执行物理步进；保留原时间设置和载荷门槛。

三个代表主工况通过原落座和载荷门槛，细化步长检查仍为 **SENSITIVE**。
步长敏感性尚未解决，候选未定型，制造贴合和锁定尚未验证。
本次整理没有新增仿真或重新生成历史数据。

## 留在工作区的其他成果

HexFrame assembly 包、场景和资产、`experiments/orbital_showcase/`、相应测试与
文档，以及 CLI/scene 的后续接线未纳入本阶段提交。README、实验总览、
理论/论文映射及开发计划中的更广范围编辑也保留给其所属阶段。
CI工作流改动、HexFrame测试收集与排除配置、自由空间绘图脚本的其他改动
同样保留。文档导航仅暂存本阶段所需入口，工作区中的后续导航保留。

本次范围整理后的未提交路径如下（目录包含其全部未提交文件）：

```text
.github/workflows/ci.yml
.github/workflows/docs.yml
README.md
docs/experiments.md
docs/paper_mapping.md
docs/theory/se3_lie_impedance.md
experiments/se3_free_space_plots.py
mkdocs.yml
pyproject.toml
src/compliant_docking/cli.py
src/compliant_docking/scene.py
assets/scenes/
docs/development_plan.md
docs/hexframe_assembly.md
experiments/orbital_showcase/
scenes/hexframe_assembly.yaml
src/compliant_docking/assembly/
tests/test_hexframe_scene.py
```
