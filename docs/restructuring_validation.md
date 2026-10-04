# 三层迁移验证记录

日期：2026-10-04。基线为本地 main 的固定提交 **16a54d4**，分支为
`refactor/three-layer-project`。本次保持控制公式、场景、资产、增益、物理参数和验收阈值。
用户已有自由空间绘图、Demo 图、`report/` 和重构计划保留；没有改历史结果或来源快照。
[机器可读证据](evidence/restructuring_20261004.json)保存结果、输入指纹与未运行项。

## 实际迁移

| 原实现 | 唯一实现与兼容关系 |
|---|---|
| insertion_suite 的记录/预览 | `research/rollout.py`；旧冠形矩阵引用共享实现 |
| petal_insertion_suite 的单次试验/评价/配置 | `research/petal_trials.py`；旧入口显式重新导出原函数 |
| 捕获网格的命名/数值标准/来源保护 | `research/protocols.py`；模型与控制编排共用，不相互反向导入 |
| 默认 Petal 矩阵 | `experiments/control/rq1_yaw.py`；旧默认入口保留历史 profiles 与增量运行 |
| petal_lateral_study | `control/rq2_lateral.py`；新 paired/numerics/speed 分开，旧调用保留 legacy-full |
| petal_capture_grid | `control/capture_range.py`；旧模块别名指向新实现 |
| petal_guidance_study / petal_selected_study | `models_interfaces/petal_guidance.py` / `selected_candidate.py`；旧模块兼容 |
| hexframe_validation 的 P0/P1/P2 | `system/validation/run_halfstep.py` / `run_grid.py` / `run_noise.py`；旧脚本调用新实现 |
| docking 正式 assembly 路由 | 保留 `assembly.runner`；新增固定 `system.hexframe` 的 precheck/accept/replay |

资产生成脚本、历史 orbital_showcase 与结果物理位置保持原样。共享研究库不导入实验或 CLI。
系统 sidecar `entrypoint_manifest.json` 记录实际入口与命令；核心 assembly 未改变。

发现并修复原默认矩阵的参数绑定错误：`telemetry` 曾作为 `run_case` 的 `error` 位置参数传入；
新旧矩阵路由均改为关键字传递。`run_case` 本体和物理行为未变，新增语义测试覆盖此绑定。

## 来源与历史复用

新指纹覆盖共享源码、实际三层编排、场景、资产清单、`pyproject.toml` 与 `uv.lock`。
复用要求完整 manifest 相等，且归档中真实共享 `run_case` 的 AST 相等。
新增来源、依赖变化、缺少快照和评价变化均拒绝；没有宽松路径或哈希 fallback。
旧迁移前归档不能继续由新代码安全复用，须按旧快照解释或复现到新目录。

候选研究的默认历史参考在仿真前明确拒绝。新 `--reference` 可指定由当前几何协议生成的
完整同源参考（来源与 artifact manifest 均检查）；这是新研究，不改写旧候选结论。
[当前可用流程](models_interfaces.md)。本次未重跑大型几何矩阵。

## 已执行检查

| 检查 | 结果与耗时 |
|---|---|
| 合并前基线 | ruff、strict docs；338 fast 测试通过，4 deselected；完整 HexFrame PASS + audit PASS |
| 全仓 ruff | PASS |
| 严格文档 | PASS，首次 2.39 s、最终证据文档构建 2.15 s；GitHub 真实结果链接经站点 hook 路由，Mermaid 渲染标记已核对 |
| 受影响模型/控制/系统目标测试 | 49 PASS，12.23 s |
| 新协议/兼容边界 | 9 PASS，1.71 s；拒绝越层参数、跨协议续写、缺来源、spawn 导入与 telemetry 绑定 |
| 完整非慢速测试 | 347 PASS，4 deselected，57.82 s；7 个现有 SciencePlots 弃用警告 |
| 标准包导入 | 10 个新入口通过 |
| 命令解析 | 5 个旧直接脚本与 6 个新模块 `--help` exit 0 |
| 模型基线入口 | PASS；原版资产哈希、11 配置双引擎一致性与初值检查 |
| 无显示系统预检 | exit 0 / INCOMPLETE，10.23 s；DISPLAY 和 MUJOCO_GL 未预设，默认 EGL 生效 |

`scripts/check_restructuring.py --baseline 16a54d4` 逐函数解析 Git 基线和当前文件：
variant、preflight、feedback_audit、run_case、summarize、JSON 转换、save_rollout、preview_rollout
共八个函数 **IDENTICAL_AST**。控制、assembly、orchestration、scenes、assets 与基线 Git diff 均为空。

## 行为回归

Petal 使用迁移前 `/tmp` 原脚本与新 RQ1 `--jobs 2`，同参数 nominal/released/baseline，
分别写入 `runs/restructuring_20261004_petal_old` 与 `_petal_new`。两组均 CANDIDATE_PASS，
实测 180.92 / 182.05 s；有效场景、评价、接触载荷、反馈审计、原门禁、警告、路点和时间完全相同，
**58 个 NPZ/contacts 数组通道逐元素完全相等**。不复用旧遥测或重新标定阈值。

新系统通过 `system.hexframe accept` 写入 `runs/restructuring_20261004_hexframe`，实测 **360.03 s**。
完整 53 s 流程 validation PASS、faults=[]、独立 audit PASS；与
`runs/merge_readiness_20261004_hexframe` 的状态、全部事件、首次接触、锁定和三个主要验收量完全相同。

| 量 | 新旧共同值 |
|---|---:|
| 首次接触 | 35.112 s |
| 锁定 | 36.611000000000004 s |
| 峰值轴向力 | 1.8550067626648206 N |
| 最大穿透 | 0.002164953330996348 mm |
| 终态模块误差 | 0.005054356134462048 mm |

复查命令（需要这些本地新运行目录及 Git 中的固定基线）：

```bash
uv run python scripts/check_restructuring.py --baseline 16a54d4 \
  --petal-old runs/restructuring_20261004_petal_old \
  --petal-new runs/restructuring_20261004_petal_new \
  --system-old runs/merge_readiness_20261004_hexframe \
  --system-new runs/restructuring_20261004_hexframe \
  --out runs/restructuring_20261004_checks/migration_check.json
```

同目录 `replay --record` exit 0，60.69 s；生成 `assembly_sequence.mp4`，
H.264、1280×720、24 fps、53.0 s、1272 帧。接触阶段视频帧和六格 storyboard 已视觉检查。
model、rollout、phases、accepted_anchor 与验收时的来源清单哈希相同；回放没有重新模拟或修改这些输入。
视频位于仓库本地 `runs/restructuring_20261004_hexframe/assembly_sequence.mp4`，不自动发布。

## 未运行项与边界

未执行四项既有 slow 测试、完整 RQ1/RQ2、捕获网格、几何/候选和 P0/P1/P2 矩阵。
这些协议的迁移通过配置/函数等价、入口检查和代表行为回归覆盖，不声称其矩阵已经重跑。
本次没有新增控制优势结论、接口晋升、SE(3) 系统集成或实机认证。


## 工作区与 Git 状态

主代理逐字节核对：已有 plot 与 Demo 图等于 `stash@{0}`，其第三父提交中的
`report/` 与 `docs/restructuring_plan.md` 共 19 文件均与当前一致。
本轮仅将本地 main 快进到 16a54d4，再从它开重构分支；origin/main 仍为 d27a5be。
远端 PR #3 没有合并，未 push 或发布。重构提交由主代理处理。
