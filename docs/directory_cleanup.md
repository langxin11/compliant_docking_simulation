# 目录归档与清理记录

本轮以 `c640a1327fbf6bb3dcba23b94516e180aae47e1a` 为归档基线，在
`refactor/three-layer-project` 完成路径整理。上一轮三层迁移的基线与物理验收保持在
[原验证记录](restructuring_validation.md)，不重新解释其历史数字。

## 目录与唯一来源

| 原位置 | 当前位置 | 处理与边界 |
|---|---|---|
| `experiments/orbital_showcase/assets/hexframe_module/` | `assets/modules/hexframe/` | 503 个生产资源逐字节搬迁；正式场景引用唯一新位置 |
| `experiments/orbital_showcase/` 历史源码、场景、资产和说明 | `archive/orbital_showcase/` | 34 个文件原样归档；旧 Python 入口只转发，不复制实现 |
| `experiments/hexframe_validation/README.md` | `archive/hexframe_validation/README.md` | 原验证协议说明原样保留；旧命令仍转发系统层 |
| 原纳入 Git 的 `runs/` 文件 | `results/historical/runs/` | 27 个文件逐字节副本；本地原运行数据仍保留，Git 不再跟踪运行目录 |
| orbital 三组本地 `outputs/` | `runs/archived_outputs/orbital_showcase/` | 125 个完整本地文件搬迁并校验，未整体纳入 Git |
| orbital `outputs/` 的 JSON/Markdown | `results/historical/orbital_showcase/` | 策展 48 个摘要，251325 字节；源码快照和大通道不作为摘要复制 |
| 原 `designs/petal_guidance/candidate_20261003T103814Z_23ff2b4e.json` | `assets/interfaces/petal_guidance/angle1_blend030/selected_design.json` | 两者原本逐字节相同；保留已冻结资产内唯一文件，移除旧副本和空目录 |
| 用户本地 `report/` | `docs/reports/research_draft/` | 18 个草稿文件按字节保留，仍不作为本次提交内容；站点排除草稿 |
| 用户本地 `figure/` | `runs/figures/` | 24 个图件按字节保留 |

完整逐文件路径、SHA-256、操作与原 Git 跟踪状态见
`docs/evidence/directory_cleanup_20261004.json`。资产自身 `IMPORT.json`、许可、CAD、网格和
历史输出未改写；内嵌旧路径属于来源记录，不伪造为本次生成结果。

## 生成文件与兼容入口

正式图件默认写入 `runs/figures/`、录制视频写入 `runs/videos/`；框架新对比表写入
`runs/framework_comparison/`，避免覆盖 `results/` 的冻结报告；调形候选保存到 `runs/designs/petal_guidance/`，
待审查模型生成到 `runs/generated_assets/`。冻结模型及候选参数进入 `assets/`，
冻结评价、指纹与必要摘要进入 `results/`。新运行继续使用独立 `runs/<name>/`，
不续写冻结证据。站点输出为 `runs/docs_site/`。

用户正在编辑的 `experiments/se3_free_space_plots.py` 和 `demo/se3_free_space_validation.png`
未修改；前者保留其旧默认值，当前文档通过显式 `--out runs/figures/se3_free_space` 选择新路径。
本地 `runs/` 和 `.venv/` 保留，只删除可再生成的站点和 Python/检查工具缓存。

旧 orbital Python 命令通过 `experiments/_archive_compat.py` 加载 `archive/` 唯一实现。
旧生产资源位置和归档内部资源位置使用 **Linux 相对符号链接**，共同解析到
`assets/modules/hexframe/`。三个归档 `outputs` 链接只指向独立可写的
`runs/archive_workspaces/orbital_showcase/`；兼容 loader 在冷启动时创建这些目标目录，
无需克隆时携带本地运行文件。直接执行归档源码用于查阅原协议；运行旧命令请使用兼容入口。

历史审计默认读取独立工作区，它不隐式读取冻结摘要。兼容审计提供
`--out <完整历史运行的可写副本>`，例如
`uv run python experiments/orbital_showcase/hexframe_assembly_audit.py --out runs/my_legacy_audit_copy`。
原算法仍校验完整通道和 `source_manifest.json`；旧来源路径指向当前包装器时会因哈希不同拒绝，
需要当时源码/快照和完整运行记录。不能用路径替换、改哈希或写入 `results/` 使审计通过。
冻结摘要不是可独立回放的完整运行包。

## 复现与检查

迁移当日由时点检查工具（2026-10-05 已从仓库移除，存于 Git 历史）完成核对：8 个研究
函数 `16a54d4` AST 等价、控制/装配/规划/仿真实现严格不变，配置仅允许 `assembly.resource`
路径迁移、编排仅允许 `plot_results` 和视频保存目录变化，既有资产及搬迁资源按原 Git blob
逐文件校验。完整口径见[原验证记录](restructuring_validation.md)；需要本地 Git 历史中的
基线提交才能重查。

```bash
uv run ruff check .
uv run pytest -m 'not slow' -q
uv run mkdocs build --strict
```

当前源码、配置路径及默认输出发生变化，不复用旧研究 manifest；同源续跑仍要求完整清单相等。
这次整理不改变接口几何、控制公式、系统增益、步长、阶段或验收门槛。

| 本轮检查 | 实际结果 |
|---|---|
| 正式模型基线 `runs/directory_cleanup_20261004_model` | exit 0，PASS |
| 无 DISPLAY / 无 MUJOCO_GL 系统预检 `runs/directory_cleanup_20261004_hexframe` | exit 0，INCOMPLETE（预检预期）；生产资源、双模型与完整路径可解析 |
| 503 个生产资源与原 Git blob 比较 | 全部逐字节一致 |
| 27 个已跟踪运行文件与本地原件比较 | 全部逐字节一致，原件仍在 `runs/` |
| `uv run ruff check .` | PASS |
| `uv run pytest -m 'not slow' -q` | 350 passed，4 deselected，62.44 s；7 条既有 SciencePlots 弃用警告 |
| 严格文档构建与实际站点检查 | PASS，2.84 s；历史报告页面存在，research_draft 页面不存在 |
| 归档行为边界 | 3 项随最终快速套件通过：无 runs 的真实包装器冷启动、单一归档实现/生产链接、拒绝审计写入冻结 results |
| 全部迁移映射本地字节检查 | 739 项通过（含仅本地保存的 125 个原输出）；维护命令重查 614 项版本化证据 |
| 8 个函数 AST、核心物理、配置路径规范化、资产 Git blob 检查 | PASS；默认出图和视频目的地是唯一编排变化 |

本轮未重跑完整 53 s 系统验收、回放、控制研究矩阵或几何矩阵；上一阶段完整验收与数值
对照记录继续按其源码指纹解释。预检成功只确认几何、模型一致性和规划，不升级为完整系统 PASS。
