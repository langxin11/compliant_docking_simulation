# 程序生成文件与冻结资产

新运行生成的日志、轨迹、NPZ、派生图、预览、报告构建文件和 Blender 导出默认进入
`runs/`，由 Git 忽略。模型运行必需的 OBJ/STL/XML、配置、许可、冻结候选和经过人工选择的
轻量证据继续维护；不能仅根据文件扩展名把它们全部忽略。

本轮基线为 `fe9825a4bb9def613e2bcec77aaddce298b76593`。从导入 HexFrame 包中移出
20 个配套生成文件：`results/` 12 个、`preview/` 7 个、`blender/module.glb` 1 个，
完整逐字节保存在本地 `runs/imported_artifacts/hexframe/`，共 6590242 字节。
不再复制一套到 `results/` 跟踪；原始导入 `IMPORT.json`、许可、几何、配置与 MJCF 未改。
另有 2 张无引用旧图由 `demo/` 移到本地 `runs/generated_archive/demo/`；7 个已发布展示
文件使用明确 allowlist 保留。

此前[目录整理](directory_cleanup.md)的 503 个资源、739 项映射是当时的真实状态，
`docs/evidence/directory_cleanup_20261004.json` 保持原样。本轮之后保留 483 个资源：
476 个逐字节不变，6 个工具脚本和 README 根据新输出策略维护。新增
`docs/evidence/generated_artifacts_20261004.json` 记录被移出的 22 个文件原 Git 哈希、
本地保存位置，以及 13 个维护文件的前后哈希、精确编辑与适用边界。
原始导入哈希表示原始包，不伪造为当前维护工具的哈希。

## 当前默认位置

以下路径均相对于仓库根目录。

| 工具或用途 | 默认输出 | 输入与覆盖边界 |
|---|---|---|
| `assets/modules/hexframe/demo.py` | `runs/hexframe_module/results/demo/` | 读取正式模型；短烟雾运行不代表完整对接通过 |
| `assets/modules/hexframe/validate_model.py` | `runs/hexframe_module/results/` | 可用 `--out`；核对模型，不写入资产 |
| `assets/modules/hexframe/create_preview.py` | `runs/hexframe_module/preview/` | 默认只生成静态图；动画显式 `--trajectory-dir`，缺输入明确拒绝 |
| `assets/modules/hexframe/generate.py` | 模型 `runs/generated_assets/hexframe/`，GLB `runs/hexframe_module/blender/` | 可用 `--out/--artifacts-out`；拒绝已有模型目录，不改正式资源 |
| HexFrame 嵌套接口 CAD 生成器 | `runs/generated_assets/petaldock_interface/` | CAD 依赖另行安装；不自动替换正式接口 |
| Blender `build_scene.py` | `runs/hexframe_module/blender/hexframe_module.blend` | 使用 Blender 执行；GLB 必须存在或显式 `--glb`，轨迹需显式 `--trajectory` |
| Petal 接口导入/导向生成器 | `runs/generated_assets/petal_dock100/`、`runs/generated_assets/petal_guidance/` | 支持显式 `--out`，已有模型不覆盖 |
| 冠形凸分解生成器 | `runs/generated_assets/convex_crown/` | `--validate-only` 默认读冻结模型，仅将验证清单副本写到运行报告目录 |
| 调形候选 | `runs/designs/petal_guidance/` | 仍标记未验证；不会自动晋升 |
| 框架报告、一般图件与录像 | `runs/framework_comparison/`、`runs/figures/`、`runs/videos/` | 手动仿真演示也使用这些位置 |
| 候选报告后处理 | 指定运行目录中的 `report.md` 和分析快照 | 不自动覆写 `results/` 冻结报告 |

例如在仓库根目录运行：

```bash
uv run python assets/modules/hexframe/demo.py --headless --seconds 0.025
uv run python assets/modules/hexframe/validate_model.py
uv run python assets/modules/hexframe/create_preview.py
# 先生成包含轨迹样本的 demo，再选择动画输入：
uv run python assets/modules/hexframe/create_preview.py \
  --trajectory-dir runs/hexframe_module/results/demo
```

冷克隆无需保留导入时的运行文件，也能加载正式模型并生成新 demo/验证/静态预览。
需要回放历史动画时，要另外取得完整记录并显式指定；冻结哈希摘要不是回放输入。
CAD/CoACD/Blender 依赖属于离线资产工具，不新增到普通仿真运行依赖。

## 验证与版本控制

维护检查继续比较 8 个迁移研究函数的 AST、控制/装配/规划源码，以及全部保留的模型资源。
仿真模块仅允许两个手动录像路径和手动绘图路径的精确改动，`MujRobot`、`RobotController`
与资产 demo 的 `ModuleDemo` 类 AST 不变；两个导入 CAD 生成器的几何/MJCF函数 AST 不变。
20 个移出项必须与原 Git blob 和前一阶段映射一致，实际正式目录不得残留它们；本地保存
文件存在时还会核对哈希。已记录的维护源码只允许清单中的精确编辑，额外资源或代码变化
会失败。忽略的本地文件不是克隆/CI 的必需输入。

```bash
uv run python scripts/check_restructuring.py --out runs/generated_artifacts_source_check.json
uv run ruff check .
uv run pytest -m 'not slow' -q
uv run mkdocs build --strict
```

CI 拒绝任何仍被忽略规则命中的已跟踪产物。人工选择需要发布的证据或展示文件时，
先记录用途和来源并新增具体 allowlist，不用 `git add -f` 绕过策略。
历史 `results/`、源码快照、来源清单和用户报告草稿不因这一策略重写。
用户原绘图 WIP 默认值保持原样，当前命令使用显式 `--out runs/figures/se3_free_space`。

本轮真实工具行为检查 5 项通过（17.14 s）：冷目录 demo/模型验证/静态预览生成、缺轨迹
输入拒绝、原模型覆盖拒绝、只验证不改冻结 manifest、候选后处理不发布冻结报告。
最终快速套件 **355 passed / 4 deselected**，75.78 s，7 条既有 SciencePlots 弃用警告；
ruff 与严格文档构建 PASS（2.64 s）。22 个旧产物位置及新 `runs/` 位置均命中忽略规则，
7 个保留展示文件不命中。前一阶段 739 项机器清单未改写。

正式模型基线 `runs/generated_policy_20261004_model` exit 0 / PASS；无 DISPLAY 和
MUJOCO_GL 的系统预检 `runs/generated_policy_20261004_hexframe` exit 0 / INCOMPLETE（预期），
说明移除配套产物不影响正式模型读取。它仍不是完整系统验收。实际结果与适用边界已记录在
本轮机器清单中。
没有重跑完整系统验收、控制/几何矩阵、CAD 重建、CoACD 分解或 Blender 导出。
