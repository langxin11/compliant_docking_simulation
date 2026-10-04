# 分层入口与数据策略

推荐从仓库根目录使用 `python -m experiments.<layer>.<entry>`；旧直接脚本命令继续兼容。
新入口按问题建立参数集合，不提供跨研究目的的任意切换。

| 职责 | 新入口 | 保留的旧入口 |
|---|---|---|
| 固定模型检查 | `models_interfaces.baseline` | Petal preflight 函数 |
| 绕轴策略 RQ1 | `control.rq1_yaw` | `petal_insertion_suite.py` 默认矩阵 |
| 横向策略 RQ2 | `control.rq2_lateral` | `petal_insertion_suite.py --lateral-study` |
| 离散误差范围 | `control.capture_range` | `petal_insertion_suite.py --grid` |
| 几何矩阵 | `models_interfaces.petal_guidance` | `petal_insertion_suite.py --geometry-study` |
| 候选复核 | `models_interfaces.selected_candidate` | `petal_selected_study.py` |
| 完整系统 | `system.hexframe precheck/accept/replay` | `docking --scene scenes/hexframe_assembly.yaml` |
| P0/P1/P2 | `system.validation.run_halfstep/run_grid/run_noise` | `hexframe_validation/*.py` |

自由空间验证沿用 `experiments/se3_free_space.py` 和用户正在编辑的 `se3_free_space_plots.py`；
框架、冠形与旧展示归历史索引。模型生成/调形脚本保留原位置及唯一资产来源。

## 输出与退出

新运行选择新的 `runs/<question>_<date_or_run>/`。已存在来源清单必须逐项相等才能续跑；
不将新输出续写进旧归档。不安全的历史复用明确拒绝。控制计算完成退出 0 不等于对照组
全通过，结论读取每组 `assessment`；系统验收失败退出 2，预检为 INCOMPLETE，回放须先审计。

每组记录有效场景、路点、F/T 与真实接触载荷、反馈协议、几何/安全门禁、运行环境和来源。
物理步长复核默认可保存核心遥测；需要逐步重算时明确选择 full。`runs/` 默认忽略，
可分享的是报告、指纹、评价和需要的图件；原始大通道依报告策略保留。

RQ2 paired 与 numerics 使用固定协议，speed 单独目录。P2 的判定策略与噪声写入运行说明。
生成视频与其验收目录关联，不以视频存在推断 PASS。

候选历史参考因源码迁移而拒绝复用；当前可用的同源新参考流程与 `--reference` 参数见
[模型与接口](models_interfaces.md)。旧候选数值仅按其归档源码解释。
