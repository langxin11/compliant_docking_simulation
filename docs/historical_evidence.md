# 历史证据索引

原报告、资产来源和结果内容保持原样。本索引按问题分类，不重新评价历史结果。
本地 `runs/` 通道可能按数据策略精简；需要完整原始数据时按原协议重跑到新目录。

| 层次 | 记录 | 证据用途 |
|---|---|---|
| 模型/接口 | [angle1 1 ms 九点任务验证](../results/angle1_task_1ms_20261005.md) | 自由空间提速、9/9 落座与载荷门槛、时序和运动限制审计 |
| 模型/接口 | [原版接入](../results/petal_iiwa14_validation.md) | 同源模型与固定刚度首轮 |
| 模型/接口 | [导向几何](../results/petal_guidance_geometry_validation.md) | 独立轮廓矩阵 |
| 模型/接口 | [候选步长](../results/petal_angle1_blend030_dtcheck_20261003.md) | 当时未晋升；当前按新 1 ms 协议设为默认 |
| 模型/接口 | [凸碰撞](../results/convex_collision_iiwa14.md) | 原冠形碰撞表示对照（冠形线） |
| 控制 | [绕轴释放](../results/petal_contact_control_validation.md) | 固定模型、同点绕轴策略 |
| 控制 | [横向配对](../results/petal_lateral_control_validation.md) | 九个离散点、组合卡滞与数值敏感性 |
| 控制 | [捕获网格](../results/petal_capture_grid_validation.md) | 原策略离散误差范围 |
| 控制 | [框架与自由空间复现](experiments.md) | 原协议下的方法/实现验证 |
| 系统 | [HexFrame 实际位姿偏差](../results/hexframe_pose_check_20261005.md) | 名义完整通过；三个偏差点未落座，保留失败与独立核对 |
| 系统 | [HexFrame 两种接口移植](../results/hexframe_interfaces_20261005.md) | 花瓣名义通过；偏差与冠形移植保留失败，提供同源视频供人工评判 |
| 系统 | [HexFrame 正式流程](hexframe_assembly.md) | 当前导纳流程验收 |

P0/P1/P2 的扩展验证入口在 `experiments/system/validation/`；
P2 包含判定策略变量，不能归因于单接口 SE(3) 控制。旧 orbital_showcase/冠形/
接触力历史实现已随 2026-10-05 清理从仓库移除，Git 历史可查；其结论按原协议
解释，不能以历史视频替代正式系统验收。

迁移后源码指纹发生变化。旧目录继续使用当时快照解释；当前指纹不匹配时拒绝续跑或复用。
分层入口见[实验复现手册](experiments.md)。
