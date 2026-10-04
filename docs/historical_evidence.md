# 历史证据索引

原报告、资产来源、源码快照和结果目录保持原样。本索引按问题分类，不重新评价历史结果。
本地 `runs/` 通道可能按数据策略精简；需要完整原始数据时按原协议重跑到新目录。

| 层次 | 记录 | 证据用途 |
|---|---|---|
| 模型/接口 | [原版接入](../results/petal_iiwa14_validation.md) | 同源模型与固定刚度首轮 |
| 模型/接口 | [导向几何](../results/petal_guidance_geometry_validation.md) | 独立轮廓矩阵 |
| 模型/接口 | [候选步长](../results/petal_angle1_blend030_dtcheck_20261003.md) | `angle1_blend030` 仍未晋升 |
| 模型/接口 | [凸碰撞](../results/convex_collision_iiwa14.md) | 原冠形碰撞表示对照；默认仍保留 SDF |
| 控制 | [绕轴释放](../results/petal_contact_control_validation.md) | 固定模型、同点绕轴策略 |
| 控制 | [横向配对](../results/petal_lateral_control_validation.md) | 九个离散点、组合卡滞与数值敏感性 |
| 控制 | [捕获网格](../results/petal_capture_grid_validation.md) | 原策略离散误差范围 |
| 控制 | [框架与自由空间复现](experiments.md) | 原协议下的方法/实现验证 |
| 系统 | [HexFrame 正式流程](hexframe_assembly.md) | 当前导纳流程验收 |
| 归档 | [接口阶段收尾](stage_closeout_20261003.md) | 阶段状态、失败与未晋升结论 |

P0/P1/P2 历史说明保存在仓库文件 `experiments/hexframe_validation/README.md`；
P2 包含判定策略变量，不能归因于单接口 SE(3) 控制。旧 orbital_showcase/冠形/接触力实验
按其原协议复现，不能以历史视频替代正式系统验收。

迁移后源码指纹发生变化。旧目录继续使用当时快照解释；当前指纹不匹配时拒绝续跑或复用。
[入口映射](experiment_entrypoints.md)与[验证记录](restructuring_validation.md)明确可比条件。
