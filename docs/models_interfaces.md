# 模型与接口基线

算法主研究固定使用 `scenes/iiwa14_petal_insertion.yaml` 与原版 PetalDock100 V2。
`angle1_blend030` 属独立接口候选，尚未晋升；不能默默替代控制对照的模型。

## 固定对象

| 项目 | 基线 |
|---|---|
| 机械臂 | iiwa14；MuJoCo arm MJCF，Pinocchio 从同一组装 MJCF 构建 |
| 接口 | `assets/interfaces/petal_dock100/active.xml` / `passive.xml`，已有凸块接触表示 |
| 单位与根坐标 | SI；控制根为工具法兰；安装变换在场景声明 |
| 配合 | `Rz(45°) Rx(180°)`，名义法兰间距 46.4 mm |
| 目标 | 世界位置 (0, 0.5, 0.35) m；固定，零重力 |
| 接触默认 | condim=3，friction=`0.15 0.003 0.0001`，solref=`0.003 1`，solimp=`0.95 0.99 0.0002` |
| 求解 | implicitfast，elliptic，60 次迭代，容差 1e-10，物理步长 0.5 ms |
| 控制与反馈 | 控制周期 0.5 ms；F/T 延迟固定一个控制周期 |
| 初值与限制 | 场景内 IK 初值；七轴关节范围与 iiwa14 URDF 一致 |

资产原始文件、适配文件与质量惯量来源由 `assets/interfaces/petal_dock100/manifest.json` 记录。
导入只读取模型/网格，保留原资产唯一来源与许可，不执行附件脚本。

以下 SHA-256 记录本次结构迁移的输入基线；运行时仍逐项核查资产清单。

| 文件 | SHA-256 |
|---|---|
| Petal manifest | `ea5babbc2eb6391f29b36662e44367eff836ce2a27014e56146801d8284a7a33` |
| Petal scene | `89046325c5cdc082b2096a9294e41a0e1d08605b19d4636838cc57009fafa87c` |
| iiwa14 arm | `da144841d2fd1a243df4e7c93f6c18ba76e3ae7219150394d8bf0e492d3b4146` |
| HexFrame scene | `ae92e5a59f2ad6f7228df3ab5f4214e1b6a3c368e56c130a1a41d00d70f4d507` |

## 资格检查

```bash
uv run python -m experiments.models_interfaces.baseline --out runs/model_baseline_my_run
```

检查初值与十个固定种子随机配置的末端平移、旋转与质量矩阵一致性（最大误差 <1e-10），
初始无接触、关节裕度，并核对资产文件哈希。结果保存检查报告与当前指纹。
模型检查通过仅支持上述声明的仿真用途。装配负载与六侧面端口另由 HexFrame 预检检查。

## 接口设计与候选

生成/调形脚本保持原路径和资产唯一来源：`prepare_petal_interface.py`、
`prepare_petal_guidance.py`、`prepare_petal_design.py`、`petal_designer.py`。
它们属于模型层，不是控制入口选项。几何矩阵与候选复核入口位于 `experiments/models_interfaces/`。
轮廓、物理步长或碰撞表示变化分别记入协议；与控制改进分开归因。

[候选独立步长复核](../results/petal_angle1_blend030_dtcheck_20261003.md)保留未晋升结论，
[接口阶段收尾](stage_closeout_20261003.md)保留历史状态。
[原版控制证据](control_research.md)与[历史模型证据](historical_evidence.md)分别引用。

迁移后旧候选归档与当前源码指纹不同，候选入口会在仿真前拒绝复用。历史复现使用其归档
`source_snapshot`。若要开始新的同源候选研究，先用当前几何矩阵生成带完整 artifact manifest
的参考目录，再显式传入该目录：

```bash
uv run python -m experiments.models_interfaces.petal_guidance --out runs/current_geometry_reference --jobs 3
uv run python -m experiments.models_interfaces.selected_candidate \
  --reference runs/current_geometry_reference --out runs/current_candidate_study --jobs 3
```

这会形成新的参考研究，旧归档仍不改写；本次结构迁移没有重跑该大型几何矩阵。
