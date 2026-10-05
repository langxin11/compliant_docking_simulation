# 模型与接口基线

2026-10-05：按项目负责人决定，单接口默认场景 `scenes/iiwa14_petal_insertion.yaml`
采用 **窄平顶＋角向斜坡（angle1_blend030 凸块模型）**，资产位于 `assets/interfaces/petal_guidance/angle1_blend030/`。
默认采用已验证的自由空间提速和 1 ms 物理/控制/反馈延迟设置；SDF 原型对照保留这些设置及原摩擦、质量惯量与门槛。
这次默认选择不等于证明最优或完成实机验证；新协议九点证据见[1 ms 任务验证](../results/angle1_task_1ms_20261005.md)。

原轮廓保存为 `scenes/iiwa14_petal_original_insertion.yaml`；RQ1/RQ2、捕获范围和历史几何
复现实验显式读取它。`iiwa14_petal_lateral_insertion.yaml` 仍是原轮廓的历史演示。
`iiwa14_petal_guided_insertion.yaml` 是已有 narrow 组合偏差演示。
旧 angle1 精细步长协议的未晋升判定保留历史口径；本次按用户决定设为新任务协议默认。

HexFrame 使用独立整模块资产，当前仍是原接口与关节伺服＋接触导纳。
新默认接口已在独立实验模型中接入 HexFrame：angle1 名义完整通过并独立审计，三个偏差点未落座；
冠形 STL 四点未完成，存储预载与提前接触仍待处理。正式默认场景未替换，旧 53 s 验收不适用于新接口。
见[两种接口实验适配](../results/hexframe_interfaces_20261005.md)。

## 固定对象

| 项目 | 基线 |
|---|---|
| 机械臂 | iiwa14；MuJoCo arm MJCF，Pinocchio 从同一组装 MJCF 构建 |
| 接口 | `assets/interfaces/petal_guidance/angle1_blend030/active.xml` / `passive.xml`，已有凸块接触表示 |
| 单位与根坐标 | SI；控制根为工具法兰；安装变换在场景声明 |
| 配合 | `Rz(45°) Rx(180°)`，名义法兰间距 46.4 mm |
| 目标 | 世界位置 (0, 0.5, 0.35) m；固定，零重力 |
| 接触默认 | condim=3，friction=`0.15 0.003 0.0001`，solref=`0.003 1`，solimp=`0.95 0.99 0.0002` |
| 求解 | implicitfast，elliptic，60 次迭代，容差 1e-10，物理步长 1 ms |
| 控制与反馈 | 控制周期 1 ms；F/T 延迟 1 ms（一个控制周期） |
| 初值与限制 | 场景内 IK 初值；七轴关节范围与 iiwa14 URDF 一致 |

当前默认资产及质量惯量由 `assets/interfaces/petal_guidance/angle1_blend030/manifest.json` 记录；原资产来源保留在 `assets/interfaces/petal_dock100/manifest.json`。
导入只读取模型/网格，保留原资产唯一来源与许可，不执行附件脚本。

以下 SHA-256 记录切换默认接口之前的结构迁移输入基线，不是当前默认场景指纹；运行时仍逐项核查资产清单。

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

此前 narrow 默认切换的预检和测试属于历史记录。当前默认与已验证的 angle1 1 ms 场景参数一致；未新增 HexFrame 验收结论。

## 用户选定接口的 1 ms 任务验证

`scenes/iiwa14_petal_angle1_blend030.yaml` 使用 1 ms 物理步长、控制周期与反馈延迟，
提高自由空间速度/加速度，插入和保持参数不变。原时间设置留在
`scenes/iiwa14_petal_angle1_blend030_025ms.yaml`，仅供历史对照。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl uv run python -m experiments.models_interfaces.angle1_task_check --out runs/angle1_task_new --jobs 3
```

该入口只运行固定九点并审计，不自动追加更细步长或调整门槛。

本轮 [1 ms 九点验证](../results/angle1_task_1ms_20261005.md) 已完成：9/9 通过原门槛及逐步审计。
这是独立的新任务协议；旧精细步长研究判定按历史口径保留。

## 有限 SDF 对照结论

[三工况对照](../results/angle1_sdf_compare_20261005.md)：凸块 3/3 通过，当前花瓣 SDF＋原止挡原型 0/3 通过，
存在提前接触、大穿透及明显载荷异常。默认保留 angle1_blend030 凸块模型，不替换为 SDF。
