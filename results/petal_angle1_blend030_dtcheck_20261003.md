# PetalDock100 angle1_blend030 步长复核（dtcheck，2026-10-03）

回应 [阶段收尾](petal_stage_closeout_20261003.json) open item 1（时间设置与数值敏感性）。
候选导向轮廓 `angle1_blend030`（1° / 0.3）此前唯一未决项：nx6 点 0.25 vs 0.125 ms
配对横向 0.10412 mm / 相位 0.11011°，超 0.1 / 0.1 限值，判 SENSITIVE。

## 预注册判定规则（先于结果固定）

> 晋升当且仅当三组配对全部 STABLE_IN_TWO_STEPS：两个组合点各补 0.125 ms 与
> 归档 0.25 ms 配对（判定敏感性是否点特异）；nx6 补 0.0625 ms 与归档 0.125 ms
> 配对（判定是否随加密收敛）。单条 rollout 仍须 CANDIDATE_PASS，限值不放宽。

## 结果（5 条新 rollout，全部 CANDIDATE_PASS）

| 配对 | 状态 | 横向 mm | 相位 ° | 峰值力差 N | 峰值轴矩差 N·m |
|---|---|---|---|---|---|
| A combo_ny6_p15 0.25 vs 0.125 | **SENSITIVE** | 0.00056 | 0.0007 | 3.707 | 0.0489 |
| A combo_py6_n15 0.25 vs 0.125 | **SENSITIVE** | 0.00734 | 0.0087 | 1.976 | 0.0611 |
| B nx6 0.125 vs 0.0625 | STABLE | 0.01059 | 0.0086 | 0.0046 | 0.0041 |
| S* combo_ny6_p15 0.125 vs 0.0625 | STABLE | 0.01431 | 0.0136 | 0.175 | 0.0428 |
| S* combo_py6_n15 0.125 vs 0.0625 | STABLE | 0.00230 | 0.0042 | 1.187 | 0.0053 |

\* 补充组：预注册判定之外的归因复核，见下。

## 判定与归因

**按预注册规则：不晋升（KEEP_CANDIDATE_NOT_PROMOTED）**——A 组两对 SENSITIVE。

归因（补充组证据）：三点的几何通道在所有步长下都远小于限值；A 组判 SENSITIVE
的实体是**接触力峰值**。而 0.125 → 0.0625 ms 在全部三个点（B + S*）均 STABLE。
结论：候选在 0.125 ms 已收敛；敏感性源于**归档 0.25 ms 记录对组合点接触瞬态
峰值的分辨率不足**，不是候选在认证步长下的发散。

## 后续路径（需研究负责人决定，本文不自行改判）

1. **修订认证规则后晋升**：把认证步长定为 0.125 ms、以 0.125/0.0625 收敛为依据。
   这属于看到数据后的规则修改，必须显式声明并记录，不能追溯适用本轮预注册结论。
2. **维持不晋升**，按 development_plan 第 1 条转碰撞离散加密 / 摩擦窗口（否决分支）。
   本轮证据提示该线的优先对象是峰值通道而非落座几何。

## 证据

- 运行目录：`runs/petal_angle1_blend030_dtcheck_20261003/`（含 `supplement_0p0625/`）；
  驱动脚本、`dtcheck_manifest.json`、`dtcheck_summary.json`、五份 rollout JSON 已入库，
  原始 NPZ（core 遥测）按数据策略留本地。
- 对照归档：`runs/petal_angle1_blend030_20261003/`（closeout 哈希钉死，未改动）。
- 门禁、增益、几何与一切限值沿用已评审实现；配对判定复用
  `petal_capture_grid.sensitivity`。控制周期与 F/T 延迟保持 0.5 ms 不变。
