# HexFrame 验证工作包（experiments/hexframe_validation/）

正式场景 `scenes/hexframe_assembly.yaml` 的 1 ms / 53 s 验收流程保持锁定
（config 校验与 CLI 拒绝逻辑未改动）。本目录是独立验证入口，用
`dataclasses.replace(scene, …)` 构造变体后直调 `simulate()` + `audit()`，
门禁、增益、相位时长与审计限值全部不变。

## P0 物理半步长（run_halfstep.py）

回答"验收结论是否 1 ms 步长的人为产物"。前置：`simulation.py` 的 10 ms
接触 IK/100 Hz 记录节拍与 `audit.py` 的 0.5 s 就位窗口改为按
`runtime.json` 的 `physics_timestep_s` 推导，1 ms 下逐位一致
（已在 `full_assembly_cad_precision` 副本上重跑审计确认 IDENTICAL）。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 uv run python \
  experiments/hexframe_validation/run_halfstep.py \
  --out runs/hexframe_halfstep_20261003 --dt 0.0005
```

2026-10-03 结果（runs/hexframe_halfstep_20261003/）：

| 量 | 1 ms 基线 | 0.5 ms | 漂移 |
|---|---|---|---|
| 状态 / 审计 | PASS / PASS | **PASS / PASS** | 事件顺序不变 |
| 首次接触 / 锁定 | 35.112 / 36.611 s | 35.059 / 36.558 s | −54 / −53 ms |
| 峰值轴向力 | 1.855 N | 0.992 N | −0.863 N（限值 10 N） |
| 最大穿透 | 0.00217 mm | 0.00177 mm | −0.0004 mm |
| 终态模块误差 | 0.0051 mm | 0.0046 mm | −0.0004 mm |

结论：验收结论对物理步长不敏感，半步长下门禁裕度更大。

## P1 误差网格（run_grid.py）

命令级布局场扰动（pick ±10 mm 抓取偏差、seed ±2/±4 mm 配合偏差），
每组完整重规划 + 全程仿真 + 独立审计；名义点复用已记录基线，不重跑。
yaw/tilt 误差需模型级注入，不在第一轮网格内。

```bash
uv run python experiments/hexframe_validation/run_grid.py --jobs 3
uv run python experiments/hexframe_validation/run_grid.py --only seed_x_m4  # 单组
```

2026-10-03 结果（runs/hexframe_grid_20261003/，12 组 + 名义基线）：
**12/12 PASS，独立审计全部 PASS**。峰值轴向力 1.849–1.910 N（限值 10 N），
穿透 ≤0.0022 mm（限值 0.3 mm），终态模块误差 ≤0.006 mm（限值 1 mm）。

| 组 | 锁定时刻 | 峰值力 N | 备注 |
|---|---|---|---|
| pick_x ±10 mm | 36.611 s | 1.850–1.856 | 与名义一致 |
| pick_y_m10 / _p10 | 38.876 / 36.611 s | 1.910 / 1.857 | m10 需额外 2.3 s 就位 |
| seed_x ±2 / ±4 mm | 36.611–39.670 s | 1.851–1.860 | +x 偏差延迟就位 2–3 s |
| seed_y ±2 / ±4 mm | 36.611–36.761 s | 1.849–1.887 | 与名义一致 |

结论：±10 mm 抓取偏差与 ±4 mm 配合偏差内，导纳 + 导向止挡都能在接触窗内
持续就位；y 向抓取偏差和 x 向配合偏差主要表现为就位时间延长而非失败。
连续捕获区域仍未验证（网格是离散点），yaw/tilt 误差待模型级注入。

## 测试

`tests/test_hexframe_validation.py`：节拍/窗口推导、replace 入口、
正式 yaml 仍拒绝非 1 ms、基线 runtime 驱动同值窗口。
