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

结果见 `runs/hexframe_grid_20261003/grid_summary.json`。

## 测试

`tests/test_hexframe_validation.py`：节拍/窗口推导、replace 入口、
正式 yaml 仍拒绝非 1 ms、基线 runtime 驱动同值窗口。
