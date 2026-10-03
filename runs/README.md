# runs/ 数据保留索引（2026-10-03 盘点与精简）

`runs/` 不入 git（见 `.gitignore`），本文件是本地数据的权威索引。总量约 660 MB
（含 dtcheck 活跃阶段的本地 NPZ）。

项目定位与数据策略见 README「项目定位与实验边界」：
结论以 JSON 门禁 + PNG/PDF 图 + `results/*.md` 报告为准；关闭阶段的原始逐步遥测
（`*.npz` / `*.contacts.npz`）已于 2026-10-03 删除，删除前逐文件记录了 SHA-256，
存于各目录的 `PURGED_NPZ_20261003.json`；需要原始时序时按报告命令重跑。

## 保留完整原始数据

| 目录 | 大小 | 说明 |
|---|---|---|
| `petal_angle1_blend030_20261003/` | 256M | 当前候选轮廓完整证据；被 `results/petal_stage_closeout_20261003.json` 逐文件 SHA-256 钉死，**一个字节都不能动** |
| `hexframe_main_integration_20261003/` | 29M | HexFrame 正式接入验收（活跃主线）；最终数据在 `full_assembly_cad_precision/`，audit PASS |
| `petal_angle1_blend030_dtcheck_20261003/` | 288M | 步长复核（含 `supplement_0p0625/`）：预注册判定**不晋升**，0.125 ms 三点收敛；报告 `results/petal_angle1_blend030_dtcheck_20261003.md`，JSON 证据已入库，NPZ（core 遥测）留本地 |
| `hexframe_halfstep_20261003/` | 9.4M | P0 半步长验证：PASS + 审计 PASS，结论对步长不敏感；入口 `experiments/hexframe_validation/run_halfstep.py` |
| `hexframe_grid_20261003/` | 74M | P1 误差网格 12 组：全 PASS（pick ±10 mm / seed ±4 mm 内）；入口 `experiments/hexframe_validation/run_grid.py` |
| `compliant_insertion/` | 36M | `insertion_suite.py` 默认输出目录（会被同名更新覆盖，历史用带日期 `--out`） |
| `petal_iiwa14_validation/` | 23M | 初版失败记录（沿用旧增益未落座）；报告声明保留，有效数据在 `_20261002` |
| `collision_comparison_20261002/`、`convex_geometry_20261002/`、`petal_model_review_20261002/` | <1M | 小体积，整体保留 |

## 已关闭阶段（结论完整保留，原始 NPZ 已精简）

以下目录保留全部 JSON（配置/门禁/峰值/审计）、PNG/PDF 图、log、`source_snapshot/`
与清单；报告中的链接全部有效。`--reanalyze` / `--reuse-from` 这类读原始 NPZ 的
流程对这些目录不再可用，需要时重跑。

| 目录 | 精简前 | 现状 | 结论出处 |
|---|---|---|---|
| `petal_guidance_geometry_20261003/` | 1.9G | 5.3M | `results/petal_guidance_geometry_validation.md` |
| `petal_capture_grid_20261003/` | 1.7G | 3.4M | `results/petal_capture_grid_validation.md` |
| `petal_lateral_control_20261003/` | 986M | 2.9M | `results/petal_lateral_control_validation.md` |
| `petal_contact_control_20261002_v2/` | 350M | 2.5M | `results/petal_contact_control_validation.md` |
| `petal_contact_control_20261002/` | 350M | 2.4M | 同上（初版 0.3 N 阈值失败记录） |
| `petal_iiwa14_validation_20261002/` | 248M | 1.5M | `results/petal_iiwa14_validation.md` |
| `convex_contact_20261002/` | 256M | 4.1M | `results/convex_collision_iiwa14.md` |
| `contact_diagnostics_20261002/` | 234M | 4.4M | `results/contact_diagnostics_iiwa14.md` |

## 已清理（2026-10-03）

- `insertion_pilot/`（11M）、`insertion_pilot_other/`（20M）：早期试点，被六组矩阵取代，无引用。
- `hexframe_new/`（12M）：`full_assembly_cad_precision/` 的逐文件一致重复副本。
- 上述 8 个关闭阶段的原始 NPZ 共 5.81 GiB（见各目录 `PURGED_NPZ_20261003.json`）。

`*_launch.log` 为对应批量的启动日志，保留。
