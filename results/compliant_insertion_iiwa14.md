# 组合对接实验：接近、定位偏差与绕轴柔顺

公头原有 40° 安装相位保留。nominal 表示零新增估计误差，不代表接口天然完美配合。
每个工况只改变绕轴刚度 25 → 0.5 N·m/rad；其他惯量、阻尼、平动/倾斜刚度完全相同。
PASS 仅表示场景声明的插入深度、保持与载荷门禁通过，不等于 CAD 完全就位或锁紧。

| 工况 | 绕轴 | 状态 | 入口进给 mm | 横偏 mm | 接触后转角 ° | 峰值力 N | 峰值轴矩 Nm | 原因 |
|---|---|---|---:|---:|---:|---:|---:|---|
| nominal | stiff | FAIL | 21.40 | 1.49 | 0.90 | 24.96 | 3.331 | axial moment limit |
| nominal | compliant | FAIL | 21.48 | 2.71 | 3.10 | 23.29 | 2.024 | axial moment limit |
| xy | stiff | PASS | 21.05 | 1.66 | 0.58 | 21.08 | 1.567 | — |
| xy | compliant | PASS | 23.02 | 0.17 | 4.10 | 14.30 | 1.577 | — |
| combined | stiff | FAIL | 19.53 | 6.39 | -0.54 | 13.29 | 1.383 | lateral offset, not settled (translation), not settled (rotation) |
| combined | compliant | PASS | 22.93 | 0.45 | -0.47 | 20.49 | 1.963 | — |

运行曲线和原始数据由 `experiments/insertion_suite.py` 写入 `runs/compliant_insertion/`。

逐步数据在同名 NPZ；完整场景、路点时间和验收结果在 JSON；运行输出在 log。

## 本次基线条件（2026-10-02）

- 场景：`scenes/iiwa14_compliant_insertion.yaml`，1 ms 步长，固定基座、零重力。
- MuJoCo 3.12.0、Pinocchio 4.1.0（pin）、NumPy 2.5.2。
- 每组约24.3 s，含8 s保持。三组 PASS、三组 FAIL；失败保留，不放宽判据。
- 表中深度从指定入口参考平面起算，含接触前空行程，不是 CAD 啮合深度。
- xy 柔顺组接触后转动4.10°，显示了允许转动的效果；combined柔顺组只转动约0.47°，不能据此宣称大范围自动角度对准。
- nominal两组都出现瞬时轴矩超限，说明仅降低绕轴刚度不能保证全部工况成功。

## 验证记录

- 新增场景及相关测试：18项通过（含完整物理插入验收）。
- 全套检查曾得到191项通过、1项旧接触锚点失败；随后新增的完整插入回归通过。
- 旧回归 `test_contact_regression_12s` 实测68.214789 mm，锚点68.095 ± 0.05 mm；在未修改主分支和同一依赖环境中复现完全相同的失败，本次未改动该锚点。
- Ruff与MkDocs严格构建通过。
