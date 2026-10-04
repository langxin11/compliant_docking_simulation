# 冠形接口几何与载荷诊断（2026-10-02）

本轮把网格参考、接触点、F/T 惯性分解和数值对照合入 `experiments/insertion_suite.py`。
执行六组完整基线和四组 nominal 数值变体；控制反馈、刚度和接口资产保持原配置。
六组基线的关节状态、速度、控制力矩、末端位置、F/T 轴矩及原门禁与上一轮逐样本完全一致。

**原门禁通过不等于接口完成。六组实际接触轴矩均超过原有 2 N·m 限值，
十组试验均未同时通过稳定保持、几何候选与真实接触载荷验收。**

## 独立几何参考

从编译网格恢复实际接口根坐标系，采样竖直射线的上下包络，扫描 0–360° yaw，
再按 0.25° / 0.25 mm 细化局部紧凑参考。公头内部 40° 安装角包含在计算中。

| 公头根相对母头根的 yaw | 根坐标系间高度 |
|---:|---:|
| 4.25° | 47.187 mm |
| 94.00° | 47.206 mm |
| 184.25° | 47.418 mm |
| 274.00° | 47.397 mm |

这些是近似等价的采样几何参考，90° 重复的高度差最大约 0.376 mm。
网格含 8 个闭合连通分量，未发现边界边或非流形边；这不证明组件无重叠或无自相交。
当前研究容差为横偏 0.5 mm、倾斜 0.5°、相位 2°、轴向/采样重叠 0.75 mm。
检查末 1 s 的五个位姿，仍未验证制造公差、连续碰撞、完整啮合或锁紧。

xy/combined 的低绕轴刚度组符合 `SEATED_CANDIDATE`；其他工况存在横偏、相位、
倾斜或高于紧凑参考的问题。这一结果只用于评估，没有把真实目标或几何真值反馈给控制器。

## 原 F/T 与真实接触载荷

接触合力及轴矩取对公头的载荷，力矩参考点统一为公头根，轴向取真实目标 +Z。
逐接触载荷独立于 F/T 读数，从求解器接触记录求和；有效惯性项由工具刚体的空间惯量、
速度和加速度计算。传感器 = 接触合载荷 - 有效惯性载荷（含重力项）。
十组力矩平衡残差均小于 1e-12 N·m。

| 工况 | 绕轴刚度 | 原门禁 | 几何状态 | F/T 峰矩 Nm | 接触峰矩 Nm | 真实接触门禁 |
|---|---|---|---|---:|---:|---|
| nominal | 高 | FAIL | NOT_SEATED | 3.331 | 4.344 | FAIL |
| nominal | 低 | FAIL | NOT_SEATED | 2.024 | 2.877 | FAIL |
| xy | 高 | PASS | NOT_SEATED | 1.567 | 3.079 | FAIL |
| xy | 低 | PASS | SEATED_CANDIDATE | 1.577 | 2.387 | FAIL |
| combined | 高 | FAIL | NOT_SEATED | 1.383 | 2.944 | FAIL |
| combined | 低 | PASS | SEATED_CANDIDATE | 1.963 | 3.576 | FAIL |

nominal 低刚度组的 F/T 峰值时刻，轴矩分别为传感器 +2.024、接触 +0.571、
有效惯性 -1.453 N·m；惯性项显著放大了读数。其他时刻惯性项会抵消接触载荷，
因此不能仅使用 F/T 峰值判定接口载荷安全。表内各峰值来自各自时序的最大值，
不能把不同列的峰值直接相减。

真实接触门禁沿用 40 N / 2 N·m 限值，评价合载荷。逐接触载荷也保留，
但尚没有接触应力或单接触点载荷的材料强度判据。

## 数值对照

只对 nominal 两种刚度增加半步长和 SDF 迭代加密，其他参数保持一致。

| 绕轴刚度 | 设置 | 原门禁 | F/T 峰矩 Nm | 接触峰矩 Nm | 真实接触门禁 | 几何状态 |
|---|---|---|---:|---:|---|---|
| 高 | 0.5 ms，SDF 10/40 | PASS | 1.029 | 2.542 | FAIL | NOT_SEATED |
| 高 | 1 ms，SDF 20/40 | PASS | 1.050 | 1.368 | PASS | NOT_SEATED |
| 低 | 0.5 ms，SDF 10/40 | PASS | 0.715 | 2.871 | FAIL | NOT_SEATED |
| 低 | 1 ms，SDF 20/40 | PASS | 0.817 | 1.490 | PASS | NOT_SEATED |

原 PASS/FAIL 对步长和求解设置敏感，尚未证明收敛。SDF 初始化点数尝试增到 80 时
触发 MuJoCo 接触点数量上限；正式对照保持 40，仅增加迭代次数。

隔离接口的下降扫描也显示，SDF 首次超过 1 µm 穿透的高度，在 yaw=0° 时比网格
首触高度高约 6.22 mm，在 yaw=4.25° 时高约 12.06 mm。加倍迭代次数及把距离场深度
从 6 改为 8，均未消除这两个相位的差异。45° 相位约差 -0.25 mm，接近扫描步长。
这提示应继续核对距离场及其候选接触；尚不能把所有冲击都归为虚假接触。

## 复现与验证

```bash
MUJOCO_GL=egl uv run python experiments/insertion_suite.py \
  --diagnose --sensitivity --preview --out runs/contact_diagnostics_20261002

# 只重算派生验收和重绘，不重复物理仿真
MUJOCO_GL=egl uv run python experiments/insertion_suite.py \
  --reanalyze --sensitivity --out runs/contact_diagnostics_20261002
```

完整数据、图件和综合表格在 `runs/contact_diagnostics_20261002/`；原基线仍保留在
`runs/compliant_insertion/`。失败退出码 2 是研究结果，图件和原始数据照常保存。
2 s 快速检查和其重新分析均标为 INCOMPLETE。

已验证接触作用反作用与力臂、旋转传感器、重力/惯性分解、已知网格射线高度、
编译网格坐标恢复，以及被动记录不改变控制和积分前姿态对齐。
全项目 198 项快速测试通过；29 项相关检查、Ruff 和 MkDocs 严格构建通过。
十组完整试验均保存数据和验收结果；本轮未重跑四项慢回归。

下一阶段先核对接触几何表示，并建立可实现的传感器惯性补偿及同一时刻反馈；
待接触载荷和几何结果随数值加密稳定后，再引入分阶段阻抗、减速或绕轴搜索。

MuJoCo 的接口定义参见 [F/T 传感器](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-force)、
[接触力 API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-contactforce)、
[数据一致性](https://mujoco.readthedocs.io/en/stable/computation/index.html#consistency-in-mjdata)。
