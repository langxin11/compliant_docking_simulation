# 实验复现手册

所有命令在仓库根执行；无显示环境渲染加 `MUJOCO_GL=egl` 前缀。图件输出到
`figure/<场景名>/`，对比研究数据落 `results/`。

## 1. 对接仿真

```bash
uv run docking --scene scenes/iiwa14_docking.yaml   # iiwa14（零摩擦锚点场景）
uv run docking --scene scenes/fr3_docking.yaml      # FR3（真实摩擦，torque 摩擦模式）
uv run docking --quick                              # 2s 冒烟（INCOMPLETE，仅链路）
```

- 控制器切换：`--controller impedance|hqp`；
- 结束后打印 Table 10 三层指标 + CLI 汇总行。

**基线数值**（impedance / hqp 实测，确定性可复现）：

| 场景 | 控制器 | 峰值轴向力 | 稳态轴向 | 终态误差 |
|---|---|---|---|---|
| iiwa14 | impedance | 7.90 N | -1.25 N | 78.5 mm |
| iiwa14 | hqp | 7.83 N | -1.32 N | 78.6 mm |
| FR3 | impedance | 27.21 N | -3.88 N | 13.8 mm |
| FR3 | hqp | 5.19 N | ≈0 | 21.0 mm |

iiwa14 终态误差 ~78 mm 是对接语义（规划行程 0.18 m 远超接触深度，接触后
末端被物理阻挡），并非跟踪失败；HQP 稳态≈0 需配合预紧力使用。

## 2. 跟踪测试与自由空间门禁

```bash
uv run docking --scene scenes/iiwa14_tracking.yaml   # PASS / FAIL 退出码 0 / 2
uv run docking --scene scenes/fr3_tracking.yaml      # FR3（velocity 摩擦模式）
uv run docking --scene scenes/fr3_tracking.yaml --quick  # 标记 INCOMPLETE
```

门禁阈值（场景 `tracking_thresholds:`，失败关闭语义）：位置分段 RMS ≤ 5 mm、
峰值 ≤ 15 mm、姿态 RMS ≤ 0.5°、力矩饱和 ≤ 1%、接触数 = 0。

**基线数值**：iiwa14 全程 RMS 0.229 mm；FR3 全程 RMS 1.309 mm（姿态 0.19°），
两者均零力矩饱和、零接触。

## 3. 无传感器 HQP-AC 与接触预紧

```bash
uv run docking --scene scenes/fr3_docking_sensorless.yaml --controller hqp
```

PI 动量观测器估计接触力（残差扣除已知耗散模型），8 N 检测死区 + 5 N 预紧：
实测首触 9.5 s、稳态轴向 +5.22 N（目标 5.0）、插入 14.9 mm。

## 4. 框架对比研究（论文 §4.2.3）

```bash
MUJOCO_GL=egl uv run python experiments/compare_frameworks.py
# 或单场景：--scene scenes/fr3_docking.yaml
```

2×2 矩阵（{单段五次, 两段式}×{CIC, HQP-AC}）按 Table 10 三层指标输出对比表
（`results/framework_comparison_*.md`）与图件（`figure/framework_comparison/`）。
核心结论：HQP-AC 在 FR3 摩擦场景峰值力 -81%；两段式规划在慢速基线上无时长收益。

## 5. 规划器选择

场景 `trajectory:` 段的 `type` 字段：

| type | 行为 |
|---|---|
| `twophase`（缺省） | 两段式五次：接近宽松限速 + 预对接点 + 对接段严格限速 |
| `se3topp` | SE(3) 测地线 + 解析时间最优剖面（同限速下 -39% 时长） |
| `tracking` | 圆+8字跟踪测试（无母头，`stroke` 置零） |

不写 `trajectory:` 段 → 历史单段解耦五次（15 s）。

## 6. 测试体系

```bash
uv run pytest -q            # 快速套件（含门禁集成测试）
uv run pytest -q -m slow    # 12s 全接触回归（数值锚点，不可随意调整）
uv run ruff check .
```

数值锚点说明见 `tests/test_regression.py` 文件头；重构改变行为时必须显式
重建基线并说明理由。
