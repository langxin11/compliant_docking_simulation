# 实验复现手册

所有命令在仓库根执行；无显示环境渲染加 `MUJOCO_GL=egl` 前缀。图件输出到
`figure/<场景名>/`，对比研究数据落 `results/`。

## 1. 对接仿真

```bash
uv run docking --scene scenes/iiwa14_docking.yaml   # iiwa14（零摩擦锚点场景）
uv run docking --scene scenes/fr3_docking.yaml      # FR3（真实摩擦，torque 摩擦模式）
uv run docking --quick                              # 2s 冒烟（INCOMPLETE，仅链路）
```

- 控制器切换：`--controller impedance|se3_lie|hqp`；
- 结束后打印 Table 10 三层指标 + CLI 汇总行。

其中 `se3_lie` 使用 `(T_d, V_d, Vdot_d)` body 运动参考和 EE body wrench；
`impedance` / `hqp` 保持既有世界轴对齐任务空间接口。任一现有规划器都可与
`se3_lie` 配合：SE(3)-TOPP 直接提供完整运动参考，纯位置规划器由适配器结合
场景固定期望姿态补齐。

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

## 3. SE(3) Lie 群阻抗自由空间验证

先运行快速链路，再运行完整验收和论文风格图件：

```bash
uv run python experiments/se3_free_space.py --quick
uv run python experiments/se3_free_space.py
uv run python experiments/se3_free_space_plots.py
# 自定义图件目录：.../se3_free_space_plots.py --out /tmp/se3-free-space
```

`se3_free_space.py` 覆盖五组实验并以退出码报告 PASS/FAIL：

| 实验 | 验证目标 |
|---|---|
| `static` | 小位姿误差收敛且无 NaN/力矩爆炸 |
| `trans_step` | 固定 D=K=20、改变 A，验证平移惯量重塑 |
| `rot_step` | 固定 D=K=10、改变 A_rot，验证转动惯量重塑 |
| `large_rot` | 179° 姿态调节在主对数分支内稳定收敛 |
| `zero_stiffness` | 零刚度方向卸载后不回位，高刚度方向恢复 |

完整平移阶跃的已验证超调为 0% / 0% / **48.6%**（A=0.5 / 5 / 100），
与二阶理论值一致。绘图脚本将 PNG 与 PDF 写入
`figure/se3_free_space/se3_free_space_validation.*`。这些实验验证的是 Kim et al.
2025 §IV-A 的标称 SE(3) 阻抗；不包含该论文 §III-B 的 NRIC 鲁棒内环。

## 4. 无传感器 HQP-AC 与接触预紧

```bash
uv run docking --scene scenes/fr3_docking_sensorless.yaml --controller hqp
```

PI 动量观测器估计接触力（残差扣除已知耗散模型），8 N 检测死区 + 5 N 预紧：
实测首触 9.5 s、稳态轴向 +5.22 N（目标 5.0）、插入 14.9 mm。

## 5. 框架对比研究（论文 §4.2.3）

```bash
MUJOCO_GL=egl uv run python experiments/compare_frameworks.py
# 或单场景：--scene scenes/fr3_docking.yaml
```

2×2 矩阵（{单段五次, 两段式}×{CIC, HQP-AC}）按 Table 10 三层指标输出对比表
（`results/framework_comparison_*.md`）与图件（`figure/framework_comparison/`）。
核心结论：HQP-AC 在 FR3 摩擦场景峰值力 -81%；两段式规划在慢速基线上无时长收益。

## 6. 规划器选择

场景 `trajectory:` 段的 `type` 字段：

| type | 行为 |
|---|---|
| `twophase`（缺省） | 两段式五次：接近宽松限速 + 预对接点 + 对接段严格限速 |
| `se3topp` | SE(3) 测地线 + 解析时间最优剖面；提供完整 `(T_d,V_d,Vdot_d)` body 参考（同限速下 -39% 时长） |
| `tracking` | 圆+8字跟踪测试（无母头，`stroke` 置零） |

不写 `trajectory:` 段 → 历史单段解耦五次（15 s）。

内置对接场景的起止姿态当前相同，所以即使选择 `se3topp`，现有数值基线仍是
恒定姿态。时变姿态接口已经接入，但尚未提供内置旋转对接场景及其端到端基线。

## 7. 测试体系

```bash
uv run pytest -q            # 快速套件（含门禁集成测试）
uv run pytest -q -m slow    # 12s 全接触回归（数值锚点，不可随意调整）
uv run ruff check .
```

数值锚点说明见 `tests/test_regression.py` 文件头；重构改变行为时必须显式
重建基线并说明理由。
