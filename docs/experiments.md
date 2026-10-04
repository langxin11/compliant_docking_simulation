# 实验复现手册

所有命令在仓库根执行；无显示环境渲染加 `MUJOCO_GL=egl` 前缀。图件输出到
`figure/<场景名>/`，对比研究数据落 `results/`。

新增组合接触实验使用独立的 `runs/compliant_insertion/` 输出目录：

```bash
MUJOCO_GL=egl uv run python experiments/insertion_suite.py --preview
```

该入口将侧方初值、上方路点、XY/yaw 偏差与绕轴刚度比较合并为六组完整仿真；
统一保存曲线、原始状态、门禁和场景配置。[任务、判据与后续方案](development_plan.md)。

几何与接触诊断使用同一入口；以下命令运行六组基线和四组 nominal 数值对照：

```bash
MUJOCO_GL=egl uv run python experiments/insertion_suite.py \
  --diagnose --sensitivity --preview --out runs/contact_diagnostics_20261002
```

原 F/T 门禁、网格就位候选、真实接触载荷门禁与综合评估分别保存；综合失败返回 2，
原始数据仍保留。已有诊断数据可加 `--reanalyze --sensitivity` 重建门禁和图件，不重新仿真。

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
| `waypoints` | 显式估计目标驱动的接近/下降/插入；与 `docking` 段及 `se3_lie` 配合 |

不写 `trajectory:` 段 → 历史单段解耦五次（15 s）。

传统对接场景的起止姿态相同，`se3topp` 的原有数值基线仍是恒定姿态。
新增 `waypoints` 组合场景覆盖接近阶段姿态过渡和被动绕轴柔顺；持续改变接触阶段
期望姿态的旋转插入仍待验证。

## 7. 测试体系

```bash
uv run pytest -q            # 快速套件（含门禁集成测试）
uv run pytest -q -m slow    # 12s 全接触回归（数值锚点，不可随意调整）
uv run ruff check .
```

数值锚点说明见 `tests/test_regression.py` 文件头；重构改变行为时必须显式
重建基线并说明理由。

## 可选凸碰撞模型

原始 STL 的凹面不能直接替换成整件凸包。已生成的 130 块接口代理通过了采样几何
验收，安装、视觉与显式惯量保留；动态对照尚未通过综合验收，默认仍是 SDF。

```bash
MUJOCO_GL=egl uv run python experiments/insertion_suite.py \
  --collision convex --diagnose --sensitivity --preview --out runs/convex_contact_20261002
uv run python experiments/compare_collision_models.py
```

`--sensitivity` 对凸模型仅增加 nominal 半步长；SDF 还增加迭代加密。
离线重新生成使用 `uv run --group geometry python experiments/prepare_convex_interface.py`；
仅校验使用 `--validate-only`，运行时不需要 CoACD。生成资产指纹与几何门禁会在选择
凸模型时检查。完整科学验收失败时退出码为 2，结果照常保存。

详细结果与下一步见仓库中的 `results/convex_collision_iiwa14.md`。

## PetalDock100 接入机械臂

`experiments/petal_insertion_suite.py` 使用与原组合实验相同的机器人控制循环。
固定高/低绕轴刚度分别为 25 和 0.5 N·m/rad；`released` 使用同一低刚度，
接触后释放到零。其余 A/D/K、初始状态和轨迹相同；工况是无估计误差、
XY 偏差 (2, −2) mm、XY 同时叠加 5° 偏航。控制周期固定为 0.5 ms，
`dt_half` 仅把物理步长从 0.5 改为 0.25 ms。

场景 `iiwa14_petal_insertion.yaml` 声明新工具的法兰根 frame、45° 配合相位、
名义落座间距 46.4 mm，以及独立的估计目标。真实目标位姿仅进入物理和评估，
不进入轨迹生成。Pinocchio 从同一份组装 MJCF 读取完整工具惯量；初始姿态无
接触，轨迹经过连续 IK 检查，运行时保留关节限位与非接口接触检查。

```bash
OPENBLAS_NUM_THREADS=1 MUJOCO_GL=egl uv run python experiments/petal_insertion_suite.py \
  --jobs 2 --preview
OPENBLAS_NUM_THREADS=1 MUJOCO_GL=egl uv run python experiments/petal_insertion_suite.py \
  --case combined --setting dt_half --jobs 2 --resume
```

输出目录默认 `runs/petal_contact_control_current`，建议每次显式指定新的 `--out`。
历史验证数据保留在 `runs/petal_contact_control_20261002_v2`。`preflight.json` 保存运动学、
质量矩阵、关节范围与初始接触核对；`source_manifest.json` 在开始时固定源码和
模型哈希。源码改变后拒绝续跑到同一目录，避免混合不同实现的结果。

F/T 在每个控制周期的第一个物理求解时刻采样，下一控制周期使用，延迟固定为
一个控制周期。先按当时的 site 位姿存为世界系 wrench，反馈时再搬移/旋转到
当前控制 body。初始反馈为零；没有额外约束求解。NPZ 中的 `control_*` 记录
控制时刻、原始样本时刻、样本世界 wrench/原点、控制 body 位姿、反馈 wrench、
接触检测滤波载荷和实际绕轴/XY 刚度。`feedback_audit` 核对延迟、坐标变换与求解位姿。

场景的 `se3_impedance.contact_yaw` 声明检测与释放参数：插入/保持阶段用
绝对轴向 F/T 载荷，10 ms 滤波，超过 0.15 N 持续 20 ms 后锁存触发；
0.25 s 平滑释放，接触短暂消失不重新加刚度。运动阶段的惯性载荷不触发释放。
策略不读取目标真值、接触对或止挡标记；这些量只进入评估与显示。

逐步 NPZ 保存求解时刻的 F/T、接触合力、接触力矩、惯性平衡和承载止挡接触数。
compact 模式不保存所有接触点事件，减少多凸块模型的内存开销；数值门禁依然
使用每个仿真步的峰值。每组 JSON 分别保存原动态门禁、接触载荷门禁、几何门禁
与综合结论。近似名义落座和持续止挡接触不等于锁紧，场景没有 weld 等式约束。

普通矩阵自动比较 combined 的 0.5 与 0.25 ms，输出 `numerical_comparison.json`。
`--setting dt_quarter` 可记录 0.125 ms，控制周期不变；普通矩阵不自动比较该组，
`--lateral-study` 会保存代表点的全部相邻比较。比较要求综合状态
相同，横向/轴向差不超过 0.1 mm、相位差不超过 0.1°；峰值力差不超过
max(0.5 N, 10%)，峰值轴矩差不超过 max(0.05 N·m, 10%)。这是声明的研究比较
阈值，不是数学收敛证明。

此前结果见 [接触后的绕轴释放验证](reports/petal_contact_control_validation.md)：
四组释放策略均达到落座候选条件，组合误差的两个物理步长通过原比较阈值。
四组配置一致的固定刚度对照由来源校验后复用；NPZ 未修改，JSON 保存 provenance。
初版 0.3 N 检测阈值及其未触发释放的无误差组记录保留在旧输出目录。

## PetalDock100 捕获范围的离散扫描

`petal_insertion_suite.py --grid` 以 X、Y 各 −6/0/6 mm，偏航 −15/0/15° 的
27 个完整流程为初始网格，直接复用 `run_case`，不重新实现控制或评分。
每组保持零重力、同一起点、原 A/D/K、0.15 N 接触检测、8 s 保持和全部验收门槛。

边界补点依据实际已运行点的通过/失败转换，预算 4 个中点；不假设正负对称。
粗网格若全通过，先检查对角 XY 或偏航的更大误差探针。最多 4 个代表性节点以
0.25 ms 物理步长复核，控制周期/反馈延迟保持 0.5 ms。相邻点或复核清单持久保存，
恢复运行沿用清单，避免预算在续跑时扩张。

`grid_plan.json` 固定轴值及预算；每组 JSON 的 `grid_error`、`grid_stage` 区分
coarse、anchor、outer、refinement、boundary_check。不可达路径单独标为
INFEASIBLE_PLAN，不作为接触捕获失败混淆。NPZ 保存逐步位姿、F/T、接触合力、
止挡承载、反馈时间和实际刚度。复用原样本时逐项验证有效场景（实验名称除外），
并核对控制、仿真、物理、模型、规划、日志与评分；原始数据不变。

图中绿色/红色只说明离散采样点。边界的两点或中点均不证明整条线段或整个色块
内的成功；两个步长不同的节点应单独标记为数值敏感，不能计入已复核范围。

## PetalDock100 接触后的横向释放

`petal_insertion_suite.py --lateral-study` 仍使用上述 `run_case` 和同一组门禁。
九个声明的误差点各跑原 `released` 与新 `lateral_released`，共十八个完整流程。
新策略只将 `contact_yaw.lateral_stiffness_after` 设为零，接触前保持 XY 刚度 80 N/m，
触发后与绕轴刚度共用 0.25 s 的平滑下降过程。轴向预紧、倾斜刚度、惯量和阻尼不变。
省略可选字段保持原 XY 行为；不得设置高于接触前的刚度或负值。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl uv run python experiments/petal_insertion_suite.py \
  --lateral-study --jobs 3 --out runs/petal_lateral_reproduction --resume
```

三组预选代表点追加半步长；敏感点再追加四分之一步长，上限三组。
若最新相邻比较仍敏感且该点落座，再按冻结的一点预算独立比较插入速度减半的
0.25/0.125 ms 两组。减速不混入主配对，也不改变门禁。所有预算和源码在启动时固定，
续跑拒绝不同配置或实现。输出包括原始 NPZ、每组 JSON/log、`study_plan.json`、
`matched_pair_checks.json`、`summary.json` 和 `numerical_comparison.json`；
入口完成代表计划已执行，具体科学结论以各组 `assessment` 和数值比较为准。

本轮九组配对的触发前参考与关节轨迹完全相同。单轴 ±6 mm 的四个点由未持续
承载止挡转为落座候选，原成功的三个点仍通过；两个 XY＋15° 组合仍卡滞。
结果、独立审计和统一风格图像见 [横向释放验证](reports/petal_lateral_control_validation.md)。
可直接运行 `iiwa14_petal_lateral_insertion.yaml` 演示单轴 −6 mm 的半步长组，
启动时显式指定 `--dt 0.00025`。没有验证多轴组合的整个区域或锁紧。

## PetalDock 导向几何对照

`petal_insertion_suite.py --geometry-study` 比较原模型和两个可选导面原型，继续复用
同一个 `run_case`、F/T 接触释放和全部验收条件。三组都使用接触后 XY/yaw 释放，
不混入其他控制策略、减速或摩擦改变。生成参数与模型清单见
[导向模型说明](reports/petal_guidance.md)。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl uv run python experiments/petal_insertion_suite.py \
  --geometry-study --jobs 3 --out runs/petal_guidance_reproduction --resume

# 完成矩阵后，从完整逐步记录重算门禁、反馈约定和图表
uv run python experiments/petal_guidance_report.py runs/petal_guidance_reproduction
```

主矩阵是 9 个离散误差点 × 3 个几何，共 27 次新复跑，物理步长 0.25 ms。
9 点包含单轴 X/Y 的 ±6 mm、两个 (0,∓6) mm / ±15° 组合、无误差、
(2,−2) mm / +5° 和纯 −15° 偏航。对三个模型逐点核对相同的参考位置和时间。

再按冻结预算追加 4 次 0.125 ms 物理步长复核：原几何的一个组合卡滞点，以及
新候选中主矩阵通过数最多、最大峰值力最低者的两个组合点与 −6 mm X 点。
控制周期和 F/T 延迟均保持 0.5 ms；比较采用原状态、残差和载荷差阈值。
两个步长比较稳定不是完整收敛证明，单个采样点也不是连续捕获区域。

运行前先使用原生凸碰撞模型核对名义首触高度和静态纠偏趋势。静态自由体的
0/0.15/0.3 摩擦探针不属于完整机器人捕获对照；机器人全过程摩擦只使用原 0.15。
新模型更新完整质心和惯量，并核对 Pinocchio/MuJoCo 位姿和质量矩阵；
质量积分网格加密不能替代接触碰撞离散的收敛验证。

`study_plan.json`、源码快照、资产哈希、环境版本、静态检查、每组 JSON/NPZ/log、
`reference_checks.json`、`numerical_plan.json`、`summary.json` 和审计/图件一并保存。
所有运行都是新的，历史数据只用于原几何回归核对。模型或运行源码变化时续跑拒绝
混用；完成并冻结的目录也拒绝再续跑，复现应使用新的输出目录。

独立研究场景 `iiwa14_petal_guided_insertion.yaml` 复现角向候选的
(0,−6) mm / +15° 主矩阵组，有效参数与对应 JSON 一致；运行时显式指定
`--dt 0.00025`。单场景 CLI 的进给门禁不等于全部几何与接触载荷验收，
完整结论以矩阵的 `assessment` 为准。

本轮全部 31 次仿真和完整数据审计已完成：0.25 ms 主矩阵中原轮廓 7/9、角向候选
9/9、径向候选 6/9 达到落座候选。四组 0.25/0.125 ms 比较均符合原差异门槛，
其中原几何卡滞点仍失败、三个角向候选复核点仍通过。径向候选三个偏航工况超限，
保留其完整数据与失败原因；没有降低摩擦、改变控制或放宽门禁。
[完整结果、图件与采用范围](reports/petal_guidance_geometry_validation.md)。
