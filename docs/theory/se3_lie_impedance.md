# SE(3) Lie 群阻抗控制器（se3_lie）

实现自：Jonghyeok Kim, Minchang Sung, Youngjin Choi, Jonghoon Park, Wan Kyun Chung,
"Impedance Control Design Framework Using Commutative Map Between SE(3) and se(3)",
*IEEE Transactions on Robotics*, Vol. 41, 2025（§II-C/II-D 的 Eq. 26-43 数学工具 +
§III-A 的 Eq. 44-66 控制律）。CLI 以 `docking --controller se3_lie` 启用。

**本仓库不包含论文 §III-B 的 NRIC 鲁棒内环**（明确留作后续独立工作）；
本控制器是论文的标称（nominal）Lie 群阻抗控制器。

## 与另两种控制器的关系

| 控制器 | 性质 | 姿态误差表达 | 惯量重塑 | 出处 |
|---|---|---|---|---|
| `impedance` | 固定增益任务空间阻抗基线 | `log3` 姿态 PD（与世界系平动阻抗并联） | 平动通道有（m/d/k） | 本仓库历史基线 |
| `se3_lie` | SE(3) Lie 群阻抗 | `log6(T̃)` 六维指数坐标（平移-旋转耦合） | 平动+转动全通道（A/D/K） | Kim et al. 2025 T-RO §III-A |
| `hqp` | HQP-AC 约束自适应控制 | `log3` + QP 硬约束 | 无显式惯量重塑 | Ren & Shan 2026 §3.2 |

## se3_lie 不是"位置阻抗 + log3 姿态 PD"

它完整使用了论文的 SE(3) 群结构，而非把平移误差和 `log3` 姿态误差拼成 6 维向量：

- **相对位姿**：`T̃ = T⁻¹·T_d`（Eq. 44），误差在当前末端系 {b} 上表达；
- **相对 twist（右平移版本，Eq. 45）**：`Ṽ = Ad_T̃·V_d − V`，对应 `[Ṽ] = Ṫ̃·T̃⁻¹`
  （有限差分测试验证该 convention，见 `tests/test_se3_impedance.py`）；
- **六维指数坐标**：`λ = log6(T̃) = [η; ξ]`（η 平移在前），`λ̇ = dexp⁻¹_λ·Ṽ`（Eq. 48）。
  注意此处 dexp 是**右平凡化**版本（`vee(ṪT⁻¹) = dexp(λ)λ̇`），不是 `dexp_{-λ}`；
- **dexp 全家桶**（`control/lie_se3.py`，Eq. 26-43 闭式实现）：SO(3)/SE(3) 的
  `dexp`、`dexp⁻¹` 及其**解析**时间导数。SE(3) dexp 的右上耦合块 `C_ξ(η)`/`D_ξ(η)`
  保留旋转-平移耦合（不得近似为 `block_diag(dexp_ξ, dexp_ξ)`）；θ→0 全系数走
  显式 Taylor 分支；
- **等效有效 wrench**（Eq. 50/56）：`F̃ = Ad_T̃^{-T}·F_d − F`、`γ = dexp_λᵀ·F̃`，
  满足虚功率不变 `λ̇ᵀγ = ṼᵀF̃`；
- **惯量重塑**（Eq. 57-58）：阻抗在 SE(3) 上设计（A、D、K），经
  `A_λ = dexpᵀAdexp`、`D_λ = dexpᵀ(D·dexp + A·d/dt dexp)` 转入 se(3)——
  `D_λ` 一般不对称，这是设计在群上而非代数上的原因，本实现不强加对称化；
- **参考加速度 + 逆动力学**（Eq. 60-65）：`λ̈_ref = −K_Vλ̇ − K_Pλ + K_Fγ`，
  `V̇_ref = Ad_T̃V̇_d − dexp·λ̈_ref + ad_Ṽ·V − d/dt dexp·λ̇`，
  `τ = M·q̈_ref + ĥ − Jᵀ·F_body`。

`impedance` 基线的姿态通道是 `log3(R_d R^T)` 的 PD——它没有 dexp 耦合、没有
指数坐标下的惯量/阻尼一致转换、也不能通过 F/T 反馈重塑表观惯量。自由空间
阶跃实验（`experiments/se3_free_space.py`）复现了论文 §IV-A：固定 D、K 改变
A，平移阶跃超调 0%/0%/**48.6%**（A=0.5/5/100，理论值 48.6%）——动力学随期望
惯量明显变化，证明不是简单 PD。

## 主循环接线

`experiments/run_docking.py` 在 `--controller se3_lie` 下使用独立的 SE(3) 通道：

1. `planning.motion_reference.get_motion_reference` 采样 `(T_d,V_d,Vdot_d)`；
2. `SE3ToppTrajectory.get_motion_state` 可直接给出常螺旋段的 body 运动量，其他
   纯位置规划器则结合场景固定姿态完成适配；
3. `wrench.wrench_to_body` 把 sensor-site F/T 读数变换到 EE body frame 和 EE
   原点；
4. 控制器输出经共享摩擦/阻尼前馈，再由主循环执行统一力矩限幅。

因此按步姿态参考的接口已经接入，不再是待扩展项。需要注意：当前内置对接
场景的起止姿态相同，现有端到端结果仍只验证恒定姿态；旋转对接场景与对应
数值基线尚未加入。

## 坐标系约定（验收重点）

- twist `V = [v; ω]`、wrench `F = [f; n]`（线量在前，与 Pinocchio `Motion` 一致）；
- 主任务雅可比用 `pin.ReferenceFrame.LOCAL`（body Jacobian，**不是**基线用的
  `LOCAL_WORLD_ALIGNED`），`V = J_body·q̇` 与 `vee(T⁻¹Ṫ)` 有限差分一致；
- F/T 传感器读数经 `wrench.wrench_to_body` 从 sensor site 系变换到 EE body 系
  （含参考点平移矩 `(p_S − p_E) × f`）——与 `J_body` 严格同 frame、同参考点，
  逆动力学 `−JᵀF_body` 项才自洽；沿用基线读取侧的负号约定；
- 7-DoF 冗余适配：`J⁻¹`（论文 6-DoF 用）→ 动力学一致广义逆
  `J̄ = M⁻¹JᵀΛ`、`Λ = (J M⁻¹Jᵀ)⁻¹`（条件数超阈值降级 pinv 并告警记录），
  零空间 `N = I − J̄J` 只注入稳定阻尼，不改变主任务。

## 参数与配置

`config.SE3ImpedanceConfig`（场景 YAML 可选 `se3_impedance:` 段覆盖）：

```yaml
se3_impedance:
  a_diag: [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]    # 期望惯量（平动质量 / 转动惯量）
  d_diag: [80.0, 80.0, 80.0, 10.0, 10.0, 10.0] # 期望阻尼
  k_diag: [50.0, 50.0, 50.0, 25.0, 25.0, 25.0] # 期望刚度
  null_damping: 10.0                            # 零空间阻尼
```

默认值由 `ImpedanceConfig` 基线数值映射而来，仅作控制器间公平对比的兼容性
初值，**不是论文最优参数**。控制器内部以一般 6×6 矩阵持有 A/D/K（对角只是
特例，构造函数可直接注入非对角矩阵）。

## 数学验证体系

- `tests/test_lie_se3.py`（63 项）：dexp/dexp⁻¹ 逆一致性（0/小角度/中等/近 π）、
  右平凡化微分与逆微分的有限差分验证、`d/dt dexp` 解析 vs 中心差分 + 链式法则
  交叉验证、Adjoint 共轭 + Pinocchio oracle、功率不变与 Eq. 56、θ→0 稳定分支；
- `tests/test_se3_impedance.py`（18 项）：相对 twist convention 的有限差分验证、
  body Jacobian/J̇ 帧一致性、误差/力方向语义、零空间不扰主任务、Eq. 57 误差
  动力学代数闭环一致性、近 π 有限性；
- `experiments/se3_free_space.py`：论文 §IV-A 的自由空间动力学渲染验证
  （静态调节 / 平移与旋转阶跃的三档惯量 / 179° 大角度 / 零刚度方向）。

## 已知边界

- 姿态主对数分支：‖ξ‖ < π（`log6` 内在性质）；π 附近连续但论文 EEC 扩展
  （超越 injectivity radius）未实现；
- dexp⁻¹ 定义域 ‖ξ‖ < 2π（实现里对越界显式抛异常，控制器不会触界）；
- NRIC 未实现：模型失配/扰动下的鲁棒内环留作后续独立工作；
- 真实机器人未验证（无 ROS 2 驱动，本任务明确不做）。
