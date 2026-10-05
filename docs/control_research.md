# 柔顺控制算法研究协议

本页历史 RQ1/RQ2 与捕获范围协议固定原轮廓 `scenes/iiwa14_petal_original_insertion.yaml`，
不跟随 narrow 默认接口切换；模型版本见[模型基线](models_interfaces.md)。本页预先声明主要变量、配对条件、
评价与适用范围；结构迁移不引入算法改进或新的研究结论。

2026-10-04 收束：以下保留可复现协议，当前不要求把数值矩阵全部重跑。
主结果是同点完成情况、误差与失败边界；峰值为辅助结果。新工作按[收束决策](research_focus.md)执行，
历史载荷门槛与评价标签仍按原规则解释。

## RQ1：接触后的绕轴释放

假设：接触后持续拉向有偏差的偏航目标可能阻碍被动相位纠偏；只释放绕轴刚度，
保留惯量、阻尼及其他轴约束，可改变承载止挡与稳定落座行为。

| 策略 | 绕轴刚度 N·m/rad | 接触后行为 |
|---|---:|---|
| stiff | 25 | 固定 |
| compliant | 0.5 | 固定 |
| released | 0.5 | 载荷触发后平滑降到 0 |

固定横向 80 N/m、轴向 1500 N/m、倾斜 25 N·m/rad；A/D、零空间阻尼、轨迹、初值和模型不变。
触发只在插入/保持阶段使用轴向 F/T：0.15 N，10 ms 滤波，20 ms 保持，250 ms 释放。
真值不进入控制。主工况 nominal、xy=(2,-2) mm、combined=(2,-2) mm/+5°；同点比较三策略。

```bash
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.control.rq1_yaw \
  --case combined --profile stiff compliant released --setting baseline dt_half \
  --out runs/rq1_my_run --jobs 2
```

## RQ2：固定绕轴策略后的横向释放

假设：保持 XY 弹簧会将工具回拉向错误目标；触发后仅将 XY 刚度由 80 降到 0 N/m，
可影响横向残差和承载止挡。两策略均采用相同绕轴释放、A/D、轴向推进与检测。

主配对采用历史九个离散点：XY 单轴 ±6 mm、两组 ±6 mm/∓15° 组合、
nominal、(2,-2) mm/+5°、零 XY/-15°。同点参考与触发前状态应完全一致。

```bash
uv run python -m experiments.control.rq2_lateral --stage paired --jobs 3 --out runs/rq2_my_run
# 需要复现历史数值协议或诊断主结论时，显式运行；不是 paired 的自动后续
uv run python -m experiments.control.rq2_lateral --stage numerics --jobs 3 --out runs/rq2_my_run
```

主配对不自动追加减速。`--stage speed` 是独立因子，单独目录、同点同策略对照
10/5 mm/s，不能替代横向配对或将卡滞改写为数值通过。旧 `--lateral-study` 仍执行历史完整协议。

## 判据、失败与数值复核

落座候选要求保持末 1 s：横偏 ≤0.5 mm、倾斜 ≤0.5°、相位误差（模 90°）≤2°、
名义间距偏差 ≤0.75 mm、承载止挡占比 ≥95%、线/角速度 ≤3 mm/s 与 2°/s。
全程净接口接触力 ≤40 N、绕轴矩 ≤2 N·m，同时满足原进给、F/T、关节安全和静止门禁。
无 weld 锁定，`CANDIDATE_PASS` 仅表示研究落座候选。未完成与规划不可达单独记录。

遥测保存求解时刻状态、载荷、参考、触发与刚度；反馈独立审计核对时间年龄、body 变换和载荷平衡。
误差网格声明离散适用范围，不宣称连续捕获区域。物理步长 baseline/dt_half/dt_quarter 固定控制周期
与延迟；比较状态及横/轴向 ≤0.1 mm、相位 ≤0.1°、峰值力差 ≤max(0.5 N,10%)、
轴矩差 ≤max(0.05 N·m,10%)。两步长一致不能证明数学收敛。

## 已有证据与缺口

[绕轴历史对照](../results/petal_contact_control_validation.md)支持其声明的同点结论；
[横向历史配对](../results/petal_lateral_control_validation.md)记录九点、两个组合卡滞及粗步长敏感结果。
两轮工况集合不同，不能合成统一成功率曲线。当前三控制器属于方法库；
[框架历史对照](historical_evidence.md)与自由空间验证不替代接触策略研究。

已完成[主结果、卡滞边界与集成选择](control_main_results.md)的摘要核对，下一步推进候选策略接入 HexFrame。
精细步长、搜索、预紧或控制周期研究不自动追加；既有 SENSITIVE 结果作为补充限制保留。
