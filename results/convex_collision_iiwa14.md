# iiwa14 冠形接口：凸碰撞模型与 SDF 对照

本轮接入了可选凸碰撞模型，完成几何校验和 8 组完整动力学运行。
**凸模型显著改善首次接触的几何一致性，但尚不能作为可靠默认。**
默认场景与 SDF 基线保留；没有为了使实验通过而调整控制器或验收阈值。

## 模型与几何验收

原 STL 的 8 个连通组件中，3 个小组件接近凸体；四个较大组件的凸包体积增加约
32.3%，主体增加约 73.2%。直接取 8 个凸包，在紧凑参考相位附近会将首次接触高度
抬高约 0.92 mm，不能用作本接口的忠实碰撞表示。

新模型按组件使用 CoACD 1.0.14：固定 seed 42、归一化阈值 0.01、禁止重建网格、
减顶点和分块膨胀。三个近凸组件直接使用凸包。共得到 **130 个凸块/接口**。
每块是独立 OBJ/mesh geom，挂在原来的 rev body 上。

- 原视觉 STL、工具内部 40° 安装角、挂载位姿、传感器和显式质量/惯量保留。
- 原名 dock_geom 的完整网格保留为禁用碰撞的几何参考；运行时只显示原视觉网格。
- CoACD 只用于离线生成，不是仿真运行依赖。
- 清单保存源资产、生成资产、版本、参数、来源和校验结果。运行入口拒绝未通过或
  修改过的生成资产；清单路径相对于仓库，允许正常移动/克隆项目。

校验覆盖同轴 0–355°（5° 步长），以及四个紧凑相位 ±2°（0.25° 步长）与
XY 各 -2/0/+2 mm 的组合，共 **684 个位姿**。高度网格通常 0.5 mm，最大误差
样本与超限样本加密到 0.25 mm；这是一套采样研究验收，不是连续碰撞或制造证书。

| 指标 | 实测 |
|---|---:|
| 分块表面采样的最大额外材料距离 | 0.1944 mm |
| 最大缺失材料采样距离 | 0.01265 mm |
| 最大接近高度误差 | 0.07821 mm |
| 四个紧凑参考相位偏移 | 0°（0.25° 扫描精度） |
| 12 个独立 native 碰撞首触扫描与 CAD 的最大偏差 | 0.10 mm（扫描步长 0.10 mm） |

首触扫描覆盖 0°、45°、四个参考相位，以及 XY=(0,0)/(2,-2) mm。
凸模型在此范围内符合原网格首触；原 SDF 在部分相位提前约 5.1–12.1 mm 接触。
形状与首触通过不代表动态接触或机械锁定通过。倾斜误差在运行后的原始网格验收中
检查，没有把这次高度采样当作所有倾斜姿态的几何保证。

![分块检查](../runs/convex_geometry_20261002/collision_parts.png)

![几何误差](../runs/convex_geometry_20261002/geometry_validation.png)

## 完整动力学对照

沿用原有 nominal、XY、XY+yaw 三工况，以及高/低绕轴刚度。真值仅用于构造误差
与评估，轨迹继续只使用目标估计。六组新模型与保存的 SDF 数据逐组核对完整配置、
控制/轨迹/仿真回路源文件指纹和运行库版本；仅更换两个接口片段的路径。
同步接触分析与 SDF 的保存数据再分析版本一致。

以下为真实接口**净**轴向接触力矩的逐步峰值，限值为 2 N·m。

| 工况 | 绕轴刚度 | SDF 峰矩 Nm | 凸模型峰矩 Nm（1 ms） | 凸模型几何 | 凸模型综合 |
|---|---|---:|---:|---|---|
| nominal | 高 | 4.344 | 0.697 | NOT_SEATED | FAIL |
| nominal | 低 | 2.877 | 4.474 | NOT_SEATED | FAIL |
| XY | 高 | 3.079 | 6.197 | NOT_SEATED | FAIL |
| XY | 低 | 2.387 | 6.715 | NOT_SEATED | FAIL |
| XY+yaw | 高 | 2.944 | 0.269 | NOT_SEATED | FAIL |
| XY+yaw | 低 | 3.576 | 0.292 | NOT_SEATED | FAIL |

nominal 加做 0.5 ms 步长：高刚度峰矩 0.218 N·m，低刚度峰矩 0.152 N·m；
两者几何仍 NOT_SEATED。这些变化说明逐步峰值尚未收敛，不能仅凭半步长通过
载荷阈值就认可模型。XY 工况尚未做半步长对照。

8 组均未通过综合验收。1 ms 的 nominal 高刚度还存在接触保持不足；其他组
原 F/T 门禁通过，但 nominal 低刚度和两组 XY 的真实接触载荷超限。
nominal/XY 的最终绕轴角接近 0°，与约 4.25° 紧凑参考相位不匹配；XY+yaw 的
相位较接近，但横向残差约 1.61/2.13 mm，且仍高于紧凑参考约 0.76/1.04 mm。

载荷平衡矩残差均小于 1.2e-13 N·m。F/T 与真实接触载荷继续受到惯性项的影响：
例如 XY 低刚度 F/T 峰矩约 0.546 N·m，真实接触峰矩约 6.715 N·m。
净力矩阈值也不能代替逐接触载荷、应力或材料强度验收。

[完整配对报告](../runs/collision_comparison_20261002/report.md)、
[凸模型运行报告](../runs/convex_contact_20261002/report.md)保留曲线、JSON、NPZ、
逐接触位置与载荷。运行中修改的路径指纹/展示辅助函数和命令帮助被运行清单记录；
控制、轨迹、完整物理回路未变，模型资产在所有运行期间未变。

## 复现与下一步

```bash
# 可选离线生成依赖；已有生成资产时不需要执行。
MUJOCO_GL=egl uv run --group geometry python experiments/prepare_convex_interface.py

# 仅重做几何验收，不需要安装 CoACD。
MUJOCO_GL=egl uv run python experiments/prepare_convex_interface.py --validate-only

# 六组对照 + nominal 两组半步长；预期当前综合 FAIL 时返回 2 并保留结果。
MUJOCO_GL=egl uv run python experiments/insertion_suite.py \
  --collision convex --diagnose --sensitivity --preview \
  --out runs/convex_contact_20261002

# 保存数据再分析，不重跑物理。
MUJOCO_GL=egl uv run python experiments/insertion_suite.py \
  --collision convex --reanalyze --sensitivity --out runs/convex_contact_20261002

# 将已保存的新旧数据配对比较。
uv run python experiments/compare_collision_models.py
```

1. 在 XY 工况补充半步长/更小步长，定位真实接触峰值对应的分块、活动接触数、
   接触距离与接触载荷，检查分块边界或重复约束是否相关。当前结果尚未证明具体原因。
2. 比较合并分块或利用 CAD 特征的更少凸块；每次都重做形状、首触和完整动力学验收。
   同时评估接触参数和步长的收敛，不预设凸分解会更快或更稳定。
3. 接触可信后再设计接触后的受限绕轴搜索与横向对中；用估计/测量驱动，不把
   4.25° 几何真值直接输入控制器。仅靠现有被动柔顺尚未实现稳定就位。
4. 继续可实现的 F/T 惯性补偿与时序统一；诊断用真值保持评估专用。

快速测试 202 项通过，4 个 slow 测试未运行；本轮实际执行了上述 8 组完整轨迹。
新增 4 项测试覆盖面内距离、凹槽误差、原质量/安装/传感器保留和错误资产拒绝。
Ruff 与文档严格构建用于代码/文档检查。

MuJoCo 对普通 mesh 的凸碰撞规则及 CoACD 的推荐见
[官方文档](https://mujoco.readthedocs.io/en/latest/computation/index.html#convex-decomposition)；
生成参数、逐块输出和阈值限制见 [CoACD 官方说明](https://github.com/SarahWeiii/CoACD)。
