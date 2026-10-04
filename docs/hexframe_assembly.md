# HexFrame 正式组装场景

`scenes/hexframe_assembly.yaml` 是正式新增的固定基座、零重力组装场景。主项目原默认 iiwa14 对接与 FR3 场景保留。HexFrame 的任务包括存储、抓取、转运、真实接触就位、卸力释放与撤离，使用独立的 `assembly_admittance` 流程；其控制器不等同于现有三个单接口阻抗对比实验。

```bash
# 全路径、几何和双模型预检；INCOMPLETE 不代表组装已通过
OPENBLAS_NUM_THREADS=1 MUJOCO_GL=egl uv run docking \
  --scene scenes/hexframe_assembly.yaml --preview-only

# 53 秒完整动力学、独立审计；输出目录必须不存在或为空
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl uv run docking \
  --scene scenes/hexframe_assembly.yaml --record --out runs/hexframe_my_run

# 回放该目录的保存状态，重新生成预览与 MP4，不重新积分
MUJOCO_GL=egl uv run docking --scene scenes/hexframe_assembly.yaml \
  --replay --record --out runs/hexframe_my_run

# 原实验入口仍有效，默认委托正式场景；旧模块也保留
uv run python experiments/orbital_showcase/crown_assembly.py --video
uv run python experiments/orbital_showcase/crown_assembly.py --module legacy --video

# 原主项目场景冒烟
uv run docking --scene scenes/iiwa14_docking.yaml --quick
uv run docking --scene scenes/fr3_docking.yaml --quick
```

未指定 `--out` 时，每次创建独立的时间戳结果目录。`--quick` 在 HexFrame 场景下只做预检；完整流程固定 1 ms 物理步长，`--duration`、`--dt` 覆盖和其他控制器不能悄悄截短或替换已验收流程。`--record` 从保存的状态生成视频，需 EGL 与中文字体；Linux 使用 Noto Sans CJK，其他系统可通过 `HEXFRAME_FONT` 指定字体文件。

## 资源、布局与模型

原始 Astra HexFrame / PetalDock100 V2 资源、CAD、网格、许可和 `IMPORT.json` 保留在 `experiments/orbital_showcase/assets/hexframe_module/`。正式配置引用此唯一资源，不重新缩放或复制原始 CAD。六侧面接口 1–6 映射为资源端口 3、4、5、0、1、2，接口 1 朝上、接口 4 朝下；端口位姿、46.4 mm 配合根间距、45° 配合相位均读取资源元数据。质量为 CAD 估算 3.5082656535 kg，完整惯量与质心同源。

工作区与工具片段位于 `assets/scenes/hexframe/`；模型构建复用主项目 `assets/iiwa14/iiwa14_arm.xml`，关节范围只读取机器人 URDF 的七个关节限制，不加载旧工具质量惯量。`AssemblyRuntime` 显式保存每次运行的布局、参考姿态、锁定 site、控制增益与输出目录，正式场景不导入或修改实验模块的共享变量。

Pinocchio 的空载臂/工具和名义刚性抓取模型从新装配模型的全精度物理字段生成；嵌套固定刚体的 site 从父 body 和源局部变换重建。两种模型分别核对控制点位姿、雅可比、质量矩阵、非线性偏置项和运动质量。固定工具声明质量 0.25 kg、惯量来自工具片段；它是当前夹持器的理想化参数，并非经过 CAD 或实测标定的整套抓取硬件。携带模型另增加原 HexFrame 的质量、惯量及真实抓取变换。

Pinocchio 用于 IK 和模型核对。实际控制继续采用验收过的 MuJoCo 偏置补偿关节伺服与轴向接触导纳，没有宣称已经迁移到主项目 Pinocchio 逆动力学阻抗控制器。携带模型用于名义刚性抓取一致性检查，物理运行中的模块仍保留自由关节并由切换 weld 求解；该检查不证明有限刚度 weld 的约束载荷与刚性模型等价。

## 交接与验收

模块 1 初始通过接口 4 锁在朝上的标准存储接口。机械臂确认接口 1 抓取锁定后，存储接口才解锁，再竖直提起；备用接口始终空闲。模块 2 的接口 4 与基座标准接口预锁定，其接口 1 接收模块 1 的接口 4。

模块间锁定仍要求真实承载止挡接触与原门限连续满足 0.5 s：轴向力 0.15–0.6 N、位置误差 ≤0.75 mm、姿态误差 ≤0.5°、相对速度 ≤0.5 mm/s、相对角速度 ≤0.5°/s、穿透 ≤0.3 mm、接触力矩 ≤0.5 Nm。接触目标保持 0.4 N。接受锚点只在持续就位通过后更新，不以理想 weld 拉近未就位模块。

机械臂释放前要求抓取约束合力 ≤0.2 N、关节速度范数 ≤0.002 rad/s 持续 0.5 s；独立审计重新核对 500 个锁定前样本、卸力窗口、交接顺序、全过程支撑、抬升后存储分离以及最终至少 70 mm 的锚点撤离。若未持续就位、未完成卸力或交接未完成，正式命令返回非零状态。

`validation.json`、`audit.json`、`geometry_check.json`、`model_consistency.json`、`planning.json` 保存结果；`rollout.npz` 为 100 Hz 状态，`contact_trace.npz` 和 `storage_trace.npz` 为独立的 1 kHz 接触日志，另存 `accepted_anchor.json`、`phases.json`、`runtime.json`。`source_snapshot/` 保存运行源码和配置，`source_manifest.json` 记录快照、原资源及原始结果的 SHA-256，后续源码修改不影响已保存快照的核对。

视频为 MuJoCo 保存状态的 24 fps 回放，说明文字放在画面说明栏和特写外，不在接口上加标签。`preview.png`、`storyboard.png`、`assembly_sequence.mp4` 和基座/存储接口特写由同一渲染器生成。

CI 运行原快速测试和新增正式场景测试，另外通过正式入口运行完整 53 秒 HexFrame 组装与独立审计，并上传诊断产物。长时原场景回归仍可用 `pytest -m slow -q` 本地执行。

## 尚未建模的机制

这是固定基座零重力名义组装验证。抓取、存储及模块间锁紧为理想 weld；基座与模块 2 是初始预锁定固定连接。尚未模拟锁销、机械预紧、解锁执行机构、电气连接、结构弹性、抓取硬件标定、自由漂浮基座反作用、轨道传播或航天器姿态控制。太阳翼为固定视觉结构。已有独立 P0 系统半步长、P1 离散布局误差网格与 P2 测量噪声/判定策略对照；范围与变量见[系统验证](system_validation.md)，历史结论见[证据索引](historical_evidence.md)。P0 的伺服/导纳/滤波随物理步长更新，P2 含判定策略变量；这些结果不能替代固定采样的单接口控制研究。尚未完成实机验证。
