# 六侧面接口空间组装场景

> HexFrame 现已接入主项目 `docking --scene scenes/hexframe_assembly.yaml`。
> 原 `crown_assembly.py --module hexframe` 入口委托给同一正式流程；新运行默认写入唯一的 `runs/hexframe_assembly_*` 目录，历史 `outputs/` 不覆盖。
> 几何、接触和渲染维护在 `src/compliant_docking/assembly/`；旧模块仍使用实验兼容入口。
> 下文“独立实验、不修改主项目”等说明描述整合前阶段。当前使用说明与限制见 [正式场景说明](../../docs/hexframe_assembly.md)。


白色 KUKA iiwa14、银色六棱柱桁架模块、六个蓝色侧接口、细长太阳翼与淡蓝灰星空。当前默认模块为用户提供的 Astra **HexFrame / PetalDock100 V2**，默认入口为 `crown_assembly.py`；下方原紧凑竖直版保留为旧版。太阳翼为光伏板外观，并未模拟太阳帆光压。

## 默认 HexFrame 模块

原资源完整保存在 `assets/hexframe_module/`，包含原始 MIT 许可、参数化生成器、STEP、GLB、MJCF、预览和随附验证。`IMPORT.json` 记录用户压缩包摘要及端口映射；包内说明仅作为资源资料读取。默认模块采用原尺寸约 **341 × 344 × 170 mm**，CAD 密度估算质量约 **3.508 kg**，沿用原 CAD 惯量，未缩放为旧模块或保留旧 2 kg 参数。

场景接口 1–6 对应资源端口 3、4、5、0、1、2。接口 1 朝上、接口 4 朝下，仍全在六个侧面。对接根间距改为 46.4 mm，模块中心距约 323.528 mm，接口安装钟向采用资源原有 67.5°。机械臂端部也换成匹配的 PetalDock 外观。初始位置、抬升和撤离距离同步调整；活跃组装面、存储和备用端口采用精细导向/止挡碰撞，其余模块接口保留包络代理，框架保留独立梁碰撞与中空结构。

```bash
# 新默认：完整 HexFrame 组装、接触验证和视频
OPENBLAS_NUM_THREADS=1 .venv/bin/python experiments/orbital_showcase/crown_assembly.py --video

# 仅重渲染保存的默认组装状态
.venv/bin/python experiments/orbital_showcase/render_hexframe.py --video

# 原花冠模块回退入口
OPENBLAS_NUM_THREADS=1 .venv/bin/python experiments/orbital_showcase/crown_assembly.py --module legacy --video

# 验证新默认的尺寸、惯量、端口与组装记录
.venv/bin/pytest -q experiments/orbital_showcase/test_hexframe_integration.py
.venv/bin/python experiments/orbital_showcase/hexframe_assembly_audit.py
```

模块 2 通过朝下的侧面接口 4 与匹配的 PetalDock100 V2 基座接口连接，初始已锁定；原组装托架替换为低矮圆形安装座。基座端口采用同一接口网格和 46.4 mm 配合根间距，模块 2 是该端口的固定子刚体。上方接口 1 继续接收机械臂携带的模块 1，未新增端面接口。

存储位与备用位同样采用朝上的标准端口。模块 1 初始用接口 4 锁定在存储端口，机械臂接上接口 1 后，先确认机械臂锁定，再解除存储接口并竖直提起。存储 weld 的锚点移至接口配合面，接触与脱离单独记录于 `storage_trace.npz`；备用端口保持空闲。三处台面连接由同一安装座/接口构建函数生成，旧叉形托架在默认场景中已移除。

默认输出独立位于 `outputs/hexframe_assembly/`。原 `outputs/vertical/` 与 `outputs/crown_integrated/` 保留，旧性能数字不能代表新模块。新资源属于原创研究模型；活动锁紧仍采用理想 weld，基座连接为预锁定固定变换，未模拟锁紧机构。CAD 质量不含紧固件、锁紧机构及电子设备，尚未进行结构强度与模态校核。`default_scene.json` 记录默认选择，主项目原始资产与控制入口没有更换。

独立物理验证新增于 [原花冠插合与锁定实验](crown_seating/README.md)：复用原花冠凸分解，比较无偏差和组合偏差下的绕轴柔顺/固定刚度，连续就位后才启用理想锁定并撤去驱动力。五组试验、半步长复核和原始轨迹审计已完成；尚未接入下方机械臂组装视频，不能把独立接口结果视为完整场景接触验证。

最新的 [带花冠接触完整组装流程](outputs/crown_integrated/README.md) 已完成名义工况的抓取、搬运、真实花冠凸分解接触、持续就位锁定、卸力确认、机械臂释放与撤离。入口为 `crown_assembly.py --video`，独立输出到 `outputs/crown_integrated/`；下方原竖直视频仍保留。真实接触仅加入模块间对接处，抓取和锁紧机构仍为理想约束。

## 紧凑基座与竖直组装（保留版本）

入口 `vertical_assembly.py` 按左侧组装、中央机械臂、右侧存储布置。基座为 **2.6 × 1.8 × 0.5 m**，包含侧壁分舱外观；两片 **2.4 × 0.42 m** 细长太阳翼从两侧对称向外展开，内端距中心 X 轴原点 1.55 m、安装中心高度在台面下方 0.23 m。固定 12° 展示倾角，未模拟展开机构或对日跟踪，也未验证发电和遮挡性能。

两个六棱柱整体旋转，使六边形端面朝前后。自然模块坐标的六个侧面接口保持不变；接口 1 的外法线朝世界 +Z，接口 4 朝 −Z。机械臂从上方捕获模块 1 的接口 1，抬升并转运到左侧，再沿 −Z 方向让接口 4 与固定模块 2 的接口 1 对接。安装确认后机械臂沿 +Z 撤离。底部采用中空托架，给朝下的侧面接口留出空间。

```bash
# 完整竖直组装、验证和视频
.venv/bin/python experiments/orbital_showcase/vertical_assembly.py --video

# 仅运行检查
.venv/bin/python experiments/orbital_showcase/vertical_assembly.py --no-render

# 重渲染已保存的竖直组装状态
.venv/bin/python experiments/orbital_showcase/vertical_assembly.py --replay --video
```

独立模型为 `assets/vertical_assembly.xml`，输出位于 `outputs/vertical/`：`preview.png` 展示双翼和初末状态，`storyboard.png` 展示分阶段动作，`assembly_sequence.mp4` 为 37 秒真实刚体状态回放。`geometry_check.json` 核对侧面接口和世界法线，`validation.json` 记录交接、关节余量及碰撞检查。物理边界仍为理想锁定约束，花冠微接触没有加入完整流程。碰撞检查包含机械臂代理、模块边杆、中空托架支撑面和基座台面；太阳翼与细节装饰仅作显示。

双侧外伸的选择用于给上方组装留出空间，外观参考常见双翼航天器布置；[NASA SWOT](https://swot.jpl.nasa.gov/resources/140/solar-panel-deployment/) 也采用从机身相对两侧展开、可对日转动的太阳翼。当前场景的尺寸和固定倾角是布局参数，不是该任务的工程复现。

## 横向抓取、搬运与安装流程（保留版本）

新增入口 `assembly_sequence.py` 在同一作业区设置模块 1 的存放架、备用空位和固定模块 2 的组装位。机械臂末端安装独立接口；模块 1 始终保留自由刚体模型，通过三个可切换的 MuJoCo weld 约束交接支撑关系。

| 阶段 | 存放架—模块 1 | 机械臂—模块 1 接口 1 | 模块 1 接口 4—模块 2 接口 1 |
|---|---|---|---|
| 初始存放 | 锁定 | 解除 | 解除 |
| 抓取确认，保持静止 | 锁定 | 锁定 | 解除 |
| 抬升与转运 | 解除 | 锁定 | 解除 |
| 安装确认，保持静止 | 解除 | 锁定 | 锁定 |
| 机械臂松开并撤离 | 解除 | 解除 | 锁定 |

规则是先确认新连接，再解除旧连接。激活锁定需要位置误差小于 1 mm、角度误差小于 0.5°、相对速度小于 3 mm/s、相对角速度小于 2°/s。抓取前解除存放锁、安装前释放机械臂、远距离直接锁定均有拒绝检查。短暂双锁阶段保持静止。

```bash
# 运行 37 秒刚体动力学流程，保存分镜与视频
.venv/bin/python experiments/orbital_showcase/assembly_sequence.py --video

# 仅运行轨迹、动力学与交接检查
.venv/bin/python experiments/orbital_showcase/assembly_sequence.py --no-render

# 重渲染已有状态
.venv/bin/python experiments/orbital_showcase/assembly_sequence.py --replay --video
```

新输出独立保存在 `outputs/sequence/`：`storyboard.png`、`assembly_sequence.mp4`、`initial_workcell.png`、`installed.png`、`platform.png`、`rollout.npz`、`validation.json` 和 `source_manifest.json`。模型为 `assets/assembly_sequence.xml`，不会覆盖原侧面对接实验模型。

该流程使用受力矩限幅的关节伺服、自由模块与动力学锁定约束；初始化后不直接改写模块位置。碰撞检查覆盖机械臂代理、模块边杆、存放台面和作业区地板。锁紧机构采用理想约束，花冠细节仅显示，名义接合间距为 50 mm，未求解花冠 SDF 微接触，也未模拟锁销、预紧力或电气连接。这里的流程 PASS 与下文原柔顺接触实验的稳定性判定不同，不能用于宣称此前接触振动已解决。

布局与动作设计参考了机器人搬运标准化模块的任务思路，例如 [NASA FFR 的轨道可更换单元抓取与转移](https://www.nasa.gov/mission/fly-foundational-robots/)。约束切换依据 [MuJoCo weld 文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-weld)；具体流程、布局和参数由本实验自行设计。

## 原横向接触实验：连接关系与坐标

机械臂固定连接模块 1 的接口 1；模块 1 的相对侧接口 4 对接固定模块 2 的接口 1。每个模块的六个接口均为模块刚体的固定子节点，接口和桁架一起运动；上下六边形端面没有接口。

模块自然坐标系 M 的 Z 轴沿六棱柱轴线。接口依次绕侧面编号 1–6，1 与 4 相对。接口位置为 `a[cos(theta), sin(theta), 0]`，其中 `theta = pi + (number-1)*pi/3`，`a = 0.115*cos(pi/6)` m。接口局部 Z 轴沿所在侧面的外法线。实际接近方向是世界坐标 +Y。

机械臂法兰到接口 1 的连接件长 15 mm；接口 1 到接口 4 为约 199.2 mm。控制点、轨迹及接触诊断均改为接口 4。具体变换矩阵见生成的 `outputs/geometry_manifest.json`。

模块 1 总质量设为 2 kg，采用等效均匀六棱柱惯量，MuJoCo 与 Pinocchio 同步更新。桁架边杆有胶囊碰撞代理；模块 1 接口 4 与模块 2 接口 1 使用已有花冠 SDF 接触几何。其余接口仅有外观和刚性连接关系，尚未配置独立碰撞、锁紧或释放功能。机械臂与接口 1 的连接也为固定安装，未模拟抓取或锁紧过程。

## 隔离与运行

场景修改位于此目录；不修改主项目 `src/`、`scenes/` 或原始资产。工程收尾还更新了根目录测试/代码检查配置与 CI 依赖安装方式。当前目录没有 Git 元数据，因此采用独立目录隔离，未创建提交或 PR。

收尾检查、基座连接定义及视频制作流程见 [工程收尾记录](ENGINEERING_CLOSEOUT.md)。默认选择由 `default_scene.json` 实际读取；HexFrame 的模型和接触行为通过显式回调传入。兼容旧场景的内部配置仍使用共享变量，但限定在上下文内，并在正常退出或异常时恢复；同一进程应串行运行场景。

从项目根目录运行：

```bash
# 生成资产，检查侧面位置、可达性与双模型一致性，输出静态预览
.venv/bin/python experiments/orbital_showcase/run.py --preview-only

# 完整侧面对接闭环，保存仿真状态，输出全景与动态特写视频
.venv/bin/python experiments/orbital_showcase/run.py --video

# 重渲染已保存的侧面对接状态
.venv/bin/python experiments/orbital_showcase/run.py --replay --video

# 黑色星空选项，默认淡蓝灰色
.venv/bin/python experiments/orbital_showcase/run.py --replay --sky black
```

`scene.yaml` 是本实验的配置快照，由独立入口 `run.py` 使用 `side_docking.py` 构建；不要交给主项目目前仅支持竖直插入的场景入口。视觉源文件是 `build_assets.py`，模块与侧面轨迹源文件是 `side_docking.py`。生成资产使用当前项目的绝对网格路径，搬动项目后需重新生成。

输出位于 `outputs/`：`preview.png` 为全景海报，`assembly.png` 为机械臂与模块近景，`hexagonal_module.png` 为接口布置特写，`orbital_docking.mp4` 为仿真状态回放，`rollout.npz` 保存状态，`insertion_gate.json` 保存接触与稳定性判定，`physics_check.json` 保存模型一致性检查。`source_manifest.json` 记录运行时源码摘要。

## 验证范围

检查覆盖全部六个接口的侧面位置、固定连接关系、50 个轨迹采样点可达性、控制点正运动学及质量矩阵一致性。闭环接触另用主项目原有判定标准评估；具体通过与否以 `insertion_gate.json` 为准，不放宽稳定性阈值。

本实验使用固定基座、零重力、1 ms 步长与现有柔顺控制器。尚未实现自由漂浮航天器、轨道传播、模块释放或机械锁紧。本场景是原创参数化结构，并非论文原始 CAD；接触判定通过也不代表完整工程装配就位。

## 资源与许可

新增参数化几何与脚本沿用项目 MIT 许可。机械臂网格、已有花冠接口和控制器保留各自原有许可。还可参考 [mjorbit](https://github.com/johnzhang3/mjorbit) 和 [CoACD](https://github.com/SarahWeiii/CoACD) 扩充轨道动力学和碰撞处理；二者不是本场景的运行依赖。
