# KUKA iiwa14 柔顺对接仿真（MuJoCo × Pinocchio）

[![CI](https://github.com/langxin11/compliant_docking_simulation/actions/workflows/ci.yml/badge.svg)](https://github.com/langxin11/compliant_docking_simulation/actions/workflows/ci.yml)

基于 **MuJoCo 物理仿真** 与 **Pinocchio 刚体动力学** 联动搭建的七轴机械臂（KUKA iiwa14）柔顺对接仿真平台。两个引擎完全独立、互不共享参数：MuJoCo 作为"物理世界"提供非凸 SDF 接触、末端六维力/力矩传感与渲染；Pinocchio 作为控制器内置的动力学模型提供质量矩阵、雅可比与逆运动学。二者在 **1 kHz** 闭环中协同运行，与真实机器人上部署模型基（model-based）控制器的结构完全一致。

## 亮点

### 1. Pinocchio × MuJoCo 双引擎联动仿真

| 引擎 | 职责 |
|---|---|
| **MuJoCo** | 物理积分（implicitfast，1 ms 步长）、SDF 非凸接触（elliptic 摩擦锥，`sdf_iterations=10`）、末端力/力矩传感、渲染与录制 |
| **Pinocchio** | 控制器侧动力学模型：CRBA 质量矩阵 `M`、Coriolis 矩阵 `C`、雅可比 `J` 及其时间导数 `J̇`、阻尼最小二乘逆运动学 |

每个控制周期（1 ms）的闭环数据流：

```
MuJoCo 状态 (q, v) ──► Pinocchio FK / J / M / C ──► 操作空间阻抗控制器 ──► 关节力矩 τ
      ▲                                                                        │
      └──────────────── MuJoCo mj_step ◄───────────────────────────────────────┘
                             │
                             └──► 力/力矩传感器 ──► 接触力前馈（旋转至世界系）
```

关键设计：控制器不依赖 MuJoCo 内部动力学——MuJoCo 只是被施加力矩的"黑盒物理世界"，接触力经虚拟传感器回读，如同真实机器人上的 F/T 传感器。这种解耦使同一套控制器可以无修改地迁移到纯 Pinocchio 仿真（`RobotSimulator`）或真实机器人。

双引擎联动的前提是动力学一致：`simulation/consistency.py` 在关节空间 PD 跟踪（五次多项式/正弦轨迹）下，将 Pinocchio 前向动力学（`pin.aba`）与 MuJoCo（`mj_step`）的结果交叉验证，确认两套模型同源，为联动闭环的可信度提供依据。

### 2. 分层抽象的软件架构

```mermaid
graph TB
    subgraph L5["实验编排层"]
        M["experiments/run_docking.py<br/>场景装配 · 仿真主循环 · 参数注入"]
    end
    subgraph L4["记录与可视化层"]
        LOG["telemetry.py<br/>跟踪误差 / 接触力 / 关节力矩曲线"]
    end
    subgraph L3["控制层 — control/task_space.py"]
        C1["阻抗操作空间控制（主）<br/>力前馈 + 零空间投影"]
        C2["操作空间 PD（对照）"]
        C3["可操作度梯度零空间优化（对照）"]
    end
    subgraph L2["规划层"]
        T["planning/trajectory.py<br/>三轴解耦五次多项式"]
        IK["planning/kinematics.py<br/>阻尼最小二乘 IK"]
    end
    subgraph L1["仿真层（可互换）"]
        S1["simulation/mujoco_env.py<br/>MuJoCo 物理 · SDF 接触 · 力传感"]
        S2["simulation/pinocchio_sim.py<br/>纯 Pinocchio RK4（无接触对照）"]
        S3["simulation/consistency.py<br/>双引擎一致性验证"]
    end
    subgraph L0["模型层 — assets/iiwa14/ + models.py"]
        D1["iiwa14_dock_updated.xml → MuJoCo"]
        D2["iiwa14_dock.urdf → Pinocchio"]
    end
    M --> C1 & C2 & C3
    M --> T & IK
    M --> S1
    C1 & C2 & C3 --> S1
    T & IK --> C1
    S1 & S2 --> D1 & D2
```

| 层 | 模块 | 职责 | 可替换性 |
|---|---|---|---|
| 实验编排层 | `experiments/run_docking.py` | 装配各层组件、运行仿真主循环 | — |
| 记录与可视化层 | `compliant_docking.telemetry` | 时序数据记录、位置跟踪/接触力/关节力矩绘图 | — |
| 控制层 | `compliant_docking.control.task_space` | 三组控制器实现（见下表） | 控制器间可切换对比 |
| 规划层 | `compliant_docking.planning`（`trajectory` / `kinematics`） | 三轴解耦五次多项式轨迹（端点速度/加速度为零）、DLS 逆运动学 | 任意轨迹发生器 |
| 仿真层 | `compliant_docking.simulation`（`mujoco_env` / `pinocchio_sim` / `consistency`） | MuJoCo 仿真封装、纯 Pinocchio RK4 仿真、双引擎验证 | 仿真器可互换 |
| 模型层 | `compliant_docking.models` + `assets/iiwa14/` | 同一套 mesh 的双描述：MuJoCo XML（主模型 `iiwa14_dock_updated.xml`）与 Pinocchio URDF，统一加载入口 | 换机器人只换模型层 |

各层之间只通过标准量（`q, v, τ, SE(3), J`）交互，替换任何一层不影响其它层——例如把 MuJoCo 仿真器换成纯 Pinocchio 仿真器做无接触验证、或在多组控制器之间切换做对比实验，都只改动编排层一行装配代码。

### 3. 柔顺对接任务与结果

场景：iiwa14 末端安装锥形对接头（SDF 非凸 mesh），对固定对接座沿 z 向下压完成对接。指令行程 18 cm，五次多项式轨迹 15 s，总仿真 18 s，控制频率 1 kHz。

- 操作空间阻抗控制 + 接触力前馈：接触后 Z 向按阻抗参数柔顺让位，无持续冲击；
- 接触回路保留 12 s 确定性安全回归：当前峰值约 **15.798 N**、12 s 时约
  **1.311 N**，且峰值硬性要求不超过旧基线 18.785 N；该回归不是最终对接精度验收；
- 非接触方向跟踪：X/Y 误差保持在 **±2 mm** 以内；
- 主循环含力矩限幅与异常捕获，接触丰富的场景下长时仿真稳定。

![docking error](demo/tracking_error.png)
[![Docking Demo](demo/docking_preview.gif)](demo/docking.mp4)

## 控制器变体

`TaskSpaceController` 提供三组可切换实现，支撑多控制器对比实验：

| 方法 | 特点 |
|---|---|
| `compute_control_task_space_with_orientation_and_imp` | **主控制器**：位置/姿态双通道阻抗模型 `(m, d, k)` + 传感器接触力前馈；采用操作空间惯性 `Λ=(JM⁻¹Jᵀ)⁺`、动力学一致广义逆 `J̄=M⁻¹JᵀΛ` 与零空间投影 `N=I−J̄J`，通过 `Nᵀ` 注入关节阻尼抑制自运动 |
| `compute_control_task_space_with_orientation` | 纯操作空间 PD（无阻抗、无力前馈），作为对照 |
| `compute_control_task_space` | 位置子空间控制 + 可操作度（manipulability）梯度零空间优化，作为对照 |

此外提供独立的 `HQPAdaptiveController`（`control/hqp_ac.py`，与主控制器同签名可互换）：
HQP-AC 约束自适应控制——把关节位置/速度/力矩极限作为 QP 硬约束（ZOH 短时域预测），
刚度按接触力自适应，零空间做奇异性规避与关节位姿阻抗；CLI 以
`docking --controller hqp` 启用。出处：Ren & Shan 2026, Acta Astronautica, §3.2。

## 快速开始

```bash
git clone https://github.com/langxin11/compliant_docking_simulation.git
cd compliant_docking_simulation

uv sync                     # 创建环境并锁定依赖（uv.lock）
uv run docking --quick      # CLI 冒烟：2 s 仿真验证环境
uv run python experiments/run_docking.py   # 完整 18 s 对接仿真
```

先运行自由空间轨迹跟踪门禁，再进入柔顺对接：

```bash
# 2 s 冒烟只检查链路，结果标记为 INCOMPLETE，不作性能判定
uv run --frozen docking --scene scenes/iiwa14_tracking.yaml --quick

# 完整覆盖圆形与 8 字轨迹；PASS 返回 0，FAIL 返回 2
uv run --frozen docking --scene scenes/iiwa14_tracking.yaml --duration 13.1

# FR3 使用同一套门禁流程
uv run --frozen docking --scene scenes/fr3_tracking.yaml --duration 13.1
```

跟踪场景不装配母头，也不把 F/T 信号反馈给控制器，以隔离接触与柔顺控制影响；
但仍记录原始传感器、真实接触数和力矩限幅情况。圆形与 8 字轨迹在段间保持位置、
速度和加速度连续。验收阈值位于对应 `scenes/*_tracking.yaml` 的
`tracking_thresholds` 段，可按实际对接精度需求调整；完整运行只有同时满足分段误差、
姿态误差、力矩限幅比例和零接触要求才会输出 `PASS`。

当前 1 ms 步长、默认参数的确定性基线：iiwa14 圆/8 字位置 RMS 分别约
`0.236/0.269 mm`，FR3 分别约 `1.545/0.938 mm`；两者均无力矩限幅和接触。
这些结果只证明自由空间跟踪链路合格，不代表柔顺接触与最终对接精度已经验收。

运行测试：

```bash
uv run pytest -m "not slow" -q   # 快速单元测试（CI 同款）
uv run pytest -m slow -q         # 12 s 接触回归：锁定行为锚点（误差/接触力数值）
```

没有 uv 时也可以直接用 pip 安装依赖：

```bash
pip install mujoco pin numpy scipy matplotlib imageio imageio-ffmpeg
python experiments/run_docking.py
```

无显示器（headless）环境渲染：

```bash
sudo apt update && sudo apt install libegl1 libegl-dev
export MUJOCO_GL=egl
```

## 场景配置

`scenes/*.yaml` 用一份 YAML 描述完整对接场景：机械臂 + 公头工具 + 母头目标 + 物理参数 + 任务初始条件。三段 MJCF 片段在运行时经 MuJoCo **MjSpec attach** 组装为单一模型（`compliant_docking.scene.load_scene` → `Scene.build_mjmodel`），无需手工拼接 XML。

通过 CLI 的 `--scene` 参数选择场景：

```bash
uv run docking --scene scenes/iiwa14_docking.yaml --quick   # iiwa14 对接（默认场景）
uv run docking --scene scenes/fr3_docking.yaml --quick      # FR3 对接
```

可选机械臂：

- **KUKA iiwa14**（URDF）：MuJoCo 用 `assets/iiwa14/iiwa14_arm.xml`，Pinocchio 读 URDF `iiwa14_dock.urdf`；
- **Franka FR3**（mujoco_menagerie 派生的力矩执行器 MJCF 变体）：MuJoCo 与 Pinocchio 均直读 `assets/fr3/fr3_arm.xml`——`load_pin_model` 按文件后缀分发解析器（`.urdf` 走 URDF，`.xml` 走 `buildModelFromMJCF`）。

新增场景：复制一份现有 YAML，替换 `robot` 段的资产路径与 `task` 段的初始条件（`ik_guess` 换成新机械臂的 home 位形）即可；公头/母头片段可直接复用 `assets/interfaces/` 下的 `male_cone.xml` / `female_socket.xml`。

## 绘图风格

项目绘图统一走 [SciencePlots](https://github.com/garrettj403/SciencePlots) 的 `["science", "ieee", "no-latex"]` 风格（IEEE 单栏、不依赖 LaTeX），并叠加 Noto CJK 中文字体回退与 `axes.unicode_minus=False`（规避中文字体缺 U+2212 负号的问题）。入口：`compliant_docking.plotting.apply_style()` 与 `plot_docking_log(log, out_dir, scene_name=...)`，每张图同时输出 PNG + PDF。运行 `docking --scene ...` 完成后图件按场景归档在 `figure/<场景名>/`（如 `figure/iiwa14_docking/`），文件名带场景前缀。

## 仓库结构

```
├── src/compliant_docking/          # 核心 Python 包
│   ├── models.py                   # 模型层入口：XML/URDF 路径与加载（重力置零统一管理）
│   ├── config.py                   # ImpedanceConfig / DockingConfig：参数集中管理
│   ├── cli.py                      # docking 命令行入口（uv run docking）
│   ├── planning/
│   │   ├── trajectory.py           # 三轴解耦五次多项式轨迹
│   │   └── kinematics.py           # 阻尼最小二乘逆运动学
│   ├── control/
│   │   └── task_space.py           # 操作空间控制器（三组可切换实现）
│   ├── simulation/
│   │   ├── mujoco_env.py           # MuJoCo 封装（step / 传感器 / 渲染 / 录制）
│   │   ├── pinocchio_sim.py        # 纯 Pinocchio RK4 仿真器（无接触对照）
│   │   └── consistency.py          # 双引擎动力学一致性验证
│   └── telemetry.py                # 数据记录与结果绘图
├── experiments/
│   ├── run_docking.py              # 对接任务编排：场景装配 + 仿真主循环
│   └── check_env.py                # 仿真环境自检脚本
├── assets/iiwa14/                  # 模型资产：MuJoCo XML 与 Pinocchio URDF（同一套 mesh）
│   ├── iiwa14_dock_updated.xml     # 主仿真模型：SDF 对接头/对接座 + 力传感器
│   └── iiwa14_dock.urdf            # Pinocchio 动力学计算用
├── tests/                          # pytest 测试
├── docs/                           # 理论文档（见"延伸阅读"）
└── demo/                           # 演示视频与结果图
```

## 延伸阅读

- [docs/main_simulation_theory_and_flow.md](docs/main_simulation_theory_and_flow.md) —— 控制理论基础与端到端数据流梳理，附关键代码锚点
- [docs/机械臂阻抗控制.md](docs/机械臂阻抗控制.md) —— 关节空间阻抗控制推导（期望阻抗模型与控制律设计）

## License

MIT
