# MuJoCo Dynamics Docking (SDF) / 基于 MuJoCo 的动力学柔顺对接（SDF）

Welcome! This repository demonstrates trajectory planning and dynamics simulation using MuJoCo, with Pinocchio for efficient kinematics/dynamics, and an operational-space impedance controller for compliant docking.
欢迎！本仓库使用 MuJoCo 进行轨迹与动力学仿真，结合 Pinocchio 的高效运动学/动力学计算，并实现操作空间阻抗控制以完成柔顺对接。

## Table of Contents / 目录

- Introduction / 简介
- Features / 特性
- Instruction / 文件说明
- Usage / 使用方式
- Result / 运行结果
- Trouble shooting / 故障排查
- Contributing / 贡献方式
- License / 许可证

## Introduction / 简介

This is a repository for MuJoCo-based dynamics simulation and compliance control of a KUKA iiwa14 arm.
本仓库面向 KUKA iiwa14 机械臂的动力学仿真与柔顺控制。

## Features / 特性

- Signed Distance Field (SDF) collision utilities / 支持 SDF 碰撞工具
- Dynamics simulation via MuJoCo / 基于 MuJoCo 的动力学仿真
- Operational-space impedance control / 操作空间阻抗控制

## Instruction / 文件说明

### 1) Model Files / 模型文件

1. `kuka_xml_urdf/iiwa14_dock.xml`：带 SDF 的 iiwa 模型（MuJoCo XML）
2. `kuka_xml_urdf/iiwa14_dock.urdf`：对应 URDF，用于 Pinocchio 的动力学计算

### 2) Core Scripts / 核心脚本

1. `dynamics1.py`：验证 Pinocchio 与 MuJoCo 的动力学接口（关节空间）
2. `Relate_class.py`：轨迹规划、逆运动学与任务空间控制等相关类
3. `main_simulation.py`：主仿真脚本，运行任务空间动力学+阻抗控制

## Usage / 使用方式

1) Clone / 克隆仓库

```bash
git clone https://github.com/ming751/initial_docking_model.git
cd initial_docking_model
```

2) Setup dependencies / 安装依赖

- For dynamics features, install Pinocchio on Linux / 若使用动力学功能，需在 Linux 安装 Pinocchio：

```bash
conda create -n pin_mjcf python=3.10
conda activate pin_mjcf
pip install pin
pip install -r requirement.txt
```

3) Run main simulation / 运行主仿真

```bash
python main_simulation.py
```

## Result / 运行结果

![docking error](demo/tracking_error.png)
[![Docking Demo](demo/docking_preview.gif)](demo/docking.mp4)

## Trouble shooting / 故障排查

Render on headless devices / 无显示设备渲染：

```bash
sudo apt update
sudo apt install libegl1 libegl-dev
export MUJOCO_GL=egl
```

## Contributing / 贡献方式

We welcome contributions! / 欢迎贡献！

1. Fork the repository / Fork 仓库
2. Create a feature branch / 新建特性分支：

```bash
git checkout -b my-feature-branch
```

3. Commit changes / 提交修改：

```bash
git commit -am 'Add new feature'
```

4. Push to your fork / 推送到 fork：

```bash
git push origin my-feature-branch
```

5. Open a PR on GitHub / 发起 Pull Request

## License / 许可证

MIT License. See [LICENSE](LICENSE). / 本项目基于 MIT 许可证，详见 [LICENSE](LICENSE)。
