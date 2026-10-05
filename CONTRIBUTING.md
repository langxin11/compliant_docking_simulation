# 贡献指南

本仓库研究机械臂柔顺对接与装配。贡献前先读[当前主线](docs/research_focus.md)与
[研究范围](docs/research_scope.md)，再决定改动属于哪一层。

## 环境安装

```bash
git clone https://github.com/langxin11/compliant_docking_simulation.git
cd compliant_docking_simulation
uv sync --frozen --dev        # 推荐 uv；标准环境可用 python -m pip install -e .
uv run docking --quick        # 2 s 链路检查，确认环境可用
```

无显示环境设 `MUJOCO_GL=egl`；Ubuntu 需要 `libegl1`、`libegl-dev`。

## 三层实验与研究数据

研究分三层，新实验入口放进对应层的目录：

- `experiments/models_interfaces/` —— 模型资格检查、接口候选与几何编排；
- `experiments/control/` —— RQ1/RQ2、误差范围等柔顺控制对照实验；
- `experiments/system/` —— HexFrame 正式验收、预检、回放与扩展工况。

`experiments/` 只负责实验编排，共享实现放 `src/compliant_docking/`，不能反向导入
实验入口。从仓库根目录运行：`uv run python -m experiments.<层>.<入口> --help`。

新运行数据一律写 `runs/` 下的新目录（Git 忽略）；正式模型在 `assets/`，
冻结证据在 `results/`，精选展示在 `demo/`。不用 `git add -f` 加入被忽略的产物。
完整命令、退出语义与默认输出位置见[实验复现手册](docs/experiments.md)。

## 修改后跑什么

```bash
uv run ruff check .
uv run pytest -m 'not slow' -q
uv run mkdocs build --strict
```

按改动范围选择：纯文档改动跑文档构建和静态检查；控制、采样、模型或系统编排
改动加跑受影响的数值锚点；正式装配行为改动才需要完整 53 s HexFrame 验收和
独立审计。慢速接触回归用 `uv run pytest -m slow -q`。请在 PR 中记录实际运行的
命令与结果，不把未执行的验证写成通过。

## 研究约定

- 一个实验回答一个问题：声明唯一主要变量和固定条件，不把不同矩阵合成统一成功率。
- 不修改历史结果、验收阈值和来源指纹；历史协议按原口径解释，失败工况是研究结果。
- 不新增 pipeline / manager / registry 之类的抽象层；优先修改现有文件，不再运行的
  旧代码直接删除（Git 历史可恢复），不建 archive 目录或第二套兼容入口。

## 代码与注释

编码代理先读根目录 [AGENTS.md](AGENTS.md)。项目自有代码遵循
[代码与注释规范](docs/coding_style.md)：注释面向中文用户，物理量说明单位、形状、
坐标系与时间约定；公共接口用 Google 风格 Args / Returns / Raises。第三方资产与
冻结源码快照不做格式或语言改写。

## 变更记录

只有影响使用方式、实验协议、默认行为、默认模型/接口或重要研究阶段的改动才更新
[CHANGELOG.md](CHANGELOG.md)；注释、typo 和局部清理不必记录。

本项目参考 [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/blob/main/CONTRIBUTING.md)
的贡献流程与许可习惯、[robosuite](https://github.com/ARISE-Initiative/robosuite/blob/master/README.md)
的模块划分，只采用职责与复现习惯，不引入它们的框架依赖。
