# 贡献指南

请先说明变更属于模型与接口、控制研究或完整系统，具体触发条件和预期结果。
缺陷修复可直接提交 PR；研究协议或大范围物理变更先在 issue 中描述研究问题和主要变量。

## 开发与检查

```bash
uv sync --frozen --dev
uv run ruff check .
uv run pytest -m 'not slow' -q
uv run mkdocs build --strict
```

修改控制、采样、模型或系统编排时运行受影响的数值锚点。正式系统变更还需完整 53 s
HexFrame 流程和独立审计；回放改动需从同一状态记录验证。慢速套件使用 `uv run pytest -m slow -q`。
CI 使用上述静态/快速检查、严格文档构建和完整系统验收。请在 PR 记录实际运行命令与结果。

## 研究与数据

- 对照声明唯一主要变量和固定条件；物理步长、控制周期、减速、接口几何分别立项。
- 保存有效配置、失败原因、运行环境、来源哈希、状态与评价；新运行写 `runs/` 的新目录。
- 临时候选保存在 `runs/designs/`，生成模型先放 `runs/generated_assets/`；经审查的唯一模型纳入 `assets/`，冻结证据摘要纳入 `results/`。
- 历史脚本维护在 `archive/`；兼容入口只转发，不保留第二份实现。归档不能修改原证据或绕过来源指纹。
- 不改旧结果/源码快照/哈希以通过检查；不把不同矩阵合成统一成功率。
- 模型/资产保留唯一来源、许可与导入清单；候选不未经验证替代基线。
- 共享库不导入实验脚本；算法不依赖 Demo 布局；旧入口兼容与变更记录一起提交。

结构变更需要新旧有效配置、退出语义和多进程入口检查。指纹变化无法证明安全复用时拒绝复用。
视频由验收记录回放；预检与渲染成功不代表系统通过。

## 采用的开源习惯

本项目参考 [MuJoCo Menagerie 的贡献流程](https://github.com/google-deepmind/mujoco_menagerie/blob/main/CONTRIBUTING.md)
统一本地/CI 检查及变更记录，并保持模型来源和许可；参考
[robosuite 的模块划分](https://github.com/ARISE-Initiative/robosuite/blob/master/README.md)
区分模型、控制、任务和回放。这里采用职责与复现习惯，不引入它们的框架依赖。

提交时更新 `CHANGELOG.md` 的 Unreleased 项，避免将未执行验证写为通过。

程序生成数据默认写 `runs/` 并忽略；模型必需资源与人工选择的冻结证据按用途维护。
CI 拒绝已跟踪但命中忽略规则的文件，人工发布需记录来源并添加具体 allowlist，
不用 `git add -f` 绕过。详细工具输出与复现边界见[生成文件策略](docs/generated_artifacts.md)。
