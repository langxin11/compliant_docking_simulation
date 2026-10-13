# models_interfaces

这里放模型资格检查、接口候选与几何对照的编排入口。

- `baseline.py`：默认 Petal 固定模型的资格检查（双引擎一致性、初值、哈希）；
- `petal_guidance.py`：三个几何候选的横向释放 rollout；
- `selected_candidate.py`：已保存 1°/0.3 导向设计的代表点复核；
- `angle1_task_check.py`：选定接口的九点任务验证（1 ms 时序）。

从仓库根目录运行：

```bash
uv run python -m experiments.models_interfaces.baseline --out runs/model_baseline_my_run
```

输出只写入 `runs/` 的新目录；经审查的唯一模型才进入 `assets/`。
