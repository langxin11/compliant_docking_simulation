# control

这里放柔顺控制策略的对照实验，固定原版 Petal 接口（original 场景）。

- `yaw_release.py`：固定接口，只比较绕轴刚度策略（stiff / compliant / released）；
- `lateral_release.py`：固定绕轴释放后，同点比较 XY 保持与释放；
- `capture_range.py`：有限的离散误差工况地图，不对采样点之间做插值声明。

从仓库根目录运行：

```bash
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.control.yaw_release \
  --case combined --setting baseline --out runs/yaw_release_my_run
```

一个实验只回答一个问题；评价读各组 JSON 的 `assessment`，成功退出不表示所有工况通过。
