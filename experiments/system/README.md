# system

hexframe precheck is INCOMPLETE, accept requires full validation and audit, replay uses saved states. validation/ contains P0/P1/P2, separate from formal acceptance and control research.

Run from repository root with `uv run python -m experiments.system.<entry> --help`.


## 有限接收位姿偏差

`hexframe_pose_check` 保留现有控制器，在名义规划完成后改变接收站的真实位姿。
固定四点：nominal、xy（世界 X +2 mm）、yaw（世界 Z +5°）、combined。
接收几何和评分锚点共同变换，规划及导纳参考保持名义估计。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 uv run python -m experiments.system.hexframe_pose_check \
  --case combined --out runs/hexframe_pose_combined_new
```

输出包含实际与名义模型、偏差核对、接触记录、原验收结果和来源指纹。
运行失败保留原记录；完整通过后运行独立交接审计。
原系统就位条件与单接口 2 N·m 历史参考值不同；此入口没有引入该参考值。


## 人工评判视频

`hexframe_review_video` 对已有成功或未完成工况生成整体视角＋接口特写的原速回放。
支持截短记录，视频结束于实际保存时间，并显示真实抓持和锁定状态。

```bash
MUJOCO_GL=egl uv run python -m experiments.system.hexframe_review_video \
  --run runs/原工况目录 --out runs/新的展示目录 --case yaw
```

输入模型、状态和锚点只读；输出视频、末帧及来源哈希，不重新积分或变更原评价。

## 两种接口的有限移植对照

`hexframe_interface_compare` 在独立模型中替换模块 1 接口 4、模块 2 接口 1 和匹配存储头。
支持 `angle1`（angle1_blend030 原凸块）与 `crown_stl`（最早 STL 原 SDF）。
四个工况与真实接收偏差入口相同；原轨迹、控制器、摩擦和模块 CAD 惯量保留。
冠形需显式挂载偏置与相位适配，几何候选不等于独立止挡证明；初始存储预载必须另行检查。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 uv run python -m experiments.system.hexframe_interface_compare \
  --interface angle1 --case nominal --out runs/新的接口对照目录
uv run python -m experiments.system.hexframe_review_video \
  --run runs/新的接口对照目录 --out runs/新的回放目录 --case nominal --interface-label angle1_blend030
```

以 `validation.json` 为评价依据；花瓣通过后执行独立审计。失败同样生成保存状态回放，
不因视频成功升级验收。当前结果见 `results/hexframe_interfaces_20261005.md`。
