# 历史实现

`orbital_showcase/` 保留原脚本、旧场景、资产与说明的唯一实现；原生产 HexFrame
资产通过相对符号链接读取 `assets/modules/hexframe/`。`hexframe_validation/` 保留
原始验证协议说明。当前正式入口见 `experiments/models_interfaces/`、
`experiments/control/` 和 `experiments/system/`。

旧说明中的原输出路径与结果数字保持历史语义，不代表当前运行位置或新验收。
默认 outputs 相对链接指向可写 `runs/archive_workspaces/`，冻结证据在 `results/`。
完整本地历史输出在 `runs/archived_outputs/`；不能将历史 source_manifest 的原路径
换成新路径来伪装同源复现。详见 [迁移记录](../docs/directory_cleanup.md)。
