# 冻结历史证据

`runs/` 保存此前纳入版本控制的 27 个运行摘要、驱动和配置的逐字节副本，原本地
`runs/` 数据继续保留。`orbital_showcase/` 策展原本地 outputs 的 48 个 JSON/Markdown
摘要，完整 NPZ、视频、模型及源码快照继续留在 `runs/archived_outputs/`。

这些文件不可写为新输出目录，也不保证搬迁后历史来源指纹仍可复用。原文中的
绝对/相对路径保留取证语义；逐文件哈希和迁移映射在
`docs/evidence/directory_cleanup_20261004.json`，边界见 `docs/directory_cleanup.md`。
