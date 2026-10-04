# 历史命令兼容

旧脚本的唯一实现迁至 `archive/orbital_showcase/`；这里的 Python 文件仅转发。
正式 HexFrame 入口为 `python -m experiments.system.hexframe`，唯一生产模型为
`assets/modules/hexframe/`。旧资源位置仅保留 Linux 相对符号链接供已保存模型解析。

历史输出摘要位于 `results/historical/orbital_showcase/`，完整本地旧输出保留在
`runs/archived_outputs/orbital_showcase/`。新旧命令的默认可写 outputs 链接指向独立
`runs/archive_workspaces/orbital_showcase/`；它不指向冻结证据。审计输入需显式选择，
详见 [归档迁移](../../docs/directory_cleanup.md)。
