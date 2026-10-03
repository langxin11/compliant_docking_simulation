"""兼容薄壳：主仿真编排已迁入 src/compliant_docking/orchestration/run_docking.py。

保留本文件是为了旧文档命令、source_manifest 中的历史路径和外层脚本导入；
新代码请直接导入包内模块。
"""
from compliant_docking.orchestration.run_docking import *  # noqa: F401,F403
from compliant_docking.orchestration.run_docking import main, run_simulation  # noqa: F401

__all__ = ["main", "run_simulation"]
