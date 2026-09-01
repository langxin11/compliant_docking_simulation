"""测试共享配置：在任何 mujoco/matplotlib 导入前锁定无头后端。

- MUJOCO_GL=egl：离屏渲染走 EGL（CI/服务器无显示环境）。
  注意用真值判断而非 setdefault：环境里可能存在 MUJOCO_GL=""（已设置但为空）。
- MPLBACKEND=Agg：matplotlib 只落盘不弹窗。
"""
import os

if not os.environ.get("MUJOCO_GL"):
    os.environ["MUJOCO_GL"] = "egl"
os.environ.setdefault("MPLBACKEND", "Agg")
