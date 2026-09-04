"""模型加载入口：MuJoCo XML 与 Pinocchio URDF 双描述统一管理。

实验路径一律由 compliant_docking.scene 的场景 YAML 驱动；本模块的 PIN_URDF
仅为向后兼容的默认值。
"""
from pathlib import Path

import numpy as np
import pinocchio as pin

ASSETS_DIR = Path(__file__).resolve().parents[2] / "assets" / "iiwa14"
PIN_URDF = ASSETS_DIR / "iiwa14_dock.urdf"


def _rotation_from_wxyz(quat: np.ndarray) -> np.ndarray:
    """将场景 YAML/MuJoCo 的 ``wxyz`` 四元数转换为旋转矩阵。"""
    w, x, y, z = np.asarray(quat, dtype=float).reshape(4)
    norm = np.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0:
        raise ValueError("tool_mount_quat 不能为零四元数")
    w, x, y, z = (w / norm, x / norm, y / norm, z / norm)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _append_tool_inertia(model: pin.Model, *, tool_frame: str,
                         tool_mount_pos: np.ndarray, tool_mount_quat: np.ndarray,
                         tool_mass: float, tool_com: np.ndarray,
                         tool_diaginertia: np.ndarray) -> None:
    """将场景中显式声明的固定工具惯量附加到 Pinocchio 末端 frame。

    MuJoCo 的工具 MJCF 在运行时挂载到 ``tool_frame``；MJCF 直读的 Pinocchio
    模型（FR3）不包含该片段。此函数以相同挂载位姿把惯量加入该 frame 的父关节。
    组合 URDF 已自带工具时不传此参数，避免重复计入质量。
    """
    if tool_mass <= 0.0:
        raise ValueError("tool_mass 必须为正")
    frame_id = model.getFrameId(tool_frame)
    if frame_id >= len(model.frames):
        raise ValueError(f"Pinocchio 模型缺少工具挂载 frame: {tool_frame!r}")
    diagonal = np.asarray(tool_diaginertia, dtype=float).reshape(3)
    if np.any(diagonal <= 0.0):
        raise ValueError("tool_diaginertia 必须逐轴为正")
    frame = model.frames[frame_id]
    mount = pin.SE3(_rotation_from_wxyz(tool_mount_quat),
                    np.asarray(tool_mount_pos, dtype=float).reshape(3))
    placement = frame.placement * mount
    inertia = pin.Inertia(float(tool_mass), np.asarray(tool_com, dtype=float).reshape(3),
                          np.diag(diagonal))
    model.appendBodyToJoint(frame.parentJoint, inertia, placement)


def load_pin_model(urdf_path=PIN_URDF, gravity: bool = False, *,
                   tool_frame: str | None = None,
                   tool_mount_pos: np.ndarray | None = None,
                   tool_mount_quat: np.ndarray | None = None,
                   tool_mass: float | None = None,
                   tool_com: np.ndarray | None = None,
                   tool_diaginertia: np.ndarray | None = None):
    """加载 Pinocchio 模型；gravity=False 时置零重力（与 MuJoCo 模型保持一致）。

    按文件后缀分发解析器：``.urdf`` 走 URDF 解析，``.xml``（MJCF，如 FR3 的
    Menagerie 模型变体）走 buildModelFromMJCF。两条路线同样置零重力。
    """
    if Path(urdf_path).suffix.lower() == ".xml":
        model = pin.buildModelFromMJCF(str(urdf_path))
    else:
        model = pin.buildModelFromUrdf(str(urdf_path))
    tool_args = (tool_frame, tool_mount_pos, tool_mount_quat,
                 tool_mass, tool_com, tool_diaginertia)
    if any(arg is not None for arg in tool_args):
        if any(arg is None for arg in tool_args):
            raise ValueError("附加工具惯量需要完整的 frame、挂载位姿和惯量参数")
        _append_tool_inertia(
            model,
            tool_frame=tool_frame,
            tool_mount_pos=tool_mount_pos,
            tool_mount_quat=tool_mount_quat,
            tool_mass=tool_mass,
            tool_com=tool_com,
            tool_diaginertia=tool_diaginertia,
        )
    if not gravity:
        # 注意：此版本 pinocchio 的 gravity.linear 需要 Eigen 向量，不能传 Python 元组 /
        # Note: this pinocchio build requires an Eigen vector, not a Python tuple
        model.gravity.linear = np.array([0.0, 0.0, 0.0])
    return model
