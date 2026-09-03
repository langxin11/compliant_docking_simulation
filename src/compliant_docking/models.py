"""模型加载入口：MuJoCo XML 与 Pinocchio URDF 双描述统一管理。

实验路径一律由 compliant_docking.scene 的场景 YAML 驱动；本模块的 PIN_URDF
仅为向后兼容的默认值。
"""
from pathlib import Path

import numpy as np
import pinocchio as pin

ASSETS_DIR = Path(__file__).resolve().parents[2] / "assets" / "iiwa14"
PIN_URDF = ASSETS_DIR / "iiwa14_dock.urdf"


def load_pin_model(urdf_path=PIN_URDF, gravity: bool = False):
    """加载 Pinocchio 模型；gravity=False 时置零重力（与 MuJoCo 模型保持一致）。

    按文件后缀分发解析器：``.urdf`` 走 URDF 解析，``.xml``（MJCF，如 FR3 的
    Menagerie 模型变体）走 buildModelFromMJCF。两条路线同样置零重力。
    """
    if Path(urdf_path).suffix.lower() == ".xml":
        model = pin.buildModelFromMJCF(str(urdf_path))
    else:
        model = pin.buildModelFromUrdf(str(urdf_path))
    if not gravity:
        # 注意：此版本 pinocchio 的 gravity.linear 需要 Eigen 向量，不能传 Python 元组 /
        # Note: this pinocchio build requires an Eigen vector, not a Python tuple
        model.gravity.linear = np.array([0.0, 0.0, 0.0])
    return model
