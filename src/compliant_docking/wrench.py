"""
wrench.py — F/T wrench 的坐标系与参考点变换

把 MuJoCo F/T 传感器（site 系、site 原点参考）读数变换为控制器使用的
EE body wrench（与 Pinocchio ``ReferenceFrame.LOCAL`` body Jacobian 同 frame、
同参考点），供 SE(3) Lie 阻抗控制器（Kim et al. 2025 Eq. 50/65）使用。

为什么不直接 ``current_ori @ force_sensor``（run_docking 旧路径）：

1. 那得到的是世界轴表达（新控制器需要 body 系 F 与 body Jacobian 配对）；
2. sensor site 与 Pinocchio EE frame 原点不重合；
3. 矩的参考点平移需要 (p_S - p_E) × f 项，纯旋转会丢失该耦合。

变换链（任务书 §8 / 论文 body wrench 定义 F = [f; n]）::

    f_W      = R_WS · f_S
    n_W_at_S = R_WS · n_S
    n_W_at_E = n_W_at_S + (p_S - p_E) × f_W
    f_E      = R_WE^T · f_W
    n_E      = R_WE^T · n_W_at_E

与 control.lie_se3.adjoint_wrench 的关系：本模块是其在"世界系中转"下
的显式实现，``transform_wrench(...)`` 数值等价于
``adjoint_wrench(T_E_S) @ [f_S; n_S]``，其中 ``T_E_S = T_WE⁻¹ · T_WS``
（测试 tests/test_wrench.py 交叉验证）。
"""
from __future__ import annotations

import numpy as np
import pinocchio as pin


def transform_wrench(force: np.ndarray, torque: np.ndarray,
                     p_source: np.ndarray, R_source: np.ndarray,
                     p_target: np.ndarray, R_target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """把 source frame（原点参考）的 wrench 变换到 target frame。

    Args:
        force: source 系表达的力 f_S (3,)
        torque: source 系表达的矩 n_S (3,)，参考点为 source 原点
        p_source, R_source: source frame 的世界位姿（p (3,)，R (3,3)）
        p_target, R_target: target frame 的世界位姿

    Returns:
        (f_target, n_target)：target 系表达、target 原点参考的 wrench
    """
    f_S = np.asarray(force, dtype=float).reshape(3)
    n_S = np.asarray(torque, dtype=float).reshape(3)
    p_S = np.asarray(p_source, dtype=float).reshape(3)
    R_S = np.asarray(R_source, dtype=float).reshape(3, 3)
    p_T = np.asarray(p_target, dtype=float).reshape(3)
    R_T = np.asarray(R_target, dtype=float).reshape(3, 3)

    f_W = R_S @ f_S
    n_W_at_S = R_S @ n_S
    n_W_at_T = n_W_at_S + np.cross(p_S - p_T, f_W)
    f_T = R_T.T @ f_W
    n_T = R_T.T @ n_W_at_T
    return f_T, n_T


def wrench_to_body(force: np.ndarray, torque: np.ndarray,
                   p_source: np.ndarray, R_source: np.ndarray,
                   T_target: pin.SE3) -> np.ndarray:
    """传感器 site wrench → EE body wrench F_body = [f; n]（6 维）。

    Args:
        force, torque: 传感器 site 系读数（含既有符号约定，本函数只做
            坐标变换，不改符号——MuJoCo 读取侧的负号由调用方保留）
        p_source, R_source: sensor site 的世界位姿（MuJoCo site_xpos/xmat）
        T_target: Pinocchio EE frame 位姿（oMf，与 body Jacobian 同源）

    Returns:
        F_body (6,) = [f_E; n_E]，EE body 系、EE frame 原点参考
    """
    f_E, n_E = transform_wrench(force, torque, p_source, R_source,
                                np.asarray(T_target.translation),
                                np.asarray(T_target.rotation))
    return np.concatenate([f_E, n_E])
