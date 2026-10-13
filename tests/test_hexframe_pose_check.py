"""验证真实接收偏差不会被规划同步消除，也不会破坏预锁底座/存储连接。"""
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from compliant_docking.assembly.runtime import AssemblyRuntime
from compliant_docking.scene import load_scene
from experiments.system.hexframe_pose_check import inject_receiver_pose


def test_receiver_moves_while_reference_and_stored_module_remain_nominal(tmp_path):
    runtime = AssemblyRuntime(load_scene('scenes/hexframe_assembly.yaml'), tmp_path)
    model = runtime.geometry.build_model()
    nominal_tip = runtime.install_tip.copy()
    original = mujoco.MjData(model)
    mujoco.mj_forward(model, original)
    actual_model, evidence = inject_receiver_pose(runtime, model, (.002, 0., 5.))
    actual = mujoco.MjData(actual_model)
    mujoco.mj_forward(actual_model, actual)
    np.testing.assert_array_equal(runtime.install_tip, nominal_tip)
    np.testing.assert_allclose(actual.body('module2').xpos-original.body('module2').xpos,
                               [.002, 0., 0.], atol=1e-12)
    np.testing.assert_allclose(actual.body('module2').xmat.reshape(3, 3),
                               Rotation.from_euler('z', 5, degrees=True).as_matrix()
                               @ original.body('module2').xmat.reshape(3, 3), atol=1e-12)
    # 独立检查：停在名义安装姿态时，真实评分必须看到 2 mm 和 5°，不能随目标一起消失。
    actual.qpos[7:10] = evidence['nominal_module_center_m']
    actual.qpos[10:14] = original.body('module1').xquat
    mujoco.mj_forward(actual_model, actual)
    distance, angle, _, _ = runtime.lock_error(actual_model, actual, 2)
    np.testing.assert_allclose(distance, .002, atol=1e-12)
    np.testing.assert_allclose(angle, np.deg2rad(5), atol=1e-12)
    np.testing.assert_array_equal(actual_model.geom_friction, model.geom_friction)
