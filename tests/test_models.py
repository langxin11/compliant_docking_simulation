"""模型资产与加载入口测试：文件存在性、DoF 数、默认零重力。"""
import numpy as np

from compliant_docking.models import MUJOCO_MODEL, PIN_URDF, load_pin_model


def test_model_files_exist():
    assert MUJOCO_MODEL.is_file(), f"MuJoCo 模型缺失: {MUJOCO_MODEL}"
    assert PIN_URDF.is_file(), f"Pinocchio URDF 缺失: {PIN_URDF}"


def test_load_pin_model_has_7_dof():
    model = load_pin_model()
    assert model.nq == 7
    assert model.nv == 7


def test_load_pin_model_zeroes_gravity_by_default():
    model = load_pin_model()
    np.testing.assert_allclose(np.asarray(model.gravity.linear), np.zeros(3), atol=1e-12)


def test_load_pin_model_keeps_gravity_when_requested():
    model = load_pin_model(gravity=True)
    # Pinocchio 默认重力为 -z 方向 9.81（非零），此处只断言未被置零 /
    # Pinocchio default gravity is 9.81 along -z; only assert it is not zeroed
    assert np.linalg.norm(np.asarray(model.gravity.linear)) > 1.0
