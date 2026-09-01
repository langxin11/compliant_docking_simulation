"""compute_ik 单元测试：用已知良好的初值求解标准初始位姿。"""
import numpy as np
import pinocchio as pin

from compliant_docking.models import load_pin_model
from compliant_docking.planning.kinematics import compute_ik

INIT_POS = np.array([0.0, 0.5, 0.5])
INIT_ORI = np.diag([1.0, -1.0, -1.0])
IK_GUESS = np.array([0.0, 0.5, 0.0, -1.0, 0.0, 1.5, 0.0])


def test_ik_known_good_guess_converges():
    model = load_pin_model()
    data = model.createData()

    target_pose = pin.SE3(INIT_ORI, INIT_POS)
    q, success = compute_ik(model, data, target_pose, initial_q=IK_GUESS, max_iters=5000)

    assert success, "IK 应在已知良好初值下收敛"

    # FK 验证：末端平动位置应回到目标（1e-5 内）
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    ee_id = model.getFrameId("cylinder_link")
    fk_pos = data.oMf[ee_id].translation
    np.testing.assert_allclose(fk_pos, INIT_POS, atol=1e-5)

    # 姿态也应回到目标（旋转矩阵逐元素一致）
    np.testing.assert_allclose(data.oMf[ee_id].rotation, INIT_ORI, atol=1e-4)


def test_ik_result_within_joint_limits():
    model = load_pin_model()
    data = model.createData()

    target_pose = pin.SE3(INIT_ORI, INIT_POS)
    q, success = compute_ik(model, data, target_pose, initial_q=IK_GUESS, max_iters=5000)

    assert success
    assert np.all(q >= model.lowerPositionLimit - 1e-9)
    assert np.all(q <= model.upperPositionLimit + 1e-9)
