"""任务空间雅可比参考系与时间导数的一致性测试。"""
import numpy as np
import pinocchio as pin

from compliant_docking.models import load_pin_model
from compliant_docking.scene import DEFAULT_SCENE_PATH, load_scene


def test_lwa_frame_velocity_and_jdot_match_finite_difference():
    """frame 原点 LWA 线速度/Jdot 应分别匹配位置/Jacobian 的有限差分。"""
    scene = load_scene(DEFAULT_SCENE_PATH)
    model = load_pin_model(scene.robot.pin_model)
    data = model.createData()
    frame_id = model.getFrameId(scene.robot.ee_frame)
    q = np.asarray(scene.task.ik_guess, dtype=float)
    v = np.array([0.31, -0.27, 0.19, -0.23, 0.17, -0.13, 0.11])
    h = 1e-6

    pin.forwardKinematics(model, data, q, v)
    pin.computeJointJacobiansTimeVariation(model, data, q, v)
    pin.updateFramePlacements(model, data)
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    Jdot = pin.getFrameJacobianTimeVariation(
        model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    def frame_position(q_sample):
        pin.forwardKinematics(model, data, q_sample)
        pin.updateFramePlacements(model, data)
        return data.oMf[frame_id].translation.copy()

    def frame_jacobian(q_sample):
        return pin.computeFrameJacobian(
            model, data, q_sample, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    q_plus = pin.integrate(model, q, h * v)
    q_minus = pin.integrate(model, q, -h * v)
    velocity_fd = (frame_position(q_plus) - frame_position(q_minus)) / (2.0 * h)
    jdot_fd = (frame_jacobian(q_plus) - frame_jacobian(q_minus)) / (2.0 * h)

    np.testing.assert_allclose(J[:3, :] @ v, velocity_fd, rtol=1e-6, atol=1e-7)
    np.testing.assert_allclose(Jdot, jdot_fd, rtol=2e-5, atol=2e-6)
    assert np.linalg.norm(Jdot) > 1e-5
