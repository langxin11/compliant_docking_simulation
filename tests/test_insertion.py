"""Independent reference/physics checks for the combined insertion experiment."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pinocchio as pin
import pytest

from compliant_docking.cli import _load_run_docking
from compliant_docking.docking_task import (
    build_docking_trajectory,
    docking_sample,
    evaluate_docking,
)
from compliant_docking.scene import load_scene


@pytest.fixture
def scene():
    return load_scene("scenes/iiwa14_compliant_insertion.yaml")


def test_side_start_does_not_move_insertion_axis_or_target(scene):
    plan = build_docking_trajectory(scene.task, scene.docking, scene.trajectory)
    moved = build_docking_trajectory(replace(scene.task, init_pos=np.array([.15, .35, .58])),
                                     scene.docking, scene.trajectory)
    # Changing the initial pose must not change any target-relative waypoint.
    for p, q in zip(plan.poses[1:], moved.poses[1:], strict=True):
        np.testing.assert_allclose(p.homogeneous, q.homogeneous)
    entry_t, final_t = plan.times[-2:]
    for t in np.linspace(entry_t, final_t, 21):
        np.testing.assert_allclose(plan.get_state(t)[0][:2], scene.docking.estimate_pos[:2])
        np.testing.assert_allclose(plan.get_state(t)[1][:2], 0.0, atol=1e-14)
    np.testing.assert_allclose(plan.poses[-1].translation,
                               np.array(scene.docking.estimate_pos)+[0, 0, .03])


def test_low_start_adds_vertical_clearance_leg(scene):
    task = replace(scene.task, init_pos=np.array([-.12, .40, .45]))
    plan = build_docking_trajectory(task, scene.docking, scene.trajectory)
    assert plan.names[:3] == ["start", "lift", "approach"]
    np.testing.assert_allclose(plan.poses[1].translation, [-.12, .4, .48])


def test_motion_reference_matches_independent_pose_differences(scene):
    plan = build_docking_trajectory(scene.task, scene.docking, scene.trajectory)
    h = 1e-5
    for t in np.linspace(.1, plan.times[1]-.1, 5):
        pose, V, Vdot = plan.get_motion_state(t)
        plus, minus = plan.get_pose(t+h), plan.get_pose(t-h)
        linear = pose.rotation.T @ ((plus.translation-minus.translation)/(2*h))
        angular = pin.log3(minus.rotation.T @ plus.rotation)/(2*h)
        np.testing.assert_allclose(V, np.r_[linear, angular], atol=1e-8)
        _, vp, _ = plan.get_motion_state(t+h)
        _, vm, _ = plan.get_motion_state(t-h)
        np.testing.assert_allclose(Vdot, (vp-vm)/(2*h), atol=1e-8)
    for t, pose in zip(plan.times, plan.poses, strict=True):
        actual, V, Vdot = plan.get_motion_state(t)
        np.testing.assert_allclose(actual.homogeneous, pose.homogeneous, atol=1e-12)
        np.testing.assert_allclose(V, 0, atol=1e-12)
        np.testing.assert_allclose(Vdot, 0, atol=1e-12)


def _settled_log(scene, plan):
    samples = []
    rotation = pin.exp3(np.array([0., 0., np.deg2rad(-10)])) @ np.diag([1., -1., -1.])
    for t in np.arange(plan.total_duration, plan.total_duration+scene.docking.hold_s+.001, .01):
        samples.append(docking_sample(
            scene.docking, scene.target, t=t, phase="hold", position=np.array([0., .5, .4]),
            rotation=rotation, velocity=np.zeros(3), angular_velocity=np.zeros(3),
            force=np.array([0., 0., 5.]), moment=np.array([0., 0., .1]),
            interface_contacts=1, other_contacts=0))
    return SimpleNamespace(docking_samples=samples, torque_saturated=[False]*len(samples))


def test_gate_accepts_settled_twist_but_rejects_shallow_or_unsettled_contact(scene):
    plan = build_docking_trajectory(scene.task, scene.docking, scene.trajectory)
    log = _settled_log(scene, plan)
    # A passive turn away from the commanded yaw is not an orientation failure.
    assert evaluate_docking(log, scene.docking, plan, .01)["status"] == "PASS"
    log.docking_samples[-1]["depth_mm"] = 1.
    assert "insufficient insertion" in evaluate_docking(log, scene.docking, plan, .01)["reasons"]
    log.docking_samples[-1]["depth_mm"] = 20.
    log.docking_samples[-1]["angular_speed_deg_s"] = 10.
    assert "not settled (rotation)" in evaluate_docking(log, scene.docking, plan, .01)["reasons"]


def test_gate_rejects_early_collision_and_never_passes_short_runs(scene):
    plan = build_docking_trajectory(scene.task, scene.docking, scene.trajectory)
    log = _settled_log(scene, plan)
    log.docking_samples = log.docking_samples[:20]
    assert evaluate_docking(log, scene.docking, plan, .01)["status"] == "INCOMPLETE"
    log.docking_samples[0]["phase"] = "approach"
    assert evaluate_docking(log, scene.docking, plan, .01)["status"] == "FAIL"
    log.docking_samples[0]["depth_mm"] = float("nan")
    assert evaluate_docking(log, scene.docking, plan, .01)["status"] == "FAIL"


def test_truth_changes_evaluation_but_not_reference(scene):
    plan = build_docking_trajectory(scene.task, scene.docking, scene.trajectory)
    truth_changed = replace(scene, target=replace(scene.target, pos=scene.target.pos+[.01, 0, 0]))
    same = build_docking_trajectory(truth_changed.task, truth_changed.docking, truth_changed.trajectory)
    for t in np.linspace(0, plan.total_duration, 20):
        np.testing.assert_array_equal(plan.get_pose(t).homogeneous, same.get_pose(t).homogeneous)
    assert _settled_log(scene, plan).docking_samples[-1]["lateral_mm"] == pytest.approx(0.)
    assert _settled_log(truth_changed, plan).docking_samples[-1]["lateral_mm"] == pytest.approx(10.)


def test_waypoint_docking_rejects_controllers_without_pose_reference(scene):
    with pytest.raises(ValueError, match="requires se3_lie"):
        _load_run_docking().main(scene=scene, controller="impedance", render=False, record=False)


def test_scene_gain_and_depth_validation(scene):
    with pytest.raises(ValueError, match="finite"):
        replace(scene.docking, estimate_yaw_deg=float("nan"))
    with pytest.raises(ValueError, match="command_depth"):
        replace(scene.docking, min_depth=1.)


@pytest.mark.slow
def test_combined_insertion_rollout(scene):
    """Full contact gate with original crown phasing plus XY/yaw uncertainty."""
    log = _load_run_docking().main(scene=scene, render=False, record=False, plot=False)
    assert log.docking_gate["status"] == "PASS", log.docking_gate
    assert log.docking_gate["post_contact_advance_mm"] > 1.0
    assert abs(log.docking_gate["contact_yaw_change_deg"]) > 0.1
