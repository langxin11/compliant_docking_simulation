"""Sensor-triggered release, causal feedback and held-rate physics checks."""
from dataclasses import replace

import numpy as np
import pinocchio as pin
import pytest

from compliant_docking.cli import _load_run_docking
from compliant_docking.control.contact_yaw import (
    ContactYawSchedule,
    ContactYawSpec,
    control_stride,
)
from compliant_docking.scene import load_scene
from compliant_docking.wrench import WrenchSample


def test_contact_requires_insertion_filter_and_dwell_then_release_stays_latched():
    spec = ContactYawSpec(filter_s=.01, dwell_s=.02, release_s=.25)
    schedule = ContactYawSchedule(spec, .5)
    for t in np.arange(0., .2, .001):
        assert schedule.update(t, "approach", 100.) == .5
    assert schedule.filtered_force_N == 0.
    # One short pulse must not release the spring.
    for t in np.arange(.2, .3, .001):
        schedule.update(t, "insert", 10. if t < .201 else 0.)
    assert schedule.trigger_t is None
    stiffness = [schedule.update(t, "insert", 2.) for t in np.arange(.3, .8, .001)]
    assert schedule.trigger_t >= .3+spec.dwell_s
    assert np.all(np.diff(stiffness) <= 0.)
    assert stiffness[-1] == 0.
    for t in np.arange(.8, 1., .001):
        assert schedule.update(t, "hold", 0.) == 0.
    # Decreasing K at a fixed error cannot add spring energy.
    assert np.all(np.diff(.5*np.asarray(stiffness)*np.deg2rad(5.)**2) <= 0.)


@pytest.mark.parametrize("kwargs", [dict(release_s=0.), dict(filter_s=-1.),
                                   dict(force_threshold_N=np.nan), dict(stiffness_after=-1.),
                                   dict(lateral_stiffness_after=-1.),
                                   dict(lateral_stiffness_after=np.inf)])
def test_invalid_contact_policy_rejected(kwargs):
    with pytest.raises(ValueError):
        ContactYawSpec(**kwargs)


def test_policy_cannot_increase_stiffness_or_reuse_timestamp():
    with pytest.raises(ValueError):
        ContactYawSchedule(ContactYawSpec(stiffness_after=1.), .5)
    schedule = ContactYawSchedule(ContactYawSpec(), .5)
    schedule.update(0., "insert", 1.)
    with pytest.raises(ValueError):
        schedule.update(0., "insert", 1.)


@pytest.mark.parametrize("after", [0., 20.])
def test_lateral_release_shares_sensor_latch_keeps_approach_gains_and_removes_spring_energy(after):
    schedule = ContactYawSchedule(ContactYawSpec(lateral_stiffness_after=after), .5, [80., 120.])
    for t in np.arange(0., .1, .001):
        schedule.update(t, "approach", 100.)
        np.testing.assert_array_equal(schedule.lateral_stiffness, [80., 120.])
    gains = []
    for t in np.arange(.1, .7, .001):
        schedule.update(t, "insert", .3 if t < .3 else 0.)
        gains.append(schedule.lateral_stiffness.copy())
    assert schedule.trigger_t is not None
    np.testing.assert_array_equal(gains[-1], [after, after])
    assert np.all(np.diff(gains, axis=0) <= 0.)
    error = np.array([.006, -.004])
    energy = .5 * (np.asarray(gains) @ error**2)
    assert np.all(np.diff(energy) <= 0.)
    assert schedule.stiffness == 0.  # same release clock, not a second detector


@pytest.mark.parametrize("before", [None, [80.], [80., np.nan], [10., 80.]])
def test_lateral_release_requires_two_valid_gains_and_cannot_increase_them(before):
    with pytest.raises(ValueError):
        ContactYawSchedule(ContactYawSpec(lateral_stiffness_after=20.), .5, before)


@pytest.mark.parametrize("period, dt, expected", [(None, .001, 1), (.0005, .0005, 1),
                                                  (.0005, .00025, 2), (.0005, .000125, 4)])
def test_control_period_is_independent_of_physics_refinement(period, dt, expected):
    assert control_stride(period, dt) == expected


@pytest.mark.parametrize("period, dt", [(.0003, .0005), (.0007, .0005), (0., .001)])
def test_unsupported_control_period_rejected(period, dt):
    with pytest.raises(ValueError):
        control_stride(period, dt)


def test_saved_sensor_pose_survives_mutation_and_transports_to_new_body():
    p, R = np.array([1., 2., 3.]), np.array(pin.exp3(np.array([0., 0., .4])))
    force, moment = np.array([3., 1., -2.]), np.array([.1, .2, .3])
    sample = WrenchSample.from_site(.1, force, moment, p, R)
    expected_force, expected_moment = R @ force, R @ moment
    p[:] = 0.
    R[:] = np.eye(3)
    target = pin.SE3(pin.exp3(np.array([.1, -.2, .3])), np.array([.9, 2.1, 3.2]))
    value = sample.at_body(target)
    np.testing.assert_allclose(target.rotation @ value[:3], expected_force, atol=1e-14)
    np.testing.assert_allclose(target.rotation @ value[3:], expected_moment
                               + np.cross(sample.origin-target.translation, expected_force), atol=1e-14)
    assert sample.t == .1


@pytest.mark.parametrize("dt", [.0005, .00025, .000125])
def test_robot_feedback_delay_and_all_solve_telemetry_share_declared_timing(dt):
    scene = load_scene("scenes/iiwa14_petal_insertion.yaml")
    scene = replace(scene, physics=replace(scene.physics, timestep=dt),
        se3_impedance=replace(scene.se3_impedance,
            contact_yaw=replace(scene.se3_impedance.contact_yaw, lateral_stiffness_after=0.)))
    log = _load_run_docking().main(scene=scene, dt=dt, duration=.006,
                                  render=False, record=False, plot=False, diagnostics="summary")
    ticks = [i for i, s in enumerate(log.se3_diagnostics) if s["control_update"]]
    for old, new in zip(ticks, ticks[1:], strict=False):
        sample = log.se3_diagnostics[new]
        previous_solve = log.contact_diagnostics[old]
        assert sample["feedback_t"] == pytest.approx(previous_solve["t"], abs=1e-12)
        assert sample["feedback_age_s"] == pytest.approx(.0005, abs=1e-12)
        np.testing.assert_allclose(sample["feedback_world_at_origin"], previous_solve["sensor_world"], atol=1e-10)
        for i in range(old, new):
            np.testing.assert_array_equal(log.tau_hist[i], log.tau_hist[old])
    for i, solve in enumerate(log.contact_diagnostics):
        assert log.t_list[i] == log.docking_samples[i]["t"] == solve["t"]
        np.testing.assert_allclose(log.pos_actual[i], solve["position"], atol=1e-12)
        np.testing.assert_array_equal(log.joint_angles[i], solve["q"])
        np.testing.assert_allclose(log.force_externals[i], solve["sensor_world"][:3], atol=1e-10)
        np.testing.assert_allclose(log.torque_externals[i], solve["sensor_world"][3:], atol=1e-10)
    assert log.se3_diagnostics[-1]["yaw_stiffness"] == .5  # no contact, no early release
    np.testing.assert_array_equal(log.se3_diagnostics[-1]["lateral_stiffness"], [80., 80.])
