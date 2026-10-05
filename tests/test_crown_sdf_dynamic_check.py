"""动态平稳与瞬态超载、首触时刻分别评价，避免混淆结论。"""
from types import SimpleNamespace

import numpy as np

from experiments.models_interfaces.crown_sdf_dynamic_check import dynamics_metrics


def sample_log(tail_speed=0.):
    t = np.arange(8001)*.001
    samples = [dict(t=float(x), phase='approach' if x < 1 else 'hold',
                    speed_m_s=.1 if x < 1 else tail_speed, angular_speed_deg_s=0.,
                    interface_contacts=1) for x in t]
    contacts = [dict(interface_world=np.array([0., 0., 100. if x < .5 else .2, 0., 0., 0.]),
                     min_distance_m=-1e-6) for x in t]
    log = SimpleNamespace(docking_samples=samples, pos_actual=np.zeros((len(t), 3)),
                          contact_diagnostics=contacts, joint_angles=np.zeros((len(t), 7)),
                          joint_velocities=np.zeros((len(t), 7)), torque_saturated=np.zeros(len(t)),
                          simulation_warnings={})
    scene = SimpleNamespace(target=SimpleNamespace(quat=[1., 0., 0., 0.]),
                            docking=SimpleNamespace(max_linear_speed=.003, max_angular_speed_deg=2.,
                                                    max_force=40., max_axial_moment=2.))
    return log, scene


def test_early_contact_and_transient_overload_do_not_mean_tail_unsettled():
    result = dynamics_metrics(*sample_log())
    assert result['first_contact_phase'] == 'approach'
    assert result['force_over_limit_duration_s'] == .5
    assert result['tail_settled_under_original_speed_limits']


def test_persistent_tail_motion_is_not_settled():
    result = dynamics_metrics(*sample_log(.02))
    assert not result['tail_settled_under_original_speed_limits']
    assert result['tail']['max_speed_mm_s'] == 20.
