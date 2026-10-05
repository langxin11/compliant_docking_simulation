"""提速只影响自由空间；控制步长不能短于真实物理反馈间隔。"""
import json
from dataclasses import asdict

import numpy as np
import pytest

from compliant_docking.control.contact_yaw import control_stride
from compliant_docking.docking_task import build_docking_trajectory
from compliant_docking.research.cases import POINTS
from compliant_docking.research.petal_trials import variant
from compliant_docking.research.protocols import error_tuple
from compliant_docking.research.rollout import _json_default
from compliant_docking.scene import load_scene
from experiments.models_interfaces.angle1_task_check import SCENE, motion_plan


def test_one_real_update_per_step_and_unchanged_contact_gates():
    new = load_scene(SCENE)
    old = load_scene('scenes/iiwa14_petal_angle1_blend030_025ms.yaml')
    assert control_stride(new.se3_impedance.control_period, new.physics.timestep) == 1
    assert new.physics.timestep == new.se3_impedance.control_period == .001
    with pytest.raises(ValueError):
        control_stride(.0005, .001)
    assert asdict(new.docking) == asdict(old.docking)
    gains = asdict(new.se3_impedance)
    gains['control_period'] = old.se3_impedance.control_period
    assert gains == asdict(old.se3_impedance)


@pytest.mark.parametrize('case', POINTS)
def test_all_nine_paths_respect_limits_and_keep_insertion(case):
    new = variant(load_scene(SCENE), case, 'lateral_released', error_tuple(POINTS[case]))
    old = variant(load_scene('scenes/iiwa14_petal_angle1_blend030_025ms.yaml'),
                  case, 'lateral_released', error_tuple(POINTS[case]))
    a, b = motion_plan(new), motion_plan(old)
    assert a['free_space_s'] < b['free_space_s']
    for key in ('duration_s', 'peak_v_a_omega_alpha', 'limits_v_a_omega_alpha'):
        np.testing.assert_allclose(a['segments'][-1][key], b['segments'][-1][key],
                                   rtol=0., atol=1e-12)
    ta = build_docking_trajectory(new.task, new.docking, new.trajectory)
    tb = build_docking_trajectory(old.task, old.docking, old.trajectory)
    for pa, pb in zip(ta.poses, tb.poses, strict=True):
        np.testing.assert_array_equal(pa.homogeneous, pb.homogeneous)


def test_default_scene_uses_verified_angle1_convex_task():
    default = load_scene('scenes/iiwa14_petal_insertion.yaml')
    selected = load_scene(SCENE)
    for field in ('tool', 'target', 'physics', 'trajectory', 'docking', 'se3_impedance'):
        left = json.dumps(asdict(getattr(default, field)), default=_json_default, sort_keys=True)
        right = json.dumps(asdict(getattr(selected, field)), default=_json_default, sort_keys=True)
        assert left == right
