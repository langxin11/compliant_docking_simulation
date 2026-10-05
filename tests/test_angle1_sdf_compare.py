"""混合碰撞原型须保持止挡、惯量、双引擎动力学及计时包装器行为。"""
import xml.etree.ElementTree as ET
from dataclasses import replace

import mujoco
import numpy as np

from compliant_docking.models import load_assembled_pin_model
from compliant_docking.research.petal_trials import preflight
from compliant_docking.scene import load_scene
from experiments.models_interfaces.angle1_sdf_compare import SOURCE, prototype, timers


def test_sdf_keeps_stops_inertia_and_shared_dynamics(tmp_path):
    destination = tmp_path/'sdf'
    prototype(destination)
    for filename in ('active.xml', 'passive.xml'):
        old = ET.parse(SOURCE/filename).getroot()
        new = ET.parse(destination/filename).getroot()
        for name in [g.get('name') for g in old.findall('.//geom') if g.get('name', '').startswith('stop_')]:
            assert old.find(f".//geom[@name='{name}']").attrib == new.find(f".//geom[@name='{name}']").attrib
        assert old.find('.//inertial').attrib == new.find('.//inertial').attrib
    scene = load_scene('scenes/iiwa14_petal_insertion.yaml')
    scene = replace(scene, tool=replace(scene.tool, mjcf=destination/'active.xml'),
                    target=replace(scene.target, mjcf=destination/'passive.xml'))
    assert preflight(scene)['status'] == 'PASS'
    load_assembled_pin_model(scene)
    assert np.count_nonzero(scene.build_mjmodel().geom_type == mujoco.mjtGeom.mjGEOM_SDF) == 8


def test_profiling_preserves_physics_step_and_restores_binding():
    original = mujoco.mj_step
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><freejoint/><geom size=".1"/></body></worldbody></mujoco>')
    a, b = mujoco.MjData(model), mujoco.MjData(model)
    original(model, a)
    with timers() as (totals, counts):
        mujoco.mj_step(model, b)
    np.testing.assert_array_equal(a.qpos, b.qpos)
    np.testing.assert_array_equal(a.qvel, b.qvel)
    assert counts['physics'] == 1 and totals['physics'] > 0
    assert mujoco.mj_step is original
