"""检查实验模型确实替换工作接口，保留抓取接口及真实接收位姿误差。"""
import mujoco
import numpy as np
import pytest

from compliant_docking.assembly.runtime import AssemblyRuntime
from compliant_docking.scene import load_scene
from experiments.system.hexframe_interface_compare import InterfaceAdapter
from experiments.system.hexframe_pose_check import inject_receiver_pose


@pytest.mark.parametrize('interface', ['angle1', 'crown_stl'])
def test_working_geometry_and_truth_are_independent_from_arm_reference(tmp_path, interface):
    r = AssemblyRuntime(load_scene('scenes/hexframe_assembly.yaml'), tmp_path)
    adapter = InterfaceAdapter(r, interface)
    r.geometry = adapter
    model = adapter.build_model()
    # 新头挂在实际工作端口子树；抓取上口仍有原几何，不能只替换展示材质。
    root = model.body('module1_port0_root')
    assert model.body_parentid[root.id] == model.body('module1_port_4').id
    assert model.body('module2_port3_root').parentid[0] == model.body('module2_port_1').id
    assert any(model.geom(g).name.startswith('module1_port3_') for g in range(model.ngeom))
    mobile = [g for g in range(model.ngeom)
              if model.geom(g).name.startswith('module1_port0_') and model.geom_contype[g]]
    assert mobile
    sdf = sum(model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SDF for g in mobile)
    assert sdf == (1 if interface == 'crown_stl' else 0)
    reference = r.install_tip.copy()
    truth, evidence = inject_receiver_pose(r, model, (.002, 0., 5.))
    data = mujoco.MjData(truth)
    mujoco.mj_forward(truth, data)
    original_quat = data.body('module1').xquat.copy()
    data.qpos[7:10] = evidence['nominal_module_center_m']
    data.qpos[10:14] = original_quat
    mujoco.mj_forward(truth, data)
    distance, angle, _, _ = r.lock_error(truth, data, 2)
    np.testing.assert_allclose([distance, angle], [.002, np.deg2rad(5)], atol=1e-12)
    np.testing.assert_array_equal(reference, r.install_tip)
