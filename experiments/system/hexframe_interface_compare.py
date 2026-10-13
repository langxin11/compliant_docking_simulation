"""HexFrame 两种接口的有限几何接入与同源视频验证。

仅替换模块1接口4、接收模块2接口1及匹配存储头；抓取头和其他接口保留原资源。
模块质量惯量、布局、规划和控制保持原系统。冠形用明确的安装偏置/相位适配，
原始 STL 和内部40°变换保留。冠形没有独立止挡，几何接触候选不得称止挡验证。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import time
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'egl')
os.environ.setdefault('MPLBACKEND', 'Agg')

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from compliant_docking.assembly.audit import audit
from compliant_docking.assembly.consistency import prepare_models
from compliant_docking.assembly.geometry import quaternion, vector
from compliant_docking.assembly.runner import provenance, write_json
from compliant_docking.assembly.runtime import AssemblyRuntime
from compliant_docking.assembly.simulation import simulate
from compliant_docking.scene import REPO_ROOT, load_scene
from experiments.system.hexframe_pose_check import CASES, inject_receiver_pose


class InterfaceAdapter:
    """在独立运行的模型中安装接口资产，保留模块 CAD 质量惯量。

    Args:
        runtime: 本次装配运行对象；模型与检查文件写入其 output 目录。
        interface: angle1（凸块花瓣）或 crown_stl（原冠形 SDF）。
            只替换工作接口与匹配存储头；其他接口保留原模块资源。
    """

    def __init__(self, runtime, interface):
        self.runtime = runtime
        self.interface = interface
        self.inputs = set()
        self.original = runtime.geometry
        self.crown_gap = .0471874034950606

    def attach(self, tree, parent, source, prefix, offset, mount_deg, storage=False):
        """导入独立命名空间的固定几何，剥离源惯量以保持模块 CAD 惯量。

        源欧拉角先转四元数，避免度/弧度混用。原地修改 XML 树，记录资产来源。

        Args:
            tree: 接收导入资产和默认类的整机 XML 根节点。
            parent: 接口固定挂载的父 body（刚体坐标系）节点。
            source: 接口 MJCF 文件路径，网格路径相对此文件解析。
            prefix: 名称前缀，同时决定工作接触和止挡的日志分类。
            offset: 接口根相对 parent 的局部 Z 偏置，单位 m。
            mount_deg: 接口根绕 parent 局部 Z 的安装角，单位度。
            storage: 为真时使用存储接触分类，否则使用模块间接触分类。
        """
        self.inputs.add(source)
        imported = ET.parse(source).getroot()
        compiler = imported.find('compiler')
        degrees = compiler is None or compiler.get('angle', 'degree') == 'degree'
        mesh_dir = source.parent / (compiler.get('meshdir', '.') if compiler is not None else '.')
        names = {item.get('name'): prefix+item.get('name') for item in imported.iter()
                 if item.get('name')}
        classes = {item.get('class'): prefix+item.get('class') for item in imported.iter('default')
                   if item.get('class')}
        for item in imported.iter():
            if item.get('name'):
                item.set('name', names[item.get('name')])
            for key in ('mesh', 'material'):
                if item.get(key) in names:
                    item.set(key, names[item.get(key)])
            for key in ('class', 'childclass'):
                if item.get(key) in classes:
                    item.set(key, classes[item.get(key)])
            if item.get('euler'):
                rotation = Rotation.from_euler('xyz', np.fromstring(item.get('euler'), sep=' '), degrees=degrees)
                item.attrib.pop('euler')
                item.set('quat', vector(rotation.as_quat(scalar_first=True)))
        for item in imported.findall('asset/*'):
            if item.get('file'):
                resolved = (mesh_dir/item.get('file')).resolve()
                item.set('file', str(resolved))
                self.inputs.add(resolved)
            tree.find('asset').append(item)
        if imported.find('default') is not None:
            tree.append(imported.find('default'))
        body = imported.find('worldbody/body')
        body.set('name', prefix+'root')
        body.set('pos', vector([0., 0., offset]))
        body.set('quat', vector(quaternion(Rotation.from_euler('z', mount_deg, degrees=True).as_matrix())))
        for owner in body.iter():
            for child in list(owner):
                if child.tag == 'inertial':
                    owner.remove(child)
        for geom in body.iter('geom'):
            visual = geom.get('contype') == '0' or 'visual' in geom.get('class', '')
            geom.set('mass', '0')
            if visual:
                geom.set('contype', '0')
                geom.set('conaffinity', '0')
                geom.set('group', '2')
            else:
                geom.set('contype', '4' if storage else '2')
                geom.set('conaffinity', '2')
                geom.set('group', '3')
                geom.set('friction', '.15 .003 .0001')
                geom.set('solref', '.008 1')
                geom.set('solimp', '.95 .99 .0002')
                geom.set('condim', '3')
        parent.append(body)

    def build_model(self):
        """写出对照模型，核对安装参考与模块惯量，并返回 MuJoCo 模型。

        使用当前 runtime.output，覆盖本次运行构建中的 model.xml 并写入
        interface_adapter.json；不修改源资产。检查固定挂载变换，不证明动态落座。
        """
        r = self.runtime
        base_model = self.original.build_model()
        tree = ET.parse(r.output/'model.xml').getroot()
        for module, port in [('module1', 'port0'), ('module2', 'port3')]:
            parent = tree.find(f".//body[@name='{module}']")
            for item in list(parent):
                if item.tag == 'geom' and item.get('name', '').startswith(module+'_'+port+'_'):
                    parent.remove(item)
        storage = tree.find(".//body[@name='storage_dock_port']")
        for item in list(storage):
            if item.tag == 'geom' and not item.get('name', '').endswith('_adapter'):
                storage.remove(item)
        mobile = tree.find(".//body[@name='module1_port_4']")
        receiver = tree.find(".//body[@name='module2_port_1']")
        if self.interface == 'angle1':
            folder = REPO_ROOT/'assets/interfaces/petal_guidance/angle1_blend030'
            active, passive = folder/'active.xml', folder/'passive.xml'
            offset, mount = 0., 0.
        else:
            active = REPO_ROOT/'assets/interfaces/male_cone.xml'
            passive = REPO_ROOT/'assets/interfaces/female_socket.xml'
            # 两侧根略向模块内退，使模块/轨迹布局不变时几何根间距为冠形参考。
            offset = (r.resource.gap-self.crown_gap)/2
            # 原端口名义45°，公头局部+40.75°使冠形外部根相位为4.25°。
            mount = 40.75
        self.attach(tree, mobile, active, 'module1_port0_', offset, mount)
        self.attach(tree, receiver, passive, 'module2_port3_', offset, 0.)
        self.attach(tree, storage, passive, 'storage_dock_port_', offset, 0., storage=True)
        ET.ElementTree(tree).write(r.output/'model.xml', encoding='unicode')
        model = mujoco.MjModel.from_xml_path(str(r.output/'model.xml'))
        for name in ('module1', 'module2'):
            np.testing.assert_allclose(model.body(name).mass, base_model.body(name).mass, atol=1e-12)
            np.testing.assert_allclose(model.body(name).inertia, base_model.body(name).inertia, atol=1e-12)
            np.testing.assert_allclose(model.body(name).ipos, base_model.body(name).ipos, atol=1e-12)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.original.check_base_dock(model, data)
        self.original.check_storage_docks(model, data)
        # 用安装位置检查新配合根：安装参考不能仅在文档中声明。
        data.qpos[7:10] = r.installed
        mujoco.mj_forward(model, data)
        a, b = data.body('module1_port0_root'), data.body('module2_port3_root')
        R = b.xmat.reshape(3, 3)
        relative = R.T @ (a.xpos-b.xpos)
        relative_rotation = R.T @ a.xmat.reshape(3, 3)
        expected_gap = r.resource.gap if self.interface == 'angle1' else self.crown_gap
        expected_angle = 45. if self.interface == 'angle1' else 4.25
        expected_rotation = Rotation.from_euler('z', expected_angle, degrees=True).as_matrix() @ np.diag([1., -1., -1.])
        np.testing.assert_allclose(relative, [0., 0., expected_gap], atol=1e-10)
        np.testing.assert_allclose(relative_rotation, expected_rotation, atol=1e-10)
        collisions = [g for g in range(model.ngeom) if model.geom(g).name.startswith('module1_port0_') and model.geom_contype[g]]
        details = dict(interface=self.interface, expected_root_gap_m=expected_gap,
                       expected_root_phase_deg=expected_angle, local_root_offset_m=offset,
                       active_local_mount_deg=mount, native_crown_internal_deg=40. if self.interface == 'crown_stl' else None,
                       moving_collision_geoms=len(collisions), sdf_geoms=int(np.sum(model.geom_type==mujoco.mjtGeom.mjGEOM_SDF)),
                       module_mass_inertia_retained=True, replacement_scope=['module1 port4','module2 port1','matching storage head'],
                       contact_parameters=dict(friction=[.15,.003,.0001], solref=[.008,1.], solimp=[.95,.99,.0002]),
                       seating_condition='原承载止挡' if self.interface == 'angle1' else '冠形接触＋几何候选；无独立止挡标签')
        write_json(r.output/'interface_adapter.json', details)
        return model

    def __getattr__(self, name):
        return getattr(self.original, name)

    def stop_loaded(self, model, data):
        """花瓣按真实止挡；冠形仅返回接触几何候选，不虚构止挡标签。"""
        if self.interface == 'angle1':
            return self.original.stop_loaded(model, data)
        f, _, count, _, _ = self.original.wrench(model, data)
        error = self.runtime.lock_error(model, data, 2)
        return bool(count and np.linalg.norm(f)>1e-5 and error[0]<=.00075 and error[1]<=np.deg2rad(.5))


def run_variant(interface, case, out):
    """对一个接口/代表点执行完整交接或保留真实超时，输出独立来源。

    Args:
        interface: angle1 或 crown_stl；模块质量惯量与原关节伺服/导纳保留。
        case: nominal、xy、yaw 或 combined；偏差施加到接收站真值，单位见 CASES。
        out: 新输出目录。物理及控制积分为 1 ms，接触参考 IK 为 10 ms，
            保存 100 Hz 状态与 1 kHz 接触记录。冠形使用接触几何候选而非止挡标签。

    Returns:
        原评价状态 PASS 或 FAIL；仅花瓣 PASS 继续执行原系统独立审计。

    Raises:
        ValueError: 输出目录非空，拒绝覆盖已有证据。
    """
    if out.exists() and any(out.iterdir()):
        raise ValueError('拒绝覆盖非空运行目录')
    out.mkdir(parents=True, exist_ok=True)
    r = AssemblyRuntime(load_scene('scenes/hexframe_assembly.yaml'), out)
    adapter = InterfaceAdapter(r, interface)
    r.geometry = adapter
    model = adapter.build_model()
    prepare_models(r)
    spline, phases, planning = r.plan(model)
    model, _ = inject_receiver_pose(r, model, CASES[case])
    planning['rejected_unsafe_events'] = r.verify_interlocks(model, spline(0))
    write_json(out/'planning.json', planning)
    write_json(out/'phases.json', [dict(key=p.key,title=p.title,seconds=p.seconds,end=p.end.tolist(),event=p.event) for p in phases])
    write_json(out/'runtime.json',dict(physics_timestep_s=.001, state_record_hz=100, contact_record_hz=1000,
        controller='原关节伺服＋轴向接触导纳',seating_gate='raw_strict',
        interface=interface,validation_context='几何对照原型，原模块惯量；冠形没有独立承载止挡标签'))
    try:
        _, report = simulate(r, model, spline, phases, planning)
        report['interface_variant'] = interface
        report['seating_condition_semantics'] = json.loads((out/'interface_adapter.json').read_text())['seating_condition']
        write_json(out/'validation.json', report)
    finally:
        provenance(r)
        manifest_path = out/'source_manifest.json'
        manifest = json.loads(manifest_path.read_text())
        for path in [Path(__file__).resolve(), REPO_ROOT/'experiments/system/hexframe_pose_check.py']:
            dest=out/'source_snapshot'/path.relative_to(REPO_ROOT)
            dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(path,dest)
            manifest[str(dest)]=hashlib.sha256(dest.read_bytes()).hexdigest()
        for path in [*adapter.inputs, out/'interface_adapter.json', out/'perturbation.json', out/'nominal_model.xml']:
            manifest[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        write_json(manifest_path,manifest)
    if report['status']=='PASS' and interface=='angle1':
        audit(out)
    return report['status']


def main():
    """选定两个接口之一和四个固定代表点，不做自动调参。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface',choices=['angle1','crown_stl'],required=True)
    parser.add_argument('--case',choices=CASES,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    start=time.monotonic()
    result=run_variant(args.interface,args.case,args.out.resolve())
    print(args.interface,args.case,result,'wall_s',time.monotonic()-start,flush=True)


if __name__=='__main__':
    main()
