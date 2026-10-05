"""单进程三点 SDF 原型对照：先剖析凸块名义任务，再替换花瓣导向碰撞。

所有输出进入新 runs 目录。只改变导向面的碰撞表示，止挡、惯量和控制不变。
分项计时采用相同包装器，墙钟结果为本机单次观测，不代表普遍性能保证。
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from collections import defaultdict
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np

from compliant_docking.contact_diagnostics import ContactDiagnostics
from compliant_docking.control.se3_impedance import SE3LieImpedanceController
from compliant_docking.research import petal_trials
from compliant_docking.research.protocols import error_tuple, source_manifest, write_json
from compliant_docking.scene import REPO_ROOT, load_scene
from compliant_docking.telemetry import Log
from experiments.models_interfaces.angle1_task_check import audit_point, motion_plan

CASES = ('nominal', 'nx6', 'combo_ny6_p15')
SOURCE = REPO_ROOT/'assets/interfaces/petal_guidance/angle1_blend030'
SCENE = 'scenes/iiwa14_petal_insertion.yaml'


def sha(path):
    """返回文件内容指纹，绑定本次资产与代码。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@contextmanager
def timers():
    """相同包装器统计互不嵌套的物理、控制、诊断和持久化耗时（秒）。"""
    totals, counts, originals = defaultdict(float), defaultdict(int), []
    targets = [(mujoco, 'mj_step', 'physics'),
               (SE3LieImpedanceController, 'compute_control', 'control'),
               (SE3LieImpedanceController, 'get_task_space_state', 'state_kinematics'),
               (ContactDiagnostics, 'capture', 'contact_logging'),
               (Log, 'store_data', 'state_logging'),
               (petal_trials, 'save_rollout', 'save_logs')]
    for obj, name, label in targets:
        original = getattr(obj, name)
        originals.append((obj, name, original))

        def measured(*args, _original=original, _label=label, **kwargs):
            start = time.perf_counter()
            try:
                return _original(*args, **kwargs)
            finally:
                totals[_label] += time.perf_counter()-start
                counts[_label] += 1
        setattr(obj, name, measured)
    try:
        yield totals, counts
    finally:
        for obj, name, original in originals:
            setattr(obj, name, original)


def prototype(destination):
    """由原花瓣闭合表面创建 SDF；原止挡和显式惯量逐项保持。"""
    destination.mkdir(parents=True, exist_ok=False)
    checks = {}
    for filename in ('active.xml', 'passive.xml'):
        root = ET.parse(SOURCE/filename).getroot()
        original_body = copy.deepcopy(root.find('worldbody/body'))
        body = root.find('worldbody/body')
        guides = [g for g in body.findall('geom') if g.get('name', '').startswith('guide_')]
        assert guides
        for g in guides:
            body.remove(g)
        for index in range(4):
            ET.SubElement(body, 'geom', name=f'guide_sdf_{index}', type='sdf',
                          mesh=f'pd_v_petal_{index}', attrib={'class': 'pd_collision'})
        # 资产仍引用同一批网格；删除不再使用的凸导向网格声明，避免无效编译开销。
        asset = root.find('asset')
        used = {g.get('mesh') for g in root.iter('geom') if g.get('mesh')}
        for mesh in list(asset.findall('mesh')):
            if mesh.get('name') not in used:
                asset.remove(mesh)
            else:
                mesh.set('file', str((SOURCE/mesh.get('file')).resolve()))
        for tag in ('inertial',):
            assert ET.tostring(body.find(tag)) == ET.tostring(original_body.find(tag))
        def stops(b):
            return [ET.tostring(g) for g in b.findall('geom')
                    if g.get('name', '').startswith('stop_')]
        assert stops(body) == stops(original_body)
        checks[filename] = dict(convex_guides_removed=len(guides), sdf_guides=4,
                                stops_unchanged=len(stops(body)), inertia_unchanged=True)
        ET.indent(root)
        ET.ElementTree(root).write(destination/filename, encoding='utf-8', xml_declaration=True)
    shutil.copy2(SOURCE/'model_info.json', destination/'model_info.json')
    write_json(destination/'prototype.json', dict(checks=checks, source_manifest_sha256=sha(SOURCE/'manifest.json'),
                representation='MuJoCo mesh SDF; original closed petal surfaces; convex stops retained',
                native_sdf_settings_unchanged=True))
    return checks


def scene_for(out, backend):
    """只切换工具与目标接口文件，保留同一场景的其余设置。"""
    base = load_scene(SCENE)
    if backend == 'convex':
        return base
    path = out/'sdf_assets'
    return replace(base, tool=replace(base.tool, mjcf=path/'active.xml'),
                   target=replace(base.target, mjcf=path/'passive.xml'))


def worker(out, backend, case):
    """单次完整任务：计时不包含离线审计；保存审计与峰值穿透。"""
    from compliant_docking.research.cases import POINTS
    scene = scene_for(out, backend)
    folder = out/backend/case
    folder.mkdir(parents=True, exist_ok=False)
    limits = petal_trials.preflight(scene)
    write_json(folder/'preflight.json', limits)
    with timers() as (totals, counts):
        start = time.perf_counter()
        record = petal_trials.run_case(folder, scene, case, 'lateral_released', 'baseline',
                    error=error_tuple(POINTS[case]), telemetry='full')
        elapsed = time.perf_counter()-start
    timing = dict(wall_s=elapsed, components_s=dict(totals), calls=dict(counts),
                  other_s=elapsed-sum(totals.values()), threads=1,
                  simulated_s=record['gate']['duration_s'], audit_excluded=True)
    write_json(folder/'timing.json', timing)
    audited = audit_point(folder, case, limits, base_scene=scene)
    write_json(folder/'audit.json', audited)
    print(backend, case, record['assessment']['status'], audited['status'], timing, flush=True)


def launch(out, backend, case):
    """顺序执行单进程任务；每点最多 300 秒，超时保留日志并停止该点。"""
    logpath = out/f'{backend}_{case}.log'
    with logpath.open('w') as log:
        try:
            result = subprocess.run([sys.executable, '-m', 'experiments.models_interfaces.angle1_sdf_compare', '--phase', 'worker',
                       '--out', str(out), '--backend', backend, '--case', case],
                       cwd=REPO_ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=300,
                       env={**os.environ, 'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1', 'MUJOCO_GL': 'egl'})
            return dict(status='DONE' if result.returncode == 0 else 'ERROR', exit_code=result.returncode)
        except subprocess.TimeoutExpired:
            return dict(status='TIMEOUT', wall_budget_s=300)


def main():
    """profile 先生成名义凸块计时；compare 再完成其余五点，不自动替换模型。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=('profile', 'compare', 'worker'), required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--backend', choices=('convex', 'sdf'))
    parser.add_argument('--case', choices=CASES)
    args = parser.parse_args()
    out = args.out.resolve()
    if args.phase == 'worker':
        worker(out, args.backend, args.case)
        return
    if args.phase == 'profile':
        if out.exists() and any(out.iterdir()):
            raise ValueError('Use a new output directory')
        out.mkdir(parents=True, exist_ok=True)
        base = load_scene(SCENE)
        motion_plan(base)
        manifest = source_manifest(base)
        for p in (REPO_ROOT/'assets/iiwa14').rglob('*'):
            if p.is_file():
                manifest['sources'][str(p.relative_to(REPO_ROOT))] = sha(p)
        write_json(out/'source_manifest.json', manifest)
        for filename in manifest['sources']:
            target = out/'source_snapshot'/filename
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(REPO_ROOT/filename, target)
        write_json(out/'plan.json', dict(cases=CASES, workers=1, threads=1, task_timeout_s=300,
                    physics_control_delay_s=.001, promotion=False, automatic_refinement=False,
                    matched_trajectory_friction_inertia_gates=True))
        write_json(out/'profile_status.json', launch(out, 'convex', 'nominal'))
        return
    manifest = json.loads((out/'source_manifest.json').read_text())
    for filename, digest in manifest['sources'].items():
        assert sha(REPO_ROOT/filename) == digest
    assert json.loads((out/'profile_status.json').read_text())['status'] == 'DONE'
    prototype(out/'sdf_assets')
    convex, sdf = scene_for(out, 'convex'), scene_for(out, 'sdf')
    # 显式惯量与关节模型一致；几何数/类型以外的动力学参数不得改变。
    models = [s.build_mjmodel() for s in (convex, sdf)]
    for attr in ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'jnt_range'):
        np.testing.assert_allclose(getattr(models[0], attr), getattr(models[1], attr), rtol=0, atol=1e-12)
    write_json(out/'model_check.json', dict(status='PASS', convex_geoms=models[0].ngeom,
                sdf_geoms=models[1].ngeom,
                sdf_count=int(np.count_nonzero(models[1].geom_type == mujoco.mjtGeom.mjGEOM_SDF)),
                masses_inertias_joint_ranges_equal=True, mujoco_version=mujoco.__version__))
    del models
    statuses = {'convex/nominal': json.loads((out/'profile_status.json').read_text())}
    for backend, cases in [('convex', CASES[1:]), ('sdf', CASES)]:
        for case in cases:
            print('START', backend, case, flush=True)
            statuses[f'{backend}/{case}'] = launch(out, backend, case)
            write_json(out/'task_status.json', statuses)
    pairs = {}
    for case in CASES:
        pair = {}
        for backend in ('convex', 'sdf'):
            folder = out/backend/case
            pair[backend] = dict(task=statuses[f'{backend}/{case}'])
            for key in ('timing', 'audit'):
                path = folder/f'{key}.json'
                if path.exists():
                    pair[backend][key] = json.loads(path.read_text())
        if all('audit' in pair[b] for b in pair):
            with np.load(out/'convex'/case/f'{case}_lateral_released.npz') as a, np.load(out/'sdf'/case/f'{case}_lateral_released.npz') as b:
                for key in ('t', 'desired_position'):
                    np.testing.assert_array_equal(a[key], b[key])
            pair['identical_reference_and_timestamps'] = True
        pairs[case] = pair
    for filename, digest in manifest['sources'].items():
        assert sha(REPO_ROOT/filename) == digest
    write_json(out/'summary.json', dict(pairs=pairs, default_backend='convex', sdf_promoted=False))


if __name__ == '__main__':
    main()
