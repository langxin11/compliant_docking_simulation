"""用户选定接口的九点任务验证：1 ms 时序、自由空间提速及原落座门槛。

输出仅写入新目录；从逐步遥测检查运动限制、反馈时序和穿透量。
不复用旧记录，不增加精细步长矩阵，不代表真实锁紧或连续捕获范围。
"""
from __future__ import annotations

import argparse
import json
import shutil
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np

from compliant_docking.config import DockingConfig
from compliant_docking.contact_diagnostics import evaluate_contact_load
from compliant_docking.docking_task import build_docking_trajectory, target_rotation
from compliant_docking.petal_geometry import evaluate_petal_seating
from compliant_docking.research.cases import POINTS
from compliant_docking.research.petal_trials import preflight, run_case, variant
from compliant_docking.research.protocols import error_tuple, source_manifest, write_json
from compliant_docking.research.rollout import _json_default
from compliant_docking.scene import REPO_ROOT, load_scene

SCENE = 'scenes/iiwa14_petal_angle1_blend030.yaml'


def motion_plan(scene):
    """采样参考轨迹并核对各段平移/转动速度与加速度，单位为 SI。"""
    trajectory = build_docking_trajectory(scene.task, scene.docking, scene.trajectory)
    segments = []
    for index, phase in enumerate(trajectory.names[1:]):
        start, end = trajectory.times[index:index+2]
        samples = [trajectory._sample(t) for t in np.linspace(start, end, 1001)]
        peaks = [max(float(np.linalg.norm(s[i])) for s in samples) for i in (1, 2, 3, 4)]
        limits = scene.trajectory
        if phase in ('approach', 'lift'):
            bounds = [limits.v_max_approach, limits.a_max_approach,
                      limits.omega_max_approach, limits.alpha_max_approach]
        else:
            bounds = [limits.v_max_docking, limits.a_max_docking,
                      limits.omega_max_docking, limits.alpha_max_docking]
            if phase == 'insert':
                bounds[:2] = [scene.docking.insertion_speed, scene.docking.insertion_acceleration]
        assert np.all(np.array(peaks) <= np.array(bounds)+1e-9)
        segments.append(dict(phase=phase, duration_s=float(end-start),
                             peak_v_a_omega_alpha=peaks, limits_v_a_omega_alpha=bounds))
    return dict(segments=segments, free_space_s=float(trajectory.times[-2]),
                total_with_hold_s=trajectory.total_duration+scene.docking.hold_s)


def run_point(args):
    """按同一场景执行一个误差点，保存完整逐步遥测供独立复算。"""
    out, case = args
    return run_case(out, load_scene(SCENE), case, 'lateral_released', 'baseline',
                    error=error_tuple(POINTS[case]), telemetry='full')


def audit_point(out, case, limits, *, base_scene=None):
    """重算几何/载荷并检查真实更新间隔、有限性、关节限制与最大穿透。"""
    name = f'{case}_lateral_released'
    record = json.loads((out/f'{name}.json').read_text())
    scene = variant(base_scene if base_scene is not None else load_scene(SCENE),
                    case, 'lateral_released', error_tuple(POINTS[case]))
    assert json.loads(json.dumps(asdict(scene), default=_json_default)) == record['scene']
    with np.load(out/f'{name}.npz') as data:
        assert all(np.isfinite(data[k]).all() for k in data.files
                   if np.issubdtype(data[k].dtype, np.number))
        times = data['t']
        assert np.all(data['control_control_update'])
        np.testing.assert_allclose(np.diff(times), .001, rtol=0., atol=1e-10)
        np.testing.assert_allclose(data['control_feedback_age_s'][1:], .001, rtol=0., atol=1e-10)
        np.testing.assert_allclose(np.diff(data['control_control_t']), .001, rtol=0., atol=1e-10)
        assert record['feedback_convention']['delay_s'] == .001
        assert record['feedback_audit']['status'] == 'PASS'
        complete = times[-1]+.001 >= record['waypoint_times'][-1]+scene.docking.hold_s
        assert complete
        tail = np.flatnonzero(times > times[-1]-1.)
        samples = [dict(t=times[i], position=data['diagnostic_position'][i],
                        rotation=data['diagnostic_rotation'][i],
                        stop_contact_count=data['diagnostic_stop_contact_count'][i]) for i in tail]
        assert evaluate_petal_seating(samples, scene, complete=True) == record['geometry_evaluation']
        assert evaluate_contact_load(data['diagnostic_interface_world'], scene.docking,
                                     target_rotation(scene.target), complete=True) == record['contact_load_gate']
        q, qd = data['q'], data['qd']
        joint_violations = int(np.count_nonzero(np.any(
            (q < np.array(limits['lower_position_limits'])-1e-6) |
            (q > np.array(limits['upper_position_limits'])+1e-6), axis=1)))
        # URDF 的速度值是仓库模型声明，不能当作硬件认证限值。
        urdf = ET.parse(REPO_ROOT/'assets/iiwa14/iiwa14_dock.urdf').getroot()
        velocity_limits = np.array([float(j.find('limit').get('velocity'))
                                    for j in urdf.findall('joint') if j.get('type') == 'revolute'])
        speed_peaks = np.max(np.abs(qd), axis=0)
        torque_peak = float(np.max(np.abs(data['torque'])))
        penetration_mm = float(max(0., -np.min(data['diagnostic_min_distance_m']))*1000)
        issues = []
        if joint_violations:
            issues.append('joint position limit')
        if np.any(speed_peaks > velocity_limits+1e-6):
            issues.append('model joint velocity limit')
        if torque_peak > DockingConfig().max_torque+1e-8:
            issues.append('command torque limit')
        if record['simulation_warnings']:
            issues.append('simulation warning')
        if np.any(data['other_contacts']):
            issues.append('unexpected contact')
        # 新增异常提示不替换旧落座门槛；超过 0.3 mm 时必须人工复核。
        if penetration_mm > .3:
            issues.append('penetration over 0.3 mm diagnostic threshold')
        free = times < record['waypoint_times'][-2]
        return dict(status='PASS' if not issues else 'REVIEW', issues=issues,
                    assessment=record['assessment'], geometry=record['geometry_evaluation'],
                    loads=record['contact_load_gate'], max_penetration_mm=penetration_mm,
                    joint_position_violation_samples=joint_violations,
                    peak_joint_speed_rad_s=speed_peaks.tolist(),
                    model_joint_speed_limits_rad_s=velocity_limits.tolist(),
                    peak_joint_acceleration_rad_s2=np.max(np.abs(np.diff(qd, axis=0)/.001), axis=0).tolist(),
                    joint_acceleration_limit_declared=False,
                    peak_torque_Nm=torque_peak,
                    torque_saturation_samples=int(np.count_nonzero(data['torque_saturated'])),
                    free_space_torque_saturation_samples=int(np.count_nonzero(data['torque_saturated'][free])),
                    feedback_period_and_delay_s=.001, one_update_per_physics_step=True,
                    geometry_and_loads_recomputed=True)


def main():
    """固定九点计划，先检查轨迹限值，再运行与复算，不自动调整参数。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--jobs', type=int, choices=(1, 2, 3), default=3)
    args = parser.parse_args()
    if args.out.exists() and any(args.out.iterdir()):
        raise ValueError('Use a new empty output directory')
    args.out.mkdir(parents=True, exist_ok=True)
    base = load_scene(SCENE)
    old = load_scene('scenes/iiwa14_petal_angle1_blend030_025ms.yaml')
    assert asdict(base.docking) == asdict(old.docking)
    assert base.physics.timestep == base.se3_impedance.control_period == .001
    plans = {case: motion_plan(variant(base, case, 'lateral_released', error_tuple(error)))
             for case, error in POINTS.items()}
    old_plans = {case: motion_plan(variant(old, case, 'lateral_released', error_tuple(error)))
                 for case, error in POINTS.items()}
    write_json(args.out/'study_plan.json', dict(cases=POINTS, new=plans, old=old_plans,
               physics_control_delay_s=.001, docking_gates_unchanged=True,
               penetration_review_threshold_mm=.3, automatic_refinement=False))
    manifest = source_manifest(base)
    import hashlib
    for p in (REPO_ROOT/'assets/iiwa14').rglob('*'):
        if p.is_file():
            manifest['sources'][str(p.relative_to(REPO_ROOT))] = hashlib.sha256(p.read_bytes()).hexdigest()
    for filename, digest in manifest['assets']['imported_files'].items():
        assert hashlib.sha256((base.tool.mjcf.parent/filename).read_bytes()).hexdigest() == digest
    write_json(args.out/'source_manifest.json', manifest)
    for filename in manifest['sources']:
        target = args.out/'source_snapshot'/filename
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPO_ROOT/filename, target)
    limits = preflight(base)
    write_json(args.out/'preflight.json', limits)
    print('Motion limits and model preflight PASS', flush=True)
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        list(pool.map(run_point, [(args.out, case) for case in POINTS]))
    audits = {case: audit_point(args.out, case, limits) for case in POINTS}
    for filename, digest in manifest['sources'].items():
        assert hashlib.sha256((REPO_ROOT/filename).read_bytes()).hexdigest() == digest
    write_json(args.out/'summary.json', dict(cases=audits,
        passes=sum(a['assessment']['status'] == 'CANDIDATE_PASS' for a in audits.values()),
        audit_passes=sum(a['status'] == 'PASS' for a in audits.values()),
        continuous_capture_verified=False, locking_verified=False))
    print(json.dumps({k: (v['assessment']['status'], v['status']) for k, v in audits.items()}), flush=True)


if __name__ == '__main__':
    main()
