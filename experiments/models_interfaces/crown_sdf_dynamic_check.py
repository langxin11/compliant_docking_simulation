"""原冠形 SDF 三点动态核对：接受导向首触，分别报告平稳、载荷和落座。

沿用原场景固定低绕轴刚度、轨迹和门槛，物理/控制/反馈延迟均为 1 ms。
末段平稳仅指有限时域观测；不构成全局稳定性证明或真实锁紧验证。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import time
from contextlib import redirect_stdout
from dataclasses import replace
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'egl')
os.environ.setdefault('MPLBACKEND', 'Agg')

import numpy as np

from compliant_docking.contact_diagnostics import evaluate_contact_load
from compliant_docking.docking_task import target_rotation
from compliant_docking.interface_geometry import InterfaceGeometry, asset_fingerprints
from compliant_docking.orchestration.run_docking import main as simulate
from compliant_docking.research.petal_trials import feedback_audit
from compliant_docking.research.protocols import write_json
from compliant_docking.research.rollout import save_rollout
from compliant_docking.scene import REPO_ROOT, load_scene
from experiments.insertion_suite import CASES, variant


def dynamics_metrics(log, scene):
    """用原速度门槛判断末 1 s 平稳，记录整个保持段趋势及异常，不处罚首触时刻。"""
    samples = log.docking_samples
    t = np.array([s['t'] for s in samples])
    position = np.asarray(log.pos_actual)
    force = np.array([np.linalg.norm(s['interface_world'][:3]) for s in log.contact_diagnostics])
    moment = np.array([abs(target_rotation(scene.target)[:, 2]@s['interface_world'][3:])
                       for s in log.contact_diagnostics])
    speed = np.array([s['speed_m_s'] for s in samples])
    angular = np.array([s['angular_speed_deg_s'] for s in samples])
    contacts = np.array([s['interface_contacts'] > 0 for s in samples])
    windows = []
    for index in range(8):
        mask = (t > t[-1]-8+index) & (t <= t[-1]-7+index+1e-10)
        windows.append(dict(window=index+1, max_speed_mm_s=float(speed[mask].max()*1000),
                            max_angular_speed_deg_s=float(angular[mask].max()),
                            position_peak_to_peak_mm=(np.ptp(position[mask], axis=0)*1000).tolist(),
                            force_min_N=float(force[mask].min()), force_max_N=float(force[mask].max()),
                            force_std_N=float(force[mask].std()),
                            contact_fraction=float(contacts[mask].mean())))
    tail = t > t[-1]-1
    settled = bool(speed[tail].max() <= scene.docking.max_linear_speed
                   and angular[tail].max() <= scene.docking.max_angular_speed_deg
                   and contacts[tail].mean() >= .95)
    finite = all(np.isfinite(a).all() for a in (position, force, moment, speed, angular,
                                               log.joint_angles, log.joint_velocities))
    force_over = force > scene.docking.max_force
    moment_over = moment > scene.docking.max_axial_moment
    onset = np.flatnonzero(contacts)
    penetration = np.maximum(0., -np.array([s['min_distance_m'] for s in log.contact_diagnostics]))
    post = t >= (t[onset[0]] if len(onset) else t[-1])
    return dict(finite=finite, simulation_warnings=log.simulation_warnings,
                tail_settled_under_original_speed_limits=settled,
                speed_limit_mm_s=scene.docking.max_linear_speed*1000,
                angular_speed_limit_deg_s=scene.docking.max_angular_speed_deg,
                tail=windows[-1], hold_windows=windows,
                first_contact_s=None if not len(onset) else float(t[onset[0]]),
                first_contact_phase=None if not len(onset) else samples[onset[0]]['phase'],
                post_contact_max_speed_mm_s=float(speed[post].max()*1000),
                post_contact_max_angular_speed_deg_s=float(angular[post].max()),
                post_contact_contact_fraction=float(contacts[post].mean()),
                force_over_limit_duration_s=float(force_over.sum()*.001),
                moment_over_limit_duration_s=float(moment_over.sum()*.001),
                max_penetration_mm=float(penetration.max()*1000),
                tail_max_penetration_mm=float(penetration[tail].max()*1000),
                joint_position_violation=bool(getattr(log, 'docking_joint_limit_violation', False)),
                torque_saturation_samples=int(np.count_nonzero(log.torque_saturated)),
                torque_saturation_ratio=float(np.mean(log.torque_saturated)),
                early_contact_is_not_instability_criterion=True)


def main():
    """固定三工况顺序执行并保存完整轨迹；不做增益、步长或几何扫描。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists() and any(args.out.iterdir()):
        raise ValueError('Use a new output directory')
    args.out.mkdir(parents=True, exist_ok=True)
    base = load_scene('scenes/iiwa14_compliant_insertion.yaml')
    base = replace(base, se3_impedance=replace(base.se3_impedance, control_period=.001))
    assert base.physics.timestep == .001
    geometry = InterfaceGeometry(base)
    archive = REPO_ROOT/'runs/convex_geometry_20261002/geometry_validation.json'
    reference = json.loads(archive.read_text())['original_calibration']
    assert reference['assets'] == asset_fingerprints(base)
    geometry.calibration = reference
    write_json(args.out/'geometry_reference.json', reference)
    sources = [*sorted((REPO_ROOT/'src').rglob('*.py')), Path(__file__).resolve(),
               REPO_ROOT/'experiments/insertion_suite.py', base.path]
    source_hashes = {str(p.relative_to(REPO_ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    write_json(args.out/'source_manifest.json', dict(sources=source_hashes, assets=asset_fingerprints(base),
               geometry_reference_sha256=hashlib.sha256(archive.read_bytes()).hexdigest()))
    for filename in source_hashes:
        target = args.out/'source_snapshot'/filename
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPO_ROOT/filename, target)
    write_json(args.out/'plan.json', dict(cases=CASES, profile='compliant',
               physics_control_delay_s=.001, hold_s=base.docking.hold_s,
               early_contact_is_not_instability_criterion=True,
               dynamic_tail_limits=dict(linear_m_s=base.docking.max_linear_speed,
                  angular_deg_s=base.docking.max_angular_speed_deg, contact_fraction=.95),
               original_benchmark_and_geometry_reported_separately=True))
    results = {}
    for case in CASES:
        scene = variant(base, case, 'compliant')
        print('START', case, flush=True)
        start = time.perf_counter()
        with (args.out/f'{case}.log').open('w') as stream, redirect_stdout(stream):
            log = simulate(scene=scene, dt=.001, render=False, record=False, plot=False, diagnostics='summary')
        complete = log.docking_samples[-1]['t']+.001 >= log.docking_trajectory.total_duration+scene.docking.hold_s
        assert complete
        log.geometry_evaluation = geometry.evaluate(log.contact_diagnostics, scene.target.pos,
                                                    target_rotation(scene.target))
        data = save_rollout(args.out, case, scene, log, telemetry='full')
        loads = evaluate_contact_load(data['diagnostic_interface_world'], scene.docking,
                                      target_rotation(scene.target), complete=True)
        feedback = feedback_audit(log)
        assert feedback['status'] == 'PASS'
        assert log.feedback_convention['delay_s'] == .001
        assert np.all(data['control_control_update'])
        np.testing.assert_allclose(data['control_feedback_age_s'][1:], .001, rtol=0, atol=1e-10)
        metrics = dynamics_metrics(log, scene)
        result = dict(dynamics=metrics, loads=loads, geometry=log.geometry_evaluation,
                      original_gate=log.docking_gate, feedback=feedback,
                      wall_s=time.perf_counter()-start,
                      mathematical_stability_proved=False, locking_verified=False)
        results[case] = result
        write_json(args.out/f'{case}_evaluation.json', result)
        print(case, 'tail settled', metrics['tail_settled_under_original_speed_limits'],
              'loads', loads['status'], 'geometry', log.geometry_evaluation['status'], flush=True)
        del log, data
    write_json(args.out/'summary.json', results)


if __name__ == '__main__':
    main()
