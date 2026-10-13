"""HexFrame 有限接收位姿偏差验证，保持名义规划和原控制器。

在规划完成后，对固定接收站整体施加世界系 XY 平移和绕世界 Z 轴旋转。
真实几何、接收接口与验收锚点共同变化，控制目标仍为名义估计。
仅核对名义、横向 2 mm、绕轴 5°、组合四点，不扫描捕获范围或优化增益。
输出完整运行、模型、偏差说明、源码快照；失败工况保留原门禁结果。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import time
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MPLBACKEND", "Agg")

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from compliant_docking.assembly.audit import audit
from compliant_docking.assembly.consistency import prepare_models
from compliant_docking.assembly.geometry import quaternion, vector
from compliant_docking.assembly.runner import provenance, run, write_json
from compliant_docking.assembly.runtime import AssemblyRuntime
from compliant_docking.assembly.simulation import simulate
from compliant_docking.scene import REPO_ROOT, load_scene

# 真值相对名义估计的 (世界 X m，世界 Y m，绕世界 Z deg)。
CASES = {"nominal": (0., 0., 0.), "xy": (.002, 0., 0.),
         "yaw": (0., 0., 5.), "combined": (.002, 0., 5.)}
SCENE = REPO_ROOT / "scenes/hexframe_assembly.yaml"


def inject_receiver_pose(runtime, model, delta):
    """移动真实接收站并重新编译模型；名义控制参考保持不变。

    Args:
        runtime: 本次运行对象；只更新 installed（最终评分用的真实中心，m）。
        model: 由名义场景构建的 MuJoCo 模型，不原地修改。
        delta: (dx_m, dy_m, yaw_deg)，接收站真值相对估计的偏差。
            绕世界 Z 轴旋转的轴线经过名义 module2 中心。

    Returns:
        (新模型, 偏差核对字典)。同时保存 nominal_model.xml、实际 model.xml、
        perturbation.json，并更新 geometry_check.json 中实际底座与存储连接核对。
        存储模块、抓取位姿、轨迹与 install_tip 不变化。
    """
    dx, dy, yaw = map(float, delta)
    if not np.isfinite([dx, dy, yaw]).all():
        raise ValueError("位姿偏差必须有限")
    before = mujoco.MjData(model)
    mujoco.mj_forward(model, before)
    rotation = Rotation.from_euler("z", yaw, degrees=True).as_matrix()
    shift = np.array([dx, dy, 0.])
    center = before.body("module2").xpos.copy()
    station = before.body("seed_station")
    parent = before.body(int(model.body_parentid[station.id]))
    parent_rotation = parent.xmat.reshape(3, 3)
    station_position = center + rotation @ (station.xpos-center) + shift
    station_rotation = rotation @ station.xmat.reshape(3, 3)
    nominal_xml = runtime.output / "model.xml"
    shutil.copy2(nominal_xml, runtime.output / "nominal_model.xml")
    root = ET.parse(nominal_xml)
    body = root.find(".//body[@name='seed_station']")
    body.set("pos", vector(parent_rotation.T @ (station_position-parent.xpos)))
    body.set("quat", vector(quaternion(parent_rotation.T @ station_rotation)))
    root.write(nominal_xml, encoding="unicode")
    actual_model = mujoco.MjModel.from_xml_path(str(nominal_xml))
    after = mujoco.MjData(actual_model)
    mujoco.mj_forward(actual_model, after)
    nominal_tip = runtime.install_tip.copy()
    nominal_installed = runtime.installed.copy()
    # installed 仅被末段位置评分读取；规划/导纳读取 install_tip，保持名义值。
    runtime.installed = after.site("assembly_anchor").xpos.copy()
    for name in ("module2", "module2_port_1", "base_dock_port"):
        np.testing.assert_allclose(after.body(name).xpos,
                                   center+rotation@(before.body(name).xpos-center)+shift, atol=1e-12)
        np.testing.assert_allclose(after.body(name).xmat.reshape(3, 3),
                                   rotation@before.body(name).xmat.reshape(3, 3), atol=1e-12)
    for name in ("module1", "storage_dock_port", "gripper"):
        np.testing.assert_array_equal(after.body(name).xpos, before.body(name).xpos)
        np.testing.assert_array_equal(after.body(name).xmat, before.body(name).xmat)
    np.testing.assert_array_equal(actual_model.body_mass, model.body_mass)
    np.testing.assert_array_equal(actual_model.body_inertia, model.body_inertia)
    np.testing.assert_array_equal(actual_model.geom_friction, model.geom_friction)
    np.testing.assert_array_equal(runtime.install_tip, nominal_tip)
    checks_path = runtime.output / "geometry_check.json"
    checks = json.loads(checks_path.read_text())
    checks["base_connection"] = runtime.geometry.check_base_dock(actual_model, after)
    checks["storage_connection"] = runtime.geometry.check_storage_docks(actual_model, after)
    checks["receiver_perturbation_verified"] = True
    write_json(checks_path, checks)
    details = dict(truth_minus_estimate=dict(world_xy_m=[dx, dy], world_yaw_deg=yaw),
                   pivot_world_m=center.tolist(), nominal_install_tip_m=nominal_tip.tolist(),
                   nominal_module_center_m=nominal_installed.tolist(),
                   actual_module_center_m=runtime.installed.tolist(),
                   actual_receiver_center_m=after.body("module2").xpos.tolist(),
                   nominal_reference_unchanged=True, storage_and_grasp_unchanged=True,
                   masses_inertias_friction_unchanged=True,
                   receiver_geometry_and_scoring_anchor_transformed_together=True)
    write_json(runtime.output / "perturbation.json", details)
    return actual_model, details


def run_case(case, output):
    """执行一个预声明工况；返回原验收退出码，失败也保存物理记录和来源。"""
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"拒绝覆盖非空目录：{output}")
    if case == "nominal":
        return run(load_scene(SCENE), output=output)
    output.mkdir(parents=True, exist_ok=True)
    runtime = AssemblyRuntime(load_scene(SCENE), output)
    nominal_model = runtime.geometry.build_model()
    prepare_models(runtime)
    spline, phases, planning = runtime.plan(nominal_model)
    if planning["spline_joint_limit_margin_rad"] <= 0:
        raise RuntimeError("名义规划超出关节范围")
    model, _ = inject_receiver_pose(runtime, nominal_model, CASES[case])
    planning["rejected_unsafe_events"] = runtime.verify_interlocks(model, spline(0))
    write_json(output / "planning.json", planning)
    write_json(output / "phases.json", [dict(key=p.key, title=p.title, seconds=p.seconds,
                                              end=p.end.tolist(), event=p.event) for p in phases])
    write_json(output / "runtime.json", dict(
        python=platform.python_version(), platform=platform.platform(),
        mujoco=mujoco.__version__, numpy=np.__version__, physics_timestep_s=runtime.dt,
        state_record_hz=100, contact_record_hz=1000, video_fps=24,
        seating_gate="raw_strict", controller="MuJoCo bias-compensated joint servo with contact admittance",
        validation_context=f"接收站实际位姿偏差 {case}，名义规划与控制目标不变"))
    try:
        _, report = simulate(runtime, model, spline, phases, planning)
        report["planning"] = planning
        write_json(output / "validation.json", report)
    finally:
        provenance(runtime)
        entry = Path(__file__).resolve()
        snapshot = output / "source_snapshot" / entry.relative_to(REPO_ROOT)
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(entry, snapshot)
        manifest_path = output / "source_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        for p in (snapshot, output / "perturbation.json", output / "nominal_model.xml",
                  output / "geometry_check.json"):
            manifest[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
        write_json(manifest_path, manifest)
    if report["status"] == "PASS":
        audit(output)
        return 0
    return 2


def main():
    """选择四个固定代表点之一；不接受隐式扫描或覆盖原始输出。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    start = time.monotonic()
    result = run_case(args.case, args.out)
    print(json.dumps(dict(case=args.case, exit_code=result,
                          wall_seconds=time.monotonic()-start)), flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
