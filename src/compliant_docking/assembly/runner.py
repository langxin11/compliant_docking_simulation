"""Formal assembly entry, immutable run outputs and recorded-state replay."""
from __future__ import annotations

import hashlib
import json
import os
import platform
import shutil
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np

from .audit import audit
from .config import ROOT
from .consistency import prepare_models
from .runtime import AssemblyRuntime
from .simulation import simulate


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+"\n")


def provenance(r):
    snapshot = r.output / "source_snapshot"
    files = [* (ROOT / "src/compliant_docking").rglob("*.py"), r.scene.source,
             r.baseline, r.baseline.parent/"gripper.xml", ROOT/"assets/iiwa14/iiwa14_arm.xml",
             ROOT/"assets/iiwa14/urdf/iiwa14.urdf", ROOT/"pyproject.toml", ROOT/"uv.lock"]
    manifest = {}
    for p in files:
        dest = snapshot / p.relative_to(ROOT)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, dest)
        manifest[str(dest)] = hashlib.sha256(dest.read_bytes()).hexdigest()
    # Keep upstream license/import/CAD/meshes in place and fingerprint every input.
    for p in r.resource.path.rglob("*"):
        if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc":
            manifest[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (ROOT/"assets/iiwa14/assets").iterdir():
        if p.is_file():
            manifest[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
    for name in ["model.xml", "arm_tool_model.xml", "carried_model.xml", "phases.json", "accepted_anchor.json", "rollout.npz", "contact_trace.npz", "storage_trace.npz", "model_consistency.json", "planning.json", "runtime.json"]:
        p = r.output/name
        if p.exists():
            manifest[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
    write_json(r.output/"source_manifest.json", manifest)


def render_saved(r, video):
    from .rendering import camera, preview_card, render, render_connection
    model = mujoco.MjModel.from_xml_path(str(r.output/"model.xml"))
    anchor = json.loads((r.output/"accepted_anchor.json").read_text())
    model.site("assembly_anchor").pos[:] = anchor["pos"]
    model.site("assembly_anchor").quat[:] = anchor["quat"]
    with np.load(r.output/"rollout.npz") as saved:
        records = {k: saved[k] for k in saved.files}
    # Read recorded phases rather than regenerate them from current defaults.
    from .sequence import Phase
    phases = [Phase(p["key"], p["title"], p["seconds"], np.array(p["end"]), p["event"])
              for p in json.loads((r.output/"phases.json").read_text())]
    render(model, records, phases, video, output=r.output,
           work_camera=camera([0, .30, .28], 2.8, 85, -23),
           wide_camera=camera([0, .25, -.02], 6.5, 75, -24),
           detail_azimuth=90, detail_elevation=-12, base_prelocked=True,
           footer="HexFrame · 固定基座零重力 · 真实导向/止挡接触 · 理想锁定后卸力释放")
    preview_card(r.output)
    render_connection(model, records, output=r.output, sites=["base_dock_mating"],
                      title="基座标准接口 ↔ 模块 2 侧面接口 4",
                      footer="初始已锁定 · PetalDock100 V2 · 基座捕获与锁紧机构未模拟",
                      filename="base_connection.png")
    for t, filename, title in [(0., "storage_connection.png", "标准存储接口锁定模块 1 · 备用标准接口空闲"),
                                (15., "storage_released.png", "机械臂已提起模块 1 · 存储接口解锁并脱离")]:
        render_connection(model, records, output=r.output, sites=["storage_dock_mating", "spare_dock_mating"],
                          title=title, footer="接口 4 朝下 · 理想锁定与受保护的解锁交接",
                          filename=filename, distance=.78, time=t)


def run(scene, *, output=None, record=False, preview_only=False, replay=False):
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("MPLBACKEND", "Agg")
    if replay and output is None:
        raise ValueError("Replay requires --out pointing to a completed run")
    if output is None:
        output = ROOT / "runs" / ("hexframe_assembly_"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ"))
    output = Path(output).resolve()
    r = AssemblyRuntime(scene, output)
    if replay:
        audit(output)
        render_saved(r, record)
        print(f"[docking] replay={output}")
        return 0
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"Refusing to overwrite a nonempty run directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    model = r.geometry.build_model()
    consistency = prepare_models(r)
    spline, phases, planning = r.plan(model)
    if planning["spline_joint_limit_margin_rad"] <= 0:
        raise RuntimeError("Planned spline exceeds joint limits")
    planning["rejected_unsafe_events"] = r.verify_interlocks(model, spline(0))
    write_json(output/"planning.json", planning)
    write_json(output/"phases.json", [dict(key=p.key, title=p.title, seconds=p.seconds, end=p.end.tolist(), event=p.event) for p in phases])
    write_json(output/"runtime.json", dict(python=platform.python_version(), platform=platform.platform(),
                mujoco=mujoco.__version__, numpy=np.__version__, physics_timestep_s=r.dt,
                state_record_hz=100, contact_record_hz=1000, video_fps=24,
                controller="MuJoCo bias-compensated joint servo with contact admittance",
                pinocchio_role="same-source IK and nominal rigid-body consistency checks"))
    if preview_only:
        write_json(output/"validation.json", dict(status="INCOMPLETE", scope="geometry, dynamics consistency and complete-path IK only", planning=planning, consistency=consistency))
        from .rendering import render_connection
        q = mujoco.MjData(model).qpos.copy()
        q[:7] = spline(0)
        render_connection(model, dict(t=np.array([0.]), qpos=np.array([q]), qvel=np.zeros((1, model.nv)), locks=np.array([[True, False, False]])),
                          output=output, sites=["gripper_tip", "base_dock_mating"], title="HexFrame 正式场景 · 初始布局",
                          footer="预览检查；未执行完整接触与交接验收", filename="preview.png", distance=2.8)
        provenance(r)
        print(f"[docking] INCOMPLETE preview={output}")
        return 0
    try:
        _, report = simulate(r, model, spline, phases, planning)
        report["planning"] = planning
        write_json(output/"validation.json", report)
    finally:
        provenance(r)
    if report["status"] != "PASS":
        print(f"[docking] FAIL output={output}")
        return 2
    audit_result = audit(output)
    if record:
        render_saved(r, True)
    print(f"[docking] {audit_result['status']} full assembly and independent audit output={output}")
    return 0
