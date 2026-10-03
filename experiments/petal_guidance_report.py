"""Audits and figures from frozen guide trials; never changes robot rollouts."""
from __future__ import annotations

import hashlib
import json
import os
from collections import Counter
from dataclasses import asdict, replace
from importlib.metadata import version
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
import numpy as np
import petal_capture_grid as grid
import petal_guidance_study as study
import petal_insertion_suite as suite
from petal_guidance_geometry import DIRECTORIES
from petal_lateral_study import POINTS

from compliant_docking.contact_diagnostics import evaluate_contact_load
from compliant_docking.docking_task import target_rotation
from compliant_docking.petal_geometry import evaluate_petal_seating
from compliant_docking.plotting import apply_style
from compliant_docking.scene import REPO_ROOT, load_scene

LABELS = {"original": "原轮廓", "narrow": "窄平顶 + 角向斜坡", "radial": "角向 + 径向导面"}
COLORS = {"original": "#666666", "narrow": "#009E73", "radial": "#D55E00"}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def arrays(out, key, fields=None):
    with np.load(out/f"{study.name(*key)}.npz") as archive:
        return {k:archive[k] for k in (archive.files if fields is None else fields)}


def effective_scene(key):
    g,case,setting = key
    scene = suite.variant(study.geometry_scene(load_scene("scenes/iiwa14_petal_insertion.yaml"),g),
        f"g_{g}_{case}","lateral_released",error=grid.error_tuple(POINTS[case]))
    divisor = {"dt_half":2,"dt_quarter":4}[setting]
    return replace(scene,physics=replace(scene.physics,timestep=scene.physics.timestep/divisor))


def audit(out, *, complete_matrix=True):
    """Recompute gates and conventions from saved every-physics-step channels."""
    manifest = json.loads((out/"source_manifest.json").read_text())
    for filename,digest in manifest["sources"].items():
        assert sha(REPO_ROOT/filename) == sha(out/"source_snapshot"/filename) == digest,filename
    robot_assets = out/"robot_assets.json"
    if robot_assets.exists():
        for filename,digest in json.loads(robot_assets.read_text())["files_sha256"].items():
            assert sha(REPO_ROOT/filename) == digest,filename
    asset_count = 0
    for g in study.GEOMETRIES:
        saved = manifest["geometry_assets"][g]
        assert saved == json.loads((DIRECTORIES[g]/"manifest.json").read_text())
        for filename,digest in saved["imported_files"].items():
            assert sha(DIRECTORIES[g]/filename) == digest,(g,filename)
            asset_count += 1
        native = json.loads((out/f"{g}_geometry_validation.json").read_text())
        assert native["assets_manifest_sha256"] == sha(DIRECTORIES[g]/"manifest.json")
        assert native["validator_sha256"] == manifest["sources"]["experiments/petal_guidance_geometry.py"]
        assert native["status"] == "PASS"
    records = study.read_records(out)
    primary = {(g,c,study.PRIMARY) for g in study.GEOMETRIES for c in POINTS}
    if complete_matrix:
        assert primary <= records.keys()
        best,checks = study.select_checks(records)
        plan = json.loads((out/"numerical_plan.json").read_text())
        assert plan == json.loads(json.dumps(dict(selected_geometry=best,jobs=checks)))
        assert records.keys() == primary | set(checks)
        assert len(records) == 31
    maxima = dict(pose_error_m=0.,transport_error=0.,force_balance_N=0.,moment_balance_Nm=0.)
    issues,history = {},{}
    for key,r in sorted(records.items()):
        g,case,setting = key
        name,scene = study.name(*key),effective_scene(key)
        assert json.loads(json.dumps(asdict(scene),default=grid._json_default)) == r["scene"],name
        a = arrays(out,key)
        assert all(np.isfinite(v).all() for v in a.values() if np.issubdtype(v.dtype,np.number)),name
        np.testing.assert_array_equal(a["t"],a["diagnostic_t"])
        np.testing.assert_array_equal(a["q"],a["diagnostic_q"])
        error = float(np.max(abs(a["position"]-a["diagnostic_position"])))
        assert error < 1e-10
        maxima["pose_error_m"] = max(maxima["pose_error_m"],error)
        np.testing.assert_allclose(a["diagnostic_state_t"]-a["t"],scene.physics.timestep,rtol=0.,atol=1e-10)
        ticks = a["control_control_update"].astype(bool)
        np.testing.assert_allclose(a["control_feedback_age_s"][ticks][1:],.0005,rtol=0.,atol=1e-10)
        np.testing.assert_allclose(np.diff(a["control_control_t"][ticks]),.0005,rtol=0.,atol=1e-10)
        R,world = a["control_control_rotation"][ticks],a["control_feedback_world_at_origin"][ticks]
        shift = a["control_feedback_origin"][ticks]-a["control_control_position"][ticks]
        transformed = np.column_stack((np.einsum("nji,nj->ni",R,world[:,:3]),
            np.einsum("nji,nj->ni",R,world[:,3:]+np.cross(shift,world[:,:3]))))
        error = float(np.max(abs(transformed-a["control_feedback_body"][ticks])))
        assert error < 1e-10
        maxima["transport_error"] = max(maxima["transport_error"],error)
        for field,sl in (("force_balance_N",slice(0,3)),("moment_balance_Nm",slice(3,6))):
            error = float(np.linalg.norm(a["diagnostic_balance_residual_world"][:,sl],axis=1).max())
            assert error < 1e-6
            maxima[field] = max(maxima[field],error)
        complete = a["t"][-1]+scene.physics.timestep >= r["waypoint_times"][-1]+scene.docking.hold_s
        assert complete,name
        tail = np.flatnonzero(a["t"] > a["t"][-1]-1.)
        samples = [dict(t=a["t"][i],position=a["diagnostic_position"][i],
            rotation=a["diagnostic_rotation"][i],stop_contact_count=a["diagnostic_stop_contact_count"][i]) for i in tail]
        assert evaluate_petal_seating(samples,scene,complete=True) == r["geometry_evaluation"],name
        assert evaluate_contact_load(a["diagnostic_interface_world"],scene.docking,
            target_rotation(scene.target),complete=True) == r["contact_load_gate"],name
        assert r["feedback_audit"]["status"] == "PASS",name
        gains = a["control_lateral_stiffness"]
        assert np.all(np.diff(gains,axis=0) <= 1e-12)
        trigger = r["feedback_audit"]["yaw_trigger_s"]
        assert trigger is not None
        np.testing.assert_allclose(gains[a["t"] <= trigger],80.,rtol=0.,atol=1e-12)
        np.testing.assert_allclose(gains[-1],0.,rtol=0.,atol=1e-12)
        assert r["feedback_audit"]["final_yaw_stiffness_Nm_rad"] == 0.
        limits = json.loads((out/f"{g}_preflight.json").read_text())
        local = dict(warnings=r["simulation_warnings"],other_contacts=int(np.count_nonzero(a["other_contacts"])),
            saturation=int(np.count_nonzero(a["torque_saturated"])),joint_limit_samples=int(np.count_nonzero(np.any(
                (a["q"] < np.array(limits["lower_position_limits"])-1e-6) |
                (a["q"] > np.array(limits["upper_position_limits"])+1e-6),axis=1))))
        if any(local.values()):
            issues[name] = local
        if g == "original" and setting == "dt_half" and case in ("nx6","combo_ny6_p15","yaw_n15"):
            previous = REPO_ROOT/"runs/petal_lateral_control_20261003"/f"{case}_lateral_released_dt_half.npz"
            with np.load(previous) as archive:
                np.testing.assert_array_equal(a["t"],archive["t"])
                error = float(np.max(abs(a["q"]-archive["q"])))
            assert error < 1e-10,name
            history[name] = dict(source=str(previous),original_sha256=sha(previous),
                maximum_joint_trace_difference=error,fresh_rollout=True)
        print("AUDIT",name,r["assessment"]["status"],flush=True)
        del a
    study.reference_audit(out)
    references = json.loads((out/"reference_checks.json").read_text())
    if complete_matrix:
        assert len(references) == 9
        saved = json.loads((out/"summary.json").read_text())
        assert study.summarize(out) == saved
    result = dict(status="PASS",complete_matrix=complete_matrix,records=len(records),
        outcomes=dict(Counter(r["assessment"]["status"] for r in records.values())),matched_references=len(references),
        all_numeric_channels_finite=True,verified_asset_files=asset_count,sources_and_assets_verified=True,
        recomputed_geometry_and_loads_match=True,no_early_lateral_release=True,
        simulation_issues=issues,maxima=maxima,history_equivalence=history)
    grid.write_json(out/("validation_audit.json" if complete_matrix else "partial_audit.json"),result)
    return result


def save_figure(fig,out,name):
    for extension in ("png","pdf"):
        fig.savefig(out/f"{name}.{extension}",dpi=180)
    plt.close(fig)


def profile_figures(out):
    import prepare_petal_guidance as generate
    apply_style("report",cjk_first=True)
    fig,axes = plt.subplots(1,2,figsize=(12.5,4.8),constrained_layout=True)
    angle,radius = np.linspace(-45.,45.,2001),np.linspace(32.,50.,1201)
    for g in study.GEOMETRIES:
        p = json.loads((DIRECTORIES[g]/"model_info.json").read_text())["parameters"]
        if g == "original":
            d = abs((angle+45.) % 90.-45.)
            u = np.clip((45.-p["guide_tip_half_angle_deg"]-d)/(45.-2*p["guide_tip_half_angle_deg"]),0.,1.)
            height = p["guide_height_mm"]*(6*u**5-15*u**4+10*u**3)
        else:
            height = generate.surface_height(np.full_like(angle,41.),angle,p)
        axes[0].plot(angle,height,color=COLORS[g],label=LABELS[g],ls="--" if g == "radial" else "-",lw=1.8)
    for g in ("narrow","radial"):
        p = json.loads((DIRECTORIES[g]/"model_info.json").read_text())["parameters"]
        for a,label,ls in ((0.,"峰顶","-"),(45.,"谷底","--")):
            h = generate.surface_height(radius,np.full_like(radius,a),p)
            axes[1].plot(radius,h,color=COLORS[g],ls=ls,lw=1.8,label=LABELS[g]+" · "+label)
    axes[0].set(xlabel="花瓣中心角 [°]",ylabel="导面高度 [mm]",xlim=(-45.,45.),ylim=(-.5,19.))
    axes[1].set(xlabel="径向位置 [mm]",ylabel="导面高度 [mm]",xlim=(32.,50.),ylim=(-.5,19.))
    for axis in axes:
        axis.legend(fontsize=9)
    fig.suptitle("角向工作斜坡与互补配合的径向导面（不含原 1 mm 边缘圆滑）",fontsize=13)
    save_figure(fig,out,"guidance_profiles")


def trial_figures(out,records):
    from matplotlib.colors import ListedColormap
    apply_style("report",cjk_first=True)
    fig,axis = plt.subplots(figsize=(10.5,5.4),constrained_layout=True)
    matrix = np.array([[grid.passed(records[(g,c,study.PRIMARY)]) for g in study.GEOMETRIES] for c in POINTS])
    axis.imshow(matrix,cmap=ListedColormap(["#F3D6CB","#C4E3D3"]),vmin=0,vmax=1,aspect="auto")
    axis.set(xticks=range(3),xticklabels=[LABELS[g] for g in study.GEOMETRIES],yticks=range(9),
        yticklabels=[f"({p[0]:g}, {p[1]:g}) mm / {p[2]:g}°" for p in POINTS.values()])
    for i in range(9):
        for j in range(3):
            axis.text(j,i,"通过" if matrix[i,j] else "未通过",ha="center",va="center",fontsize=11)
    axis.grid(False)
    axis.set_title("0.25 ms 物理步长 · 9 个离散工况 · 相同控制与验收门槛",pad=15)
    save_figure(fig,out,"guidance_cases")
    trace_figures(out)


def trace_figures(out):
    apply_style("report",cjk_first=True)
    fig,axes = plt.subplots(2,3,figsize=(13.8,7.4),constrained_layout=True)
    visible_bounds = {0:[float("inf"),-float("inf")],1:[float("inf"),-float("inf")]}
    for g in study.GEOMETRIES:
        key = (g,"combo_ny6_p15",study.PRIMARY)
        scene = effective_scene(key)
        truth = target_rotation(scene.target)
        a = arrays(out,key,("t","diagnostic_position","diagnostic_rotation",
            "diagnostic_interface_world","lateral_mm","diagnostic_stop_contact_count"))
        rotation = np.einsum("ij,njk->nik",truth.T,a["diagnostic_rotation"])
        position = np.einsum("ij,nj->ni",truth.T,a["diagnostic_position"]-scene.target.pos)
        phase = abs((np.rad2deg(np.arctan2(rotation[:,1,0],rotation[:,0,0]))-45.+45.) % 90.-45.)
        info = json.loads((DIRECTORIES[g]/"model_info.json").read_text())
        signals = [1000*(position[:,2]-info["nominal_flange_separation_m"]),a["lateral_mm"],phase,
            np.linalg.norm(a["diagnostic_interface_world"][:,:3],axis=1),
            abs(a["diagnostic_interface_world"][:,3:]@truth[:,2]),(a["diagnostic_stop_contact_count"] > 0).astype(float)]
        for index,bound in visible_bounds.items():
            value = signals[index][a["t"] >= 9.]
            bound[0],bound[1] = min(bound[0],float(value.min())),max(bound[1],float(value.max()))
        for axis,value in zip(axes.flat,signals,strict=True):
            axis.plot(a["t"],value,label=LABELS[g],color=COLORS[g],lw=1.)
            axis.set(xlim=(9.,a["t"][-1]),xlabel="时间 [s]")
            axis.axvspan(a["t"][-1]-1.,a["t"][-1],color=".94",zorder=-1)
        for axis,value in ((axes[1,0],signals[3]),(axes[1,1],signals[4])):
            peak = int(np.argmax(value))
            axis.scatter(a["t"][peak],value[peak],color=COLORS[g],s=25,zorder=5)
    for axis,label in zip(axes.flat,("相对名义落座间隙 [mm]","横向残差 [mm]","配合相位残差 [°]",
        "接口净接触力 [N]","绕轴净接触力矩 [N·m]","止挡是否承载"),strict=True):
        axis.set_ylabel(label)
    axes[0,0].legend(fontsize=9)
    for index,(low,high) in visible_bounds.items():
        span = max(high-low,1.)
        axes.flat[index].set_ylim(low-.04*span,high+.08*span)
    from matplotlib.ticker import ScalarFormatter
    axes[1,0].set_yscale("symlog",linthresh=1.,linscale=.7)
    axes[1,0].set_yticks((0.,1.,10.,40.,100.,300.))
    axes[1,0].yaxis.set_major_formatter(ScalarFormatter())
    axes[1,0].set_ylabel("接口净接触力 [N]（对称对数）")
    axes[1,0].axhline(40.,color="#B54A39",lw=.9,ls=":")
    axes[1,2].set(yticks=(0.,1.),yticklabels=("否","是"),ylim=(-.1,1.2))
    fig.suptitle("(0, −6) mm / +15°：几何对照的完整接触过程",fontsize=13)
    save_figure(fig,out,"guidance_trace")


def state_figures(out,records):
    import mujoco
    import pinocchio as pin

    from compliant_docking.rendering import CAMERAS, apply_render_theme, scene_options
    from compliant_docking.simulation.mujoco_env import MujRobot
    apply_style("report",cjk_first=True)
    fig,axes = plt.subplots(1,3,figsize=(14,5.2))
    fig.subplots_adjust(left=.01,right=.99,bottom=.16,top=.81,wspace=.035)
    for axis,g in zip(axes,study.GEOMETRIES,strict=True):
        key = (g,"combo_ny6_p15",study.PRIMARY)
        r,scene = records[key],effective_scene(key)
        robot = MujRobot(scene.build_mjmodel(),render=False,record=False,eef_body=scene.eef_body)
        apply_render_theme(robot.model,tool_prefix=scene.tool.prefix,target_prefix=scene.target.prefix)
        robot.data.qpos[:] = arrays(out,key,("q",))["q"][-1]
        mujoco.mj_forward(robot.model,robot.data)
        body = robot.data.body(scene.eef_body)
        robot.coordinate_frame_labels = False
        robot.set_coordinate_frames(dict(actual=pin.SE3(body.xmat.reshape(3,3),body.xpos),
            truth=pin.SE3(target_rotation(scene.target),scene.target.pos)))
        option = scene_options(robot.model,interface_only=True,prefixes=(scene.tool.prefix,scene.target.prefix))
        camera = CAMERAS["contact"].camera()
        camera.lookat[:] = scene.target.pos+target_rotation(scene.target)@np.array([0.,0.,.035])
        camera.distance = .16  # shared interface close-up; preserve target and angles
        with mujoco.Renderer(robot.model,height=600,width=800) as renderer:
            renderer.update_scene(robot.data,camera=camera,scene_option=option)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            robot._add_coordinate_frames(renderer.scene)
            axis.imshow(renderer.render())
        verdict = "通过：落座候选" if grid.passed(r) else "落座但载荷超限" if r["geometry_evaluation"]["status"] == "SEATED_CANDIDATE" else "未通过：卡滞"
        axis.set_title(LABELS[g]+"\n"+verdict,fontsize=12,linespacing=1.6)
        metric = r["geometry_evaluation"]
        axis.text(.5,-.035,f"末 1 s 间隙 {metric['last_second_max_abs_axial_gap_mm']:.4f} mm\n"
            f"承载止挡占比 {metric['last_second_stop_contact_fraction']:.0%}\n"
            f"全过程峰值力 {r['contact_load_gate']['peak_contact_force_N']:.2f} N",
            transform=axis.transAxes,ha="center",va="top",fontsize=10)
        axis.axis("off")
    fig.suptitle("(0, −6) mm / +15° · 记录的最终状态 · 同一视角",fontsize=13)
    fig.text(.02,.01,"金色机械臂侧 / 蓝色目标侧；RGB = X/Y/Z。相机距离统一为 0.16 m；通过仅表示仿真落座候选。",fontsize=10)
    save_figure(fig,out,"guidance_states")


def numerical_figures(out,records):
    apply_style("report",cjk_first=True)
    checks = json.loads((out/"numerical_plan.json").read_text())["jobs"]
    fig,axes = plt.subplots(1,2,figsize=(12.,5.),constrained_layout=True)
    for axis,field,label in zip(axes,("peak_contact_force_N","peak_contact_axial_moment_Nm"),
                                ("全过程峰值合力 [N]","全过程峰值绕轴力矩 [N·m]"),strict=True):
        for i,(g,c,setting) in enumerate(checks):
            values = [records[(g,c,s)]["contact_load_gate"][field] for s in (study.PRIMARY,setting)]
            axis.plot(values,[i,i],color=COLORS[g],lw=1.5)
            axis.scatter(values,[i,i],color=COLORS[g],marker="o",s=45)
            for x,s in zip(values,("0.25","0.125"),strict=True):
                axis.annotate(s,(x,i),xytext=(0,8 if s == "0.25" else -16),
                              textcoords="offset points",ha="center",fontsize=8)
        axis.set(xlabel=label,yticks=range(len(checks)),
            yticklabels=[LABELS[g]+"\n"+f"({POINTS[c][0]:g}, {POINTS[c][1]:g}) mm / {POINTS[c][2]:g}°"
                         for g,c,_ in checks],ylim=(-.6,len(checks)-.4))
    fig.suptitle("物理步长 0.25 → 0.125 ms：峰值逐步保留，控制周期/延迟不变",fontsize=13)
    save_figure(fig,out,"guidance_numerics")


def report_text(out,records,summary,checked):
    best = json.loads((out/"numerical_plan.json").read_text())["selected_geometry"]
    lines = ["# PetalDock100 导向几何验证","",
        "三种几何全部新复跑，机械臂、初始位置、运动参考、摩擦、控制和验收门槛相同。","",
        "| 几何 | 0.25 ms 主矩阵落座候选 |","|---|---:|"]
    lines += [f"| {LABELS[g]} | {summary['primary_passes'][g]}/9 |" for g in study.GEOMETRIES]
    lines += ["",f"按预定的通过数、峰值力顺序，追加数值检查的候选为 **{LABELS[best]}**。","",
        "## 几何与配合","",
        "外径 100 mm、导向高度 18 mm、安装头和止挡保持不变。名义法兰间距 46.4 mm，两端为相同四花瓣结构，以 45° 相位互补配合。",
        "角向候选同时将平顶半角从 8.4375° 减至 4.21875°、将五次曲线改为端部圆滑的线性工作斜坡；本轮评估这两项的组合效果，不能归因于单独收窄。",
        "在不含端部过渡和原边缘圆滑的角向工作区，斜坡角由保留的高度/直径与周期配合约束确定：半径 32–50 mm 处约为 44.4°–32.1°（相对水平面）。",
        "径向候选仅在角向候选上增加宽 6 mm、峰顶下降 3 mm 的导面，并同步抬升谷底，满足 h(r,θ)+h(r,45°−θ)=18 mm。原 1 mm 边缘圆滑保留。","",
        "![轮廓](guidance_profiles.png)","",
        "| 几何 | 导向凸块数/侧 | 装配质量 kg | 名义首触 mm |","|---|---:|---:|---:|"]
    for g in study.GEOMETRIES:
        info = json.loads((DIRECTORIES[g]/"model_info.json").read_text())
        native = json.loads((out/f"{g}_geometry_validation.json").read_text())
        lines.append(f"| {LABELS[g]} | {info['guide_convex_cells']} | {info['body_mass_kg']:.9f} | {1000*native['nominal_first_touch_m']:.6f} |")
    lines += ["","碰撞块与可视表面一起重建，径向表面按多带分块，避免凸包跨越谷底。完整装配的 COM 和六项惯量由原装配加导面网格差值更新，保留安装孔/基座/止挡的质量贡献。",
        "两个新模型的网格加密质量差小于 1.7×10⁻⁶ kg，原点惯量差小于 4.1×10⁻⁹ kg·m²；三个模型的 MuJoCo/Pinocchio 位姿和质量矩阵检查通过。",
        "这是仿真几何原型：已生成 OBJ/MJCF 和碰撞模型，尚未生成制造 STEP，也未完成加工、强度或真实材料接触验证。","",
        "## 完整机械臂对照","",
        "物理步长 0.25 ms，控制周期和 F/T 延迟均为 0.5 ms；原摩擦 0.15、原速度 10 mm/s。F/T 接触触发后，XY 与绕轴刚度在 0.25 s 内释放到零，保留惯量、阻尼和轴向刚度。",
        "末 1 s 门槛：横向 ≤0.5 mm、倾斜 ≤0.5°、相位 ≤2°、轴向间隙 ≤0.75 mm、承载止挡占比 ≥95%；全过程净接触合力 ≤40 N、绕轴净接触轴矩 ≤2 N·m，同时保留原进给、静止、关节范围、饱和、F/T 和内部安全门禁。","",
        "![工况](guidance_cases.png)","",
        "| XY mm / yaw ° | 几何 | 结论 | 横向 mm | 相位 ° | 间隙 mm | 止挡占比 | 峰值力 N | 轴矩 N·m |","|---|---|---|---:|---:|---:|---:|---:|---:|"]
    for c,p in POINTS.items():
        for g in study.GEOMETRIES:
            r = records[(g,c,study.PRIMARY)]
            metric,load = r["geometry_evaluation"],r["contact_load_gate"]
            values = [metric[k] for k in ("last_second_max_lateral_mm","last_second_max_phase_error_deg","last_second_max_abs_axial_gap_mm")]
            lines.append(f"| ({p[0]:g}, {p[1]:g}) / {p[2]:g} | {LABELS[g]} | {'通过' if grid.passed(r) else '未通过'} | "
                +" | ".join(f"{v:.4f}" for v in values)+f" | {metric['last_second_stop_contact_fraction']:.1%} | "
                f"{load['peak_contact_force_N']:.4f} | {load['peak_contact_axial_moment_Nm']:.5f} |")
    lines += ["","![组合工况最终姿态](guidance_states.png)","","![完整接触过程](guidance_trace.png)","",
        "力/力矩逐物理步记录，图件未删去峰值，并用圆点标记；合力轴使用对称对数刻度以同时显示预紧载荷与瞬态峰值。两种方向的组合误差分别实测，不假设正负对称。","","未通过工况及原门禁原因：",""]
    lines += [f"- `{study.name(*key)}`："+"；".join(r["assessment"]["reasons"]) for key,r in sorted(records.items()) if not grid.passed(r)]
    lines += ["","## 步长检查","",
        "4 组追加复跑将物理步长减至 0.125 ms，控制周期和传感延迟不变。两步稳定仅表示现有差异门槛内稳定，两个步长不足以证明完整数值收敛。",
        "沿用原比较门槛：结论相同，横向/间隙差 ≤0.1 mm、相位差 ≤0.1°；峰值力差 ≤max(0.5 N,10%)、轴矩差 ≤max(0.05 N·m,10%)。","",
        "![步长峰值](guidance_numerics.png)","",
        "| 几何/工况 | 比较 | 力差 N | 轴矩差 N·m | 横向差 mm | 间隙差 mm | 相位差 ° |","|---|---|---:|---:|---:|---:|---:|"]
    for name,value in summary["numerical_comparisons"].items():
        d = value["differences"]
        lines.append(f"| `{name}` | {value['status']} | "+" | ".join(f"{d[k]:.6f}" for k in
            ("peak_force_N","peak_axial_moment_Nm","lateral_mm","axial_gap_mm","phase_deg"))+" |")
    lines += ["","## 静态诊断和数据审计","",
        "下面是保持轴线竖直的 (0,−6) mm / +15° 首触，不等于机器人倾斜后的实际卡滞间隙。","",
        "| 几何 | 竖直首触间隙 mm | 瞬时 Fy N | 瞬时 Mz N·m |","|---|---:|---:|---:|"]
    for g in study.GEOMETRIES:
        item = json.loads((out/f"{g}_geometry_validation.json").read_text())["geometry_checks"]["combo_ny6_p15"]
        probe = next(p for p in item["solver_probes"] if p["friction"] == .15)
        w = probe["interface_wrench_world"]
        lines.append(f"| {LABELS[g]} | {item['first_touch_gap_mm']:.5f} | {w[1]:.5f} | {w[5]:.5f} |")
    lines += ["","自由体瞬时探针在首触下 2 μm 施加 10 N 轴向力，分别探测摩擦 0/0.15/0.3；接触反力小于施加力时自由体仍会加速，这不是静力平衡或捕获证明。完整机器人对照只用摩擦 0.15。","",
        f"数据审计 **{checked['status']}**：{checked['records']} 组完整复跑，{checked['matched_references']} 个工况的三组参考位置/时刻逐值一致，数值通道有限，反馈延迟、力矩参考点变换、力平衡和重算门禁通过。",
        f"仿真警告、额外接触、饱和与关节超限审计：`{json.dumps(checked['simulation_issues'],ensure_ascii=False)}`。",
        "3 个原几何半步长工况与上一轮的完整关节轨迹一致；仍是新复跑，历史数据只用于回归核对。原模型和前轮输出按哈希核验保留。",
        "机器人资产的哈希另行补充保存；其文件修改时间早于本轮运行元数据创建，之后再次核验内容一致。",
        "软件检查：256 项非慢速测试通过，4 项慢速测试未运行；7 条为既有绘图库弃用警告。48 项针对控制、门禁和新几何的检查通过。","",
        "固定计划、运行源码快照、资产清单、环境、每组 JSON/NPZ/日志、静态检查、数据审计和数值对照一并保存。后处理源码单独记录，不属于冻结的物理运行源码。","",
        "```bash","MUJOCO_GL=egl uv run python experiments/petal_insertion_suite.py --geometry-study --jobs 3 --out runs/petal_guidance_reproduction --resume",
        "uv run python experiments/petal_guidance_report.py runs/petal_guidance_reproduction","```","",
        "仅验证这些离散偏差、固定起点和零重力条件下的仿真落座候选；不声明连续捕获区域、真实硬件能力或锁紧完成。",""]
    return "\n".join(lines)


def build_report(out):
    out = Path(out)
    frozen = out/"artifact_manifest.json"
    if frozen.exists():
        manifest = json.loads(frozen.read_text())
        assert manifest["status"] == "FROZEN"
        for filename,digest in manifest["artifacts_sha256"].items():
            assert sha(out/filename) == digest,filename
        print("Frozen report verified; existing artifacts retained.",flush=True)
        return
    records = study.read_records(out)
    assert len(records) == 31,"Complete the fixed geometry and numerical matrix first"
    checked = audit(out)
    summary = study.summarize(out)
    profile_figures(out)
    trial_figures(out,records)
    state_figures(out,records)
    numerical_figures(out,records)
    (out/"report.md").write_text(report_text(out,records,summary,checked))
    snapshot = out/"analysis_snapshot"/Path(__file__).name
    snapshot.parent.mkdir(exist_ok=True)
    snapshot.write_bytes(Path(__file__).read_bytes())
    grid.write_json(out/"analysis_manifest.json",dict(postprocessor_sha256=sha(snapshot),
        postprocessor=str(Path(__file__).relative_to(REPO_ROOT)),runtime_sources_unchanged=True,
        environment={p:version(p) for p in ("mujoco","pin","numpy","scipy","matplotlib")}))


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out",type=Path)
    parser.add_argument("--profiles-only",action="store_true")
    parser.add_argument("--partial-audit",action="store_true")
    args = parser.parse_args()
    if args.profiles_only:
        profile_figures(args.out)
    elif args.partial_audit:
        audit(args.out,complete_matrix=False)
    else:
        build_report(args.out)
