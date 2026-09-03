"""场景组装测试：YAML 配置加载、MjSpec 组装结构、与 legacy 全量 XML 的物理等价性。

组装器（compliant_docking.scene）按 attach 机制把"纯机械臂 + 公头片段 + 母头片段
+ 环境胶水"拼成 MjModel；本文件锚定两条行为基准：

1. 结构与质量：自由度、命名对象（body/site/geom/sensor/camera）、总质量 24.5 kg；
2. 物理等价：home 位姿下逐对象世界位姿、以及 120 步确定性 rollout 的 qpos
   与 legacy `assets/iiwa14/iiwa14_dock_updated.xml` 逐位一致（atol=1e-12）。

若等价性测试失败，说明组装配方或片段被改动，行为不再与 legacy 一致。
"""
import mujoco
import numpy as np
import pytest

from compliant_docking.models import ASSETS_DIR, load_pin_model
from compliant_docking.scene import REPO_ROOT, load_scene

SCENE_YAML = REPO_ROOT / "scenes" / "iiwa14_docking.yaml"
LEGACY_XML = ASSETS_DIR / "iiwa14_dock_updated.xml"
FR3_SCENE_YAML = REPO_ROOT / "scenes" / "fr3_docking.yaml"
TWO_PHASE_SCENE_YAML = REPO_ROOT / "scenes" / "iiwa14_docking_twophase.yaml"

# FR3 home 位形（fr3_docking.yaml 的 ik_guess，IK 初猜锚点）
FR3_HOME = np.array([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853])

# legacy XML 的 home 关键帧 qpos（等价性检查的初始状态）
HOME_QPOS = np.array(
    [1.06808021, 0.34036467, -2.49507952, 1.59293802, 2.93006762, 1.27942644, -1.39355401]
)

# 逐对象等价检查表：按对象类型分组的 (legacy 名 → 新组装名)
NAME_MAP = {
    mujoco.mjtObj.mjOBJ_BODY: {
        "dock1": "tool_dock",  # 公头根 body
        "rev": "tool_rev",  # 公头几何 body（euler 0 0 40）
        "dock2": "target_dock",  # 母头根 body
    },
    mujoco.mjtObj.mjOBJ_GEOM: {
        "dock1_geom": "tool_dock_geom",
        "dock1_visual": "tool_dock_visual",
        "dock2_geom": "target_dock_geom",
        "dock2_visual": "target_dock_visual",
    },
    mujoco.mjtObj.mjOBJ_SITE: {
        "sensor_site": "tool_sensor_site",
    },
}


@pytest.fixture(scope="module")
def scene():
    """默认场景（模块级共享，避免重复解析 YAML）。"""
    return load_scene(SCENE_YAML)


@pytest.fixture(scope="module")
def pair(scene):
    """(legacy, 新组装) 模型与各自 MjData；测试内先 reset 到 home 再使用。"""
    legacy = mujoco.MjModel.from_xml_path(str(LEGACY_XML))
    assembled = scene.build_mjmodel()
    return (
        (legacy, mujoco.MjData(legacy)),
        (assembled, mujoco.MjData(assembled)),
    )


def _reset_to_home(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """复位到 home 关键帧并做一次 forward，填充位姿缓存。

    顺带锚定：home 关键帧 qpos 必须等于规格中的 HOME_QPOS（若有人改动
    XML 关键帧或片段导致 DoF 变化，这里第一时间报警）。
    """
    mujoco.mj_resetDataKeyframe(model, data, 0)
    np.testing.assert_allclose(data.qpos, HOME_QPOS, atol=1e-12, rtol=0)
    mujoco.mj_forward(model, data)


def test_load_default_scene():
    """默认场景 YAML 解析：末端锚点、任务行程、时间步长与规格一致。"""
    loaded = load_scene(SCENE_YAML)

    assert loaded.name == "iiwa14_docking"
    assert loaded.robot.ee_site == "attachment_site"
    np.testing.assert_array_equal(loaded.task.stroke, [0.0, 0.0, -0.18])
    assert loaded.physics.timestep == 0.001
    assert loaded.task.init_ori.shape == (3, 3)


def test_build_mjmodel_structure(scene):
    """新组装模型结构：7 DoF、命名对象齐全、总质量与 legacy 相同（24.5 kg）。"""
    model = scene.build_mjmodel()

    assert model.nq == 7
    assert model.nu == 7
    assert model.nv == 7

    required = {
        mujoco.mjtObj.mjOBJ_BODY: ["tool_dock", "target_dock"],
        mujoco.mjtObj.mjOBJ_SITE: ["tool_sensor_site"],
        mujoco.mjtObj.mjOBJ_GEOM: ["target_dock_geom"],
        mujoco.mjtObj.mjOBJ_SENSOR: ["force_sensor", "torque_sensor"],
        mujoco.mjtObj.mjOBJ_CAMERA: ["track_cam"],
    }
    for objtype, names in required.items():
        for name in names:
            obj_id = mujoco.mj_name2id(model, objtype, name)
            assert obj_id >= 0, f"新组装模型缺少 {objtype.name} {name!r}"

    # legacy 全量模型总质量（机械臂 + 公头 + 母头）
    assert model.body_mass.sum() == pytest.approx(24.5, abs=1e-6)


def test_assembly_equivalent_to_legacy_xml(pair):
    """物理等价（静态）：home 位姿下逐对象世界位姿与 legacy XML 一致（atol=1e-12）。"""
    (legacy, legacy_data), (assembled, assembled_data) = pair
    _reset_to_home(legacy, legacy_data)
    _reset_to_home(assembled, assembled_data)

    for objtype, names in NAME_MAP.items():
        for legacy_name, assembled_name in names.items():
            legacy_id = mujoco.mj_name2id(legacy, objtype, legacy_name)
            assembled_id = mujoco.mj_name2id(assembled, objtype, assembled_name)
            assert legacy_id >= 0, f"legacy 模型缺少 {legacy_name!r}"
            assert assembled_id >= 0, f"新组装模型缺少 {assembled_name!r}"

            if objtype == mujoco.mjtObj.mjOBJ_BODY:
                np.testing.assert_allclose(
                    legacy_data.xpos[legacy_id], assembled_data.xpos[assembled_id],
                    atol=1e-12, rtol=0,
                )
                np.testing.assert_allclose(
                    legacy_data.xmat[legacy_id], assembled_data.xmat[assembled_id],
                    atol=1e-12, rtol=0,
                )
            elif objtype == mujoco.mjtObj.mjOBJ_GEOM:
                np.testing.assert_allclose(
                    legacy_data.geom_xpos[legacy_id], assembled_data.geom_xpos[assembled_id],
                    atol=1e-12, rtol=0,
                )
                np.testing.assert_allclose(
                    legacy_data.geom_xmat[legacy_id], assembled_data.geom_xmat[assembled_id],
                    atol=1e-12, rtol=0,
                )
            else:  # site
                np.testing.assert_allclose(
                    legacy_data.site_xpos[legacy_id], assembled_data.site_xpos[assembled_id],
                    atol=1e-12, rtol=0,
                )


def test_rollout_120_steps_equivalent(pair):
    """物理等价（动态）：120 步同力矩 rollout 的 qpos 与 legacy XML 逐点一致。"""
    (legacy, legacy_data), (assembled, assembled_data) = pair
    _reset_to_home(legacy, legacy_data)
    _reset_to_home(assembled, assembled_data)

    for step in range(120):
        tau = np.array([0.3 * np.sin(0.01 * step + i) for i in range(7)])
        legacy_data.ctrl[:] = tau
        assembled_data.ctrl[:] = tau
        mujoco.mj_step(legacy, legacy_data)
        mujoco.mj_step(assembled, assembled_data)
        np.testing.assert_allclose(
            legacy_data.qpos, assembled_data.qpos, atol=1e-12, rtol=0,
            err_msg=f"rollout 第 {step} 步 qpos 与 legacy 不一致",
        )


# ---- 两段式对接场景（可选 trajectory 段） ----

def test_load_twophase_scene():
    """两段式场景：trajectory 段解析为 TrajectorySpec；默认场景无 trajectory。"""
    loaded = load_scene(TWO_PHASE_SCENE_YAML)

    assert loaded.name == "iiwa14_docking_twophase"
    assert loaded.trajectory is not None
    assert loaded.trajectory.standoff == 0.10
    assert loaded.trajectory.v_max_approach == 0.10
    assert loaded.trajectory.a_max_approach == 0.20
    assert loaded.trajectory.v_max_docking == 0.02
    assert loaded.trajectory.a_max_docking == 0.05

    # 默认场景不含 trajectory 段 → 走历史单段轨迹路径
    default = load_scene(SCENE_YAML)
    assert default.trajectory is None


# ---- FR3 场景（Menagerie MJCF 变体，Pinocchio 直读 MJCF） ----

@pytest.fixture(scope="module")
def fr3_scene():
    """FR3 对接场景（模块内共享）。"""
    return load_scene(FR3_SCENE_YAML)


def test_load_fr3_scene(fr3_scene):
    """FR3 场景 YAML 加载：pin_model 为 MJCF、末端锚点与 home 初猜正确。"""
    assert fr3_scene.name == "fr3_docking"
    assert fr3_scene.robot.pin_model.suffix == ".xml"
    assert fr3_scene.robot.ee_site == "attachment_site"
    assert fr3_scene.robot.ee_frame == "attachment_site"
    np.testing.assert_array_equal(fr3_scene.task.ik_guess, FR3_HOME)


def test_build_fr3_mjmodel(fr3_scene):
    """FR3 组装结构：7 自由度、公/母头对象存在、总质量 ≈ 20.315 kg。"""
    m = fr3_scene.build_mjmodel()
    assert m.nq == 7 and m.nv == 7 and m.nu == 7
    assert m.body_mass.sum() == pytest.approx(20.315485, abs=1e-3)
    for kind, name in [
        (mujoco.mjtObj.mjOBJ_BODY, "tool_dock"),
        (mujoco.mjtObj.mjOBJ_SITE, "tool_sensor_site"),
        (mujoco.mjtObj.mjOBJ_BODY, "target_dock"),
    ]:
        assert mujoco.mj_name2id(m, kind, name) >= 0, f"缺少对象: {name}"


def test_fr3_pin_model_from_mjcf(fr3_scene):
    """Pinocchio 经 MJCF 直读 FR3：7 自由度、零重力、末端 frame 有效。"""
    model = load_pin_model(fr3_scene.robot.pin_model)
    assert model.nq == 7
    np.testing.assert_array_equal(model.gravity.linear, np.zeros(3))
    assert model.getFrameId("attachment_site") < model.nframes
