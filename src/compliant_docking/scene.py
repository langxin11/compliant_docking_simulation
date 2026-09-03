"""场景配置加载与 MjSpec 运行时组装器（阶段 A1 新增，暂未接入现有仿真回路）。

把"机械臂 + 公头 + 母头 + 环境"从单一硬编码 XML 拆成可配置的 MJCF 片段组合：

- 机械臂 MJCF（assets/iiwa14/iiwa14_arm.xml）：纯机械臂，不含工具/环境元素；
- 工具片段（公头）：根 body 必须命名为 ``dock``（挂载位姿由场景 YAML 的
  ``tool.pose`` 提供且相对 ee_site），并且片段内必须提供名为 ``sensor_site``
  的 site（力/力矩传感器的锚点，attach 后自动加前缀）；
- 目标片段（母头）：根 body 必须命名为 ``dock``，组装时经 worldbody frame
  固定于世界系。

组装配方（MjSpec attach 机制）与 legacy 全量 XML（assets/iiwa14/
iiwa14_dock_updated.xml）物理逐位等价，等价性测试见 tests/test_scene.py。
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import yaml

# 仓库根目录（与 models.py 的 ASSETS_DIR 同口径：src/<pkg>/scene.py 上溯 3 级）
REPO_ROOT = Path(__file__).resolve().parents[2]

# 默认场景：历史行为（iiwa14 对接）收编为配置文件
DEFAULT_SCENE_PATH = REPO_ROOT / "scenes" / "iiwa14_docking.yaml"

# physics.integrator / physics.cone 的 YAML 字符串 → MuJoCo 枚举 int 映射
_INTEGRATORS = {
    "euler": int(mujoco.mjtIntegrator.mjINT_EULER),
    "implicit": int(mujoco.mjtIntegrator.mjINT_IMPLICIT),
    "implicitfast": int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
    "rk4": int(mujoco.mjtIntegrator.mjINT_RK4),
}
_CONES = {
    "pyramidal": int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
    "elliptic": int(mujoco.mjtCone.mjCONE_ELLIPTIC),
}


def _enum_value(table: dict[str, int], kind: str, value: str) -> int:
    """查 YAML 字符串对应的 MuJoCo 枚举 int；未知取值报清晰错误。"""
    key = str(value).strip().lower()
    if key not in table:
        options = ", ".join(sorted(table))
        raise ValueError(f"scene 配置 physics.{kind} 不支持 {value!r}，可选值: {options}")
    return table[key]


@dataclass(frozen=True)
class RobotSpec:
    """机械臂描述：MJCF（组装基底）+ URDF（Pinocchio）+ 末端锚点/frame 名。"""

    mjcf: Path
    urdf: Path
    ee_site: str
    ee_frame: str


@dataclass(frozen=True)
class ToolSpec:
    """公头工具片段：根 body 必须叫 ``dock``，须含 ``sensor_site`` site。"""

    mjcf: Path
    prefix: str
    pose_pos: np.ndarray  # 相对 robot.ee_site 的平移
    pose_quat: np.ndarray  # 相对 robot.ee_site 的旋转（wxyz）


@dataclass(frozen=True)
class TargetSpec:
    """母头片段：根 body 必须叫 ``dock``，经 worldbody frame 固定于世界系。"""

    mjcf: Path
    prefix: str
    pos: np.ndarray  # 世界系平移
    quat: np.ndarray  # 世界系旋转（wxyz）


@dataclass(frozen=True)
class PhysicsSpec:
    """物理参数（与历史 XML option 对应）。"""

    timestep: float
    gravity: np.ndarray
    integrator: str  # YAML 字符串，编译时映射为 mujoco.mjtIntegrator
    cone: str  # YAML 字符串，编译时映射为 mujoco.mjtCone
    sdf_iterations: int
    sdf_initpoints: int


@dataclass(frozen=True)
class TaskSpec:
    """对接任务初始条件（历史硬编码值收编）。"""

    init_pos: np.ndarray
    init_ori: np.ndarray  # (3, 3) 旋转矩阵
    ik_guess: np.ndarray
    stroke: np.ndarray


@dataclass(frozen=True)
class Scene:
    """完整对接场景：机械臂 + 公头 + 母头 + 物理 + 任务初始条件。"""

    name: str
    robot: RobotSpec
    tool: ToolSpec
    target: TargetSpec
    physics: PhysicsSpec
    task: TaskSpec
    path: Path  # 场景 YAML 的绝对路径

    # ---- 解析后的名称属性（下阶段接线时使用） ----

    @property
    def eef_body(self) -> str:
        """公头挂载后的根 body 名（framepos 传感器跟踪对象）。"""
        return f"{self.tool.prefix}dock"

    @property
    def sensor_site(self) -> str:
        """公头片段提供的力/力矩传感器锚点 site 名（attach 后带前缀）。"""
        return f"{self.tool.prefix}sensor_site"

    @property
    def camera(self) -> str:
        """组装器统一添加的跟随相机名。"""
        return "track_cam"

    def build_mjmodel(self) -> mujoco.MjModel:
        """按已验证配方组装 MjSpec 并编译为 MjModel。

        配方顺序敏感（与 legacy XML 物理逐位等价，勿改 attach 语义与数值）：
        先设公头根 body 位姿再 attach（attach 把 body 位姿解释为相对 site 系）；
        母头经 worldbody frame 挂载；胶水与传感器在 attach 之后添加。
        """
        # 1) 机械臂基底
        arm = mujoco.MjSpec.from_file(str(self.robot.mjcf))

        # 2) 公头：先给根 body 设挂载位姿（相对 ee_site），再挂到法兰 site
        tool = mujoco.MjSpec.from_file(str(self.tool.mjcf))
        tool_root = tool.body("dock")
        tool_root.pos = self.tool.pose_pos
        tool_root.quat = self.tool.pose_quat
        arm.site(self.robot.ee_site).attach_body(tool_root, prefix=self.tool.prefix)

        # 3) 母头：worldbody 加 frame，母头根 body 挂到 frame（世界系固定）
        female = mujoco.MjSpec.from_file(str(self.target.mjcf))
        frame = arm.worldbody.add_frame(
            name="target_frame", pos=self.target.pos, quat=self.target.quat
        )
        frame.attach_body(female.body("dock"), prefix=self.target.prefix)

        # 4) 环境胶水：地面/平行光/可视化 site/跟随相机
        arm.worldbody.add_geom(
            name="floor",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[0, 0, 0.05],
            material="groundplane",
        )
        arm.worldbody.add_light(
            pos=[0, 0, 1.5],
            dir=[0, 0, -1],
            type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        )
        arm.worldbody.add_site(
            name="eef_marker",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.015, 0.015, 0.015],
            pos=[0, 0, 0],
            rgba=[1, 0, 0, 0.6],
        )
        arm.worldbody.add_site(
            name="vis",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.015, 0.015, 0.015],
            pos=[0, 0, 0],
            rgba=[0, 0, 1, 0.6],
        )
        # 实测结论：mode 须用 mjtCamLight 枚举 int（不是字符串）
        arm.worldbody.add_camera(
            name="track_cam",
            pos=[0.5, 1.3, 0.8],
            mode=int(mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY),
            targetbody=f"{self.tool.prefix}rev",
        )

        # 5) 传感器：framepos 跟踪公头根 body，force/torque 锚在公头 sensor_site
        arm.add_sensor(
            name="body1_position",
            type=mujoco.mjtSensor.mjSENS_FRAMEPOS,
            objtype=mujoco.mjtObj.mjOBJ_BODY,
            objname=f"{self.tool.prefix}dock",
        )
        arm.add_sensor(
            name="force_sensor",
            type=mujoco.mjtSensor.mjSENS_FORCE,
            objtype=mujoco.mjtObj.mjOBJ_SITE,
            objname=f"{self.tool.prefix}sensor_site",
        )
        arm.add_sensor(
            name="torque_sensor",
            type=mujoco.mjtSensor.mjSENS_TORQUE,
            objtype=mujoco.mjtObj.mjOBJ_SITE,
            objname=f"{self.tool.prefix}sensor_site",
        )

        # 6) 物理参数（integrator/cone 从 YAML 字符串映射为枚举 int）
        opt = arm.option
        opt.timestep = self.physics.timestep
        opt.gravity = list(self.physics.gravity)
        opt.integrator = _enum_value(_INTEGRATORS, "integrator", self.physics.integrator)
        opt.cone = _enum_value(_CONES, "cone", self.physics.cone)
        opt.sdf_iterations = self.physics.sdf_iterations
        opt.sdf_initpoints = self.physics.sdf_initpoints

        return arm.compile()


def load_scene(path: str | Path) -> Scene:
    """加载场景 YAML 并解析为 Scene。

    YAML 相对路径相对仓库根解析（与 models.py 的 ASSETS_DIR 同口径）；
    场景文件自身传相对路径时也按仓库根解析。
    """
    scene_path = Path(path)
    if not scene_path.is_absolute():
        scene_path = REPO_ROOT / scene_path
    raw = yaml.safe_load(scene_path.read_text(encoding="utf-8"))

    def asset(rel: str) -> Path:
        p = Path(rel)
        return p if p.is_absolute() else REPO_ROOT / p

    scene = raw["scene"]
    robot = raw["robot"]
    tool = raw["tool"]
    target = raw["target"]
    physics = raw["physics"]
    task = raw["task"]

    return Scene(
        name=str(scene["name"]),
        robot=RobotSpec(
            mjcf=asset(robot["mjcf"]),
            urdf=asset(robot["urdf"]),
            ee_site=str(robot["ee_site"]),
            ee_frame=str(robot["ee_frame"]),
        ),
        tool=ToolSpec(
            mjcf=asset(tool["mjcf"]),
            prefix=str(tool["prefix"]),
            pose_pos=np.asarray(tool["pose"]["pos"], dtype=float),
            pose_quat=np.asarray(tool["pose"]["quat"], dtype=float),
        ),
        target=TargetSpec(
            mjcf=asset(target["mjcf"]),
            prefix=str(target["prefix"]),
            pos=np.asarray(target["pos"], dtype=float),
            quat=np.asarray(target["quat"], dtype=float),
        ),
        physics=PhysicsSpec(
            timestep=float(physics["timestep"]),
            gravity=np.asarray(physics["gravity"], dtype=float),
            integrator=str(physics["integrator"]),
            cone=str(physics["cone"]),
            sdf_iterations=int(physics["sdf_iterations"]),
            sdf_initpoints=int(physics["sdf_initpoints"]),
        ),
        task=TaskSpec(
            init_pos=np.asarray(task["init_pos"], dtype=float),
            init_ori=np.asarray(task["init_ori"], dtype=float).reshape(3, 3),
            ik_guess=np.asarray(task["ik_guess"], dtype=float),
            stroke=np.asarray(task["stroke"], dtype=float),
        ),
        path=scene_path,
    )
