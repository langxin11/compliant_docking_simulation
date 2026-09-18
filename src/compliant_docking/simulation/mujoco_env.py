"""
MuJoCo 机器人仿真实验辅助类（含中文注释）
MuJoCo Robot Simulation Interface (with expanded comments)

本模块提供一个围绕 MuJoCo 的轻量封装，便于做机器人仿真实验。
`MujRobot` 类负责：模型加载、仿真步进、可选的交互式渲染（viewer），
以及离屏渲染录帧（可稍后写成 MP4）。

注释意图：解释每个方法的职责、常见故障点（无显示/驱动问题）、
以及使用注意事项，功能保持与原始实现兼容。

This module provides a small, focused wrapper around MuJoCo for robotic
simulation experiments. The class `MujRobot` encapsulates model loading,
simulation stepping, optional interactive rendering (viewer), and
offscreen recording to frames which can be saved to MP4 later.

The comments added here explain intent, common failure modes (headless vs
display), and the responsibilities of each method. Functionality is kept
compatible with the original implementation.

Author: langxin11
Date: 2025
"""

import os
import warnings

import imageio
import mujoco
import numpy as np

from ..models import ASSETS_DIR


class MujRobot:
    """高层 MuJoCo 机器人辅助类 / High-level MuJoCo robot helper.

    Attributes
    ----------
    model : mujoco.MjModel | str | os.PathLike
        仿真模型：可直接传已组装的 MjModel，或传模型 XML 路径（按原逻辑加载）。
    dt : float
        仿真步长（会写入 model.opt.timestep）。
    render : bool
        是否尝试创建交互式可视化窗口（mujoco.viewer）。
    record : bool
        是否通过 MuJoCo 的离屏渲染器采样帧（稍后可写成 MP4）。
    target_pos : array-like (3,)
        目标点的世界系坐标（写入一个 site 以可视化目标）。
    eef_body / eef_marker_site / target_site / camera : str
        末端 body、末端可视化 site、目标 site 与录帧相机的名称（默认值即
        历史硬编码名称；场景驱动时应从场景解析结果传入）。
    """

    def __init__(self, model: mujoco.MjModel | str | os.PathLike,
                 render: bool = True,
                 record: bool = True,
                 dt: float = 0.001,
                 target_pos: np.ndarray | None = None,
                 *,
                 eef_body: str = "dock1",
                 eef_marker_site: str = "eef_marker",
                 target_site: str = "vis",
                 camera: str = "track_cam"):

        # ---- 基本配置 ----
        # 保存构造参数到实例属性
        if target_pos is None:
            target_pos = np.zeros(3)
        self.model_source = model
        self.dt = dt
        self.target_pos = np.asarray(target_pos, dtype=float)

        # 名称参数（默认值 = 历史硬编码名称，保证旧调用行为不变）
        self.eef_body_name = eef_body
        self.eef_marker_site_name = eef_marker_site
        self.target_site_name = target_site
        self.camera_name = camera

        # 轨迹可视化状态：规划路径（青）、实际轨迹（橙）和当前期望点（绿）。
        # 自定义几何只参与 viewer/离屏渲染，不进入碰撞与动力学计算。
        self.planned_path = np.empty((0, 3), dtype=float)
        self.actual_path: list[np.ndarray] = []
        self.desired_pos: np.ndarray | None = None
        self._trail_stride_steps = max(1, int(round(0.1 / self.dt)))  # 10 Hz 轨迹尾迹
        self._max_trail_points = 240

        # ---- MuJoCo 结构初始化 ----
        # 加载模型与数据结构；若路径错误会抛异常
        self.setup_mujoco()

        # viewer 稍后按需创建；先占位，便于测试时判断是否存在
        self.viewer = None
        self.render = bool(render)

        # ---- 头less（无显示）环境探测 ----
        # 在没有显示服务器（如 Linux 无 Xorg/Wayland 或 CI）时，不应创建交互式窗口；
        # 这里通过环境变量 DISPLAY 的存在性做一个保守判断。
        self.headless = "DISPLAY" not in os.environ or not os.environ["DISPLAY"]
        if self.headless and render:
            warnings.warn("Running in headless environment. Interactive rendering disabled.", stacklevel=2)
            self.render = False

        # ---- 尝试创建交互式 viewer ----
        # 在 headless 下会跳过；创建失败会降级为不渲染。
        self.setup_viewer()

        # ---- 离屏渲染录帧相关 ----
        # 这里只准备渲染选项；Renderer 实例在 record=True 时由 setup_renderer 创建。
        self.record = bool(record)
        if self.record:
            self.renderer_options = self.setup_renderer()

    def setup_mujoco(self):
        """加载模型并初始化 MuJoCo 数据结构。

        这里会：
        1) 设置求解容差、步长等默认项；
        2) 解析后续会用到的一些元素 ID（末端执行器、可视化 site、摄像机等）。
           名称由构造参数 eef_body / eef_marker_site / target_site / camera 决定
           （默认 'dock1'、'eef_marker'、'vis'、'track_cam'，需与模型中保持一致）。
        """

        # 模型来源：直接用已组装的 MjModel；否则按 XML 路径加载（可能抛异常）
        if isinstance(self.model_source, mujoco.MjModel):
            self.model = self.model_source
        else:
            self.model = mujoco.MjModel.from_xml_path(str(self.model_source))

        # 设置一些求解器容差参数（较小的容差有助于控制稳定）
        self.model.opt.tolerance = 0.001

        # 为该模型创建运行时数据结构
        self.data = mujoco.MjData(self.model)

        # 设置仿真步长（必要时覆盖模型内的值）
        self.model.opt.timestep = self.dt

        # 通过名称解析常用 ID。若名称在模型中不存在，会返回 -1（或在不同版本中抛异常）。
        # - eef_id: 末端执行器所在的 BODY 的 id
        # - eef_marker_id / vis_id: 用于可视化 ee/目标点的 SITE id
        self.eef_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.eef_body_name)
        self.eef_marker_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.eef_marker_site_name)
        self.vis_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.target_site_name)

        # 录帧时所用摄像机的 id（必须在模型中定义）
        self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)

    def init_simulators(self, init_qpos: np.ndarray):
        """重置仿真并设置初始姿态。

        Parameters
        ----------
        init_qpos : ndarray
            初始广义坐标（qpos），长度需与模型匹配。
        """

        # 重置 data 到默认状态，然后设置给定状态
        mujoco.mj_resetData(self.model, self.data)

        # 这里使用 mj_setState 来设置 qpos；不同版本签名可能略有差异。
        # 如果你遇到报错，可以改为直接赋值 data.qpos[...] = init_qpos，并调用 mj_forward。
        mujoco.mj_setState(self.model, self.data, init_qpos, mujoco.mjtState.mjSTATE_QPOS)

        # 向前计算一次，使得派生量（如 xpos、雅可比等）有效
        mujoco.mj_forward(self.model, self.data)

        # 计数器与录帧缓存
        self.steps = 0
        self.frames = []

    def setup_viewer(self):
        """在可用且请求渲染时，创建交互式 viewer。

        使用 `mujoco.viewer.launch_passive` 返回一个“上下文样”的对象。
        我们打开若干可视化标志位，便于调试接触力等效果。
        """

        if self.render and not self.headless:
            try:
                # 创建 viewer 并设置有用的可视化选项
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                self.viewer.opt.flags[:] = False
                self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TEXTURE] = True
                self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_STATIC] = True
                self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_SKIN] = True
                self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
                self.viewer.opt.geomgroup[3] = False
            except Exception as e:
                # viewer 可能因多种原因创建失败（GLFW 初始化失败、驱动/权限问题、
                # 无显示环境等），此时降级为不渲染以避免崩溃。
                warnings.warn(f"Failed to create viewer: {e}. Rendering disabled.", stacklevel=2)
                self.render = False

    def setup_renderer(self):
        """当需要录帧时，准备离屏渲染的选项并创建渲染器。

        返回一个 `MjvOption` 对象，可在 `update_scene` 时用于控制可视化标志。
        """

        # 创建默认可视化选项，并打开一些调试标志
        render_options = mujoco.MjvOption()
        mujoco.mjv_defaultOption(render_options)
        render_options.flags[:] = False
        render_options.flags[mujoco.mjtVisFlag.mjVIS_TEXTURE] = True
        render_options.flags[mujoco.mjtVisFlag.mjVIS_STATIC] = True
        render_options.flags[mujoco.mjtVisFlag.mjVIS_SKIN] = True
        render_options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        render_options.geomgroup[3] = False

        # 创建离屏 Renderer（在 headless 与非 headless 下都可以工作，
        # 后端由环境变量 MUJOCO_GL 选择：eg. egl/osmesa/glfw）。
        if self.record:
            # 默认较大的分辨率；若内存/性能有限可下调。
            self.renderer = mujoco.Renderer(self.model, height=1080, width=1920)

        return render_options

    def set_trajectory_visualization(self, planned_path: np.ndarray,
                                     *, max_points: int = 160) -> None:
        """设置视频/交互窗口中的完整规划路径，并清空实际轨迹尾迹。

        Args:
            planned_path: 世界系路径点，形状 ``(N, 3)``。
            max_points: 最多保留的路径点数；超出时等距下采样，限制渲染开销。
        """
        points = np.asarray(planned_path, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("planned_path 必须为形状 (N, 3) 的数组")
        if not np.all(np.isfinite(points)):
            raise ValueError("planned_path 包含非有限值")
        if max_points < 2:
            raise ValueError("max_points 必须至少为 2")
        if len(points) > max_points:
            indices = np.linspace(0, len(points) - 1, max_points, dtype=int)
            points = points[indices]
        self.planned_path = points.copy()
        self.actual_path = []

    def set_desired_position(self, position: np.ndarray) -> None:
        """更新当前期望末端位置（世界系），用于绿色球形标记。"""
        position = np.asarray(position, dtype=float).reshape(3)
        if not np.all(np.isfinite(position)):
            raise ValueError("期望位置包含非有限值")
        self.desired_pos = position.copy()

    @staticmethod
    def _append_marker(scene: mujoco.MjvScene, point: np.ndarray,
                       rgba: np.ndarray, radius: float) -> None:
        """向 MjvScene 追加一个不参与物理的球形轨迹标记。"""
        if scene.ngeom >= scene.maxgeom:
            return
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom],
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.full(3, radius),
            point,
            np.eye(3).reshape(-1),
            np.asarray(rgba, dtype=np.float32),
        )
        scene.ngeom += 1

    def _add_trajectory_overlays(self, scene: mujoco.MjvScene,
                                 *, reset_scene: bool = False) -> None:
        """绘制规划路径、实际尾迹和当前期望点。"""
        if reset_scene:
            scene.ngeom = 0

        for point in self.planned_path:
            self._append_marker(scene, point, [0.10, 0.85, 1.00, 0.95], 0.0025)
        # 实际轨迹用 2 Hz 左右的橙色采样点表示，与较细的青色规划点列区分。
        actual_markers = self.actual_path[::5]
        if self.actual_path and (not actual_markers or actual_markers[-1] is not self.actual_path[-1]):
            actual_markers = [*actual_markers, self.actual_path[-1]]
        for point in actual_markers:
            self._append_marker(scene, point, [1.00, 0.40, 0.05, 0.95], 0.004)

        if self.desired_pos is not None and scene.ngeom < scene.maxgeom:
            mujoco.mjv_initGeom(
                scene.geoms[scene.ngeom],
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.full(3, 0.012),
                self.desired_pos,
                np.eye(3).reshape(-1),
                np.array([0.15, 1.00, 0.25, 0.95], dtype=np.float32),
            )
            scene.ngeom += 1

    def step(self, tau: np.ndarray | None = None):
        """推进一步仿真，并在需要时渲染/录帧。

        Parameters
        ----------
        tau : ndarray, optional
            机器人驱动力/力矩输入（维数由 model.nu 决定）。

        Returns
        -------
        qpos, qvel, eef_pos : tuple
            当前关节位置、速度与末端执行器位置。
        """

        if tau is None:
            tau = np.zeros(self.model.nv)

        # 写入控制（裁剪到模型执行器维数 nu；nq==nv 时力矩执行器一一对应）
        self.data.ctrl[:self.model.nu] = tau

        # 物理步进
        mujoco.mj_step(self.model, self.data)

        # 输出常用状态
        qpos = self.data.qpos[:self.model.nq]
        qvel = self.data.qvel[:self.model.nq]

        # 末端执行器位置（基于 body/site 的数据，依模型而定）
        eef_pos = self.data.xpos[self.eef_id]

        if self.steps % self._trail_stride_steps == 0:
            self.actual_path.append(np.asarray(eef_pos, dtype=float).copy())
            if len(self.actual_path) > self._max_trail_points:
                self.actual_path.pop(0)

        # 更新可视化用的 site：把目标点与 ee 位置写入，便于在 viewer/renderer 中观察
        self.data.site_xpos[self.vis_id] = self.target_pos
        self.data.site_xpos[self.eef_marker_id] = eef_pos

        # 若存在交互式 viewer，则同步其显示（被动模式下需要手动 sync）
        if self.render and self.viewer:
            if hasattr(self.viewer, "user_scn"):
                self._add_trajectory_overlays(self.viewer.user_scn, reset_scene=True)
            self.viewer.sync()

        # 录帧：为了降低开销，这里做简单抽样（每 20 步采一帧）
        if self.record and hasattr(self, 'renderer'):
            if self.steps % 20 == 0:
                if self.steps % 1000 == 0:
                    print(f"Frames collected so far: {len(self.frames)}")
                try:
                    # 更新渲染场景；指定摄像机与可视化选项
                    self.renderer.update_scene(self.data, camera=self.camera_name, scene_option=self.renderer_options)
                    # 自定义轨迹点会被场景方向光拉出很长的阴影；视频模式关闭阴影，
                    # 避免把这些阴影视为力矢量或轨迹误差。
                    self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                    self._add_trajectory_overlays(self.renderer.scene)
                    frame = self.renderer.render()
                    # 渲染返回 float32 [0..1]；某些场景可能需要后续转 uint8
                    self.frames.append(frame)
                    if self.steps == 0:
                        print(f"First frame collected, shape: {frame.shape}")
                except Exception as e:
                    # 渲染可能偶发失败（GL/驱动问题等）；打印告警但不终止仿真。
                    if self.steps % 1000 == 0:
                        print(f"Failed to render frame at step {self.steps}: {e}")
                        import traceback
                        traceback.print_exc()

        # 递增步计数并返回关键状态
        self.steps += 1
        return qpos, qvel, eef_pos

    def get_ee_state(self):
        """返回末端执行器的位置与线速度（以 body 为参考）。

        注意：body_xvelp 的意义与模型中 body 定义有关；
        对 site，你可能更偏好使用 site_xpos / site_xvelp。
        """

        ee_pos = self.data.xpos[self.eef_id]
        ee_vel = self.data.body_xvelp[self.eef_id]
        return ee_pos, ee_vel

    def get_joint_state(self):
        """便捷地返回全部关节的位置与速度（维数由 model.nq 决定）。"""

        qpos = self.data.qpos[:self.model.nq]
        qvel = self.data.qvel[:self.model.nq]
        return qpos, qvel

    def to_mp4(self, filepath: str):
        """把录制的帧写成 MP4（若失败则回落为 PNG 样本）。

        主要通过 imageio.mimsave（内部调用 ffmpeg 后端）。若无该后端，
        会退化为按步保存部分 PNG 以便快速检查。

        Parameters
        ----------
        filepath : str
            输出 MP4 的完整路径。
        """

        # 解析输出目录并检查存在性
        directory = os.path.dirname(filepath)

        print(f"Attempting to save video to {filepath}")
        print(f"Directory: {directory}")
        print(f"Total frames collected: {len(self.frames)}")

        if directory and not os.path.exists(directory):
            try:
                os.makedirs(directory)
                print(f"Created directory: {directory}")
            except Exception as e:
                print(f"Failed to create directory {directory}: {e}")
                import traceback
                traceback.print_exc()

        # 若无帧可写则给出告警
        if not self.frames:
            warnings.warn("No frames to save. Video not created.", stacklevel=2)
            return

        # 首选：写 MP4
        try:
            # 注意：imageio 期望 uint8 帧；若当前为 float32 [0..1]，
            # 调用方可以在外部转换，或在此处转换（但会有额外开销）。
            imageio.mimsave(filepath, self.frames, fps=50)
            print(f"Video saved to {filepath}")
        except Exception as e:
            # 失败时给出诊断，并尝试保存样本 PNG
            print(f"Failed to save video: {e}")
            import traceback
            traceback.print_exc()

            try:
                print("Attempting to save frames as individual images...")
                img_dir = os.path.join(directory, "frames")
                if not os.path.exists(img_dir):
                    os.makedirs(img_dir)
                for i, frame in enumerate(self.frames):
                    # 仅保存稀疏子集，避免大量占用磁盘
                    if i % 10 == 0:
                        imageio.imwrite(os.path.join(img_dir, f"frame_{i:04d}.png"), frame)
                print(f"Sample frames saved to {img_dir}")
            except Exception as e2:
                print(f"Failed to save individual frames: {e2}")
                traceback.print_exc()


def test_render():
    """小型手动测试：创建机器人并运行交互式渲染循环。"""
    model_path = str(ASSETS_DIR / "iiwa14.xml")
    robot = MujRobot(model_path, render=True, record=False)
    init_qpos = np.array([0, -np.pi/2, 0, 0, 0, 0, 0])
    robot.init_simulators(init_qpos)
    for _i in range(2000):
        tau = np.zeros(7)
        qpos, qvel, eef_pos = robot.step(tau)
        print("qpos:", qpos)
        print("qvel:", qvel)
        print("eef_pos:", eef_pos)
    robot.to_mp4("test.mp4")


def test_record():
    """小型测试：在无交互式 viewer 的情况下录帧。"""
    model_path = str(ASSETS_DIR / "iiwa14.xml")
    robot = MujRobot(model_path, render=False, record=True)
    init_qpos = np.array([0, -np.pi/2, 0, 0, 0, 0, 0])
    robot.init_simulators(init_qpos)
    for _i in range(10):
        tau = np.zeros(7)
        qpos, qvel, eef_pos = robot.step(tau)
        print("qpos:", qpos)
        print("qvel:", qvel)
        print("eef_pos:", eef_pos)
    robot.to_mp4("result/video/simulation.mp4")


if __name__ == "__main__":
    test_record()
