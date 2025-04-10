"""
muj_class.py - MuJoCo Robot Simulation Interface

This module provides a simplified interface to the MuJoCo physics simulator
specifically designed for robotic control experiments. It handles the initialization,
stepping, and visualization of robot models in the MuJoCo environment.

Key features:
- Robot state management (joint positions and velocities)
- End-effector state tracking
- Control input application
- Target position visualization
- Integration with the MuJoCo viewer for real-time simulation display

Author: RQM
Date: 2024
"""

import mujoco
import numpy as np
import os
import imageio

class MujRobot:
    def __init__(self, model_path: str,
                 render:bool= True,
                 record:bool= True,
                 dt:float=0.001,
                 target_pos =np.zeros(3)):

        self.model_path = model_path
        self.dt = dt
        self.target_pos = target_pos

        self.setup_mujoco()
        self.viewer = None
        self.render = render 

        self.setup_viewer()
        self.record = record
        if self.record:
            self.renderer_options = self.setup_renderer()

    def setup_mujoco(self):
        """设置MuJoCo环境"""
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.model.opt.tolerance = 0.001
        # self.model.nconmax = 100  # 根据需要调整数值
        self.data = mujoco.MjData(self.model)

        self.model.opt.timestep = self.dt

        self.eef_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "dock1")  # 替换为实际末端执行器名称
        self.eef_marker_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "eef_marker") #用来可视化eef
        self.vis_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "vis") #用来可视化目标位置

        self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, 'track_cam')

    def init_simulators(self,init_qpos:np.ndarray):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_setState(self.model, self.data, init_qpos, mujoco.mjtState.mjSTATE_QPOS)
        mujoco.mj_forward(self.model, self.data)

        self.steps = 0
        self.frames = []

    def setup_viewer(self):
        if self.render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = True
            self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True
            self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
            self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True

    def setup_renderer(self):
        if self.record:
            self.renderer =  mujoco.Renderer(self.model, width=1920, height=1080)
        render_options = mujoco.MjvOption()
        mujoco.mjv_defaultOption(render_options)
        render_options.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = True
        render_options.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True
        render_options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        render_options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        return render_options

    def step(self,tau:np.ndarray = np.zeros(7)):

        self.data.ctrl[:7] = tau
        mujoco.mj_step(self.model, self.data)

        
        qpos = self.data.qpos[:7]
        qvel = self.data.qvel[:7]

        eef_pos = self.data.xpos[self.eef_id]
        self.data.site_xpos[self.vis_id] = self.target_pos
        self.data.site_xpos[self.eef_marker_id] = eef_pos
        if self.render:
            self.viewer.sync()

        if self.record:
            # 视频以50hz保存
            if self.steps % 20 == 0:
                if self.steps % 1000 == 0:
                    self.renderer_options.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = not self.renderer_options.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT]
                self.renderer.update_scene(self.data,camera='track_cam',scene_option=self.renderer_options)
                frame = self.renderer.render()
                self.frames.append(frame)

        return qpos,qvel,eef_pos
    
    def get_ee_state(self):

        ee_pos = self.data.xpos[self.eef_id]
        ee_vel = self.data.body_xvelp[self.eef_id]
        return ee_pos,ee_vel
    
    def get_joint_state(self):
        qpos = self.data.qpos[:7]
        qvel = self.data.qvel[:7]
        return qpos,qvel
    
    def to_mp4(self, filepath:str):
        """
        Save recorded frames to an MP4 video file.
        
        Args:
            filepath: Complete path including directory and filename (e.g., 'results/video/simulation.mp4')
        """
        # Extract directory path
        directory = os.path.dirname(filepath)
        
        # Create directory if it doesn't exist
        if directory and not os.path.exists(directory):
            os.makedirs(directory)
            
        # Save video file
        imageio.mimsave(filepath, self.frames, fps=50)
        print(f"Video saved to {filepath}")

def test_render():
    model_path = "kuka_xml_urdf/iiwa14.xml"
    robot = MujRobot(model_path,render=True,record=False)
    init_qpos = np.array([0, -np.pi/2, 0, 0, 0, 0,0])
    robot.init_simulators(init_qpos)
    for i in range(2000):
        tau = np.zeros(7)
        qpos,qvel,eef_pos = robot.step(tau)
        print("qpos:",qpos)
        print("qvel:",qvel)
        print("eef_pos:",eef_pos)
    robot.to_mp4("test.mp4")

def test_record():
    model_path = "kuka_xml_urdf/iiwa14.xml"
    robot = MujRobot(model_path,render=False,record=True)
    init_qpos = np.array([0, -np.pi/2, 0, 0, 0, 0,0])
    robot.init_simulators(init_qpos)
    for i in range(10):
        tau = np.zeros(7)
        qpos,qvel,eef_pos = robot.step(tau)
        print("qpos:",qpos)
        print("qvel:",qvel)
        print("eef_pos:",eef_pos)
    robot.to_mp4("result/video/simulation.mp4")

if __name__ == "__main__":
    test_record()


    