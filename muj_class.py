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
class MujRobot:
    def __init__(self, model_path: str,render:bool= True,dt:float=0.001,target_pos =np.zeros(3)):

        self.model_path = model_path
        self.dt = dt
        self.target_pos = target_pos

        self.setup_mujoco()
        self.viewer = None

        self.render = render
        
        self.setup_viewer()

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

        # self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, 'track_cam')

    def init_simulators(self,init_qpos:np.ndarray):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_setState(self.model, self.data, init_qpos, mujoco.mjtState.mjSTATE_QPOS)
        mujoco.mj_forward(self.model, self.data)

    def setup_viewer(self):
        if self.render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            # self.viewer.cam.trackingcamid = self.camera_id
            # self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING

    def step(self,tau:np.ndarray ):

        self.data.ctrl[:7] = tau
        mujoco.mj_step(self.model, self.data)

        
        qpos = self.data.qpos[:7]
        qvel = self.data.qvel[:7]

        eef_pos = self.data.xpos[self.eef_id]
        self.data.site_xpos[self.vis_id] = self.target_pos
        #self.data.site_xpos[self.eef_marker_id] = eef_pos
        if self.render:
            # with self.viewer.lock():
            #      self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.data.time % 2)
            #      self.viewer.opt.flags[mujoco.mjtVisFlag. mjVIS_CONTACTFORCE] = True

            self.viewer.sync()

        return qpos,qvel,eef_pos
    
    def get_ee_state(self):

        ee_pos = self.data.xpos[self.eef_id]
        ee_vel = self.data.body_xvelp[self.eef_id]
        return ee_pos,ee_vel
    
    def get_joint_state(self):
        qpos = self.data.qpos[:7]
        qvel = self.data.qvel[:7]
        return qpos,qvel