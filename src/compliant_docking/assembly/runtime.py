"""Per-run assembly state; no legacy scenario modules are imported."""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .config import ROOT
from .geometry import HexFrameGeometry
from .sequence import Phase, Sequence


class AssemblyRuntime(Sequence):
    def __init__(self, scene, output):
        self.scene = scene
        self.root = ROOT
        self.output = Path(output)
        self.resource, self.baseline = scene.resource, scene.baseline
        self.pick, self.seed = np.array(scene.pick), np.array(scene.seed)
        self.tip_rotation = np.diag([1., -1., -1.])
        self.module_rotation = self.tip_rotation @ np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
        self.pick_tip = self.pick + [0, 0, self.resource.apothem+self.resource.gap]
        self.installed = self.seed + [0, 0, 2*self.resource.apothem+self.resource.gap]
        self.install_tip = self.installed + [0, 0, self.resource.apothem+self.resource.gap]
        self.initial_q = np.array([1.20122253, .55704019, -.68567805, -1.35044302,
                                  .34976560, 1.35536911, -2.62515672])
        robot = ET.parse(ROOT/"assets/iiwa14/urdf/iiwa14.urdf").getroot()
        limits = [robot.find(f"joint[@name='iiwa_joint_{i}']/limit") for i in range(1, 8)]
        self.joint_limits = np.array([[float(v.get("lower")) for v in limits],
                                     [float(v.get("upper")) for v in limits]])
        self.dt = scene.timestep
        self.kp = np.array([600., 600., 500., 500., 180., 140., 80.])
        self.kd = np.array([70., 70., 55., 55., 20., 16., 9.])
        self.torque_limits = np.array([320., 320., 176., 176., 110., 40., 40.])
        self.lock_sites = [("storage_anchor", "module1_port_4_mating"),
                           ("gripper_anchor", "module1_anchor"),
                           ("assembly_anchor", "module1_anchor")]
        self.geometry = HexFrameGeometry(self)

    def phases(self):
        pick, installed = self.pick_tip, self.install_tip
        ready = pick+[0., 0., .16]
        above_pick = np.r_[pick[:2], self.scene.lift_height]
        above_seed = np.r_[installed[:2], self.scene.lift_height]
        return [
            Phase("stowed", "01  左侧基座锁定 · 中央机械臂 · 右侧标准存储接口", 1.5, ready),
            Phase("approach", "02  从上方接近模块 1 的接口 1", 4., pick+[0., 0., .04]),
            Phase("capture", "03  竖直下降，对准并进入捕获位置", 4., pick),
            Phase("grip_check", "04  锁定机械臂与模块 1，确认抓取", 1.5, pick, "grip_on"),
            Phase("rack_release", "05  机械臂锁定确认后，存储接口解锁", 1., pick, "rack_off"),
            Phase("lift", "06  竖直提起模块 1，脱离存储接口", 3., above_pick),
            Phase("transfer", "07  越过中央区域，转运到左侧组装位", 5., above_seed),
            Phase("align", "08  模块 1 接口 4 朝下，对准模块 2 接口 1", 3., installed+[0., 0., .075]),
            Phase("mate", "09  降至接口上方 6 mm，准备低速接触", 6., installed+[0, 0, .006]),
            Phase("assembly_check", "10  实际接触 · 柔顺插合 · 持续就位后锁定", self.scene.contact_duration, installed),
            Phase("gripper_release", "11  模块间锁定确认，卸力后释放机械臂", 1., installed, "grip_off"),
            Phase("retreat", "12  机械臂竖直抬升撤离", 3., installed+[0., 0., self.scene.retract_distance]),
            Phase("complete", "完成：两个模块竖直连接，机械臂已释放", 2., installed+[0., 0., self.scene.retract_distance]),
        ]
