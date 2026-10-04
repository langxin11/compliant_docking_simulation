"""Resource-driven HexFrame geometry and contact diagnostics."""
from __future__ import annotations

import copy
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


def vector(values):
    return " ".join(f"{x:.16g}" for x in values)


def quaternion(rotation):
    return Rotation.from_matrix(rotation).as_quat(scalar_first=True)


def beam(parent, start, end, radius=.003, material="orb_metal"):
    ET.SubElement(parent, "geom", type="capsule", fromto=vector([*start, *end]),
                  size=str(radius), material=material, contype="0", conaffinity="0",
                  mass="0", group="4")

def label_port(parent, number):
    # A compact raised numeral above the port, made from original line geometry.
    strokes = {1: [((0, 0), (0, 1))],
               2: [((0, 1), (1, 1)), ((1, 1), (1, .5)), ((1, .5), (0, .5)),
                   ((0, .5), (0, 0)), ((0, 0), (1, 0))],
               3: [((0, 1), (1, 1)), ((0, .5), (1, .5)), ((0, 0), (1, 0)), ((1, 0), (1, 1))],
               4: [((0, 1), (0, .5)), ((0, .5), (1, .5)), ((1, 0), (1, 1))],
               5: [((1, 1), (0, 1)), ((0, 1), (0, .5)), ((0, .5), (1, .5)),
                   ((1, .5), (1, 0)), ((1, 0), (0, 0))],
               6: [((1, 1), (0, 1)), ((0, 1), (0, 0)), ((0, 0), (1, 0)),
                   ((1, 0), (1, .5)), ((1, .5), (0, .5))]}
    for a, b in strokes[number]:
        beam(parent, [(a[0]-.5)*.009, .060+a[1]*.014, .004],
             [(b[0]-.5)*.009, .060+b[1]*.014, .004], .001, "orb_dark")

class HexFrameGeometry:
    def __init__(self, runtime):
        self.r = runtime
        self.resource = runtime.resource

    def build_model(self):
        root = ET.parse(self.r.baseline).getroot()
        if root.find("worldbody/body[@name='base']") is None:
            environment = root
            root = ET.parse(self.r.root/"assets/iiwa14/iiwa14_arm.xml").getroot()
            root.remove(root.find("keyframe"))
            root.remove(root.find("visual"))
            root.append(copy.deepcopy(environment.find("visual")))
            for item in environment.find("asset"):
                # The environment owns palette and sky, the robot owns meshes.
                old = root.find(f"asset/{item.tag}[@name='{item.get('name')}']") if item.get("name") else root.find(f"asset/{item.tag}[@type='{item.get('type')}']")
                if old is not None:
                    root.find("asset").remove(old)
                root.find("asset").append(copy.deepcopy(item))
            for name in ["gray", "light_gray"]:
                root.find(f"asset/material[@name='{name}']").set("rgba", ".87 .91 .94 1")
            root.find("asset/material[@name='orange']").set("rgba", ".22 .35 .48 1")
            root.find("asset/material[@name='groundplane']").set("rgba", "0 0 0 0")
            for item in environment.find("worldbody"):
                root.find("worldbody").append(copy.deepcopy(item))
            root.append(copy.deepcopy(environment.find("equality")))
            root.find(".//body[@name='link7']").append(ET.parse(self.r.baseline.parent/"gripper.xml").getroot())
            for i, motor in enumerate(root.find("actuator")):
                motor.set("biastype", "affine")
                motor.set("biasprm", vector([0, -self.r.kp[i], -self.r.kd[i]]))
                motor.set("forcelimited", "true")
                motor.set("forcerange", vector([-self.r.torque_limits[i], self.r.torque_limits[i]]))
        root.set("model", "hexframe_fixed_base_assembly")
        # Rebind the copied baseline's mesh directory to the current checkout.
        root.find("compiler").set("meshdir", str(self.r.root/"assets/iiwa14/assets"))
        asset, world = root.find("asset"), root.find("worldbody")
        external = ET.parse(self.resource.path / "mjcf/assets.xml").getroot()
        root.append(copy.deepcopy(external.find("default")))
        for element in external.find("asset"):
            item = copy.deepcopy(element)
            if item.get("file"):
                item.set("file", str((self.resource.path / "mjcf" / item.get("file")).resolve()))
            asset.append(item)
        template = ET.parse(self.resource.path / "mjcf/a_contents.xml").getroot()
        for name, center in [("module1", self.r.pick), ("module2", self.r.seed)]:
            existing = world.find(f"body[@name='{name}']")
            if existing is not None:
                world.remove(existing)
            module = ET.SubElement(world, "body", name=name, pos=vector(center),
                                   quat=vector(quaternion(self.r.module_rotation)))
            active_port = 0 if name == "module1" else 3
            for original in template:
                item = copy.deepcopy(original)
                if item.tag == "inertial":
                    info = self.resource.info
                    inertia = np.asarray(info["body_inertia_kg_m2"])
                    item.set("mass", str(info["body_mass_kg"]))
                    item.set("pos", vector(info["body_com_m"]))
                    item.set("fullinertia", vector([inertia[0, 0], inertia[1, 1], inertia[2, 2],
                                                   inertia[0, 1], inertia[0, 2], inertia[1, 2]]))
                old_name = item.get("name", "")
                if "_guide_" in old_name or "_stop_" in old_name:
                    # Only the assembly faces need detailed insertion collision.
                    if not old_name.startswith(f"a_port{active_port}_"):
                        continue
                    item.set("contype", "2")
                    item.set("conaffinity", "2")
                    item.set("solref", ".008 1")
                if old_name:
                    item.set("name", name+old_name[1:])
                module.append(item)
            for number in range(1, 7):
                port = self.resource.port(number)
                body = ET.SubElement(module, "body", name=f"{name}_port_{number}",
                                     pos=vector(port["position_m"]), quat=vector(port["quaternion_wxyz"]))
                ET.SubElement(body, "site", name=f"{name}_port_{number}_frame", size=".001")
                ET.SubElement(body, "site", name=f"{name}_port_{number}_mating",
                              pos=vector([0, 0, self.resource.gap/2]), size=".001")
                label_port(body, number)
                if port["id"] != active_port:
                    ET.SubElement(body, "geom", name=f"{name}_port_{number}_head_envelope",
                                  type="cylinder", pos="0 0 .016", size=".05 .016", mass="0",
                                  group="3", contype="1", conaffinity="1", friction=".15 .003 .0001")
            if name == "module1":
                ET.SubElement(module, "freejoint", name="module1_free")
                ET.SubElement(module, "site", name="module1_anchor", size=".001")
            else:
                ET.SubElement(module, "site", name="assembly_anchor", pos=vector([-2*self.resource.apothem-self.resource.gap, 0, 0]), size=".001")
        existing = world.find("site[@name='storage_anchor']")
        if existing is not None:
            world.remove(existing)
        for name in ["storage", "spare_slot", "seed_station"]:
            existing = world.find(f"body[@name='{name}']")
            if existing is not None:
                world.remove(existing)
        asset.append(ET.Element("mesh", name="hf_robot_adapter",
                                file=str(self.resource.path / "interface/meshes/visual/adapter.obj")))
        self.add_base_dock(world)
        for station, port, center in [("storage", "storage_dock_port", self.r.pick),
                                      ("spare_slot", "spare_dock_port", self.r.pick+[.40, 0, 0])]:
            head, _, _ = self.add_deck_socket(world, station, port, center, detailed=True)
            if station == "storage":
                ET.SubElement(head, "site", name="storage_anchor", pos=vector([0, 0, self.resource.gap/2]),
                              quat=vector(quaternion(self.resource.pair_rotation)), size=".001")
        root.find("equality/weld[@name='storage_lock']").set("site2", "module1_port_4_mating")
        eff = root.find(".//body[@name='gripper']")
        old_head = eff.find("body[@name='gripper_crown']")
        if old_head is not None:
            eff.remove(old_head)
        top = self.r.module_rotation@np.asarray(self.resource.info["ports"][3]["rotation_matrix"])
        self.add_petal_head(eff, "gripper_petal_head", np.zeros(3), self.r.tip_rotation.T@top@self.resource.pair_rotation)
        eff.find("site[@name='gripper_anchor']").set("pos", vector([0, 0, self.resource.apothem+self.resource.gap]))
        self.r.output.mkdir(parents=True, exist_ok=True)
        if hasattr(self.r, "joint_limits"):
            for i, joint in enumerate(root.findall(".//joint[@name]")):
                joint.set("limited", "true")
                joint.set("range", vector(np.rad2deg(self.r.joint_limits[:, i])))
        xml = ET.tostring(root, encoding="unicode")
        (self.r.output / "model.xml").write_text(xml)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        checks = []
        for name in ["module1", "module2"]:
            body = model.body(name)
            assert abs(float(body.mass[0])-self.resource.info["body_mass_kg"]) < 1e-8
            inertia_rotation = Rotation.from_quat(body.iquat, scalar_first=True).as_matrix()
            np.testing.assert_allclose(inertia_rotation@np.diag(body.inertia)@inertia_rotation.T,
                                       self.resource.info["body_inertia_kg_m2"], atol=1e-12)
            np.testing.assert_allclose(body.ipos, self.resource.info["body_com_m"], atol=1e-12)
            module = data.body(name)
            for number in range(1, 7):
                p = data.body(f"{name}_port_{number}")
                local = module.xmat.reshape(3, 3).T@(p.xpos-module.xpos)
                np.testing.assert_allclose(local, self.resource.port(number)["position_m"], atol=1e-10)
                assert abs(local[2]) < 1e-10
                checks.append(dict(module=name, port=number, resource_port=self.resource.port(number)["id"]))
        base_connection = self.check_base_dock(model, data)
        storage_connection = self.check_storage_docks(model, data)
        (self.r.output / "geometry_check.json").write_text(json.dumps(dict(
            status="PASS", module="HexFrame", ports=checks, original_dimensions_mm=self.resource.info["overall_dimensions_mm"],
            mass_kg=self.resource.info["body_mass_kg"], nominal_center_spacing_m=self.resource.info["nominal_module_center_separation_m"],
            base_connection=base_connection,
            storage_connection=storage_connection,
            collision_scope="frame boxes and mount/base proxies; assembly faces detailed guide/stop cells; other heads envelope proxies"), indent=2))
        return model

    def add_petal_head(self, parent, name, position, rotation):
        """Use the same standard head for the robot and spacecraft mounting port."""
        head = ET.SubElement(parent, "body", name=name, pos=vector(position),
                             quat=vector(quaternion(rotation)))
        for mesh, material in [("head_plate", "hf_blue"), ("stop_land", "hf_silver"),
                               *[(f"petal_{i}", "hf_blue") for i in range(4)]]:
            ET.SubElement(head, "geom", name=f"{name}_{mesh}", type="mesh", mesh="hf_v_"+mesh, material=material,
                          mass="0", contype="0", conaffinity="0")
        ET.SubElement(head, "geom", name=f"{name}_adapter", type="mesh", mesh="hf_robot_adapter", material="hf_silver",
                      mass="0", contype="0", conaffinity="0")
        return head

    def add_base_dock(self, world):
        """Attach module 2 at side port 4 with a prelocked, matched base port."""
        head, position, rotation = self.add_deck_socket(world, "seed_station", "base_dock_port", self.r.seed)
        # self.resource.apothem fixed parent-child transform represents an already locked interface.
        # This is not a newly simulated capture/latch event at the base.
        module = world.find("body[@name='module2']")
        world.remove(module)
        module.set("pos", vector(rotation.T@(self.r.seed-position)))
        module.set("quat", vector(quaternion(rotation.T@self.r.module_rotation)))
        head.append(module)

    def add_deck_socket(self, world, station_name, port_name, center, *, detailed=False):
        """Build one matching upward socket for base, storage and spare positions."""
        lower = self.resource.info["ports"][0]
        module_port_position = center+self.r.module_rotation@np.asarray(lower["position_m"])
        module_port_rotation = self.r.module_rotation@np.asarray(lower["rotation_matrix"])
        rotation = module_port_rotation@self.resource.pair_rotation.T
        position = module_port_position-rotation[:, 2]*self.resource.gap
        mount_height = float(position[2]-.002)  # Back of adapter sits on the pedestal.
        if mount_height <= 0:
            raise ValueError(f"{station_name} is too low for the docking adapter")
        bus = world.find("body[@name='compact_bus']")
        station = ET.SubElement(bus, "body", name=station_name, pos=vector([*position[:2], 0]))
        prefix = port_name.removesuffix("_port")
        ET.SubElement(station, "geom", name=f"{prefix}_mount", type="cylinder",
                      pos=vector([0, 0, mount_height/2]), size=vector([.075, mount_height/2]),
                      material="hf_silver", mass="0", contype="1", conaffinity="1")
        for number, angle in enumerate(np.arange(4)*np.pi/2):
            ET.SubElement(station, "geom", name=f"{prefix}_bolt_{number}", type="cylinder",
                          pos=vector([.064*np.cos(angle), .064*np.sin(angle), mount_height+.001]),
                          size=".004 .001", material="orb_dark", mass="0", contype="0", conaffinity="0")
        ET.SubElement(station, "geom", name=f"{station_name}_led", type="sphere", pos="0 -.082 .009",
                      size=".005", material="orb_cyan", mass="0", contype="0", conaffinity="0")
        head = self.add_petal_head(station, port_name, np.array([0, 0, position[2]]), rotation)
        ET.SubElement(head, "site", name=f"{prefix}_mating", pos=vector([0, 0, self.resource.gap/2]), size=".001")
        if detailed:
            # The two petals interleave: a solid cylinder would falsely overlap.
            source = ET.parse(self.resource.path / "mjcf/a_contents.xml").getroot()
            port_rotation = np.asarray(lower["rotation_matrix"])
            for original in source:
                name = original.get("name", "")
                if not name.startswith("a_port0_guide_") and not name.startswith("a_port0_stop_"):
                    continue
                item = copy.deepcopy(original)
                item.set("name", port_name+name[len("a_port0"):])
                item.set("pos", vector(port_rotation.T@(np.fromstring(item.get("pos"), sep=" ")-np.asarray(lower["position_m"]))))
                orientation = Rotation.from_quat(np.fromstring(item.get("quat"), sep=" "), scalar_first=True).as_matrix()
                item.set("quat", vector(quaternion(port_rotation.T@orientation)))
                item.set("contype", "4")
                item.set("conaffinity", "2")
                item.set("solref", ".008 1")
                head.append(item)
        else:
            ET.SubElement(head, "geom", name=f"{prefix}_head_envelope", type="cylinder", pos="0 0 .016",
                          size=".05 .016", mass="0", group="3", contype="1", conaffinity="1")
        return head, position, rotation

    def check_storage_docks(self, model, data):
        base, module = data.body("storage_dock_port"), data.body("module1_port_4")
        np.testing.assert_allclose(base.xmat.reshape(3, 3).T@module.xmat.reshape(3, 3), self.resource.pair_rotation, atol=1e-10)
        np.testing.assert_allclose(base.xmat.reshape(3, 3).T@(module.xpos-base.xpos), [0, 0, self.resource.gap], atol=1e-10)
        np.testing.assert_allclose(data.site("storage_dock_mating").xpos, data.site("module1_port_4_mating").xpos, atol=1e-10)
        assert self.r.lock_error(model, data, 0)[0] < 1e-10
        assert self.r.lock_error(model, data, 0)[1] < 1e-10
        weld = model.equality("storage_lock")
        assert weld.obj1id[0] == model.site("storage_anchor").id
        assert weld.obj2id[0] == model.site("module1_port_4_mating").id
        spare = data.body("spare_dock_port")
        np.testing.assert_allclose(spare.xpos-base.xpos, [.40, 0, 0], atol=1e-10)
        np.testing.assert_allclose(spare.xmat, base.xmat, atol=1e-10)
        return dict(status="PASS", storage_port="storage_dock_port", module_port="module1_port_4",
                    spare_port="spare_dock_port", spare_empty=True, root_gap_m=self.resource.gap,
                    lock_model="site weld at mating plane; unlock after gripper confirmation")

    def is_storage_contact(self, names):
        return ((names[0].startswith("module1_port0_") and names[1].startswith("storage_dock_port_")) or
                (names[1].startswith("module1_port0_") and names[0].startswith("storage_dock_port_")))

    def storage_wrench(self, model, data):
        force, depth, count = np.zeros(3), 0., 0
        for i in range(data.ncon):
            c = data.contact[i]
            names = [model.geom(g).name or "" for g in [c.geom1, c.geom2]]
            if not self.is_storage_contact(names):
                continue
            local = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, local)
            sign = 1 if names[1].startswith("module1_") else -1
            force += sign*c.frame.reshape(3, 3).T@local[:3]
            depth = max(depth, -float(c.dist))
            count += int(local[0] > 1e-7)
        return force, depth, count

    def check_base_dock(self, model, data):
        base = data.body("base_dock_port")
        module = data.body("module2_port_4")
        np.testing.assert_allclose(base.xmat.reshape(3, 3).T@module.xmat.reshape(3, 3), self.resource.pair_rotation, atol=1e-10)
        np.testing.assert_allclose(base.xmat.reshape(3, 3).T@(module.xpos-base.xpos), [0, 0, self.resource.gap], atol=1e-10)
        error = float(np.linalg.norm(data.site("base_dock_mating").xpos-data.site("module2_port_4_mating").xpos))
        assert error < 1e-10
        assert model.body("module2").parentid[0] == model.body("base_dock_port").id
        assert model.body("module2").jntnum[0] == 0
        return dict(status="PASS", base_port="base_dock_port", module_port="module2_port_4",
                    interface="PetalDock100 V2", root_gap_m=self.resource.gap, mating_position_error_m=error,
                    prelocked=True, lock_model="fixed parent-child transform; base capture not simulated")

    def wrench(self, model, data):
        f, m = np.zeros(3), np.zeros(3)
        unexpected, depth, count = [], 0., 0
        origin = data.body("module1_port_4").xpos
        for i in range(data.ncon):
            c = data.contact[i]
            names = [model.geom(g).name or "" for g in [c.geom1, c.geom2]]
            matched = ((names[0].startswith("module1_port0_") and names[1].startswith("module2_port3_")) or
                       (names[1].startswith("module1_port0_") and names[0].startswith("module2_port3_")))
            if not matched:
                if self.is_storage_contact(names):
                    continue
                unexpected.append((int(c.geom1), int(c.geom2)))
                continue
            local = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, local)
            sign = 1 if names[1].startswith("module1_") else -1
            fi = sign*c.frame.reshape(3, 3).T@local[:3]
            f += fi
            m += sign*c.frame.reshape(3, 3).T@local[3:]+np.cross(c.pos-origin, fi)
            depth = max(depth, -float(c.dist))
            count += int(np.linalg.norm(fi) > 1e-7)
        return f, m, count, unexpected, depth

    def stop_loaded(self, model, data):
        for i in range(data.ncon):
            c = data.contact[i]
            names = [model.geom(g).name or "" for g in [c.geom1, c.geom2]]
            if ((names[0].startswith("module1_port0_stop_") and names[1].startswith("module2_port3_stop_")) or
                    (names[1].startswith("module1_port0_stop_") and names[0].startswith("module2_port3_stop_"))):
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, i, force)
                if force[0] > 1e-5:
                    return True
        return False

