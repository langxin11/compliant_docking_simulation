"""Run inside Blender to import the CAD-derived GLB and replay MuJoCo poses.

blender --background --python blender/build_scene.py
Output: blender/hexframe_module.blend
No physics is recomputed in Blender. Saved animation replays trajectory.json.
"""
import json,math
from pathlib import Path
import bpy
from mathutils import Vector,Quaternion
ROOT=Path(__file__).resolve().parents[1]

# This script intentionally creates a new scene; run it in a new Blender file.
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.gltf(filepath=str(ROOT/'blender/module.glb'))
imported=list(bpy.context.scene.objects)
root_a=bpy.data.objects.new('Module_A',None);bpy.context.collection.objects.link(root_a)
for obj in imported:
 if obj.parent is None:obj.parent=root_a
root_a.location=(0,0,.2)
# The GLB is in metres and Y-up; Blender's glTF importer restores Z-up.
bpy.context.scene.unit_settings.system='METRIC';bpy.context.scene.unit_settings.scale_length=1
root_b=bpy.data.objects.new('Module_B',None);bpy.context.collection.objects.link(root_b)
mapping={}
for obj in imported:
 new=obj.copy();new.data=obj.data;bpy.context.collection.objects.link(new);mapping[obj]=new
for old,new in mapping.items():new.parent=mapping.get(old.parent,root_b)
root_b.rotation_mode='QUATERNION'
meta=json.loads((ROOT/'model_info.json').read_text());traj=ROOT/'results/demo/trajectory.json'
scene=bpy.context.scene;scene.render.fps=30
if traj.exists():
 data=json.loads(traj.read_text())
 root_b.location=tuple(meta['nominal_pair_translation_m'][i]+([.04,0,.2][i]) for i in range(3));root_b.rotation_quaternion=Quaternion(meta['nominal_pair_quaternion_wxyz'])
 for item in data:
  frame=1+item['time_s']*scene.render.fps
  root_b.location=item['position_m'];root_b.rotation_quaternion=Quaternion(item['quaternion_wxyz'])
  root_b.keyframe_insert(data_path='location',frame=frame);root_b.keyframe_insert(data_path='rotation_quaternion',frame=frame)
 scene.frame_end=math.ceil(data[-1]['time_s']*scene.render.fps)+1
else:
 root_b.location=tuple(meta['nominal_pair_translation_m'][i]+([0,0,.2][i]) for i in range(3));root_b.rotation_quaternion=Quaternion(meta['nominal_pair_quaternion_wxyz'])
scene.frame_start=1;scene.frame_set(1)
# Studio setup; objects remain individually selectable and share mesh data.
bpy.ops.mesh.primitive_plane_add(size=4,location=(0,0,0));floor=bpy.context.object;floor.name='Studio_Floor'
mat=bpy.data.materials.new('Floor');mat.diffuse_color=(.7,.75,.82,1);floor.data.materials.append(mat)
for name,pos,power,size in [('Key',(0,-1,1.5),350,1.5),('Fill',(.5,1,1),200,1),('Rim',(-1,0,.8),180,.8)]:
 light=bpy.data.lights.new(name,'AREA');light.energy=power;light.shape='DISK';light.size=size
 obj=bpy.data.objects.new(name,light);bpy.context.collection.objects.link(obj);obj.location=pos;obj.rotation_euler=(Vector((.16,0,.2))-obj.location).to_track_quat('-Z','Y').to_euler()
camdata=bpy.data.cameras.new('Camera');cam=bpy.data.objects.new('Camera',camdata);bpy.context.collection.objects.link(cam);cam.location=(.8,-.9,.85);cam.rotation_euler=(Vector((.16,0,.2))-cam.location).to_track_quat('-Z','Y').to_euler();camdata.type='ORTHO';camdata.ortho_scale=.95;scene.camera=cam
scene.render.resolution_x=1600;scene.render.resolution_y=1000;scene.render.resolution_percentage=100
scene.world.color=(.2,.2,.2)
bpy.ops.wm.save_as_mainfile(filepath=str(ROOT/'blender/hexframe_module.blend'))
print('Saved Blender scene with CAD-derived meshes and MuJoCo pose replay.')
