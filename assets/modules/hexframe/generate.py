"""Parametric hexagonal open-frame module with PetalDock100 V2 ports.

Original MIRROR-inspired research geometry, not a reproduction or certified
HOTDOCK-compatible product. CAD uses mm; mesh, MJCF and GLB use metres.
"""
from __future__ import annotations
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import cadquery as cq
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

ROOT=Path(__file__).resolve().parent

def fmt(x):return ' '.join(f'{float(v):.12g}' for v in np.atleast_1d(x))
def quat(r):
 q=Rotation.from_matrix(r).as_quat();return np.r_[q[3],q[:3]]
def write_xml(root,path):
 ET.indent(root,space='  ');path.write_text(ET.tostring(root,encoding='unicode')+'\n')
def location(p,r):return cq.Location(cq.Plane(origin=tuple(p),xDir=tuple(r[:,0]),normal=tuple(r[:,2])))
def transform(p,r):
 t=np.eye(4);t[:3,:3]=r;t[:3,3]=np.array(p)/1000;return t

def axis_frame(z):
 z=np.array(z,dtype=float);z/=np.linalg.norm(z)
 ref=np.array([0.,0.,1.]) if abs(z[2])<.9 else np.array([1.,0.,0.])
 x=np.cross(ref,z);x/=np.linalg.norm(x);return np.column_stack([x,np.cross(z,x),z])

def shape_mesh(shape):
 vs,fs=shape.tessellate(.12,.15)
 mesh=trimesh.Trimesh([v.toTuple() for v in vs],fs,process=True);mesh.fix_normals();mesh.apply_scale(.001)
 return mesh

def solid_properties(s,density):
 return dict(mass=s.Volume()*1e-9*density,com=np.array(cq.Shape.centerOfMass(s).toTuple())*.001,
             inertia=np.array(cq.Shape.matrixOfInertia(s))*1e-15*density)

def combine(parts):
 mass=sum(p['mass'] for p in parts);com=sum(p['mass']*p['com'] for p in parts)/mass
 inertia=np.zeros((3,3))
 for p in parts:
  d=p['com']-com;inertia+=p['inertia']+p['mass']*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
 return mass,com,inertia

def geometry(p):
 R,H=p['frame_vertex_radius_mm'],p['frame_ring_separation_mm'];apothem=R*np.cos(np.pi/6)
 vertices=np.array([[R*np.cos(np.deg2rad(30+60*k)),R*np.sin(np.deg2rad(30+60*k)),z]
                     for z in [-H/2,H/2] for k in range(6)])
 edge_ids=[(j*6+k,j*6+(k+1)%6) for j in range(2) for k in range(6)]+[(k,k+6) for k in range(6)]
 beams=[];solids=[]
 def add_beam(a,b,width,name):
  a,b=np.array(a),np.array(b);r=axis_frame(b-a);length=np.linalg.norm(b-a)
  s=cq.Solid.makeBox(width,width,length,cq.Vector(-width/2,-width/2,0)).moved(location(a,r))
  solids.append(s);beams.append(dict(name=name,p=((a+b)/2/1000).tolist(),quat=quat(r).tolist(),size=[width/2000,width/2000,length/2000]))
 for i,(a,b) in enumerate(edge_ids):add_beam(vertices[a],vertices[b],p['beam_width_mm'],f'edge_{i:02}')
 ports=[];plate_mesh=None
 for k in range(6):
  theta=np.deg2rad(60*k);n=np.array([np.cos(theta),np.sin(theta),0]);u=np.array([-np.sin(theta),np.cos(theta),0]);v=np.array([0,0,1])
  frame=np.column_stack([u,v,n]);rot=frame@Rotation.from_euler('z',p['port_clocking_deg'],degrees=True).as_matrix();pos=apothem*n
  plate=cq.Workplane('XY').workplane(offset=p['mount_start_z_mm']).circle(p['mount_outer_diameter_mm']/2).circle(6).extrude(p['mount_thickness_mm']).val()
  for ang in [45,135,225,315]:
   x,y=41*np.array([np.cos(np.deg2rad(ang)),np.sin(np.deg2rad(ang))])
   bore=cq.Solid.makeCylinder(1.65,p['mount_thickness_mm']+2,cq.Vector(x,y,p['mount_start_z_mm']-1))
   plate=plate.cut(bore)
  solids.append(plate.moved(location(pos,rot)))
  for su in [-1,1]:
   for sv in [-1,1]:
    outer=pos+frame@np.array([su*R/2,sv*H/2,0])
    inner=pos+frame@np.array([su*35,sv*35,p['mount_start_z_mm']+p['mount_thickness_mm']/2-p['brace_width_mm']/2])
    add_beam(outer,inner,p['brace_width_mm'],f'brace_{k}_{su}_{sv}')
  if plate_mesh is None:plate_mesh=shape_mesh(plate)
  ports.append(dict(id=k,enabled=k in p['enabled_ports'],position_m=(pos/1000).tolist(),rotation_matrix=rot.tolist(),quaternion_wxyz=quat(rot).tolist(),normal=n.tolist()))
 print('Fusing frame and support plates...',flush=True)
 frame=solids[0].fuse(*solids[1:],tol=1e-5).clean()
 if not frame.isValid() or len(frame.Solids())!=1:raise ValueError('Frame must form one valid connected solid')
 return frame,ports,beams,plate,plate_mesh

def main():
 p=json.loads((ROOT/'config.json').read_text());interface=json.loads((ROOT/'interface/model_info.json').read_text());ip=interface['parameters']
 assert len(set(p['enabled_ports']))==len(p['enabled_ports']) and set(p['enabled_ports'])<=set(range(6))
 assert set(p['detailed_collision_ports'])<=set(p['enabled_ports'])
 if p['frame_vertex_radius_mm']<p['mount_outer_diameter_mm']+2*p['beam_width_mm']:raise ValueError('Side face too narrow for mounting plate')
 if p['frame_ring_separation_mm']<p['mount_outer_diameter_mm']+2*p['beam_width_mm']:raise ValueError('Frame too short for mounting plate')
 if abs(p['mount_start_z_mm']+p['mount_thickness_mm']-ip['adapter_thickness_mm'])>1e-8:raise ValueError('Mount must end at head base z=8 mm')
 if ip['outer_diameter_mm']!=100 or ip['head_bolt_pcd_mm']!=82:raise ValueError('This plate pattern requires PetalDock100 / PCD82')
 for sub in ['cad','meshes','mjcf','results','preview','blender']:(ROOT/sub).mkdir(exist_ok=True)
 frame,ports,beams,plate,plate_mesh=geometry(p)
 head=cq.importers.importStep(str(ROOT/'interface/cad/docking_head_mm.step')).val()
 if not head.isValid():raise ValueError('Invalid docking head CAD')
 cq.exporters.export(frame,str(ROOT/'cad/frame_with_mounts_mm.step'))
 cq.exporters.export(plate,str(ROOT/'cad/mount_plate_mm.step'))
 cq.exporters.export(frame,str(ROOT/'cad/frame_with_mounts_mm.stl'),tolerance=.15,angularTolerance=.15)
 assembly=cq.Assembly(name='HexFrameModule');assembly.add(frame,name='frame_with_mounts',color=cq.Color(.58,.63,.67))
 frame_mesh=shape_mesh(frame);frame_mesh.export(ROOT/'meshes/frame_visual.obj',include_normals=True)
 scene=trimesh.Scene();frame_mesh.visual.vertex_colors=[150,163,174,255];scene.add_geometry(frame_mesh,geom_name='frame',node_name='frame')
 properties=[solid_properties(frame,p['density_kg_m3'])];hp=solid_properties(head,p['density_kg_m3'])
 visual_names=['head_plate','stop_land']+[f'petal_{k}' for k in range(4)]
 for port in ports:
  if not port['enabled']:continue
  r=np.array(port['rotation_matrix']);pos=np.array(port['position_m']);k=port['id']
  assembly.add(head,name=f'dock_head_{k}',loc=location(pos*1000,r),color=cq.Color(.24,.46,.68))
  properties.append(dict(mass=hp['mass'],com=pos+r@hp['com'],inertia=r@hp['inertia']@r.T))
  for name in visual_names:
   m=trimesh.load(ROOT/f'interface/meshes/visual/{name}.obj',force='mesh',process=False)
   m.visual.vertex_colors=[177,187,197,255] if name=='stop_land' else [61,117,174,255]
   scene.add_geometry(m,node_name=f'port_{k}_{name}',geom_name=f'port_{k}_{name}',transform=transform(pos*1000,r))
 print('Exporting assembly...',flush=True);assembly.export(str(ROOT/'cad/module_assembly_mm.step'))
 # glTF uses Y up; convert from the engineering Z-up frame. Blender converts back.
 export_scene=scene.copy();tf=np.eye(4);tf[:3,:3]=Rotation.from_euler('x',-90,degrees=True).as_matrix();export_scene.apply_transform(tf)
 export_scene.export(ROOT/'blender/module.glb')
 mass,com,inertia=combine(properties)
 bb=scene.bounds
 meta=dict(name='HexFrameModule',version='1.0',parameters=p,interface_version=interface['version'],
           body_mass_kg=mass,body_com_m=com.tolist(),body_inertia_kg_m2=inertia.tolist(),
           frame_mass_kg=properties[0]['mass'],head_mass_kg=hp['mass'],ports=ports,
           bounds_m=bb.tolist(),overall_dimensions_mm=((bb[1]-bb[0])*1000).tolist(),
           frame_connected_solids=len(frame.Solids()),cad_assembly_parts=1+len(p['enabled_ports']),
           port_mating_site_offset_m=interface['mating_site_z_m'],port_pair_rotation_wxyz=interface['mating_relative_quaternion_wxyz'],
           nominal_module_center_separation_m=2*p['frame_vertex_radius_mm']*np.cos(np.pi/6)/1000+interface['nominal_flange_separation_m'],
           material='Aluminium density assumption; no screws, latch, wiring or equipment mass included',
           representation='One rigid body, fused-frame mass plus each mounted head; independent visual and collision geoms')
 # Compose mating transform from the two selected port frames.
 pa,pb=ports[0],ports[3]
 ra,rb=np.array(pa['rotation_matrix']),np.array(pb['rotation_matrix'])
 q=np.array(interface['mating_relative_quaternion_wxyz']);mate_r=Rotation.from_quat(np.r_[q[1:],q[0]]).as_matrix()
 body_r=ra@mate_r@rb.T
 body_t=np.array(pa['position_m'])+ra@np.array([0,0,interface['nominal_flange_separation_m']])-body_r@np.array(pb['position_m'])
 meta['nominal_pair_translation_m']=body_t.tolist();meta['nominal_pair_quaternion_wxyz']=quat(body_r).tolist()
 (ROOT/'model_info.json').write_text(json.dumps(meta,indent=2)+'\n')
 make_mjcf(meta,beams,visual_names,interface)
 print(json.dumps({k:meta[k] for k in ['body_mass_kg','frame_mass_kg','head_mass_kg','overall_dimensions_mm','nominal_module_center_separation_m']},indent=2))

def make_mjcf(meta,beams,visual_names,interface):
 p=meta['parameters'];assetdoc=ET.Element('mujocoinclude');defaults=ET.SubElement(assetdoc,'default')
 vd=ET.SubElement(defaults,'default',{'class':'hf_visual'});ET.SubElement(vd,'geom',contype='0',conaffinity='0',group='2',density='0')
 cd=ET.SubElement(defaults,'default',{'class':'hf_collision'});ET.SubElement(cd,'geom',contype='1',conaffinity='1',group='3',density='0',condim='3',friction='.15 .003 .0001',solref='.003 1',solimp='.95 .99 .0002',rgba='.8 .35 .1 .2')
 assets=ET.SubElement(assetdoc,'asset')
 for name,color in [('frame','.58 .64 .70 1'),('blue','.20 .40 .63 1'),('silver','.67 .73 .80 1')]:ET.SubElement(assets,'material',name=f'hf_{name}',rgba=color,specular='.4',shininess='.3')
 ET.SubElement(assets,'mesh',name='hf_frame_visual',file='../meshes/frame_visual.obj')
 for name in visual_names:ET.SubElement(assets,'mesh',name=f'hf_v_{name}',file=f'../interface/meshes/visual/{name}.obj')
 nc=interface['guide_convex_cells']+interface['stop_convex_cells']
 for i in range(nc):ET.SubElement(assets,'mesh',name=f'hf_c_{i:03}',file=f'../interface/meshes/collision/cell_{i:03}.obj')
 # Head plate proxy starts at z=8; the iiwa adapter and pilot are absent.
 for name,r,z0,z1 in [('head_base',50,8,14),('mount',p['mount_outer_diameter_mm']/2,p['mount_start_z_mm'],p['mount_start_z_mm']+p['mount_thickness_mm'])]:
  m=trimesh.creation.cylinder(radius=r/1000,height=(z1-z0)/1000,sections=64);m.apply_translation([0,0,(z1+z0)/2000]);m.export(ROOT/f'meshes/{name}_proxy.obj')
  ET.SubElement(assets,'mesh',name=f'hf_{name}_proxy',file=f'../meshes/{name}_proxy.obj')
 write_xml(assetdoc,ROOT/'mjcf/assets.xml')
 def contents(prefix):
  node=ET.Element('mujocoinclude');I=np.array(meta['body_inertia_kg_m2'])
  ET.SubElement(node,'inertial',mass=fmt(meta['body_mass_kg']),pos=fmt(meta['body_com_m']),fullinertia=fmt([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
  ET.SubElement(node,'geom',name=f'{prefix}_frame_visual',type='mesh',mesh='hf_frame_visual',material='hf_frame',**{'class':'hf_visual'})
  for b in beams:ET.SubElement(node,'geom',name=f'{prefix}_{b["name"]}',type='box',pos=fmt(b['p']),quat=fmt(b['quat']),size=fmt(b['size']),**{'class':'hf_collision'})
  for port in meta['ports']:
   k=port['id'];base=dict(pos=fmt(port['position_m']),quat=fmt(port['quaternion_wxyz']))
   ET.SubElement(node,'geom',name=f'{prefix}_port{k}_mount',type='mesh',mesh='hf_mount_proxy',**base,**{'class':'hf_collision'})
   if not port['enabled']:continue
   for name in visual_names:ET.SubElement(node,'geom',name=f'{prefix}_port{k}_visual_{name}',type='mesh',mesh=f'hf_v_{name}',material='hf_silver' if name=='stop_land' else 'hf_blue',**base,**{'class':'hf_visual'})
   ET.SubElement(node,'geom',name=f'{prefix}_port{k}_head_base',type='mesh',mesh='hf_head_base_proxy',**base,**{'class':'hf_collision'})
   if k in p['detailed_collision_ports']:
    for i in range(nc):
     kind='guide' if i<interface['guide_convex_cells'] else 'stop'
     ET.SubElement(node,'geom',name=f'{prefix}_port{k}_{kind}_{i:03}',type='mesh',mesh=f'hf_c_{i:03}',**base,**{'class':'hf_collision'})
   else:
    # Coarse full-envelope proxy; this port cannot be used for insertion.
    pos=np.array(port['position_m'])+np.array(port['rotation_matrix'])@np.array([0,0,.023])
    ET.SubElement(node,'geom',name=f'{prefix}_port{k}_coarse',type='cylinder',size='.05 .009',pos=fmt(pos),quat=base['quat'],**{'class':'hf_collision'})
   for tag,z in [('mount',0),('mating',meta['port_mating_site_offset_m'])]:
    pos=np.array(port['position_m'])+np.array(port['rotation_matrix'])@np.array([0,0,z])
    ET.SubElement(node,'site',name=f'{prefix}_port{k}_{tag}_site',pos=fmt(pos),quat=base['quat'],size='.003',group='4',rgba='1 .5 .05 1')
  return node
 for prefix in ['a','b']:write_xml(contents(prefix),ROOT/f'mjcf/{prefix}_contents.xml')
 def scene(pair=False):
  root=ET.Element('mujoco',model='HexFrameModule_pair' if pair else 'HexFrameModule_single')
  ET.SubElement(root,'compiler',angle='radian',autolimits='true')
  ET.SubElement(root,'option',timestep='.0005',gravity='0 0 0',integrator='implicitfast',solver='Newton',iterations='60',cone='elliptic',tolerance='1e-10')
  ET.SubElement(root,'include',file='assets.xml')
  sky=ET.SubElement(root,'asset');ET.SubElement(sky,'texture',name='hf_sky',type='skybox',builtin='gradient',rgb1='.94 .96 .99',rgb2='.8 .86 .93',width='256',height='1536')
  visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='1800',offheight='1200');ET.SubElement(visual,'headlight',ambient='.5 .5 .5',diffuse='.6 .6 .6',specular='.2 .2 .2')
  ET.SubElement(root,'statistic',center=fmt([meta['nominal_module_center_separation_m']/2 if pair else 0,0,.2]),extent='.9' if pair else '.45')
  world=ET.SubElement(root,'worldbody');ET.SubElement(world,'light',pos='0 -.5 1',dir='0 .3 -1',diffuse='.7 .7 .7')
  ET.SubElement(world,'geom',type='plane',size='1 1 .01',rgba='.94 .95 .97 1',contype='0',conaffinity='0',group='0')
  a=ET.SubElement(world,'body',name='module_a',pos='0 0 .2');ET.SubElement(a,'include',file='a_contents.xml')
  if pair:
   sep=meta['nominal_module_center_separation_m'];b=ET.SubElement(world,'body',name='module_b',pos=fmt(np.array(meta['nominal_pair_translation_m'])+[.04,0,.2]),quat=fmt(meta['nominal_pair_quaternion_wxyz']));ET.SubElement(b,'freejoint',name='module_b_free');ET.SubElement(b,'include',file='b_contents.xml')
   eq=ET.SubElement(root,'equality');ET.SubElement(eq,'weld',name='module_lock',body1='module_a',body2='module_b',active='false',relpose=fmt(meta['nominal_pair_translation_m']+meta['nominal_pair_quaternion_wxyz']),solref='.003 1',solimp='.95 .99 .0002',torquescale='.15')
  return root
 write_xml(scene(False),ROOT/'mjcf/single.xml');write_xml(scene(True),ROOT/'mjcf/pair.xml')
 # A three-module lattice layout checks compatibility around a closed loop.
 if {0,1,2,3,4,5} <= set(p['enabled_ports']) and abs(p['port_clocking_deg']-67.5)<1e-8:
  write_xml(contents('c'),ROOT/'mjcf/c_contents.xml')
  cluster=scene(False);cluster.set('model','HexFrameModule_three_module_layout');world=cluster.find('worldbody')
  pitch=meta['nominal_module_center_separation_m']
  for label,pos in [('b',[pitch,0,.2]),('c',[pitch/2,pitch*np.sqrt(3)/2,.2])]:
   b=ET.SubElement(world,'body',name=f'module_{label}',pos=fmt(pos));ET.SubElement(b,'freejoint',name=f'module_{label}_free');ET.SubElement(b,'include',file=f'{label}_contents.xml')
  cluster.find('statistic').set('center',fmt([pitch/2,pitch/3,.2]));cluster.find('statistic').set('extent','1.0')
  write_xml(cluster,ROOT/'mjcf/three_modules.xml')
 else:
  for filename in ['three_modules.xml','c_contents.xml']:(ROOT/'mjcf'/filename).unlink(missing_ok=True)

if __name__=='__main__':main()
