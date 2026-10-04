"""Render actual MJCF geometry and replay recorded MuJoCo poses as a GIF."""
import os
os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('MPLCONFIGDIR','/tmp/hexframe_matplotlib')
import json
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'preview'

def font(n,bold=False):
 p=Path('/usr/share/fonts/truetype/dejavu')/('DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf')
 return ImageFont.truetype(str(p),n) if p.exists() else ImageFont.load_default()
def camera(target,distance,az=130,el=-30):
 c=mujoco.MjvCamera();c.lookat[:]=target;c.distance=distance;c.azimuth=az;c.elevation=el;return c

def render(m,d,c,w=1000,h=800):
 opt=mujoco.MjvOption();opt.geomgroup[0]=0;opt.geomgroup[3:]=0
 with mujoco.Renderer(m,width=w,height=h) as rr:
  rr.update_scene(d,camera=c,scene_option=opt);return Image.fromarray(rr.render())

def main():
 OUT.mkdir(exist_ok=True);meta=json.loads((ROOT/'model_info.json').read_text())
 m=mujoco.MjModel.from_xml_path(str(ROOT/'mjcf/single.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
 single=render(m,d,camera([0,0,.2],.67,130,-33),1100,800);single.save(OUT/'module.png')
 top=render(m,d,camera([0,0,.2],.65,90,-88),600,450);top.save(OUT/'top.png')
 # Exploded view of the six heads; base frame and mounting plates stay fixed.
 for k in range(6):
  n=np.array(meta['ports'][k]['normal'])
  for g in range(m.ngeom):
   if m.geom(g).name.startswith(f'a_port{k}_visual_'):m.geom_pos[g]+=.045*n
 mujoco.mj_forward(m,d);exploded=render(m,d,camera([0,0,.2],.82,130,-32),800,580);exploded.save(OUT/'exploded.png')
 pair=mujoco.MjModel.from_xml_path(str(ROOT/'mjcf/pair.xml'));pd=mujoco.MjData(pair);qa=int(pair.jnt_qposadr[pair.joint('module_b_free').id]);sep=meta['nominal_module_center_separation_m']
 pd.qpos[qa:qa+3]=np.array(meta['nominal_pair_translation_m'])+[0,0,.2];pd.qpos[qa+3:qa+7]=meta['nominal_pair_quaternion_wxyz'];mujoco.mj_forward(pair,pd)
 for g in range(pair.ngeom):
  if pair.geom(g).name.startswith('b_port') and '_visual_' in pair.geom(g).name and 'stop_land' not in pair.geom(g).name:pair.geom_matid[g]=-1;pair.geom_rgba[g]=[.84,.43,.13,1]
 paired=render(pair,pd,camera([sep/2,0,.2],1.0,105,-35),900,620);paired.save(OUT/'mated_pair.png')
 canvas=Image.new('RGB',(1800,1250),(240,244,249));draw=ImageDraw.Draw(canvas)
 draw.text((45,28),'HEXFRAME / PETALDOCK 100',font=font(42,True),fill=(27,47,65))
 draw.text((48,89),'Six-port rigid module | MIRROR-inspired concept | CAD + MuJoCo + Blender GLB',font=font(22),fill=(65,86,107))
 canvas.paste(single,(10,150));canvas.paste(top,(1170,150));canvas.paste(paired.resize((700,482)),(1090,625))
 draw.text((1190,155),'OPEN FRAME / TOP VIEW',font=font(19,True),fill=(27,47,65))
 draw.text((1120,620),'SIDE-TO-SIDE MATING',font=font(19,True),fill=(27,47,65))
 specs=[('OVERALL ENVELOPE','341 x 344 x 170 mm'),('PORTS','6 x OD 100 mm'),('ALUMINIUM MASS',f"{meta['body_mass_kg']:.2f} kg"),('MATING PITCH',f'{sep*1000:.1f} mm')]
 for i,(label,value) in enumerate(specs):
  x=35+i*445;draw.rounded_rectangle((x,1120,x+425,1230),radius=12,fill=(220,231,241));draw.text((x+18,1134),label,font=font(16,True),fill=(69,92,114));draw.text((x+18,1172),value,font=font(25,True),fill=(27,47,65))
 canvas.save(OUT/'hexframe_module_preview.png')
 if (ROOT/'mjcf/three_modules.xml').exists():
  cluster=mujoco.MjModel.from_xml_path(str(ROOT/'mjcf/three_modules.xml'));cd=mujoco.MjData(cluster);mujoco.mj_forward(cluster,cd)
  render(cluster,cd,camera([sep/2,sep/3,.2],1.2,115,-55),1400,1000).save(OUT/'three_modules.png')
 traj=ROOT/'results/demo/trajectory.json'
 if traj.exists():
  rows=json.loads(traj.read_text());opt=mujoco.MjvOption();opt.geomgroup[0]=0;opt.geomgroup[3:]=0;frames=[]
  lock_time=json.loads((ROOT/'results/demo/summary.json').read_text())['locked_at_s']
  with mujoco.Renderer(pair,width=800,height=600) as rr:
   for item in rows[::5]:
    pd.qpos[qa:qa+3]=item['position_m'];pd.qpos[qa+3:qa+7]=item['quaternion_wxyz'];mujoco.mj_forward(pair,pd)
    rr.update_scene(pd,camera=camera([sep/2,0,.2],1.0,105,-30),scene_option=opt);frame=Image.fromarray(rr.render());label=ImageDraw.Draw(frame)
    label.rounded_rectangle((15,15,785,78),radius=8,fill=(240,244,249))
    phase='LOCKED (abstract weld)' if lock_time is not None and item['time_s']>=lock_time else 'CONTACT-GUIDED APPROACH'
    label.text((28,23),f"t = {item['time_s']:.1f} s | {phase}",font=font(21,True),fill=(27,47,65));label.text((28,52),'Recorded MuJoCo trajectory | gravity = 0',font=font(16),fill=(65,86,107));frames.append(frame)
  frames[0].save(OUT/'module_docking.gif',save_all=True,append_images=frames[1:],duration=100,loop=0,optimize=True)
 print('Saved preview images and recorded-trajectory animation.')
if __name__=='__main__':main()
