"""Rigid module docking bench. Contact first, optional pose-gated abstract weld.

No qpos writes after initialization. The module is force-driven, not an iiwa
robot or free-flying spacecraft controller. Gravity is zero.
"""
from __future__ import annotations
import argparse,csv,json,time
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent

def rot(q):return Rotation.from_quat(np.r_[q[1:],q[0]])
def quat(r):
 q=r.as_quat();return np.r_[q[3],q[:3]]
def cap(v,limit):return v*min(1,limit/max(np.linalg.norm(v),1e-15))

class ModuleDemo:
 def __init__(self,offset_y_mm=2,offset_z_mm=-1,twist_deg=2,tilt_deg=1,lock=True,guide_only=False):
  self.info=json.loads((ROOT/'model_info.json').read_text());p=self.info['parameters']
  if not {0,3}<=set(p['detailed_collision_ports']):raise ValueError('Pair bench needs detailed ports 0 and 3')
  self.model=m=mujoco.MjModel.from_xml_path(str(ROOT/'mjcf/pair.xml'));self.data=d=mujoco.MjData(m)
  self.a=m.body('module_a').id;self.b=m.body('module_b').id;self.eq=m.equality('module_lock').id
  self.qa=int(m.jnt_qposadr[m.joint('module_b_free').id]);self.lock_allowed=lock;self.guide_only=guide_only
  self.locked_at=None;self.first_contact=None;self.dwell=0.;self.rows=[];self.trajectory=[]
  mujoco.mj_forward(m,d);self.goal,self.goal_r=self.target();self.start_x=float(d.qpos[self.qa])
  d.qpos[self.qa+1:self.qa+3]+=np.array([offset_y_mm,offset_z_mm])/1000
  d.qpos[self.qa+3:self.qa+7]=quat(Rotation.from_euler('xy',[twist_deg,tilt_deg],degrees=True)*self.goal_r)
  mujoco.mj_forward(m,d)
  self.jp=np.zeros((3,m.nv));self.jr=np.zeros((3,m.nv))
  self.initial=dict(offset_y_mm=offset_y_mm,offset_z_mm=offset_z_mm,twist_deg=twist_deg,tilt_deg=tilt_deg)
 def target(self):
  ra=Rotation.from_matrix(self.data.xmat[self.a].reshape(3,3))
  return self.data.xpos[self.a]+ra.apply(self.info['nominal_pair_translation_m']),ra*rot(np.array(self.info['nominal_pair_quaternion_wxyz']))
 def error(self):
  goal,rg=self.target();rc=Rotation.from_matrix(self.data.xmat[self.b].reshape(3,3))
  return goal-self.data.xpos[self.b],(rg*rc.inv()).as_rotvec()
 def seating(self):
  m=self.model
  return any((m.geom(c.geom1).name.startswith('a_port0_stop_') and m.geom(c.geom2).name.startswith('b_port3_stop_') or
              m.geom(c.geom2).name.startswith('a_port0_stop_') and m.geom(c.geom1).name.startswith('b_port3_stop_')) and c.dist<=1e-6 for c in self.data.contact)
 def state(self):
  ep,er=self.error();force=np.zeros(6);total=0.;penetration=0.
  for i,c in enumerate(self.data.contact):
   mujoco.mj_contactForce(self.model,self.data,i,force);total+=max(0,float(force[0]));penetration=max(penetration,-float(c.dist))
  return dict(time_s=float(self.data.time),axial_error_mm=float(ep[0]*1000),y_error_mm=float(ep[1]*1000),z_error_mm=float(ep[2]*1000),
              lateral_error_mm=float(np.linalg.norm(ep[1:])*1000),position_error_mm=float(np.linalg.norm(ep)*1000),angle_error_deg=float(np.rad2deg(np.linalg.norm(er))),
              contacts=int(self.data.ncon),normal_force_n=total,penetration_mm=penetration*1000,seating=self.seating(),locked=bool(self.data.eq_active[self.eq]))
 def step(self):
  m,d=self.model,self.data;ep,er=self.error();mujoco.mj_jacBodyCom(m,d,self.jp,self.jr,self.b);v=self.jp@d.qvel;w=self.jr@d.qvel
  if d.ncon and self.first_contact is None:self.first_contact=self.state()
  d.xfrc_applied[:]=0
  if not d.eq_active[self.eq]:
   enabled=not self.guide_only and self.first_contact is not None
   xref=max(self.goal[0]-.0002,self.start_x-.010*d.time)
   f=np.array([1500*(xref-d.xpos[self.b,0]),(150 if enabled else 0)*ep[1],(150 if enabled else 0)*ep[2]])-np.array([140,35,35])*v
   torque=(1.5 if enabled else 0)*er-.35*w
   d.xfrc_applied[self.b,:3]=cap(f,12);d.xfrc_applied[self.b,3:]=cap(torque,.5)
   ready=np.linalg.norm(ep)<.0007 and abs(ep[0])<.0003 and np.linalg.norm(er)<np.deg2rad(1) and np.linalg.norm(v)<.003 and np.linalg.norm(w)<.05 and self.seating()
   self.dwell=self.dwell+m.opt.timestep if ready else 0
   if self.lock_allowed and self.dwell>=.1:d.eq_active[self.eq]=True;self.locked_at=float(d.time);d.xfrc_applied[self.b]=0
  elif self.locked_at is not None and d.time>self.locked_at+.7:d.xfrc_applied[self.b,:3]=[2,.5,0]
  mujoco.mj_step(m,d)
  if not np.isfinite(d.qpos).all():raise RuntimeError('Non-finite simulation state')
  if round(d.time/m.opt.timestep)%40==0:
   self.rows.append(self.state());self.trajectory.append(dict(time_s=float(d.time),position_m=d.qpos[self.qa:self.qa+3].tolist(),quaternion_wxyz=d.qpos[self.qa+3:self.qa+7].tolist()))
 def report(self):
  mujoco.mj_forward(self.model,self.data)
  return dict(mujoco_version=mujoco.__version__,initial=self.initial,guide_only=self.guide_only,lock_enabled=self.lock_allowed,first_contact=self.first_contact,
              locked_at_s=self.locked_at,final=self.state(),warnings={str(mujoco.mjtWarning(i)):int(w.number) for i,w in enumerate(self.data.warning) if w.number},
              sampled_peak_force_n=max((r['normal_force_n'] for r in self.rows),default=0),sampled_peak_penetration_mm=max((r['penetration_mm'] for r in self.rows),default=0),
              controller=dict(approach_speed_m_s=.01,axial_stiffness_n_m=1500,lateral_stiffness_n_m=0 if self.guide_only else 150,rotational_stiffness_nm_rad=0 if self.guide_only else 1.5,
                              linear_damping_ns_m=[140,35,35],angular_damping_nms_rad=.35,force_cap_n=12,torque_cap_nm=.5),
              note='Rigid module bench. No spacecraft control, flexibility, physical latch or calibrated capture envelope.')
 def save(self,out):
  out=Path(out);out.mkdir(parents=True,exist_ok=True);r=self.report();(out/'summary.json').write_text(json.dumps(r,indent=2)+'\n')
  (out/'trajectory.json').write_text(json.dumps(self.trajectory)+'\n')
  if self.rows:
   with (out/'log.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(self.rows[0]));w.writeheader();w.writerows(self.rows)
  return r

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--headless',action='store_true');ap.add_argument('--seconds',type=float,default=9)
 ap.add_argument('--offset-y-mm',type=float,default=2);ap.add_argument('--offset-z-mm',type=float,default=-1);ap.add_argument('--twist-deg',type=float,default=2);ap.add_argument('--tilt-deg',type=float,default=1)
 ap.add_argument('--no-lock',action='store_true');ap.add_argument('--guide-only',action='store_true');ap.add_argument('--output',type=Path,default=ROOT/'results/demo');a=ap.parse_args()
 s=ModuleDemo(a.offset_y_mm,a.offset_z_mm,a.twist_deg,a.tilt_deg,not a.no_lock,a.guide_only)
 if a.headless:
  while s.data.time<a.seconds:s.step()
 else:
  import mujoco.viewer
  with mujoco.viewer.launch_passive(s.model,s.data) as viewer:
   viewer.cam.lookat[:]=[s.goal[0]/2,0,.2];viewer.cam.distance=1.2;viewer.cam.azimuth=110;viewer.cam.elevation=-28;viewer.opt.geomgroup[3:]=0
   start=time.monotonic()
   while viewer.is_running() and s.data.time<a.seconds:
    s.step()
    if round(s.data.time/s.model.opt.timestep)%40==0:
     viewer.sync();ahead=s.data.time-(time.monotonic()-start)
     if ahead>0:time.sleep(min(ahead,.02))
 print(json.dumps(s.save(a.output),indent=2))
if __name__=='__main__':main()
