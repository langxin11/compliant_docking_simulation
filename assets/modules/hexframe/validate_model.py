"""Validate module pose conventions, empty frame space and mating collisions."""
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from demo import ModuleDemo
ROOT=Path(__file__).resolve().parent

def validate(out=None):
 out=Path(out) if out is not None else ROOT.parents[2]/'runs/hexframe_module/results';out.mkdir(parents=True,exist_ok=True)
 info=json.loads((ROOT/'model_info.json').read_text());m=mujoco.MjModel.from_xml_path(str(ROOT/'mjcf/single.xml'))
 assert abs(m.body_mass[m.body('module_a').id]-info['body_mass_kg'])<1e-8
 assert np.linalg.eigvalsh(info['body_inertia_kg_m2']).min()>0
 s=ModuleDemo(0,0,0,0,lock=False);goal,gr=s.target();q=gr.as_quat();wxyz=np.r_[q[3],q[:3]]
 def probe(gap):
  s.data.qpos[s.qa:s.qa+3]=goal+[gap/1000,0,0];s.data.qpos[s.qa+3:s.qa+7]=wxyz;s.data.qvel[:]=0;mujoco.mj_forward(s.model,s.data)
  return s.state()
 separated=probe(30);nominal=probe(0);overtravel=probe(-.1)
 assert separated['contacts']==0,separated
 assert nominal['penetration_mm']<.02,nominal
 assert .05<overtravel['penetration_mm']<.2,overtravel
 probe(0)
 a=s.model.site('a_port0_mating_site').id;b=s.model.site('b_port3_mating_site').id
 pe=float(np.linalg.norm(s.data.site_xpos[a]-s.data.site_xpos[b]))
 ra=s.data.site_xmat[a].reshape(3,3);rb=s.data.site_xmat[b].reshape(3,3)
 q=np.array(info['port_pair_rotation_wxyz']);target=Rotation.from_quat(np.r_[q[1:],q[0]]).as_matrix()
 re=float(np.linalg.norm(ra.T@rb-target))
 assert pe<1e-8 and re<1e-8
 # The module centre is deliberately empty. A centre ray along +Z must pass
 # through the open top/bottom rather than hit a full-frame convex hull.
 d=mujoco.MjData(m);mujoco.mj_forward(m,d);group=np.zeros(6,dtype=np.uint8);group[3]=1;gid=np.array([-1],dtype=np.int32)
 distance=float(mujoco.mj_ray(m,d,np.array([0.,0.,.2]),np.array([0.,0.,1.]),group,True,-1,gid))
 assert distance<0,('Centre opening blocked',distance,gid)
 result=dict(mujoco_version=mujoco.__version__,body_mass_kg=info['body_mass_kg'],single_geoms=int(m.ngeom),pair_geoms=int(s.model.ngeom),
             port_mating_position_error_m=pe,port_mating_rotation_matrix_error=re,empty_centre_ray_distance=distance,
             separated=separated,nominal=nominal,overtravel_01mm=overtravel)
 (out/'geometry_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path);validate(ap.parse_args().out)
