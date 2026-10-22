"""Native Hand / spring button contact bench; no arm dynamics or full cell replay."""
from pathlib import Path
import json,xml.etree.ElementTree as E
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;PORT=HERE.parent

def vec(x):return ' '.join(map(str,np.asarray(x).ravel()))
def quat(R):return vec(Rotation.from_matrix(R).as_quat()[[3,0,1,2]])
plan=json.load(open(HERE/'active-fixture-plan.json'));q=plan['button_finish'][0]['b'];full=mujoco.MjModel.from_xml_path(str(PORT/'native_hand_feasibility/rendered_poses/native-hand-scene.xml'));fd=mujoco.MjData(full)
for j,v in enumerate(q,1):fd.qpos[full.joint(f'right_arm_fr3v2_joint{j}').qposadr[0]]=v
mujoco.mj_kinematics(full,fd);h=full.body('right_native_hand').id;R=fd.xmat[h].reshape(3,3).copy();tcp=fd.xpos[h]+R@np.array([0,0,.1034]);origin=tcp-R@np.array([0,0,.1034]);pre=origin-[0,.04,0]
root=E.parse(HERE/'best-recipe-contact-ring1/ring1-bench.xml').getroot();world=root.find('worldbody')
for node in list(world):
 if node.tag!='body' or node.get('name')not in ('hand_target','hand_base'):world.remove(node)
for name in ('hand_target','hand_base'):
 b=root.find(f".//body[@name='{name}']");b.set('pos',vec(pre));b.set('quat',quat(R))
button=E.SubElement(world,'body',name='button',pos='.3559690421575 -.24725 1.527');E.SubElement(button,'inertial',mass='.03',pos='0 0 0',diaginertia='.000002 .000002 .000002');E.SubElement(button,'joint',name='button_slide',type='slide',axis='0 1 0',range='0 .006',stiffness='1000',springref='0',damping='.5',solreflimit='.002 1');E.SubElement(button,'geom',name='button_face',type='cylinder',size='.018 .002',quat=quat(Rotation.from_euler('x',90,degrees=True).as_matrix()),contype='1',conaffinity='2',rgba='.8 .12 .08 1')
path=HERE/'native-button-bench.xml';E.ElementTree(root).write(path,encoding='unicode');m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m);mid=m.body('hand_target').mocapid[0];adr=m.joint('button_slide').qposadr[0];samples=[];forces=[];dwell=0.;maxdwell=0.
for k in range(2300):
 t=k*.001
 def smooth(u):u=np.clip(u,0,1);return u**3*(10-15*u+6*u*u)
 shift=.04*smooth((t-.2)/.65)if t<1.2 else .04*(1-smooth((t-1.2)/.6));d.mocap_pos[mid]=pre+[0,shift,0];mujoco.mj_step(m,d);travel=float(d.qpos[adr]);normal=0
 # q spring force is a conservative quasi-static force report; dynamic contact
 # normal is captured separately using the indexed contact API below.
 for ci in range(d.ncon):
  c=d.contact[ci]
  if m.geom(c.geom1).name=='button_face' or m.geom(c.geom2).name=='button_face':
   f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);normal+=abs(f[0])
 forces.append(normal);dwell=dwell+.001 if travel>=.0018 else 0;maxdwell=max(maxdwell,dwell)
 if k%10==0:samples.append(dict(time_s=t,travel_m=travel,normal_force_n=normal))
r=dict(physics='MuJoCo 1ms contact bench, standard native tips and spring-loaded button',arm_dynamics_simulated=False,full_cell_assembly_simulated=False,spring_n_m=1000,maximum_travel_m=max(s['travel_m']for s in samples),verified_dwell_s=maxdwell,maximum_normal_force_n=max(forces),final_travel_m=float(d.qpos[adr]),passed=bool(maxdwell>=.1 and max(forces)<5 and abs(d.qpos[adr])<.0002),samples=samples)
(HERE/'native-button-contact-result.json').write_text(json.dumps(r,indent=2));print({k:v for k,v in r.items()if k!='samples'},flush=True)
