"""Imported FR3 + native Hand model screen, not an actual SDK or hardware audit."""
from pathlib import Path
import json,sys,os,xml.etree.ElementTree as E
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;PORT=HERE.parent
source=PORT/'native_hand_feasibility/rendered_poses/native-hand-scene.xml'
root=E.parse(source).getroot()
for side in ['left','right']:
 body=root.find(f".//body[@name='{side}_native_hand']")
 E.SubElement(body,'inertial',mass='.7',pos='0 .01 .03',diaginertia='.001 .0025 .0017')
# All moving arm/finger bodies have explicit imported inertials. Static CAD and
# visual meshes add no arm dynamics; omit them from inverse-dynamics evaluation.
world=root.find('worldbody')
for child in list(world):
 if child.tag!='body'or child.get('name')!='official_FR3_Duo':world.remove(child)
for parent in root.iter():
 for child in list(parent):
  if child.tag in('geom','site','camera','light'):parent.remove(child)
asset=root.find('asset')
if asset is not None:root.remove(asset)
path=HERE/'torque-native-model.xml';xml=E.tostring(root,encoding='unicode');temporary=path.with_suffix('.'+str(os.getpid())+'.tmp');temporary.write_text(xml);temporary.replace(path);m=mujoco.MjModel.from_xml_string(xml);m.body_gravcomp[:]=0;m.dof_frictionloss[:]=0
m.geom_contype[:]=0;m.geom_conaffinity[:]=0
m.opt.disableflags|=int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)
d=mujoco.MjData(m);AD={};VD={}
for side in ['left','right']:
 joints=[m.joint(f'{side}_arm_fr3v2_joint{j}').id for j in range(1,8)];AD[side]=m.jnt_qposadr[joints];VD[side]=m.jnt_dofadr[joints];d.qpos[AD[side]]=[0,-.785398,0,-2.356194,0,1.570796,.785398]
parts=json.load(open(PORT/'generated/contact_task/contact-source.json'))['parts'];record=json.load(open(HERE/'comparison-physical-band.json'));cache={};reports=[]

def screen(seg,side,recipe):
 a=np.array(seg['a']);b=np.array(seg['b']);original=seg['duration_s'];delta=b-a
 if np.max(abs(delta))<1e-10:return original,None
 holding=seg['label']in ['clear_datum','extract','lift','transfer','insert','clamp'];hand=m.body(side+'_native_hand').id
 # Approximate rigid attached payload, CAD-derived masses. Tool inertia below is
 # a documented approximation, not the manufacturer's calibrated load model.
 m.body_mass[hand]=.7;m.body_ipos[hand]=[0,.01,.03];m.body_inertia[hand]=[.001,.0025,.0017]
 if holding:
  mass=parts[recipe['part']]['mass'];m.body_mass[hand]+=(mass);m.body_ipos[hand]=(.7*np.array([0,.01,.03])+mass*(np.array([0,0,.1034])-np.array(recipe['localR']).T@np.array(recipe['localP'])))/(.7+mass)
  m.body_inertia[hand]+=mass*.10**2
 u=np.linspace(0,1,401);h=u**3*(10-15*u+6*u*u);h1=30*u*u*(1-u)**2;h2=60*u*(1-u)*(1-2*u);torque=[];dynamic=[]
 for phase,v,acc in zip(h,h1,h2):
  d.qpos[AD[side]]=a+delta*phase;d.qvel[:]=0;d.qacc[:]=0;mujoco.mj_inverse(m,d);gravity=d.qfrc_inverse[VD[side]].copy()
  d.qvel[VD[side]]=delta*v/original;d.qacc[VD[side]]=delta*acc/original**2;mujoco.mj_inverse(m,d);tau=d.qfrc_inverse[VD[side]].copy();torque.append(tau);dynamic.append(tau-gravity)
 torque=np.array(torque);dynamic=np.array(dynamic);dt=original/400;rate=float(np.max(abs(np.diff(dynamic,axis=0)/dt)));peak=np.max(abs(torque),axis=0)
 factor=max(1.,(rate/950)**(1/3));limits=np.array([87]*4+[12]*3)*.95
 gravity=torque-dynamic
 for _ in range(50):
  if np.all(np.max(abs(gravity+dynamic/factor**2),axis=0)<=limits):break
  factor*=1.05
 if factor>10:
  print('MODEL_STATIC_WARNING',side,recipe['part'],seg['label'],np.max(abs(gravity),axis=0),flush=True)
  factor=max(1.,(rate/950)**(1/3))
  static_warning=True
 else:static_warning=False
 return original*factor*1.01,dict(original_s=original,retimed_s=original*factor*1.01,initial_model_without_gravity_peak_rate_nm_s=rate,initial_model_peak_torque_nm=peak.tolist(),retimed_model_without_gravity_peak_rate_nm_s=rate/(factor*1.01)**3,payload_included=holding,static_gravity_exceeds_95percent_budget=static_warning,static_gravity_peak_nm=np.max(abs(gravity),axis=0).tolist())

def main():
 for layout in record['layouts']:
  if len(layout.get('recipes',{}))!=3:continue
  for name,recipe in layout['recipes'].items():
   traces=[]
   for seg in recipe['segments']:
    new,r=screen(seg,recipe['side'],recipe);seg['duration_s']=new
    if r:traces.append(dict(label=seg['label'],**r))
   recipe['duration_s']=sum(s['duration_s']for s in recipe['segments']);reports.append(dict(layout=layout['label'],part=name,model_traces=traces));print('MODEL_RETIMED',layout['label'],name,recipe['duration_s'],flush=True)
 (HERE/'comparison-model-screened.json').write_text(json.dumps(record,indent=2));(HERE/'model-torque-retiming.json').write_text(json.dumps(dict(model='Imported MuJoCo FR3 + assumed native Hand/payload inertia',official_native_hand_total_mass_kg=.73,tool_inertia_approximate=True,real_sdk_model_tested=False,hardware_ready=False,results=reports),indent=2))
if __name__=='__main__':main()
