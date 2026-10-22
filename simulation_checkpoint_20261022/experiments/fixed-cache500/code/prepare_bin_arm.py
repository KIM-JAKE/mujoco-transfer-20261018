"""Prepare the first full-arm contact test from the unchanged existing basket."""
from pathlib import Path
import sys,json
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parent)]
import screen_bin_reach as env
from roas_kinematics import solve
cc=env.cc;OUT=HERE/'bin-picking'
r=json.load(open(OUT/'reach-screen.json'));basket=r['baskets'][0];cad=env.assets['baskets'][0];B=np.array(cad['basket_to_world_R']);O=np.array(cad['floor_origin_world_m'])
request_path=OUT/'arm-request.json';request=json.load(open(request_path))if request_path.exists()else{}
part=request.get('part','ring1');side='right';pin=part=='pin';width=.022 if pin else .08
Rp=np.eye(3)if pin else B
P=np.array([.230,-.790,.8843+.018-.009+.01775])if pin else O+B@np.array([-.06,request.get('source_y',0.),.00775 if part=='ring1'else .007])
offset=request.get('grasp_z_offset',.006 if pin else .003)
tcp=P+Rp@np.array([0,0,offset]);R=Rp@np.diag([1.,-1.,-1.]);seeds=[q for t in basket['trials']for q in t['clear_ik']if t['side']==side and t['yaw']==0]
seeds+=[cc.READY[side].tolist()]
height=request.get('lift_m',.05 if pin else .145)
attempts=[];solution=None
for seed in seeds:
    pick=solve(side,tcp,R,seed)
    if pick is None:continue
    path=[pick];valid=True
    for h in np.linspace(.002,height,max(3,int(height/.002))):
        q=solve(side,tcp+[0,0,h],R,path[-1])
        if q is None:attempts.append(dict(failed_height=float(h),reason='IK'));valid=False;break
        ok,why=env.fe.clear(side,q,width)
        if not ok:attempts.append(dict(failed_height=float(h),reason=why));valid=False;break
        path.append(q)
    if valid:solution=path;break
if solution is None:
    (OUT/'arm-preparation-failure.json').write_text(json.dumps(dict(state='no_full_arm_extraction_path',attempts=attempts,full_task_pass=False),indent=2));raise RuntimeError('No extraction path: '+str(attempts))
ready={s:cc.READY[s].tolist()for s in ['left','right']};ready[side]=solution[-1].tolist()
previous=json.load(open(HERE.parents[1]/'runs/native-roas-table-03/executed-pickup-plan.json'))
ready['left']=previous['ready_q']['left']
ok,why=env.fe.clear('left',np.array(ready['left']),.08)
if not ok:raise RuntimeError('Parked left arm fails collision screen: '+str(why))
if not all(cc.arm_pair_clear((np.array(ready['left']),.08),(q,width))for q in solution):raise RuntimeError('Parked left arm intersects the right pickup path')
plan=dict(scope='Full articulated FR3/native Hand free-part pickup from original CAD supply; initial diagnostic, no full task timing',parts={part:dict(side=side,recipe=dict(gap=.013 if pin else .075 if part=='ring1'else .062,localP=[0,0,offset],localR=np.diag([1.,-1.,-1.]).tolist()),cartesian_lift_q=[x.tolist()for x in solution],nominal_loaded_clear=None,source_P=P.tolist(),source_R=Rp.tolist())},ready_q=ready,initial_apertures_m={'left':.08,'right':width},lift_m=height,diagnostic_descend_duration_s=4.,diagnostic_lift_duration_s=5.,grasp_command_compression_m=.008,proposed_pin_holder=pin)
(OUT/'arm-plan.json').write_text(json.dumps(plan,indent=2))
m=cc.mujoco.MjModel.from_xml_path(str(HERE.parent/'torque-native-model.xml'));d=cc.mujoco.MjData(m);m.body_pos[m.body('official_FR3_Duo').id]=env.spec['duo_world_position']
for s in ready:
    for j,v in enumerate(ready[s],1):d.qpos[m.jnt_qposadr[m.joint(f'{s}_arm_fr3v2_joint{j}').id]]=v
    for label in ['left','right']:d.qpos[m.jnt_qposadr[m.joint(f'{s}_native_finger_{label}').id]]=plan['initial_apertures_m'][s]/2
cc.mujoco.mj_kinematics(m,d)
poses={m.body(i).name:dict(position=d.xpos[i].tolist(),quaternion=d.xquat[i].tolist())for i in range(1,m.nbody)}
(OUT/'initial-robot-pose.json').write_text(json.dumps(dict(ready_q=ready,body_world_poses=poses),indent=2))
print('BIN_ARM_PREPARED',len(solution),flush=True)
