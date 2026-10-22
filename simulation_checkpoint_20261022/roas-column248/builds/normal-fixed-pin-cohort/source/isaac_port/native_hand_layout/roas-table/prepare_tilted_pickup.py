"""Search controlled tilt while extracting the ring over its wide rod hole.

Preserves the successful original grasp relative to the ring, so the original
insertion contact band is retained. This is CAD screening, not a PhysX pass.
"""
from pathlib import Path
import os,sys,json,copy
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE));os.environ['M80_IMPORT_ENV_ONLY']='1'
import prepare_collision_targets as e
from roas_kinematics import solve,alternatives
from audit_five_geometry import carrying
raw=json.load(open(HERE/'collision-global-centered.json'))['rows']
previous=json.load(open(HERE.parents[1]/'runs/native-roas-table-03/executed-pickup-plan.json'))
out=copy.deepcopy(previous);out.update(scope='Original insertion grasp preserved; CAD-screened tilted ring extraction for PhysX',physics_pass=False,cycle_time_s=None);out['parts'].pop('ring2')
part='ring2';side='left';r=e.ot.original_recipe(part);P=np.array(r['source_part_P']);Rp=np.array(r['source_part_R']);L=np.array(r['localR']);lp=np.array(r['localP']);e.fe.set_env(e.scene(part));attempts=[];winner=None
seeds=next(row['candidates']for row in raw if row['part']==part and row['stage']=='pickup')
axes=[np.array(a,dtype=float)/np.linalg.norm(a)for a in [[1,0,0],[0,1,0],[1,1,0],[1,-1,0]]]
for angle in [20,-20,35,-35,50,-50,65,-65]:
    for axis in axes:
        for seed in seeds:
            q=np.array(seed['q']);path=[q];okay=True;reason=None
            for u in np.linspace(0,1,29)[1:]:
                Rpart=Rotation.from_rotvec(axis*np.deg2rad(angle)*u).as_matrix()@Rp;Rhand=Rpart@L;tcp=P+[0,0,.035*u]+Rpart@lp
                selected=None
                for b in alternatives(side,tcp,Rhand,path[-1]):
                    valid=True
                    for gap in [.064,r['gap']]:
                        good,why=e.cc.path_clear(side,[path[-1],b],gap,3)
                        if not good:valid=False;reason=dict(stage='hand_and_arm',gap=gap,why=why);break
                    if not valid:continue
                    if u*.035>=.002:
                        with carrying(r):good,why=e.fe.clear(side,b,r['gap'])
                        if not good:reason=dict(stage='carried_ring',why=why);continue
                    selected=b;break
                if selected is None:okay=False;reason=reason or'IK_or_branch_jump';break
                path.append(selected)
            attempt=dict(tilt_deg=angle,world_axis=axis.tolist(),clear=okay,why=None if okay else reason);attempts.append(attempt);print('TILTED_EXTRACTION',attempt,flush=True)
            if okay:winner=path;chosen=attempt;break
        if winner is not None:break
    if winner is not None:break
out['tilted_extraction_search']=attempts
if winner is None:
    out.update(state='no_tilted_extraction_found');(HERE/'tilted-pickup-failure.json').write_text(json.dumps(out,indent=2));raise SystemExit(2)
out['ready_q'][side]=winner[-1].tolist();out['parts'][part]=dict(recipe=r,side=side,grasp_yaw_change_deg=0,q_pick=winner[0].tolist(),q_pre=winner[-1].tolist(),cartesian_lift_q=[q.tolist()for q in winner],lift_m=.035,nominal_loaded_clear=True,tilted_extraction=chosen,first_2mm_payload_support_contact_deferred_to_PhysX=True)
out['parts']['ring1']['lift_m']=.025
out['initial_dual_arm_pair_clear']=e.cc.arm_pair_clear((winner[-1],.064),(np.array(out['ready_q']['right']),.077))
if not out['initial_dual_arm_pair_clear']:out['state']='initial_pair_blocked';(HERE/'tilted-pickup-failure.json').write_text(json.dumps(out,indent=2));raise SystemExit(2)
(HERE/'pickup-probe-plan.json').write_text(json.dumps(out,indent=2))
m=e.cc.mujoco.MjModel.from_xml_path(str(HERE.parent/'torque-native-model.xml'));d=e.cc.mujoco.MjData(m);m.body_pos[m.body('official_FR3_Duo').id]=e.spec['duo_world_position']
for side in ['left','right']:
    for j,value in enumerate(out['ready_q'][side],1):d.qpos[m.jnt_qposadr[m.joint(f'{side}_arm_fr3v2_joint{j}').id]]=value
    for label in ['left','right']:d.qpos[m.jnt_qposadr[m.joint(f'{side}_native_finger_{label}').id]]=out['initial_apertures_m'][side]/2
e.cc.mujoco.mj_forward(m,d);poses={m.body(i).name:dict(position=d.xpos[i].tolist(),quaternion=d.xquat[i].tolist())for i in range(1,m.nbody)}
(HERE/'initial-robot-pose-pickup.json').write_text(json.dumps(dict(ready_q=out['ready_q'],body_world_poses=poses),indent=2));print('TILTED_PICKUP_PREPARED',chosen,flush=True)
