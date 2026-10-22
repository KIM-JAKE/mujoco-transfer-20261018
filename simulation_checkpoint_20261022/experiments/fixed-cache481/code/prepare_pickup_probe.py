"""Build a contact diagnostic, not an accepted collision-free task plan.

Both native Hands descend to the archived ring stocks, close and lift. PhysX
must independently show retention and unexpected contacts. No task time is
reported by this deliberately slow diagnostic.
"""
from pathlib import Path
import os, sys, json
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
os.environ['M80_IMPORT_ENV_ONLY']='1'
sys.path.insert(0,str(HERE))
import prepare_collision_targets as e
from roas_kinematics import solve as robust
from audit_five_geometry import carrying
raw=json.load(open(HERE/'collision-global-centered.json'))['rows']
RISE=.025
out=dict(scope='Native Hand physical pickup diagnostic; not a full cycle or accepted path',physics_pass=False,cycle_time_s=None,parts={},ready_q={},lift_m=RISE,source_rod_top_m=1.310,initial_apertures_m={'right':.077,'left':.064})
def screen(side,path,gap):
    for a,b in zip(path[:-1],path[1:]):
        okay,why=e.cc.path_clear(side,[a,b],gap,max(3,int(np.ceil(np.max(abs(a-b))/.01))+1))
        if not okay:return False,why
    return True,None
for part,side in [('ring1','right'),('ring2','left')]:
    t=next(t for t in e.prepared['targets']if t['part']==part and t['stage']=='pickup')
    candidates=next(r['candidates']for r in raw if r['part']==part and r['stage']=='pickup')
    e.fe.set_env(e.scene(part));attempts=[]
    base_recipe=e.ot.original_recipe(part);Rp=np.array(base_recipe['source_part_R']);P=np.array(base_recipe['source_part_P']);rng=np.random.default_rng(381)
    seeds=[np.array(c['q'])for c in candidates]+[rng.uniform(e.cc.LOWER+.05,e.cc.UPPER-.05)for _ in range(40)]
    variants=[0,30,-30,60,-60,90,-90,120,-120,150,180]
    insertion_variants={}
    for ci,(yaw,seed)in enumerate((angle,seed)for angle in variants for seed in seeds):
        Z=Rotation.from_euler('z',yaw,degrees=True).as_matrix();L=Z@np.array(base_recipe['localR']);lp=Z@np.array(base_recipe['localP']);R=Rp@L;tcp=P+Rp@lp
        if yaw not in insertion_variants:
            Pi,Rpi=e.original_goals[part];Ri=Rpi@L;tcpi=Pi+Rpi@lp
            original=next(row['candidates']for row in raw if row['part']==part and row['stage']=='insert')
            insseeds=[np.array(c['q'])for c in original]+[rng.uniform(e.cc.LOWER+.01,e.cc.UPPER-.01)for _ in range(100)]
            insertion_variants[yaw]=None
            for insseed in insseeds:
                qi=robust(side,tcpi,Ri,insseed)
                if qi is not None and e.fe.clear(side,qi,out['initial_apertures_m'][side])[0]and e.fe.clear(side,qi,base_recipe['gap'])[0]:
                    insertion_variants[yaw]=qi;break
            print('CONSISTENT_GRASP_INSERT',part,yaw,insertion_variants[yaw]is not None,flush=True)
        if insertion_variants[yaw]is None:continue
        pre=robust(side,tcp+[0,0,RISE],R,seed)
        if pre is None:continue
        opening=out['initial_apertures_m'][side]
        open_clear,why=e.fe.clear(side,pre,opening)
        if not open_clear:
            attempts.append(dict(seed=ci,grasp_yaw_deg=yaw,stage='prelift_pose',clear=False,why=why))
            if ci%20==0:print('PICKUP_PRE_REJECT',part,ci,yaw,why,flush=True)
            continue
        descending=[pre];valid=True
        for rise in np.linspace(RISE-RISE/25,0.,25):
            b=robust(side,tcp+[0,0,rise],R,descending[-1])
            if b is None or not e.fe.clear(side,b,opening)[0]:valid=False;break
            descending.append(b)
        if not valid:continue
        path=list(reversed(descending));q=path[0]
        # This is a slow contact diagnostic, not a final path acceptance test.
        # The independent full planner screens every span and payload separately.
        open_clear,why=e.fe.clear(side,path[-1],opening)
        r=dict(base_recipe,localR=L.tolist(),localP=lp.tolist(),R=R.tolist(),tcp=tcp.tolist(),grasp_yaw_change_deg=yaw,q_insert=insertion_variants[yaw].tolist())
        loaded_clear,lwhy=None,'Deferred to real PhysX diagnostic; no nominal payload pass claimed'
        attempts.append(dict(open_clear=open_clear,open_failure=why,loaded_clear=loaded_clear,loaded_failure=lwhy))
        print('PICKUP_PROBE_CANDIDATE',part,attempts[-1],flush=True)
        if not open_clear:continue
        out['ready_q'][side]=path[-1].tolist()
        out['parts'][part]=dict(recipe=r,grasp_yaw_change_deg=yaw,side=side,q_pick=q.tolist(),q_pre=path[-1].tolist(),cartesian_lift_q=[x.tolist()for x in path],nominal_loaded_clear=loaded_clear,attempts=attempts)
        break
    if side not in out['ready_q']:
        out.update(state='no_diagnostic_approach',failed_part=part,failed_attempts=attempts)
        (HERE/'pickup-probe-search-failure.json').write_text(json.dumps(out,indent=2))
        raise RuntimeError('No diagnostic approach for '+part)
out['initial_dual_arm_pair_clear']=e.cc.arm_pair_clear((np.array(out['ready_q']['left']),.064),(np.array(out['ready_q']['right']),.077))
if not out['initial_dual_arm_pair_clear']:raise RuntimeError('Unsafe initial arm pair')
(HERE/'pickup-probe-plan.json').write_text(json.dumps(out,indent=2))
# Export initial poses using kinematics only; dynamics are exclusively PhysX.
m=e.cc.mujoco.MjModel.from_xml_path(str(HERE.parent/'torque-native-model.xml'));d=e.cc.mujoco.MjData(m)
m.body_pos[m.body('official_FR3_Duo').id]=e.spec['duo_world_position']
for side in ['left','right']:
    for j,value in enumerate(out['ready_q'][side],1):d.qpos[m.jnt_qposadr[m.joint(f'{side}_arm_fr3v2_joint{j}').id]]=value
    for label in ['left','right']:d.qpos[m.jnt_qposadr[m.joint(f'{side}_native_finger_{label}').id]]=out['initial_apertures_m'][side]/2
e.cc.mujoco.mj_forward(m,d)
poses={m.body(i).name:dict(position=d.xpos[i].tolist(),quaternion=d.xquat[i].tolist())for i in range(1,m.nbody)}
(HERE/'initial-robot-pose-pickup.json').write_text(json.dumps(dict(ready_q=out['ready_q'],body_world_poses=poses),indent=2))
print('PICKUP_PROBE_PREPARED',flush=True)
