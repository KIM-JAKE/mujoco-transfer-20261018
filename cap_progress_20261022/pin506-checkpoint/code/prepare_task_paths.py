"""Prepare fully screened original-stock native-Hand motions for PhysX.
Offline planning is not accepted as a dynamic task/timing result.
"""
from pathlib import Path
import os,sys,json,copy,itertools,hashlib
import numpy as np
HERE=Path(__file__).resolve().parent
os.environ['M80_IMPORT_ENV_ONLY']='1';os.environ['M80_PIN_FIRST']='1';sys.path.insert(0,str(HERE));sys.path.insert(0,str(HERE.parent))
import prepare_collision_targets as env
from audit_five_geometry import carrying,payload_bridge
from five_compare import retime,screened_button,prepare_lane
from roas_kinematics import solve as robust
cc=env.cc;ct=env.ct;fe=env.fe
raw=json.load(open(HERE/'collision-global-centered.json'))['rows'];targets=env.prepared['targets']
OUT=HERE/'prepared-task-paths.json';report=dict(state='planning',physics_pass=False,cycle_time_s=None,recipes={},failures=[])
OPEN={'ring1':.077,'ring2':.064,'pin':.02}
PICKUP_OPEN=OPEN.copy()
GRASPS={}
probe_path=HERE/'pickup-probe-plan.json'
PROBE_PLAN=json.load(open(probe_path))if probe_path.exists()else None
if probe_path.exists():
    report['pickup_plan_sha256']=hashlib.sha256(probe_path.read_bytes()).hexdigest()
    GRASPS={p:row['recipe']for p,row in PROBE_PLAN['parts'].items()}
    for p,row in PROBE_PLAN['parts'].items():PICKUP_OPEN[p]=PROBE_PLAN['initial_apertures_m'][row['side']]
    for part,r in GRASPS.items():
        if not r.get('grasp_yaw_change_deg'):continue
        for stage in ['pickup','insert']:
            t=next(t for t in targets if t['part']==part and t['stage']==stage);row=next(r for r in raw if r['part']==part and r['stage']==stage)
            P,Rp=(np.array(r['source_part_P']),np.array(r['source_part_R']))if stage=='pickup'else env.original_goals[part]
            R=Rp@np.array(r['localR']);tcp=P+Rp@np.array(r['localP']);t.update(R=R.tolist(),tcp=tcp.tolist())
            side=t['side'];env.fe.set_env(env.scene(part));rng=np.random.default_rng(771);good=[]
            seeds=[np.array(c['q'])for c in row['candidates']]+[rng.uniform(cc.LOWER+.01,cc.UPPER-.01)for _ in range(150)]
            for seed in seeds:
                q=robust(side,tcp,R,seed)
                if q is None or any(np.linalg.norm(q-np.array(c['q']))<.05 for c in good):continue
                if env.fe.clear(side,q,OPEN[part]if stage=='pickup'else r['gap'])[0]:good.append(dict(q=q.tolist()))
                if len(good)>=5:break
            row['candidates']=good;print('ROTATED_GRASP_ENDPOINTS',part,stage,len(good),flush=True)
def json_safe(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,(list,tuple)):return [json_safe(x)for x in value]
    if isinstance(value,dict):return {k:json_safe(v)for k,v in value.items()}
    return value
def save():OUT.write_text(json.dumps(json_safe(report),indent=2))
def target(part,stage):return next(t for t in targets if t['part']==part and t['stage']==stage)
def candidates(part,stage):return next(r['candidates']for r in raw if r['part']==part and r['stage']==stage)
def fine(side,path,gap):return env.ot.fine_path(side,[np.array(q)for q in path],gap)

def cartesian_line(side,tcp,R,seed,delta):
    path=[np.array(seed)]
    for u in np.linspace(0,1,max(3,int(np.ceil(np.linalg.norm(delta)/.002))+1))[1:]:
        q=robust(side,np.array(tcp)+u*np.array(delta),R,path[-1])
        if q is None or np.max(abs(q-path[-1]))>.25:return None
        path.append(q)
    return path

def cartesian_clear(side,path,gap):
    for a,b in zip(path[:-1],path[1:]):
        okay,why=cc.path_clear(side,[a,b],gap,max(3,int(np.ceil(np.max(abs(a-b))/.01))+1))
        if not okay:return False,why
    return True,None

def insertion_bands(part):
    t=target(part,'insert');side=t['side'];R=np.array(t['R']);tcp=np.array(t['tcp']);opening=OPEN[part];fe.set_env(env.scene(part));out=[]
    for entry in candidates(part,'insert'):
        qi=np.array(entry['q'])
        for distance in [.055,.035,.020,.010,.005]:
            path=cartesian_line(side,tcp,R,qi,[0,0,-distance])
            if path is None:continue
            pre=path[-1]
            inward,why=cartesian_clear(side,path,t['gap']);outward,why2=cartesian_clear(side,path,opening)
            if inward and outward:out.append((pre,qi,path));break
            print('INSERT_SPAN_REJECT',part,distance,why,why2,flush=True)
    print('INSERT_BANDS',part,len(out),flush=True);return out

def pickup_bands(part):
    t=target(part,'pickup');side=t['side'];R=np.array(t['R']);tcp=np.array(t['tcp']);opening=PICKUP_OPEN[part];fe.set_env(env.scene(part));out=[]
    probe_path=HERE/'pickup-probe-plan.json'
    if PROBE_PLAN:
        probe=PROBE_PLAN['parts'].get(part)
        if probe is not None:
            path=[np.array(q)for q in probe['cartesian_lift_q']];r=probe['recipe']
            lift=probe.get('lift_m',PROBE_PLAN['lift_m']);separated=max(1,int(np.ceil(.002/lift*(len(path)-1))))if probe.get('first_2mm_payload_support_contact_deferred_to_PhysX')else 1
            open_okay,open_why=cartesian_clear(side,path,opening)
            if not open_okay:print('PICKUP_PROBE_OPEN_REJECT',part,open_why,flush=True)
            with carrying(r):okay,why=cartesian_clear(side,path[separated:],t['gap'])
            if okay and open_okay:
                print('PICKUP_PROBE_BRANCH_REUSED',part,flush=True)
                return [(path[0],path[-1],lift,path)]
            print('PICKUP_PROBE_BRANCH_REJECT',part,why,flush=True)
    for entry in candidates(part,'pickup'):
        q=np.array(entry['q'])
        for lift in [.025,.035,.05,.07]:
            path=cartesian_line(side,tcp,R,q,[0,0,lift])
            if path is None:continue
            pre=path[-1]
            okay,why=cartesian_clear(side,path,opening)
            if not okay:print('PICKUP_OPEN_REJECT',part,lift,why,flush=True);continue
            recipe=env.ot.original_recipe(part)
            with carrying(recipe):okay,why=cartesian_clear(side,path[1:],t['gap'])
            if not okay:print('PICKUP_LOADED_REJECT',part,lift,why,flush=True);continue
            out.append((q,pre,lift,path));break
    print('PICKUP_BANDS',part,len(out),flush=True);return out

def shortcut(side,path,gap):
    path=[np.array(x)for x in path];out=[path[0]];k=0
    while k<len(path)-1:
        chosen=k+1
        for j in range(len(path)-1,k+1,-1):
            if cc.segment_time(path[k],path[j])>sum(cc.segment_time(path[l],path[l+1])for l in range(k,j))+1e-8:continue
            if fine(side,[path[k],path[j]],gap)[0]:chosen=j;break
        out.append(path[chosen]);k=chosen
    return out

def recipe(part,start,picks,inserts):
    t=target(part,'pickup');side=t['side'];opening=PICKUP_OPEN[part];release=OPEN[part]
    choices=sorted(itertools.product(picks,inserts),key=lambda c:cc.segment_time(c[0][1],c[1][0]))
    for pi,((q,pre,lift,lift_path),(approach,qi,withdraw_path))in enumerate(choices):
        fe.set_env(env.scene(part));r=copy.deepcopy(GRASPS.get(part,env.ot.original_recipe(part)));r.update(q_pick=q.tolist(),q_exit=approach.tolist(),q_insert=qi.tolist(),q_preinsert=approach.tolist(),pregrasp_delta_m=[0,0,lift])
        print('TASK_ROUTE',part,pi,'entry',flush=True)
        entry=[np.array(start),pre]if np.linalg.norm(np.array(start)-pre)<1e-6 else cc.route(side,start,pre,opening,limit=300)
        if entry is None:report['failures'].append(dict(part=part,choice=pi,stage='entry'));save();continue
        entry=shortcut(side,entry,opening)
        print('TASK_ROUTE',part,pi,'carried transfer',flush=True)
        with carrying(r):
            transfer=cc.route(side,pre,approach,r['gap'],limit=250,allow_search=False)
            if transfer is None:
                transfer=payload_bridge(r,pre,approach)
            if transfer is None:transfer=cc.route(side,pre,approach,r['gap'],limit=350)
            if transfer is not None:transfer=shortcut(side,transfer,r['gap'])
        if transfer is None:report['failures'].append(dict(part=part,choice=pi,stage='transfer'));save();continue
        ss=[cc.seg(a,b,'empty_approach',part,opening)for a,b in zip(entry[:-1],entry[1:])if np.linalg.norm(a-b)>1e-7]
        for label,path,gap in [('pregrasp',list(reversed(lift_path)),opening)]:
            ss += [cc.seg(a,b,label,part,gap)for a,b in zip(path[:-1],path[1:])]
        ss += [dict(a=q.tolist(),b=q.tolist(),duration_s=.18,label='grasp',part=part,gap_m=r['gap'])]
        ss += [cc.seg(a,b,'lift',part,r['gap'])for a,b in zip(lift_path[:-1],lift_path[1:])]
        ss += [cc.seg(a,b,'transfer',part,r['gap'])for a,b in zip(transfer[:-1],transfer[1:])]
        inward=list(reversed(withdraw_path));ss += [cc.seg(a,b,'insert',part,r['gap'])for a,b in zip(inward[:-1],inward[1:])]
        ss += [dict(a=qi.tolist(),b=qi.tolist(),duration_s=.18,label='clamp',part=part,gap_m=r['gap']),dict(a=qi.tolist(),b=qi.tolist(),duration_s=.18,label='release',part=part,gap_m=release)]
        ss += [cc.seg(a,b,'withdraw',part,release)for a,b in zip(withdraw_path[:-1],withdraw_path[1:])]
        r.update(segments=ss,duration_s=sum(s['duration_s']for s in ss),opening_gap_m=opening,release_gap_m=release);r['approximate_model_screens']=retime(r)
        for s in r['segments']:
            if s['label']in('grasp','release'):s['duration_s']=max(.18,((opening if s['label']=='grasp'else release)-r['gap'])/.095+(.08 if s['label']=='grasp'else .05))
        r['duration_s']=sum(s['duration_s']for s in r['segments']);return r
    return None

picks={};inserts={}
for part in ['ring1','ring2','pin']:
    inserts[part]=insertion_bands(part);picks[part]=pickup_bands(part)
    report['contact_bands']={p:dict(inserts=inserts[p],pickups=picks[p])for p in picks};save()
    if not inserts[part]or not picks[part]:report.update(state='missing_contact_approach',failed_part=part);save();raise SystemExit(2)

# Start above the two original ring locations. Both start poses are screened,
# and the initial pair must also be clear. Return-to-ready is timed separately.
starts=[]
for right,left in itertools.product(picks['ring1'],picks['pin']):
    if cc.arm_pair_clear((left[1],PICKUP_OPEN['pin']),(right[1],PICKUP_OPEN['ring1'])):starts.append((right,left))
if not starts:report.update(state='no_initial_dual_arm_pair');save();raise SystemExit(2)
right,left=starts[0]
cc.READY['right'][:]=right[1];cc.READY['left'][:]=left[1]
report['ready_q']={s:q.tolist()for s,q in cc.READY.items()};report['start_condition']='Right above original large-ring pickup; left above original pin pickup; open hands, validated pair'
report['left_task_order']=['pin','ring2'];report['source_occupancy_condition']='Pin must be physically inserted before small-ring pickup; all other unprocessed active parts are screened at source'
report['initial_apertures_m']={'left':PICKUP_OPEN['pin'],'right':PICKUP_OPEN['ring1']};save()
for part in ['ring1','pin','ring2']:
    start=cc.READY['right'if part=='ring1'else'left']if part!='ring2'else report['recipes']['pin']['q_exit']
    selected=[right]if part=='ring1'else[left]if part=='pin'else picks[part]
    r=recipe(part,start,selected,inserts[part])
    if r is None:report.update(state='no_individual_task_path',failed_part=part);save();raise SystemExit(2)
    report['recipes'][part]=r;save();print('PHYSX_PATH_PREPARED',part,r['duration_s'],len(r['segments']),flush=True)
fe.set_env(env.scene(None));button=screened_button(report['recipes']['ring1']['q_exit'])
if button is None:report.update(state='no_button_path');save();raise SystemExit(2)
_,approach,press=button;report.update(state='individual_paths_prepared_for_PhysX',button_approach=approach,button_finish=press,nominal_paths_only=True,placement=env.spec);save();print('ALL_TASK_PATHS_PREPARED',flush=True)
from audit_five_schedule import limits,dense_cross
recipes=report['recipes'];ql=recipes['pin']['q_exit'];qr=recipes['ring1']['q_exit']
preshape=dict(a=ql,b=ql,duration_s=(PICKUP_OPEN['ring2']-.02)/.095+.05,label='open_hand_for_ring',part='ring2',gap_m=PICKUP_OPEN['ring2'])
close=dict(a=qr,b=qr,duration_s=OPEN['ring1']/.095+.05,label='close_empty_hand_for_button',part='button',gap_m=0.)
lanes=dict(left=recipes['pin']['segments']+[preshape]+recipes['ring2']['segments'],right=recipes['ring1']['segments']+[close]+approach)
print('ROAS_COORDINATION_BEGIN',len(lanes['left']),len(lanes['right']),flush=True);schedule=cc.coordinate(lanes,.04,report['initial_apertures_m'])
if schedule is None:report.update(state='cross_arm_coordination_failed');save();raise SystemExit(2)
checks=limits(lanes)+limits(dict(right=press));bad,samples,last=dense_cross(lanes,schedule,report['initial_apertures_m'])
button_clear=all(cc.arm_pair_clear(last['left'],cc.state_at(s,t))for s in press for t in np.linspace(0,s['duration_s'],max(3,int(np.ceil(s['duration_s']/.01))+1)))
report.update(state='coordinated_paths_prepared_for_PhysX'if not bad and button_clear and all(c['passed']for c in checks)else'coordinated_path_audit_failed',lanes=lanes,coordination=schedule,command_joint_limit_checks=checks,cross_arm_failures=bad,cross_arm_sample_count=samples,button_cross_clear=button_clear,offline_command_duration_s=schedule['duration_s']+sum(s['duration_s']for s in press),physics_pass=False,cycle_time_s=None);save()
if report['state']!='coordinated_paths_prepared_for_PhysX':raise SystemExit(2)
import roas_kinematics as kin
for side in ['left','right']:
    kin.data.qpos[kin.AD[side]]=report['ready_q'][side]
    for label in ['left','right']:kin.data.qpos[kin.model.jnt_qposadr[kin.model.joint(f'{side}_native_finger_{label}').id]]=report['initial_apertures_m'][side]/2
kin.mujoco.mj_forward(kin.model,kin.data)
poses={kin.model.body(i).name:dict(position=kin.data.xpos[i].tolist(),quaternion=kin.data.xquat[i].tolist())for i in range(1,kin.model.nbody)}
(HERE/'initial-robot-pose-task.json').write_text(json.dumps(dict(ready_q=report['ready_q'],body_world_poses=poses),indent=2))
print('ROAS_COORDINATED_PATHS_READY',report['offline_command_duration_s'],flush=True)
