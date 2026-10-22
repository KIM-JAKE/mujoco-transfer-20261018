"""Retest the archived 9.467 s supply poses with standard Franka Hand.

Research only: offline CAD/kinematic planning and separate contact benches.
Never connects to a physical robot or changes the production fieldkit.
"""
from pathlib import Path
import sys, json, copy, hashlib, os, time
import numpy as np
from scipy.spatial.transform import Rotation

import compact_compare as cc
import compact_table_search as ct
from shallow_supply_search import robust, freeik
from fast_environment import set_env, clear, FixedScene
from audit_five_geometry import carrying
from payload_shapes import shape_for

OUT = Path(__file__).resolve().parent / 'original-9s-native-table'
OUT.mkdir(exist_ok=True)
SOURCE = cc.PORT / 'generated/contact_task/contact-source.json'
source = json.loads(SOURCE.read_text())
TABLE = dict(center_m=[-.315, -.28, 1.04], size_m=[.8, .35], thickness_m=.025)

def original_props():
    result = ct.table(*TABLE['center_m'], w=.8, d=.35)
    for row in source['geoms']:
        if row['body'] != 'cap_supply_fixture' and not row['body'].startswith('cap_stock_'):
            continue
        R = cc.g.checker.rot(row['quaternion'])
        result.append((row['name'], cc.fcl.CollisionObject(cc.g.checker.shape(row), cc.fcl.Transform(R, np.array(row['position'])))))
    result.append(ct.shape_box('original_pin_socket_floor', [-.555,-.16,1.28025],[.028,.028,.004]))
    result.append(ct.shape_box('independent_pin_bank_post',[-.603,-.16,1.157125],[.012,.012,.23425]))
    # The original task added this hollow sleeve around the active pin tip.
    from stock_fixture_geometry import pin_stock_sleeve
    v, f, center = pin_stock_sleeve()
    mesh = cc.g.mesh_shape(v, f)
    result.append(('original_pin_tip_sleeve',cc.fcl.CollisionObject(mesh,cc.fcl.Transform(np.eye(3),center))))
    return result

PROPS = original_props()
BASE_TOKEN = hashlib.sha256(SOURCE.read_bytes()+json.dumps(TABLE).encode()).hexdigest()
SCENES = {}

def scene(part=None, installed=True):
    key=(BASE_TOKEN,part,installed)
    if key in SCENES:return SCENES[key]
    rows=list(PROPS)
    if installed:
        for p,b in ct.BANDS.items():
            if p==part:continue
            tcp,R=cc.fk('right' if p=='ring1' else 'left',np.array(b['q_insert']))
            Rp=R@np.array(b['localR']).T;P=tcp-Rp@np.array(b['localP'])
            rows.append(('installed_'+p,cc.fcl.CollisionObject(shape_for(p),cc.fcl.Transform(Rp,P))))
    SCENES[key]=FixedScene(rows,key)
    return SCENES[key]

def original_recipe(part, band=None):
    b=copy.deepcopy(ct.BANDS[part] if band is None else band)
    row=source['parts'][part]
    Rp=cc.g.checker.rot(row['quaternion']);R=Rp@np.array(b['localR'])
    P=np.array(row['position']);tcp=P+Rp@np.array(b['localP'])
    return dict(part=part,side='right' if part=='ring1' else 'left',tcp=tcp.tolist(),R=R.tolist(),localP=b['localP'],localR=b['localR'],gap=b['gap'],source_part_P=P.tolist(),source_part_R=Rp.tolist(),q_insert=b['q_insert'],q_preinsert=b['q_preinsert'])

def endpoint_search(recipe):
    part=recipe['part'];side=recipe['side'];tcp=np.array(recipe['tcp']);R=np.array(recipe['R'])
    old=next(x for x in cc.parts if x['part']==part)['endpoints'][0]
    seeds=[np.array(x['qpos']) for x in old['arm_clear_ik']]+[cc.READY[side],np.array(recipe['q_preinsert'])]
    candidates=[];attempts=[]
    set_env(scene(part))
    for seed in seeds:
        for bias in [0,.5,-.5,1.,-1.,1.5,-1.5]:
            qseed=seed.copy();qseed[0]+=bias
            q=robust(side,tcp,R,np.clip(qseed,cc.LOWER+.01,cc.UPPER-.01))
            if q is None:continue
            if any(np.linalg.norm(q-x['q'])<.02 for x in candidates):continue
            okay,why=clear(side,q,.08 if part!='pin' else .02)
            attempts.append(dict(q=q.tolist(),open_clear=okay,why=why))
            if not okay:continue
            # Remove the rod/socket vertically before moving sideways.
            for rise in [.04,.06,.08]:
                pre=robust(side,tcp+[0,0,rise],R,q)
                if pre is None:continue
                if not cc.path_clear(side,[pre,q],.08 if part!='pin' else .02,41)[0]:continue
                separated=robust(side,tcp+[0,0,.002],R,q)
                if separated is None:continue
                with carrying(recipe):
                    # Stored rings start in intentional support contact. During
                    # the first 2 mm only the payload distance reserve is zero;
                    # intersections and all hand/arm reserves remain checked.
                    cc.g.layout_payload_collision_only=True
                    cc.g.layout_payload_token=(cc.g.layout_payload_token,'support_release')
                    okay,why=cc.path_clear(side,[q,separated],recipe['gap'],21)
                if okay:
                    with carrying(recipe):
                        okay,why=cc.path_clear(side,[separated,pre],recipe['gap'],41)
                if not okay:
                    attempts.append(dict(stage='loaded_vertical_lift',rise_m=rise,why=why));continue
                score=cc.segment_time(cc.READY[side],pre)+cc.segment_time(pre,np.array(recipe['q_preinsert']))
                candidates.append(dict(q=q,pre=pre,rise=rise,score=score));break
            print('ENDPOINT_BRANCH',part,len(candidates),len(attempts),flush=True)
            if len(candidates)>=4:break
        if len(candidates)>=4:break
    candidates.sort(key=lambda r:r['score'])
    return candidates,attempts

def json_ready(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,dict):return {k:json_ready(v)for k,v in value.items()}
    if isinstance(value,list):return [json_ready(x)for x in value]
    if isinstance(value,np.generic):return value.item()
    return value

def screen():
    begin=time.monotonic()
    table_hits=ct.machine_hits(ct.table(*TABLE['center_m'],w=.8,d=.35))
    support_hits=ct.machine_hits([(n,o)for n,o in PROPS if n=='independent_pin_bank_post'])
    print('TABLE',table_hits,'SUPPORT',support_hits,flush=True)
    report=dict(source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),archived_9p467_source_sha256='273bbe48021cd3ca4a98f41a17ba66658b049c71d7313a12406358703bd9da8b',table=TABLE,table_machine_hits=table_hits,new_support_machine_hits=support_hits,parts={},physical_full_assembly_pass=False,hardware_tested=False)
    for part in ['ring1','ring2','pin']:
        r=original_recipe(part);candidates,attempts=endpoint_search(r)
        report['parts'][part]=dict(recipe=r,candidates=json_ready(candidates),attempts=attempts)
        (OUT/'endpoint-screen.json').write_text(json.dumps(report,indent=2))
        print('ORIGINAL_ENDPOINT',part,len(candidates),len(attempts),flush=True)
    report['seconds']=time.monotonic()-begin
    (OUT/'endpoint-screen.json').write_text(json.dumps(report,indent=2))
    return report

def fine_path(side,path,gap):
    for a,b in zip(path[:-1],path[1:]):
        count=max(41,int(np.ceil(np.max(abs(np.asarray(b)-a))/.01))+1)
        okay,why=cc.path_clear(side,[np.asarray(a),np.asarray(b)],gap,count)
        if not okay:return False,why
    return True,None

def make_recipe(part,record,start):
    from audit_five_geometry import payload_bridge
    base=copy.deepcopy(record['recipe']);side=base['side'];opening=.02 if part=='pin'else .08
    target=np.array(base['q_preinsert']);qi=np.array(base['q_insert']);failures=[]
    for index,candidate in enumerate(record['candidates']):
        r=copy.deepcopy(base);q=np.array(candidate['q']);pre=np.array(candidate['pre'])
        r['q_pick']=q.tolist();r['q_exit']=target.tolist()
        set_env(scene(part))
        okay,why=fine_path(side,[target,qi],r['gap'])
        if not okay:failures.append(dict(stage='insert_hand',why=why));continue
        if not fine_path(side,[qi,target],opening)[0]:failures.append(dict(stage='withdraw'));continue
        print('ORIGINAL_ROUTE_ENTRY',part,index,flush=True)
        entry=cc.route(side,np.asarray(start),pre,opening,limit=350)
        if entry is None:failures.append(dict(stage='entry',candidate=index));continue
        if not fine_path(side,entry,opening)[0]:failures.append(dict(stage='entry_fine',candidate=index));continue
        print('ORIGINAL_ROUTE_TRANSFER',part,index,flush=True)
        with carrying(r,coarse=True):
            transfer=cc.route(side,pre,target,r['gap'],limit=260,allow_search=False)
            if transfer is None:transfer=payload_bridge(r,pre,target)
            if transfer is None:transfer=cc.route(side,pre,target,r['gap'],limit=450)
        if transfer is None:failures.append(dict(stage='transfer',candidate=index));continue
        with carrying(r):okay,why=fine_path(side,transfer,r['gap'])
        if not okay:failures.append(dict(stage='transfer_fine',candidate=index,why=why));continue
        ss=[cc.seg(a,b,'empty_approach',part,opening)for a,b in zip(entry[:-1],entry[1:])]
        ss += [cc.seg(pre,q,'pregrasp',part,opening),dict(a=q.tolist(),b=q.tolist(),duration_s=.18,label='grasp',part=part,gap_m=r['gap']),cc.seg(q,pre,'lift',part,r['gap'])]
        ss += [cc.seg(a,b,'transfer',part,r['gap'])for a,b in zip(transfer[:-1],transfer[1:])]
        ss += [cc.seg(target,qi,'insert',part,r['gap']),dict(a=qi.tolist(),b=qi.tolist(),duration_s=.18,label='clamp',part=part,gap_m=r['gap']),dict(a=qi.tolist(),b=qi.tolist(),duration_s=.18,label='release',part=part,gap_m=opening),cc.seg(qi,target,'withdraw',part,opening)]
        r.update(segments=ss,q_path=[x.tolist()for x in [q,pre]+transfer[1:]+[qi]],duration_s=sum(x['duration_s']for x in ss),pregrasp_delta_m=[0,0,candidate['rise']])
        print('ORIGINAL_RECIPE',part,r['duration_s'],len(ss),flush=True)
        return r,failures
    return None,failures

def plan():
    from five_compare import retime,prepare_lane,screened_button
    from audit_five_schedule import limits,dense_cross
    endpoints=json.load(open(OUT/'endpoint-screen.json'))
    report=dict(state='planning',table=TABLE,source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),original_supply_positions_preserved=True,original_supply_shapes_preserved=True,source_fixture_not_attached_to_machine=True,common_start_q={s:cc.READY[s].tolist()for s in cc.AD},recipes={},failures=[],hardware_tested=False,full_dynamic_assembly_pass=False)
    def save(): (OUT/'task-test.json').write_text(json.dumps(json_ready(report),indent=2))
    for part in ['ring1','ring2','pin']:
        rec=endpoints['parts'].get(part)
        if rec is None or not rec['candidates']:
            report.update(state='no_valid_pickup_path',failed_part=part);save();return
        start=cc.READY['right'if part=='ring1'else'left']if part!='pin'else np.array(report['recipes']['ring2']['q_exit'])
        r,failures=make_recipe(part,rec,start);report['failures'] += [dict(part=part,**f)for f in failures]
        if r is None:report.update(state='no_valid_individual_path',failed_part=part);save();return
        report['recipes'][part]=r;save()
    recipes=report['recipes'];model_reports=[]
    for r in recipes.values():model_reports+=retime(r)
    set_env(scene())
    button=screened_button(recipes['ring1']['q_exit'])
    if button is None:report.update(state='no_button_path');save();return
    _,approach,press=button
    for s in approach+press:s['duration_s'],_=cc.model_screen(s,'right',dict(part='button'))
    ql=recipes['ring2']['q_exit'];qr=recipes['ring1']['q_exit']
    left=recipes['ring2']['segments']+[dict(a=ql,b=ql,duration_s=.06/.095+.05,label='pin_preshape',part='pin',gap_m=.02)]+recipes['pin']['segments']
    right=recipes['ring1']['segments']+[dict(a=qr,b=qr,duration_s=.08/.095+.05,label='close_empty_hand_for_button',part='button',gap_m=0.)]+approach
    best=None
    for parked in [False,True]:
        if parked:
            extra=cc.route('left',left[-1]['b'],cc.READY['left'],.02,limit=350)
            if extra is None:continue
            for a,b in zip(extra[:-1],extra[1:]):
                s=cc.seg(a,b,'park','pin',.02);s['duration_s'],_=cc.model_screen(s,'left',dict(part='pin'));left.append(s)
        for split in [False,True]:
            lanes=dict(left=prepare_lane(left,recipes,split),right=prepare_lane(right,recipes,split))
            print('ORIGINAL_COORD_BEGIN',parked,split,flush=True)
            schedule=cc.coordinate(lanes,.03)
            if schedule is None:continue
            last=cc.state_at(lanes['left'][-1],lanes['left'][-1]['duration_s'])
            if not all(cc.arm_pair_clear(last,cc.state_at(s,t))for s in press for t in np.linspace(0,s['duration_s'],41)):continue
            elapsed=schedule['duration_s']+sum(s['duration_s']for s in press)
            trial=dict(duration_s=elapsed,lanes=lanes,coordination=schedule,button_finish=press,left_parks=parked,split_free_motions=split)
            if best is None or elapsed<best['duration_s']:best=trial
            print('ORIGINAL_COORD',elapsed,parked,split,flush=True)
        if best:break
    if best is None:report.update(state='no_dual_arm_schedule');save();return
    report.update(state='nominal_task_scheduled',**best,model_reports=model_reports);save()
    checks=limits(best['lanes'])+limits(dict(right=press));cross,samples,last=dense_cross(best['lanes'],best['coordination'])
    for s in press:
        for t in np.linspace(0,s['duration_s'],max(3,int(np.ceil(s['duration_s']/.01))+1)):
            if not cc.arm_pair_clear(last['left'],cc.state_at(s,t)):cross.append(dict(button_label=s['label'],t_s=float(t)))
            samples+=1
    sweeps=[]
    for side,lane in best['lanes'].items():
        for s in lane:
            if s['label']not in('grasp','release','pin_preshape','close_empty_hand_for_button'):continue
            set_env(scene(s['part']if s['label']in('grasp','release')else None))
            for gap in np.linspace(s['gap_start_m'],s['gap_finish_m'],41):
                okay,why=clear(side,np.array(s['a']),float(gap))
                if not okay:sweeps.append(dict(side=side,label=s['label'],why=why));break
    report.update(joint_limit_checks=checks,cross_arm_failures=cross,dense_cross_samples=samples,gripper_sweep_failures=sweeps,planning_fraction=.95,command_kinematics_pass=all(r['passed']for r in checks),nominal_dual_arm_geometry_pass=not cross and not sweeps)
    report['state']='nominal_task_audit_pass'if report['command_kinematics_pass']and report['nominal_dual_arm_geometry_pass']else'nominal_task_audit_fail'
    report['limitations']=['Stopped quintics at motion waypoints; not a global minimum-time result','Native Hand aperture speed 0.095 m/s; 80 ms grasp allowance','M80 clamp latency 0.18 s assumed','First 2 mm of stock lift has zero payload reserve for intentional support separation; intersections remain forbidden','Exact CAD free transfers, sampled arm proxies and 1 mm inter-arm reserve','Imported Hand/robot inertia approximate; no calibrated libfranka torque test','Pickup contact benches separate; complete PhysX insertion and real robot untested','Original archive contains four visible stocks per type; five-cycle feeding is not validated by this one-cycle test']
    save();print('ORIGINAL_TASK_DONE',report['state'],best['duration_s'],samples,flush=True)
    return report

if __name__=='__main__':
    if '--plan' in sys.argv:plan()
    else:screen()
