"""Collision-aware endpoint preparation for subsequent real PhysX execution."""
from pathlib import Path
import sys,json,copy,time,os
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent))
import original_native_table_test as ot
import compact_compare as cc
import compact_table_search as ct
import fast_environment as fe
from shallow_supply_search import robust
from payload_shapes import shape_for

spec=json.load(open(HERE/'placement-spec-candidate.json'));prepared=json.load(open(HERE/'prepared-ik-targets-candidate.json'));init=json.load(open(HERE/'initial-robot-pose-candidate.json'))
table_center=spec.get('robot_table_center_m',spec['duo_world_position'][:2]).copy()
if os.environ.get('M80_ROOT_Y'):spec['duo_world_position'][1]=float(os.environ['M80_ROOT_Y'])
if os.environ.get('M80_ROOT_X'):spec['duo_world_position'][0]=float(os.environ['M80_ROOT_X'])
original_goals={}
for part,b in ct.BANDS.items():
    side='right'if part=='ring1'else'left';tcp,R=cc.fk(side,np.array(b['q_insert']));Rp=R@np.array(b['localR']).T;P=tcp-Rp@np.array(b['localP']);original_goals[part]=(P,Rp)
cc.model.body_pos[cc.model.body('official_FR3_Duo').id]=spec['duo_world_position']
for side in cc.READY:cc.READY[side][:]=init['ready_q'][side];cc.data.qpos[cc.AD[side]]=cc.READY[side]
cc.mujoco.mj_kinematics(cc.model,cc.data)
fe._base_shapes=[]
for row,_ in cc.g.checker.robot:
    if row['body']!='official_FR3_Duo':continue
    b=cc.model.body(row['body']).id;R=cc.data.xmat[b].reshape(3,3);P=cc.data.xpos[b]+R@row['position'];fe._base_shapes.append(cc.fcl.CollisionObject(cc.g.checker.shape(row,False),cc.fcl.Transform(R@cc.g.checker.rot(row['quaternion']),P)))
fe._base=cc.fcl.DynamicAABBTreeCollisionManager();fe._base.registerObjects(fe._base_shapes);fe._base.setup();fe._cache.clear()
base=[(name,obj)for name,obj in ot.PROPS if not name.startswith('independent_table_')and name!='independent_pin_bank_post']
base += ct.table(*spec['supply_table']['center'],w=.8,d=float(os.environ.get('M80_SUPPLY_DEPTH','.15')))
z=spec['supply_table']['center'][2]
if z<1.04:
    for part,x in [('ring1',-.02),('ring2',-.45)]:base.append(ct.shape_box(part+'_post_extension',[x,-.16,(z+1.04)/2],[.024,.024,1.04-z]))
base.append(ct.shape_box('pin_post',[-.603,-.16,(z+1.26625)/2],[.012,.012,1.26625-z]));base.append(ct.shape_box('pin_rail_crossbar',[-.603,-.16,1.27025],[.030,.036,.008]))
base += [ct.shape_box('ROAS_closed_cabinet_envelope',[*table_center,.36],[.68,.66,.6])]

def scene(part):
    rows=list(base)
    for p,(P,R)in original_goals.items():
        if p!=part:rows.append(('installed_'+p,cc.fcl.CollisionObject(shape_for(p),cc.fcl.Transform(R,P))))
        if p!=part and not(p=='pin'and part=='ring2'and os.environ.get('M80_PIN_FIRST')=='1'):
            source=ot.source['parts'][p];Rp=cc.g.checker.rot(source['quaternion']);rows.append(('active_unprocessed_'+p,cc.fcl.CollisionObject(shape_for(p),cc.fcl.Transform(Rp,np.array(source['position'])))))
    return fe.FixedScene(rows,('ROAS_collision_with_active_source_parts',tuple(spec['duo_world_position']),part,os.environ.get('M80_PIN_FIRST')))

results=[]
for target in (prepared['targets']if os.environ.get('M80_IMPORT_ENV_ONLY')!='1'else[]):
    part=target['part'];side=target['side'];tcp=np.array(target['tcp']);R=np.array(target['R']);opening=.08 if part!='pin'else .02;gap=opening if target['stage']=='pickup'else target['gap'];fe.set_env(scene(part));good=[];attempts=[];seen=[]
    oldcase=next(x for x in cc.parts if x['part']==part)
    oldendpoint=oldcase['endpoints'][1 if target['stage']=='insert'else 0]
    seeds=[np.array(target['q']),cc.READY[side],np.array(ct.BANDS[part]['q_insert'])]
    seeds += [np.array(x['qpos'])for x in oldendpoint['arm_clear_ik']]
    rng=np.random.default_rng(121)
    seeds += [rng.uniform(cc.LOWER+.05,cc.UPPER-.05)for _ in range(int(os.environ.get('M80_RANDOM_IK_SEEDS','0')))]
    for seed in seeds:
        for bias in ([0]if len(seeds)>15 else[0,.3,-.3,.6,-.6,1.,-1.,1.5,-1.5,2.,-2.]):
            qseed=seed.copy();qseed[0]+=bias;q=robust(side,tcp,R,np.clip(qseed,cc.LOWER+.01,cc.UPPER-.01))
            if q is None or any(np.linalg.norm(q-prev)<.01 for prev in seen):continue
            seen.append(q);okay,why=fe.clear(side,q,gap);attempts.append(dict(q=q.tolist(),clear=okay,why=why))
            if okay:good.append(dict(q=q.tolist(),score=cc.segment_time(cc.READY[side],q)))
            if len(good)>=int(os.environ.get('M80_ENDPOINT_COUNT','4')):break
        if len(good)>=int(os.environ.get('M80_ENDPOINT_COUNT','4')):break
    good.sort(key=lambda r:r['score']);row=dict(part=part,stage=target['stage'],candidates=good,attempts=attempts);results.append(row);print('COLLISION_ENDPOINT',part,target['stage'],len(good),len(attempts),attempts[:2],flush=True)
    (HERE/os.environ.get('M80_ENDPOINT_OUTPUT','collision-aware-endpoints.json')).write_text(json.dumps(dict(scope='Offline exact-M80/stock/table screening plus conservative arm hulls and native finger boxes; PhysX required',placement=spec,rows=results),indent=2))
    if not good and os.environ.get('M80_ENDPOINT_STOP_ON_FAILURE')=='1':break
