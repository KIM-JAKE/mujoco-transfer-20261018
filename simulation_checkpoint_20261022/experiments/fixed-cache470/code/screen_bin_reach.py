"""Screen original basket reach and consistent-grasp insertion, kinematics only."""
from pathlib import Path
import sys,json,math,os,collections
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parent)]
import compact_compare as cc
import compact_table_search as ct
import fast_environment as fe
from roas_kinematics import solve
from payload_shapes import shape_for

OUT=HERE/'bin-picking';assets=json.load(open(OUT/'asset-audit.json'))
# Capture original fixture goals before relocating the robot's base.
goals={}
for p,b in ct.BANDS.items():
    tcp,R=cc.fk('right'if p=='ring1'else'left',np.array(b['q_insert']));Rp=R@np.array(b['localR']).T;goals[p]=(tcp-Rp@np.array(b['localP']),Rp)
spec=json.load(open(HERE/'placement-spec-candidate.json'))
cc.model.body_pos[cc.model.body('official_FR3_Duo').id]=spec['duo_world_position']
init=json.load(open(HERE/'initial-robot-pose-candidate.json'))
for s in ['left','right']:cc.READY[s][:]=init['ready_q'][s];cc.data.qpos[cc.AD[s]]=cc.READY[s]
cc.mujoco.mj_kinematics(cc.model,cc.data)
fe._base_shapes=[]
for row,_ in cc.g.checker.robot:
    if row['body']!='official_FR3_Duo':continue
    b=cc.model.body(row['body']).id;R=cc.data.xmat[b].reshape(3,3);P=cc.data.xpos[b]+R@row['position'];fe._base_shapes.append(cc.fcl.CollisionObject(cc.g.checker.shape(row,False),cc.fcl.Transform(R@cc.g.checker.rot(row['quaternion']),P)))
fe._base=cc.fcl.DynamicAABBTreeCollisionManager();fe._base.registerObjects(fe._base_shapes);fe._base.setup();fe._cache.clear()
base=[ct.shape_box('ROAS_cabinet',[*spec['duo_world_position'][:2],.36],[.68,.66,.6])]
fe.set_env(fe.FixedScene(base,('original_bins_no_added_table',tuple(spec['duo_world_position']))))
rng=np.random.default_rng(10024)
report=dict(scope=__doc__,physics_pass=False,vision_tested=False,full_task_pass=False,cycle_time_s=None,robot_position=spec['duo_world_position'],baskets=[],parts={},goals={p:dict(P=P.tolist(),R=R.tolist())for p,(P,R)in goals.items()})
def save():(OUT/'reach-screen.json').write_text(json.dumps(report,indent=2))
def branches(side,tcp,R,gap,n=18):
    seeds=[cc.READY[side],np.array(ct.BANDS['ring1'if side=='right'else'ring2']['q_insert'])]+[rng.uniform(cc.LOWER+.02,cc.UPPER-.02)for _ in range(n)]
    good=[];fail=[];exists=0
    for seed in seeds:
        q=solve(side,tcp,R,seed)
        if q is None:continue
        exists+=1;ok,why=fe.clear(side,q,gap)
        if ok and not any(np.linalg.norm(q-x)<.05 for x in good):good.append(q)
        elif not ok:fail.append(why)
        if len(good)>=2:break
    return good,dict(ik_solutions=exists,rejected=fail)

if __name__=='__main__':
    for basket in assets['baskets'][:3]:
        B=np.array(basket['basket_to_world_R']);O=np.array(basket['floor_origin_world_m']);row=dict(id=basket['id'],trials=[],reachable=False)
        for xy in [[-.06,0],[0,0],[.06,0],[-.06,.10],[-.06,-.10]]:
            for side in ['right','left']:
                for yaw in [0,45,90,135]:
                    R=B@Rotation.from_euler('z',yaw,degrees=True).as_matrix()@np.diag([1.,-1.,-1.]);P=O+B@np.array([*xy,.011]);good,detail=branches(side,P,R,.08,n=8)
                    entry=dict(side=side,xy=xy,yaw=yaw,tcp=P.tolist(),R=R.tolist(),clear_ik=[q.tolist()for q in good],diagnostic=detail);row['trials'].append(entry)
                    if good:row['reachable']=True
                    print('BIN_REACH',basket['id'],side,xy,yaw,len(good),detail['rejected'][:1],flush=True)
            if row['reachable']:break
        report['baskets'].append(row);save()
    # Every source grasp needs a physically reachable goal with that same
    # hand-to-part transform. Rings permit axial yaw, not arbitrary tilt.
    for part in ['ring1','ring2','pin']:
        P,Rp=goals[part];wins=[];failures=[]
        offsets=[.003,.005]if part!='pin'else[.004,.008,.012,-.012]
        for offset in offsets:
            for yaw in range(0,360,30):
                L=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@np.diag([1.,-1.,-1.]);lp=np.array([0,0,offset]);R=Rp@L;tcp=P+Rp@lp
                for side in ['right','left']:
                    good,detail=branches(side,tcp,R,.075 if part=='ring1'else .062 if part=='ring2'else .013,n=12)
                    if good:wins.append(dict(side=side,localP=lp.tolist(),localR=L.tolist(),tcp=tcp.tolist(),R=R.tolist(),q=[x.tolist()for x in good],grasp_z_offset=offset,yaw_deg=yaw))
                    else:failures.append(dict(side=side,offset=offset,yaw=yaw,**detail))
            if wins:break
        report['parts'][part]=dict(insertion_candidates=wins,failures=failures);save();print('BIN_CONSISTENT_INSERT',part,len(wins),flush=True)
