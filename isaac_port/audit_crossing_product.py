"""Screen cross-arm pose combinations, including arbitrarily delayed contacts."""
from pathlib import Path
import argparse,json
import numpy as np,mujoco,fcl
from scipy.interpolate import CubicSpline
from source_model import load_model
from plan_right_posture import Checker
from optimization_clock import require_optimization_window,install_cutoff_alarm


def audit(candidate,left_interval,right_interval,samples,output,respect_cutoff=False):
    if respect_cutoff:require_optimization_window(120);install_cutoff_alarm()
    root=Path(__file__).resolve().parent;z=np.load(candidate/'trajectory.npz');plan=np.load(candidate/'arm-posture-fr3.npz')
    src=json.loads((root/'generated/contact_task/contact-source.json').read_text());model=load_model();data=mujoco.MjData(model)
    geometry=Checker.__new__(Checker);geometry.arr=np.load(root/'generated/contact_task/collision.npz');geometry.cache={};geometry.layout='physical'
    robots={s:[]for s in ['left','right']};armids={s:np.array([int(model.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0])for j in range(1,8)])for s in robots}
    for row in src['geoms']:
        for side in robots:
            if not row['body'].startswith(side+'_'):continue
            vertices=geometry.arr[row['mesh']+'_vertices']if row['kind']==7 else np.array([row.get('size',[.1]*3),-np.array(row.get('size',[.1]*3))])
            robots[side].append((row,fcl.CollisionObject(geometry.shape(row,True)),float(np.linalg.norm(vertices,axis=1).max()),int(model.body(row['body']).id)))
    radius=(np.array([r[2]for r in robots['left']])[:,None]+np.array([r[2]for r in robots['right']])[None,:])**2
    poses={};times={s:np.linspace(*interval,samples)for s,interval in [('left',left_interval),('right',right_interval)]}
    for side in robots:
        spline=CubicSpline(plan['time_s'],plan[side+'_qpos'],axis=0);poses[side]=[]
        for t in times[side]:
            data.qpos[:]=[np.interp(t,z['time_s'],z['qpos'][:,j])for j in range(model.nq)];data.qpos[armids[side]]=spline(t);mujoco.mj_kinematics(model,data)
            centers=[];rotations=[]
            for row,obj,r,b in robots[side]:
                rot=data.xmat[b].reshape(3,3);centers.append(data.xpos[b]+rot@row['position']);rotations.append(rot@geometry.rot(row['quaternion']))
            poses[side].append((np.array(centers),rotations))
    failures=[];count=0
    for il,(cl,rl)in enumerate(poses['left']):
        if respect_cutoff:require_optimization_window()
        for i,(_,obj,_,_)in enumerate(robots['left']):obj.setTransform(fcl.Transform(rl[i],cl[i]))
        for ir,(cr,rr)in enumerate(poses['right']):
            count+=1
            for i,(_,obj,_,_)in enumerate(robots['right']):obj.setTransform(fcl.Transform(rr[i],cr[i]))
            close=np.argwhere(np.sum((cl[:,None,:]-cr[None,:,:])**2,axis=2)<=radius)
            for a,b in close:
                la,oa,_,_=robots['left'][a];rb,ob,_,_=robots['right'][b]
                if fcl.collide(oa,ob,fcl.CollisionRequest(),fcl.CollisionResult()):
                    failures.append(dict(left_reference=float(times['left'][il]),right_reference=float(times['right'][ir]),pair=[la['body'],rb['body']]))
                    break
            if failures:break
        if failures:break
    result=dict(sampled_cross_reference_product_pass=not failures,left_interval=list(left_interval),right_interval=list(right_interval),samples_per_axis=samples,checked_combinations=count,failures=failures,hardware_ready=False,scope='Sampled convex robot hulls only, no tracking-error reserve or carried-part pair proof; arbitrary waiting within the stated reference intervals')
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--left',type=float,nargs=2,required=True);p.add_argument('--right',type=float,nargs=2,required=True);p.add_argument('--samples',type=int,default=61);p.add_argument('--output',type=Path,required=True);p.add_argument('--respect-cutoff',action='store_true');a=p.parse_args();audit(a.candidate,a.left,a.right,a.samples,a.output,a.respect_cutoff)
