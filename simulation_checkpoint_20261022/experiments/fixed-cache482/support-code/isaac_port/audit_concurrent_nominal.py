"""Nominal cross-arm scheduling screen. Not a contact/physics acceptance test."""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
import mujoco
import fcl
from concurrent_profile import ConcurrentProfile
from plan_right_posture import Checker
from source_model import load_model
from optimization_clock import require_optimization_window


def audit(base, plan_root, output, respect_cutoff=False):
    if respect_cutoff:
        require_optimization_window(120)
        from optimization_clock import install_cutoff_alarm
        install_cutoff_alarm()
    root=Path(__file__).resolve().parent
    names=json.loads((base/'dof-names.json').read_text())
    source=json.loads((root/'generated/contact_task/contact-source.json').read_text())
    lookup={j['name']:j['qpos_address'] for j in source['joints']}
    z=np.load(base/'recording/trajectory.npz')
    plan=np.load(plan_root/'arm-posture-fr3.npz')
    q=z['qpos'][:,[lookup[n] for n in names]].copy()
    for i,n in enumerate(names):
        if '_arm_' in n:q[:,i]=plan[n.split('_')[0]+'_qpos'][:,int(n[-1])-1]
    profile=ConcurrentProfile(z['time_s'],q,names,json.loads((base/'recording/independent_schedule.json').read_text()),plan_root/'fr3-concurrent-profile.npz')
    model=load_model();data=mujoco.MjData(model)
    addresses=[int(model.joint(n).qposadr[0])for n in names]
    geometry=Checker.__new__(Checker)
    geometry.arr=np.load(root/'generated/contact_task/collision.npz');geometry.cache={};geometry.layout='physical'
    robots={side:[] for side in ['left','right']}
    for row in source['geoms']:
        for side in robots:
            if not row['body'].startswith(side+'_'):continue
            obj=fcl.CollisionObject(geometry.shape(row,True))
            vertices=geometry.arr[row['mesh']+'_vertices'] if row['kind']==7 else np.array([row.get('size',[.1]*3),-np.array(row.get('size',[.1]*3))])
            radius=float(np.linalg.norm(vertices,axis=1).max())
            robots[side].append((row,obj,radius,int(model.body(row['body']).id)))
    radii=(np.array([r[2]for r in robots['left']])[:,None]+np.array([r[2]for r in robots['right']])[None,:])**2
    release=next(r['end_s'] for r in json.loads((base/'recording/independent_schedule.json').read_text())['lanes']['left']if r['stage']['label']=='release_last_pin')
    references=dict(profile.last_reference);dt=1/120;failures=[];holds_log=[];previous_holds=None
    for step in range(120*30):
        if respect_cutoff and step%120==0:require_optimization_window()
        proposed=profile.peek(dt)
        proposed,holds=profile.coordinate(references,proposed,{'left':None,'right':None},nominal_only=True)
        gate=release-1e-8
        if references['right']<=gate and proposed['right']>gate and references['left']<release-1e-8:
            proposed['right']=gate;holds['right']='button_wait_for_all_releases'
        profile.gate(proposed,holds,True)
        profile.advance(dt)
        references=dict(profile.last_reference)
        # Match the controller's retained gate reference while an arm waits.
        for side in references:
            if holds[side]:references[side]=proposed[side]
        if holds!=previous_holds:
            holds_log.append(dict(t_s=step*dt,holds=dict(holds),references=dict(references)))
            previous_holds=dict(holds)
        data.qpos[addresses]=profile.full_path(profile.last_reference)
        mujoco.mj_kinematics(model,data)
        centers={}
        for side,items in robots.items():
            centers[side]=[]
            for row,obj,radius,body in items:
                rotation=data.xmat[body].reshape(3,3)
                pos=data.xpos[body]+rotation@row['position']
                obj.setTransform(fcl.Transform(rotation@geometry.rot(row['quaternion']),pos))
                centers[side].append(pos)
            centers[side]=np.array(centers[side])
        close=np.argwhere(np.sum((centers['left'][:,None,:]-centers['right'][None,:,:])**2,axis=2)<=radii)
        for ia,ib in close:
            ra,oa,_,_=robots['left'][ia];rb,ob,_,_=robots['right'][ib]
            if fcl.collide(oa,ob,fcl.CollisionRequest(),fcl.CollisionResult()):
                failures.append(dict(t_s=(step+1)*dt,references=dict(references),pair=[ra['body'],rb['body']]))
                break
        if len(failures)>=10:break
        if all(references[side]>=arm.bounds[-1]-1e-8 for side,arm in profile.arms.items()):break
    complete=all(references[side]>=arm.bounds[-1]-1e-8 for side,arm in profile.arms.items())
    result=dict(sampled_cross_arm_pass=not failures and complete,complete=complete,
                nominal_cycle_s=(step+1)*dt,failures=failures,coordination_events=holds_log,
                profile_sha256=hashlib.sha256((plan_root/'fr3-concurrent-profile.npz').read_bytes()).hexdigest(),
                scope='Nominal robot hull cross-arm collisions at 120Hz; no actual contact delays, dynamics, carried-part or static-environment proof',
                hardware_ready=False,physical_simulation_pass=False)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,required=True)
    p.add_argument('--plan-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--respect-cutoff',action='store_true')
    a=p.parse_args();audit(a.base,a.plan_root,a.output,a.respect_cutoff)
