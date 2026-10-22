"""C2 endpoint-matched joint shortcuts; a new unvalidated spatial candidate."""
from pathlib import Path
import argparse,json,shutil
import numpy as np
import mujoco
from scipy.interpolate import CubicSpline,make_interp_spline
from scipy.spatial.transform import Rotation
from source_model import load_model
from plan_right_posture import Checker
from task_timing_fast import source_time
from task_path_offsets import pin_clearance_z
from optimization_clock import require_optimization_window,install_cutoff_alarm


SEGMENTS={
 'ring2':('left',1,.5797916666666667,1.538125,[.819375,1.2985416666666667]),
 'left-return':('left',None,2.121458333333333,2.65,[2.3857291666666667]),
 'pin':('left',2,3.357916666666667,5.107916666666667,[]),
 'ring1':('right',0,2.3,3.408333333333333,[]),
 'right-return':('right',None,4.15,5.1,[4.3,4.6,4.9])}


def prepare(base,output,segments,*,midpoint_mode='original',respect_cutoff=False,pin_begin=None):
    if respect_cutoff:require_optimization_window(120);install_cutoff_alarm()
    z=np.load(base/'trajectory.npz');plan=np.load(base/'arm-posture-fr3.npz');ts=z['time_s']
    curves={s:CubicSpline(ts,plan[s+'_qpos'],axis=0)for s in ['left','right']}
    rows={s:plan[s+'_qpos'].copy()for s in curves};positions=z['mocap_pos'].copy();quats=z['mocap_quat'].copy()
    model=load_model();old_data=mujoco.MjData(model);data=mujoco.MjData(model)
    addresses={s:np.array([int(model.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0])for j in range(1,8)])for s in curves}
    checker=Checker(base,source_clock=source_time,include_cross_arm=False);records=[]
    for label in segments:
        side,index,begin,end,mids=SEGMENTS[label];original=curves[side]
        if label=='pin' and pin_begin is not None:
            if not begin<=pin_begin<end-.2:raise ValueError('Pin shortcut must start after extraction and before final approach')
            begin=float(pin_begin)
        if midpoint_mode=='none':mids=[]
        elif midpoint_mode=='dense':mids=np.linspace(begin,end,7)[1:-1].tolist()
        points=[begin,*mids,end]
        curve=make_interp_spline(points,original(points),k=5,axis=0,
              bc_type=([(1,original(begin,1)),(2,original(begin,2))],
                       [(1,original(end,1)),(2,original(end,2))]))
        checker.side=side
        for t in np.linspace(begin,end,181):
            if respect_cutoff:require_optimization_window()
            data.qpos[:]=[np.interp(t,ts,z['qpos'][:,j])for j in range(model.nq)]
            for s in curves:data.qpos[addresses[s]]=curve(t)if s==side else curves[s](t)
            mujoco.mj_kinematics(model,data)
            checker.current_t={side:float(t),('right'if side=='left'else'left'):float(ts[-1])}
            passed,reason=checker.check(model,data)
            if not passed:raise RuntimeError(f'{label} static screen failed at {t}: {reason}')
        active=np.flatnonzero((ts>=begin)&(ts<=end));rows[side][active]=curve(ts[active])
        if index is not None:
            hand=int(model.body(side+'_grip_base').id)
            for k in active:
                old_data.qpos[:]=z['qpos'][k]
                for s in curves:old_data.qpos[addresses[s]]=curves[s](ts[k])
                mujoco.mj_kinematics(model,old_data);data.qpos[:]=old_data.qpos;data.qpos[addresses[side]]=rows[side][k];mujoco.mj_kinematics(model,data)
                delta=data.xmat[hand].reshape(3,3)@old_data.xmat[hand].reshape(3,3).T
                part=z['mocap_pos'][k,index].copy()
                clearance=pin_clearance_z(source_time('left',float(ts[k])))if index==2 else 0.
                part[2]+=clearance;positions[k,index]=data.xpos[hand]+delta@(part-old_data.xpos[hand]);positions[k,index,2]-=clearance
                old_r=Rotation.from_quat(z['mocap_quat'][k,index,[1,2,3,0]]).as_matrix()
                quats[k,index]=Rotation.from_matrix(delta@old_r).as_quat()[[3,0,1,2]]
        records.append(dict(segment=label,side=side,begin=begin,end=end,midpoints=mids,static_samples=181))
    output.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(output/'arm-posture-fr3.npz',time_s=ts,**{s+'_qpos':q for s,q in rows.items()})
    np.savez_compressed(output/'trajectory.npz',time_s=ts,qpos=z['qpos'],mocap_pos=positions,mocap_quat=quats)
    for f in ['config.json','independent_schedule.json','retiming.json']:shutil.copy2(base/f,output/f)
    report=dict(state='new_spatial_candidate_requires_full_screen_and_PhysX',hardware_ready=False,base=str(base),interpolation='quintic, matching endpoint q/dq/ddq',segments=records)
    (output/'arm-posture-fr3.json').write_text(json.dumps(report,indent=2)+'\n')
    stage=output/'planning-base';(stage/'recording').mkdir(parents=True,exist_ok=True)
    for f in ['trajectory.npz','config.json','independent_schedule.json','retiming.json']:shutil.copy2(output/f,stage/'recording'/f)
    for f in ['arm-posture-fr3.npz','arm-posture-fr3.json']:shutil.copy2(output/f,stage/f)
    shutil.copy2(Path(__file__).parent/'assets/reference-dof-names.json',stage/'dof-names.json')
    print(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--segments',nargs='+',choices=list(SEGMENTS),required=True);p.add_argument('--midpoint-mode',choices=['original','none','dense'],default='original');p.add_argument('--respect-cutoff',action='store_true')
    p.add_argument('--pin-begin',type=float)
    a=p.parse_args();prepare(a.base,a.output,a.segments,midpoint_mode=a.midpoint_mode,respect_cutoff=a.respect_cutoff,pin_begin=a.pin_begin)
