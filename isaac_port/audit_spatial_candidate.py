"""Sample each arm against original physical CAD and carried-part geometry.

Other-arm timing is intentionally excluded here and checked separately. This
screen never changes collision settings in the native PhysX acceptance run.
"""
from pathlib import Path
import argparse,json
import numpy as np
import mujoco,fcl
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation,Slerp
from source_model import load_model
from plan_right_posture import Checker
from task_timing_fast import source_time
from task_path_offsets import pin_clearance_z
from optimization_clock import require_optimization_window,install_cutoff_alarm


def audit(candidate,output,respect_cutoff=False):
    if respect_cutoff:
        require_optimization_window(120);install_cutoff_alarm()
    root=Path(__file__).resolve().parent
    z=np.load(candidate/'trajectory.npz');plan=np.load(candidate/'arm-posture-fr3.npz');ts=z['time_s']
    model=load_model();data=mujoco.MjData(model)
    checker=Checker(candidate,source_clock=source_time,include_cross_arm=False)
    armids={s:np.array([int(model.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0])for j in range(1,8)])for s in ['left','right']}
    curves={s:CubicSpline(ts,plan[s+'_qpos'],axis=0)for s in armids}
    schedule=json.loads((candidate/'independent_schedule.json').read_text())
    source=json.loads((root/'generated/contact_task/contact-source.json').read_text())
    roles={'ring1':('right','lift_two_parts_30mm','transfer_two_parts',0),'ring2':('left','lift_two_parts_30mm','transfer_two_parts',1),'pin':('left','lift_last_pin_30mm','transfer_last_pin',2)}
    shapes={};windows={};rotations={}
    for name,(side,lift,transfer,index)in roles.items():
        shapes[name]=[(row,fcl.CollisionObject(checker.shape(row,True)))for row in source['parts'][name]['colliders']]
        lane=schedule['lanes'][side];windows[name]=(next(r['end_s']for r in lane if r['stage']['label']==lift),next(r['end_s']for r in lane if r['stage']['label']==transfer))
        rotations[name]=Slerp(ts,Rotation.from_quat(z['mocap_quat'][:,index][:,[1,2,3,0]]))
    failures=[];counts={};last=float(ts[-1])
    for side in ['left','right']:
        checker.side=side;counts[side]=0
        for t in ts:
            if respect_cutoff:require_optimization_window()
            data.qpos[:]=np.array([np.interp(t,ts,z['qpos'][:,j])for j in range(model.nq)])
            for arm in armids:data.qpos[armids[arm]]=curves[arm](t)
            mujoco.mj_kinematics(model,data)
            checker.current_t={side:float(t),('right'if side=='left'else'left'):last}
            valid,why=checker.check(model,data)
            counts[side]+=1
            if not valid:failures.append(dict(side=side,reference=float(t),scope='robot',reason=why));break
            for name,(arm,lift,transfer,index)in roles.items():
                begin,end=windows[name]
                if arm!=side or not begin+.01<t<end-.01:continue
                p=np.array([np.interp(t,ts,z['mocap_pos'][:,index,j])for j in range(3)])
                if name=='pin':p[2]+=pin_clearance_z(source_time('left',float(t)))
                rot=rotations[name](float(t)).as_matrix()
                for row,obj in shapes[name]:
                    obj.setTransform(fcl.Transform(rot@checker.rot(row['quaternion']),p+rot@row['position']))
                    query=fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=1))
                    checker.manager.collide(obj,query,fcl.defaultCollisionCallback)
                    if query.result.is_collision:
                        failures.append(dict(side=side,reference=float(t),part=name,scope='carried_part_vs_machine'));break
                    for part,installed_side,released,manager,objects in checker.installed:
                        if part==name or source_time(installed_side,checker.current_t[installed_side])<released:continue
                        query=fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=1))
                        manager.collide(obj,query,fcl.defaultCollisionCallback)
                        if query.result.is_collision:
                            failures.append(dict(side=side,reference=float(t),part=name,scope='carried_part_vs_installed_'+part));break
                    if failures:break
                if failures:break
            if failures:break
        if failures:break
    result=dict(sampled_spatial_pass=not failures,samples_by_side=counts,failures=failures,
                scope='Sampled robot self/static and conservative carried-part hulls vs physical CAD; other arm assumed clear and its ring already installed. Requires stock-pin, simultaneous-path, dynamics, contact and native FR3 tests.',hardware_ready=False,physical_simulation_pass=False)
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--respect-cutoff',action='store_true')
    a=p.parse_args();audit(a.candidate,a.output,a.respect_cutoff)
