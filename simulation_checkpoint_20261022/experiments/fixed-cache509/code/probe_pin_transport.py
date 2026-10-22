"""Static pin transport search: saved RGB-D/encoders, axial yaw free.

No physical body state is changed and this cannot claim task completion.
"""
from pathlib import Path
import json,time,numpy as np
from isaac_kinematics import frame,exp
from rgbd_geometry import axis_frame

def execute(c):
    h=Path(__file__).resolve().parent;r=h.parents[1]/'runs'/c['config']['pin_transport_probe_source_run'];report=json.loads((r/'pipeline-results.json').read_text());raw=np.load(r/'executed-actuator-commands.npz');cmd=raw['command'][-1];times=raw['time_s'];actual=raw['actual_dof_position'];ri=[report['dof_names'].index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)];li=[report['dof_names'].index('left_arm_fr3v2_joint'+str(i))for i in range(1,8)];ob=next(o for o in reversed(json.loads((r/'rgb-observations.json').read_text()))if o.get('part')=='pin'and o['mode']=='held'and 'pose'in o['result']);k=np.abs(times-ob['captured_wall_s']).argmin();arm=c['arm'];rc=c['checkers']['right'];lc=c['checkers']['left'];pc=c['pin_context'];cand=pc['candidate'];p=ob['result']['pose'];G=np.linalg.inv(arm.fk(actual[k,ri]))@frame(p['center_m'],axis_frame(p['axis']));old=np.asarray(cand['grasp_transform']);a=old[:3,2];b=G[:3,2];v=np.cross(a,b);G[:3,:3]=exp(v/max(np.linalg.norm(v),1e-12)*np.arctan2(np.linalg.norm(v),a@b))@old[:3,:3];goal=np.asarray(pc['goal']);start=cmd[ri];seeds=json.loads((h/'bin-picking/direct-pin-robot-seeds.json').read_text())['seeds'];pool=[seeds['pin_tail_carry_to_preinsert'],start,seeds['pin_tail_orient_free']]+list(np.random.default_rng(6153).uniform(arm.bounds[:,0],arm.bounds[:,1],(48,7)));started=time.perf_counter();counts={};examples={};found=[]
    def reject(key,why=None):
        counts[key]=counts.get(key,0)+1
        if why is not None and key not in examples:examples[key]=why
    left_width=float(cmd[[report['dof_names'].index('left_native_'+s)for s in ['left','right']]].sum())
    left_q=c['cmd'][0,c['left_ids']].copy() if c['config'].get('pin_transport_left_standby') else cmd[li]
    left_clear_route=None
    if c['config'].get('pin_transport_left_standby'):
        rc.attach_payload(pc['paths']['workpiece_pin'],frame([0,0,.1034],np.eye(3))@G)
        try:
            with lc.other_arm_at(rc,start,.0105):left_clear_route=lc.route(cmd[li],left_q,left_width,seconds=2.)
        finally:rc.detach_payload()
        print('PIN_LEFT_CLEARANCE_PROBE',left_clear_route is not None,flush=True)
    with rc.other_arm_at(lc,left_q,left_width):
        for yaw in np.radians([0,45,-45,90,-90,135,-135,180,30,-30,60,-60,120,-120,150,-150]):
            R=exp(np.array([0.,0.,yaw]))@np.asarray(cand['goal_part_R']);pre=frame(goal-[0,0,.045],R)@np.linalg.inv(G);end=frame(goal,R)@np.linalg.inv(G);solutions=[]
            for seed in pool:
                q=arm.solve(pre[:3,3],pre[:3,:3],seed)
                if q is not None and all(np.linalg.norm(q-old)>.05 for old in solutions):solutions.append(q)
            solutions.sort(key=lambda q:np.linalg.norm(q-start))
            for q in solutions:
                clear,why=rc.clear(q,.0105,pc['paths']['workpiece_pin'])
                if not clear:reject('pre_robot_collision',why);continue
                stroke=[q];valid=True
                for u in np.linspace(0,1,9)[1:]:
                    nq=arm.solve(pre[:3,3]*(1-u)+end[:3,3]*u,end[:3,:3],stroke[-1])
                    if nq is None:reject('stroke_IK');valid=False;break
                    clear,why=rc.path([stroke[-1],nq],.0105,pc['paths']['workpiece_pin'])
                    if not clear:reject('stroke_robot_collision',why);valid=False;break
                    stroke.append(nq)
                if not valid:continue
                rc.attach_payload(pc['paths']['workpiece_pin'],frame([0,0,.1034],np.eye(3))@G)
                try:route=rc.route(start,q,.0105,pc['paths']['workpiece_pin'],seconds=10.)
                finally:rc.detach_payload()
                if route is None:reject('carry_route');continue
                found.append(dict(goal_yaw_rad=float(yaw),goal_part_R=R.tolist(),pre_q=q.tolist(),stroke=np.asarray(stroke).tolist(),route=np.asarray(route).tolist()));break
            if found:break
    result=dict(state='pin_transport_geometry_only',full_task_pass=False,inputs='Saved pin RGB-D and robot encoders; fixed CAD',uses_free_body_poses=False,source_run=r.name,grasp_from_RGBD=G.tolist(),counts=counts,examples=examples,solutions=found,planning_s=time.perf_counter()-started,pin_axial_yaw_free_assumed=True,left_standby_required=bool(c['config'].get('pin_transport_left_standby')),left_clear_route=None if left_clear_route is None else np.asarray(left_clear_route).tolist(),left_standby_q=left_q.tolist())
    print('PIN_TRANSPORT_DIAGNOSTIC',json.dumps(result),flush=True);return result
