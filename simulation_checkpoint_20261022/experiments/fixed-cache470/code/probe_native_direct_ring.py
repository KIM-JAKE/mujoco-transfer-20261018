"""Geometry-only diagnostic of direct ring placement and opposite-arm pin."""
import json,time,numpy as np
from isaac_kinematics import frame

def execute(c):
    arm=c['arm'];check=c['checkers']['right'];cmd=c['cmd'];ids=c['ids'];seed0=cmd[0,ids].copy();pin=c['pin_context']['candidate'];seeds=[np.array(pin['insert_waypoints'][0]),seed0];results={};R0=np.diag([1.,-1.,-1.])
    for part,D,half in ([] if c['config'].get('left_pin_only_probe') else [('ring1',.075,.00775),('ring2',.062,.007)]):
        found=[];rejections={};started=time.perf_counter()
        for tilt_deg in [60,45,75,30,80,0]:
            tilt=np.radians(tilt_deg);Ry=np.array([[np.cos(tilt),0,np.sin(tilt)],[0,1,0],[-np.sin(tilt),0,np.cos(tilt)]]);offset=max(.0028,.0089*np.cos(tilt)+.00875*abs(np.sin(tilt))-half+.0012);grasp=frame([0,0,offset],R0@Ry);G=np.linalg.inv(grasp)
            for yaw in np.arange(0,360,30):
                a=np.radians(yaw);Rz=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1.]]);H=frame(c['goals'][part]['P'],Rz)@np.linalg.inv(G)
                for seed in seeds:
                    q=arm.solve(H[:3,3],H[:3,:3],seed)
                    if q is None:rejections['IK']=rejections.get('IK',0)+1;continue
                    ok,why=check.clear(q,D)
                    if ok:found.append(dict(tilt_deg=tilt_deg,axial_yaw_deg=int(yaw),q=q.tolist(),H=H.tolist(),grasp=G.tolist()));break
                    actor=why['actor'];rejections[actor]=rejections.get(actor,0)+1
                if found:break
            if found:break
        results[part]=dict(geometry_only=True,feasible_candidates=found,rejections=rejections,planning_s=time.perf_counter()-started);print('DIRECT_GOAL_PROBE',part,results[part],flush=True)
    left=c['left_arm'];checker=c['checkers']['left'];li=c['left_ids'];left_seeds=[cmd[0,li],np.array(c['handoff_reference']['ring1']['left_receive_q'])];pin_points=[('pickup',np.array(pin['pickup_H']),.014),('insert',frame(c['goals']['pin']['P'],np.array(pin['goal_part_R']))@np.linalg.inv(np.array(pin['grasp_transform'])),.0105)];left_pin={}
    for label,H,width in pin_points:
        rows=[]
        for seed in left_seeds:
            q=left.solve(H[:3,3],H[:3,:3],seed)
            if q is None:continue
            clear,why=checker.clear(q,width);rows.append(dict(q=q.tolist(),clear=clear,why=why))
        left_pin[label]=rows
    left_candidates=[]
    if c['config'].get('left_pin_only_probe'):
        from isaac_kinematics import exp
        initial_pin=np.array(pin['pickup_H'])@np.array(pin['grasp_transform']);P=initial_pin[:3,3];pickupR=np.diag([1.,-1.,-1.])
        for tilt_deg in [-45,-60,-75,-30,30,45,60,75]:
            RG=exp(np.array([0,np.radians(tilt_deg),0]));G=frame(np.array([0,0,.008])+RG@np.array([0,0,.0155]),RG)
            for pickup_yaw in [0,45,-45,90,-90,135,-135,180]:
                PR=pickupR@exp(np.array([0,0,np.radians(pickup_yaw)]));H=frame(P,PR)@np.linalg.inv(G);found_pick=None
                for seed in left_seeds+[np.array(pin['pickup_q'])]:
                    q=left.solve(H[:3,3],H[:3,:3],seed)
                    if q is not None and checker.clear(q,.014)[0]:found_pick=q;break
                if found_pick is None:continue
                for goal_yaw in [0,45,-45,90,-90,135,-135,180]:
                    GR=exp(np.array([0,0,np.radians(goal_yaw)]));goalH=frame(c['goals']['pin']['P'],GR)@np.linalg.inv(G);found_goal=None
                    for seed in left_seeds+[found_pick]:
                        q=left.solve(goalH[:3,3],goalH[:3,:3],seed)
                        if q is not None and checker.clear(q,.0105)[0]:found_goal=q;break
                    if found_goal is None:continue
                    pre=goalH.copy();pre[2,3]-=.045;preq=left.solve(pre[:3,3],pre[:3,:3],found_goal)
                    if preq is None or not checker.path([preq,found_goal],.0105)[0]:continue
                    row=dict(tilt_deg=tilt_deg,contact_z_m=.008,grip_local_pin_z_m=-.0155,yaw_deg=goal_yaw,pickup_yaw_deg=pickup_yaw,pickup_q=found_pick.tolist(),pickup_H=H.tolist(),pickup_part_R=PR.tolist(),grasp_transform=G.tolist(),insert_waypoints=[preq.tolist(),found_goal.tolist()],goal_part_R=GR.tolist());left_candidates.append(row);print('LEFT_PIN_CANDIDATE',row,flush=True);break
                if left_candidates:break
            if left_candidates:break
    return dict(state='direct_goal_geometric_probe',ring_goals=results,left_pin=left_pin,left_pin_candidates=left_candidates,physical_task_pass=False,full_task_pass=False)
