"""Production-scene native pin grasp catalog; geometry only, no actuator writes.

Inputs are class CAD, pin RGB-D and current robot encoders. Saved trial actor
poses never initialize or control this diagnostic. Query payloads are planning
shapes only. Every candidate still needs a physical closing/retention trial.
"""
from pathlib import Path
import json,time
import numpy as np
from scipy.spatial import ConvexHull
from isaac_kinematics import frame,exp
from rgbd_geometry import axis_frame
from receiver_pad_contact import contact_state
from fr3_path_timing import retime,retime_safe_polyline
from aperture_path_check import path_openings
from pin_native_contact import native_support,axis_to_body_grasp




def execute(c):
    started=time.perf_counter();h=Path(__file__).resolve().parent;cfg=c['config'];pc=c['pin_context'];arm=c['arm'];rc=c['checkers']['right'];lc=c['checkers']['left'];art=c['art'];ids=c['ids'];fi=c['fi'];path=pc['paths']['workpiece_pin'];dt=c['dt'];source_name=cfg.get('pin_grasp_catalog_source_run','native-roas-table-scatter-stable-v246-deeper-pin-first-all3-446');source=h.parents[1]/'runs'/source_name
    rows=json.loads((source/'rgb-observations.json').read_text());ob=next(v for v in rows if v.get('part')=='pin'and v['mode']=='pin_initial'and v['result'].get('pose'));pose=ob['result']['pose'];pickup_P=np.asarray(pose['center_m']);pickup_R=axis_frame(pose['axis']);goal=np.asarray(cfg.get('pin_axis_goal_world_m',pc['goal']),float);actual=art.get_dof_positions()[0,ids].copy();command=c['cmd'][0,ids].copy();left_q=art.get_dof_positions()[0,c['left_ids']].copy();left_width=float(art.get_dof_positions()[0,c['left_fi']].sum());desired_left_width=.010;counts={};examples={};found=[];geometry=[];deadline=started+float(cfg.get('pin_grasp_catalog_budget_s',90.));legacy=pc['candidate'];hints=json.loads((h/'bin-picking/direct-pin-robot-seeds.json').read_text())['seeds'];oldres=getattr(rc,'resolution_rad',.035);rc.resolution_rad=min(oldres,.01)
    def reject(label,why=None):
        counts[label]=counts.get(label,0)+1
        if why is not None and label not in examples:examples[label]=why
    def clear(result,label):
        ok,why=result
        if not ok:reject(label,why)
        return ok
    def check_line(H0,H1,q0,opening,selected=path):
        qs=[np.asarray(q0)]
        for u in np.linspace(0,1,17)[1:]:
            q=arm.solve(H0[:3,3]*(1-u)+H1[:3,3]*u,H0[:3,:3],qs[-1])
            if q is None:reject('Cartesian_IK');return None
            if not clear(rc.path([qs[-1],q],opening,selected),'Cartesian_structure_or_pair'):return None
            qs.append(q)
        return np.asarray(qs)
    saved=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative);rc.detach_payload()
    try:
        # This is a checked alternative empty-hand aperture, not an unreported
        # physical state change. Runtime must actually perform this preshape.
        if abs(left_width-desired_left_width)>.0001:
            if not c.get('left_hand_known_empty',False):raise RuntimeError('Pin catalog requires logical empty LEFT before virtual checked preshape')
            with lc.other_arm_at(rc,actual,float(art.get_dof_positions()[0,fi].sum())):
                if not all(clear(lc.clear(left_q,float(w)),'empty_LEFT_preshape')for w in np.linspace(left_width,desired_left_width,25)):raise RuntimeError('No checked empty LEFT aperture sweep for pin catalog')
        left_width=desired_left_width
        for grip_z in cfg.get('pin_grasp_catalog_grip_z_m',[-.008,-.010,-.012,-.0155]):
            for tilt_deg in [-10,10,-15,15,-20,20,-25,25,0,-30,30,-35,35,-40,40,-45,45]:
                for contact_z in [0.,.002,.004,.006]:
                    RG=exp(np.array([0.,np.radians(tilt_deg),0.]));G=frame(np.array([0.,0.,contact_z])-RG@np.array([0.,0.,float(grip_z)]),RG);support=native_support(c,G)
                    row=dict(tilt_deg=tilt_deg,contact_z_m=contact_z,grip_local_pin_z_m=float(grip_z),G=G,support=support)
                    geometry.append(dict(tilt_deg=tilt_deg,contact_z_m=contact_z,grip_local_pin_z_m=float(grip_z),support=support))
                    if support['admitted']:found.append(row)
        found.sort(key=lambda row:(np.linalg.norm(row['G'][:3,3]),-row['support']['minimum_flat_pin_axis_span_m'],abs(row['tilt_deg'])))
        classes=found;found=[]
        with rc.other_arm_at(lc,left_q,left_width):
            if not all(clear(rc.clear(actual,float(w),path),'empty_RIGHT_preshape')for w in np.linspace(float(art.get_dof_positions()[0,fi].sum()),.014,25)):
                raise RuntimeError('No checked empty RIGHT aperture sweep for pin catalog')
            for row in classes:
                class_start_count=len(found)
                if time.perf_counter()>deadline:reject('budget');break
                G=row['G'];opening=.0105
                for yaw_deg in [180,165,-165,150,-150,135,-135,120,-120,105,-105,90,-90,75,-75,60,-60,45,-45,30,-30,15,-15,0]:
                    if time.perf_counter()>deadline:break
                    R=exp(np.array([0.,0.,np.radians(yaw_deg)]));end=frame(goal,R)@np.linalg.inv(G);pre=frame(goal-[0,0,.045],R)@np.linalg.inv(G)
                    if not clear(rc.hand_clear(end,opening,path),'goal_hand'):continue
                    if not all(clear(rc.hand_clear(end,float(w),path),'release_hand')for w in np.linspace(opening,.018,5)):continue
                    if not all(clear(rc.hand_clear(frame(end[:3,3]*(1-u)+pre[:3,3]*u,end[:3,:3]),.018,path),'withdraw_hand')for u in np.linspace(0,1,17)):continue
                    seeds=[hints.get('pin_direct_preinsert',legacy['insert_waypoints'][0]),legacy['insert_waypoints'][0],actual]+list(np.random.default_rng(62123).uniform(arm.bounds[:,0],arm.bounds[:,1],(12,7)));pre_solutions=[]
                    for seed in seeds:
                        q=arm.solve(pre[:3,3],pre[:3,:3],seed)
                        if q is not None and all(np.linalg.norm(q-old)>.05 for old in pre_solutions):pre_solutions.append(q)
                    for pre_q in pre_solutions:
                        stroke=check_line(pre,end,pre_q,opening)
                        if stroke is None:continue
                        if not all(clear(rc.clear(stroke[-1],float(w),path),'release_robot')for w in np.linspace(opening,.018,5)):continue
                        retreat=check_line(end,pre,stroke[-1],.018)
                        if retreat is None:continue
                        for pickup_yaw in [0,15,-15,30,-30,45,-45,60,-60,75,-75,90,-90,105,-105,120,-120,135,-135,150,-150,165,-165,180]:
                            if time.perf_counter()>deadline:break
                            PR=pickup_R@exp(np.array([0.,0.,np.radians(pickup_yaw)]));pick=frame(pickup_P,PR)@np.linalg.inv(G);pickpre=pick.copy();pickpre[2,3]+=.070
                            if not clear(rc.hand_clear(pick,.014,path),'pickup_hand'):continue
                            pickup_q=arm.solve(pick[:3,3],pick[:3,:3],legacy['pickup_q'])
                            if pickup_q is None:reject('pickup_IK');continue
                            pre_pick_q=arm.solve(pickpre[:3,3],pickpre[:3,:3],pickup_q)
                            if pre_pick_q is None:reject('pickup_pre_IK');continue
                            descent=check_line(pickpre,pick,pre_pick_q,.014)
                            if descent is None:continue
                            lift=check_line(pick,pickpre,descent[-1],opening)
                            if lift is None:continue
                            empty_route=rc.route(command,pre_pick_q,.014,path,seconds=min(.75,max(.05,deadline-time.perf_counter())))
                            if empty_route is None:reject('empty_pickup_route');continue
                            empty_finger=np.full(2,.007);empty_knots=np.asarray(empty_route)
                            if len(empty_knots)>2:empty_curve,_,_=retime_safe_polyline(empty_knots,dt,empty_finger,empty_finger,scale=1.,velocity_fraction=.95,acceleration=9.5,jerk=4750.)
                            else:empty_curve,_,_=retime(empty_knots,.05,dt,empty_finger,empty_finger,velocity_fraction=.95,acceleration=9.5,jerk=4750.,optimize=True)
                            if not clear(path_openings(rc,np.vstack([actual,empty_curve.astype(np.float32).astype(float)]),[.014],path),'issued_empty_pickup'):continue
                            rc.attach_payload(path,frame([0,0,.1034],np.eye(3))@axis_to_body_grasp(G))
                            try:
                                if not clear(rc.clear(lift[-1],opening,path),'payload_lift_endpoint'):continue
                                if not clear(rc.clear(pre_q,opening,path),'payload_preinsert'):continue
                                carry=rc.route(lift[-1],pre_q,opening,path,seconds=min(3.,max(.05,deadline-time.perf_counter())))
                                if carry is None:reject('payload_carry');continue
                                finger=np.full(2,opening/2);knots=np.asarray(carry)
                                if len(knots)>2:curve,_,_=retime_safe_polyline(knots,dt,finger,finger,scale=1.,velocity_fraction=.95,acceleration=9.5,jerk=4750.)
                                else:curve,_,_=retime(knots,.05,dt,finger,finger,velocity_fraction=.95,acceleration=9.5,jerk=4750.,optimize=True)
                                if not clear(path_openings(rc,curve.astype(np.float32).astype(float),[opening],path),'issued_payload_carry'):continue
                            finally:rc.detach_payload()
                            candidate=dict(tilt_deg=row['tilt_deg'],contact_z_m=row['contact_z_m'],grip_local_pin_z_m=row['grip_local_pin_z_m'],pose_origin='RGB-D physical symmetry-axis midpoint',CAD_body_to_axis_offset_m=[0.,-.00025,0.],yaw_deg=yaw_deg,goal_IK=True,goal_clear=True,line_clear=True,pickup_clear=True,pickup_yaw_deg=pickup_yaw,pickup_q=pickup_q.tolist(),pickup_H=pick.tolist(),pickup_part_R=PR.tolist(),pickup_center_from_RGBD=pickup_P.tolist(),grasp_transform=G.tolist(),insert_waypoints=stroke.tolist(),goal_part_R=R.tolist(),withdraw_waypoints=retreat.tolist(),native_pad_support=row['support'],requires_empty_left_opening_m=left_width,empty_pickup_route=np.asarray(empty_route).tolist(),loaded_carry_route=np.asarray(carry).tolist(),physical_grasp_tested=False,physical_insertion_tested=False)
                            found.append(candidate);print('PIN_NATIVE_CATALOG_CANDIDATE',dict(tilt_deg=row['tilt_deg'],contact_z_m=row['contact_z_m'],yaw_deg=yaw_deg,pickup_yaw_deg=pickup_yaw,support_span_m=row['support']['minimum_flat_pin_axis_span_m']),flush=True);break
                        if len(found)>class_start_count:break
                    if len(found)>class_start_count:break
                if len(found)>=int(cfg.get('pin_grasp_catalog_max_candidates',1)):break
        result=dict(state='native_pin_geometry_catalog',full_task_pass=False,hardware_tested=False,robot_motion_executed=False,uses_free_body_poses=False,source_run=source_name,pickup_center_from_RGBD=pickup_P.tolist(),physical_axis_goal_m=goal.tolist(),left_empty_preshape_virtual_checked=True,left_query_opening_m=left_width,candidates=found,class_grasp_geometries=geometry,counts=counts,examples=examples,planning_s=time.perf_counter()-started)
        out=Path(c['vision'].out)/'pin-native-grasp-catalog.json';out.write_text(json.dumps(result,indent=2));print('PIN_NATIVE_CATALOG_RESULT',json.dumps(dict(candidates=len(found),counts=counts,planning_s=result['planning_s'],output=str(out))),flush=True);return result
    finally:
        rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved;rc.resolution_rad=oldres
