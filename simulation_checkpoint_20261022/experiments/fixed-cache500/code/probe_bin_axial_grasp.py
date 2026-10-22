"""Fresh RGB-D bin axial-cap grasp feasibility screen; no robot commands.

Only measured RGB-D, known class/native CAD and current robot encoders enter
proposals. Live scene queries check fixed obstacles and the other arm. This
probe never reads/writes a free-body pose or executes a pickup.
"""
from pathlib import Path
import json,time
import numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame
from axial_rim_receiver import geometric_candidates


def execute(c):
    began=time.perf_counter();arm,art,ids,fi,vision,checker=[c[k]for k in ['arm','art','ids','fi','vision','checker']]
    measurements=[];view_reports=[];part='ring1'
    previous=vision.calibration.get('initial_eye_offset_m',[.04,-.04,.65])
    for offset in [previous,[.35,-.25,.50],[-.30,.25,.50]]:
        event=dict(mode='initial',part=None,bin_axial_probe=True,pickup_camera_eye_offset_m=list(offset))
        vision.calibration['initial_eye_offset_m']=offset
        try:future=vision.request(event,arm.fk(art.get_dof_positions()[0,ids]))
        finally:vision.calibration['initial_eye_offset_m']=previous
        deadline=time.perf_counter()+3.
        while not future.done():
            vision.poll();c['step']('bin_axial_probe_RGBD')
            if time.perf_counter()>deadline:raise RuntimeError('Bin axial probe RGB-D deadline')
        value=future.result();event['delivered_wall_s']=time.perf_counter()-c['started']
        view_reports.append(dict(offset_m=list(offset),captured_wall_s=value.get('captured_wall_s'),error=value['result'].get('error'),ring1_instances=sum(p.get('part')==part for p in value['result'].get('instances',[]))))
        if not value['result'].get('error')and 'packet'in value:measurements.append(value)
    poses=[]
    for value in measurements:
        for pose in value['result'].get('instances',[]):
            if pose.get('part')!=part or not pose.get('front_back_from_RGBD'):continue
            if any(np.linalg.norm(np.asarray(pose['center_m'])-old['center_m'])<.012 and np.asarray(pose['axis'])@old['axis']>.98 for old in poses):continue
            poses.append(pose)
    counts={};examples={};pose_reports=[];approved=[]
    def reject(kind,detail=None):
        counts[kind]=counts.get(kind,0)+1
        if detail is not None and len(examples.setdefault(kind,[]))<3:examples[kind].append(detail)
    actual=art.get_dof_positions().copy();right=np.asarray(actual[0,ids],float);actual_width=float(actual[0,fi].sum())
    report=dict(state='fresh_RGBD_bin_axial_cap_grasp_probe_only',part=part,views=view_reports,poses=pose_reports,counts=counts,examples=examples,approved=approved,uses_free_body_poses=False,robot_commands_issued=False,robot_motion_executed=False,physical_pickup_validated=False,full_task_pass=False,right_encoder_q=right.tolist(),right_encoder_opening_m=actual_width)
    def finish():
        report['elapsed_s']=time.perf_counter()-began
        (vision.out/'bin-axial-grasp-probe.json').write_text(json.dumps(report,indent=2,default=lambda v:v.tolist()if hasattr(v,'tolist')else str(v)))
        print('BIN_AXIAL_GRASP_PROBE',json.dumps(report,default=lambda v:v.tolist()if hasattr(v,'tolist')else str(v)),flush=True)
        return report
    if not measurements:reject('no_initial_RGBD_measurements');return finish()
    if not poses:reject('no_measured_large_ring_poses');return finish()
    if 'left'not in c.get('checkers',{})or 'left_ids'not in c or 'left_fi'not in c:
        reject('missing_current_other_arm_context');return finish()
    clearance=c['observed_grasp_clearance'];points=np.concatenate([clearance.points(v['packet'],c['calibration'])for v in measurements])
    ctx=dict(c,config=dict(c['config'],receiver_axial_only=True,stable_receiver_grasp=True));ctx.pop('_receiver_stable_contact_cache',None)
    budget=float(c['config'].get('axial_receiver_fixture_budget_s',45.));deadline=time.perf_counter()+budget
    left=np.asarray(actual[0,c['left_ids']],float);left_width=float(actual[0,c['left_fi']].sum());report.update(left_encoder_q=left.tolist(),left_encoder_opening_m=left_width,other_arm_and_current_payload_proxy_checked=True)
    old_resolution=getattr(checker,'resolution_rad',.035);checker.resolution_rad=min(old_resolution,.005)
    try:
        with checker.other_arm_at(c['checkers']['left'],left,left_width):
            for pose in poses:
                incoming=frame(pose['center_m'],axis_frame(pose['axis']));pose_row=dict(image_pose=pose,geometric_candidates=0,grasp_fixed_clear=0,pregrasp_fixed_clear=0,observed_stock_clear=0,IK_branches=0);pose_reports.append(pose_row)
                for H,G,width,metadata in geometric_candidates(ctx,part,incoming):
                    pose_row['geometric_candidates']+=1
                    if time.perf_counter()>deadline:reject('probe_budget');return finish()
                    opening=.0235
                    if width>=opening:reject('contact_width_exceeds_probe_opening');continue
                    good=True
                    for gap in np.linspace(width,opening,5):
                        ok,why=checker.hand_clear(H,float(gap))
                        if not ok:reject('grasp_hand_fixed',dict(image_center_m=pose['center_m'],parameters=list(metadata),opening_m=float(gap),collision=why));good=False;break
                    if not good:continue
                    pose_row['grasp_fixed_clear']+=1
                    for standoff in [.070,.105]:
                        pre=H.copy();pre[:3,3]-=H[:3,2]*standoff
                        ok,why=checker.hand_clear(pre,opening)
                        if not ok:reject('pregrasp_hand_fixed',dict(standoff_m=standoff,collision=why));continue
                        pose_row['pregrasp_fixed_clear']+=1
                        if not clearance.palm_clear(points,pose,pre,H):reject('observed_stock_palm');continue
                        if not clearance.clear(points,pose,pre,H,opening):reject('observed_other_stock_fingers');continue
                        if any(not checker.hand_clear(frame(pre[:3,3]*(1-u)+H[:3,3]*u,H[:3,:3]),opening)[0]for u in np.linspace(0,1,13)):
                            reject('approach_hand_fixed');continue
                        pose_row['observed_stock_clear']+=1
                        seeds=[right,c['pickup_seed']]+list(np.random.default_rng(9127).uniform(arm.bounds[:,0],arm.bounds[:,1],(32,7)));seen=[]
                        for seed in seeds:
                            if time.perf_counter()>deadline:reject('probe_budget');return finish()
                            q=arm.solve(pre[:3,3],pre[:3,:3],seed)
                            if q is None:reject('pregrasp_IK');continue
                            if any(np.linalg.norm(q-old)<.05 for old in seen):continue
                            seen.append(q);pose_row['IK_branches']+=1;ok,why=checker.clear(q,opening)
                            if not ok:reject('pregrasp_robot_fixed_or_pair',why);continue
                            # Arrive with the current empty-hand aperture,
                            # then preshape at this stationary pregrasp pose.
                            gaps=np.linspace(opening,max(actual_width,opening),5)
                            if any(not checker.clear(q,float(gap))[0]for gap in gaps):
                                reject('pregrasp_preshape_fixed_or_pair');continue
                            if any(not clearance.clear(points,pose,pre,pre,float(gap))for gap in gaps):
                                reject('pregrasp_preshape_observed_stock');continue
                            descent=[q];bad=False
                            for u in np.linspace(0,1,9)[1:]:
                                nxt=arm.solve(pre[:3,3]*(1-u)+H[:3,3]*u,H[:3,:3],descent[-1])
                                if nxt is None:reject('descent_IK');bad=True;break
                                ok,why=checker.path([descent[-1],nxt],opening)
                                if not ok:reject('descent_robot_fixed_or_pair',why);bad=True;break
                                descent.append(nxt)
                            if bad:continue
                            if any(not checker.clear(descent[-1],float(gap))[0]for gap in np.linspace(width,opening,5)):
                                reject('closing_robot_fixed_or_pair');continue
                            route=[right,q]if checker.path([right,q],max(actual_width,opening))[0]else None
                            if route is None:
                                for via in c.get('empty_return_waypoints',[]):
                                    if checker.path([right,via,q],max(actual_width,opening))[0]:route=[right,via,q];break
                            if route is None:reject('empty_approach_route');continue
                            approved.append(dict(image_pose=pose,parameters=list(metadata),grasp_TCP=H.tolist(),pregrasp_TCP=pre.tolist(),measured_part_to_gripper=G.tolist(),contact_width_m=float(width),opening_m=opening,standoff_m=standoff,pregrasp_route=np.asarray(route).tolist(),descent=np.asarray(descent).tolist(),lift_and_transfer_not_screened=True))
                            return finish()
    finally:checker.resolution_rad=old_resolution
    return finish()
