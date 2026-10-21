"""No-motion saved RGB-D/encoder donor endpoint cause probe for317."""
from pathlib import Path
import json,types,numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame

def execute(c):
    here=Path(__file__).resolve().parent;source=here.parents[1]/'runs'/c['config']['handover_probe_source_run'];observations=json.loads((source/'rgb-observations.json').read_text());o=max((o for o in observations if o['mode']=='held'and o.get('part')=='ring1'and o['result'].get('pose')),key=lambda o:o['captured_wall_s']);P=frame(o['result']['pose']['center_m'],axis_frame(o['result']['pose']['axis']));G=np.linalg.inv(o['hand_TCP_world'])@P;a=np.load(source/'executed-actuator-commands.npz');command=a['command'][-1:].copy();actual=a['actual_dof_position'][-1:].copy()
    class RobotSnapshot:
        def get_dof_positions(self):return actual
    snapshot=dict(c,cmd=command,art=RobotSnapshot(),config=dict(c['config'],temporary_axial_double_handover_component=True,receiver_axial_only=True,allow_direct_right_insert=False))
    rc,lc=c['checkers']['right'],c['checkers']['left'];old_clear=rc.clear;rejections=[]
    def record(q,opening,selected=None):
        prospective=rc.preview_other_boxes is not None
        if prospective:ok,why=old_clear(q,opening,selected)
        else:
            with rc.other_arm_at(lc,actual[0,c['left_ids']],float(actual[0,c['left_fi']].sum())):ok,why=old_clear(q,opening,selected)
        if not ok:rejections.append(dict(q=np.asarray(q).tolist(),opening_m=float(opening),why=why,prospective_other_arm=prospective))
        return ok,why
    rc.clear=record;module=types.ModuleType('paired_handover');module.__file__=str(here/'paired_handover.py');planner_source=Path(module.__file__).read_text();assert planner_source.count('reference[:3,3]+=[-.10,0.,-.20]')==1;planner_source=planner_source.replace('reference[:3,3]+=[-.10,0.,-.20]','reference[:3,3]+=[-.15,0.,-.20]');exec(compile(planner_source,module.__file__,'exec'),module.__dict__)
    try:
        try:plan=module.plan(snapshot,'ring1',G,donor_release_width=.08);planner_result=dict(found=True,incoming=plan['incoming'].tolist())
        except RuntimeError as e:planner_result=dict(found=False,error=str(e))
    finally:rc.clear=old_clear
    unique=[]
    for r in rejections:
        if not r['prospective_other_arm']and all(np.max(abs(np.asarray(r['q'])-v['q']))>.01 for v in unique):unique.append(dict(r,q=np.asarray(r['q'])))
    from grasp_transfer_feasibility import receiving_poses
    partP=np.asarray(c['handoff_reference']['ring1']['part_H']).copy();partP[:3,:3]=np.eye(3);partP[:3,3]+=[-.15,0,-.20];target,_=next(receiving_poses(snapshot,'ring1',partP));pre=target.copy();pre[:3,3]-=pre[:3,2]*.020;seeds=[c['handoff_reference']['ring1']['left_receive_q'],command[0,c['left_ids']]]+list(np.random.default_rng(317).uniform(c['left_arm'].bounds[:,0],c['left_arm'].bounds[:,1],(8,7)));left_solutions=[]
    for seed in seeds:
        q=c['left_arm'].solve(pre[:3,3],pre[:3,:3],seed)
        if q is not None and all(np.max(abs(q-old))>.01 for old in left_solutions):left_solutions.append(q)
    comparisons=[];preposition_routes=[];future_plans=[];saved=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative);rc.attach_payload(c['part_collision_templates']['ring1'],frame([0,0,.1034],np.eye(3))@G)
    try:
        for lq in left_solutions:
            with lc.other_arm_at(rc,command[0,c['ids']],float(actual[0,c['fi']].sum())):
                left_clear,left_why=lc.clear(lq,.0235)
                empty_route=lc.route(command[0,c['left_ids']],lq,.0235,seconds=.5)if left_clear else None
            preposition_routes.append(dict(left_pre_q=lq.tolist(),found=empty_route is not None,route=None if empty_route is None else np.asarray(empty_route).tolist(),endpoint_rejection=left_why,RIGHT_held_payload_protected=True,robot_motion_executed=False))
            if empty_route is not None:
                future_command=command.copy();future_command[0,c['left_ids']]=lq;future_actual=actual.copy();future_actual[0,c['left_ids']]=lq;future_actual[0,c['left_fi']]=.0235/2
                class FutureRobotSnapshot:
                    def get_dof_positions(self):return future_actual
                future_snapshot=dict(snapshot,cmd=future_command,art=FutureRobotSnapshot())
                with rc.other_arm_at(lc,lq,.0235):
                    try:
                        replanned=module.plan(future_snapshot,'ring1',G,donor_release_width=.08)
                        future_plans.append(dict(found=True,left_pre_q=lq.tolist(),donor_route=np.asarray(replanned['donor_route']).tolist(),receiver_route=np.asarray(replanned['receiver_route']).tolist(),receiver_q=replanned['receiver_q'].tolist(),incoming=replanned['incoming'].tolist(),model_only=True))
                    except RuntimeError as error:future_plans.append(dict(found=False,left_pre_q=lq.tolist(),error=str(error),model_only=True))
            rc.attach_payload(c['part_collision_templates']['ring1'],frame([0,0,.1034],np.eye(3))@G)
            for row in unique:
                with rc.other_arm_at(lc,lq,.0235):ok,why=old_clear(row['q'],row['opening_m'])
                comparisons.append(dict(right_q=row['q'].tolist(),left_pre_q=lq.tolist(),original_rejection=row['why'],future_LEFT_endpoint_clear_with_current_RIGHT=left_clear,future_LEFT_endpoint_rejection=left_why,right_endpoint_clear_with_future_LEFT=ok,remaining_rejection=why))
    finally:rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved
    for r in unique:r['q']=r['q'].tolist()
    result=dict(state='saved_image_robot_endpoint_cause_probe_only',full_task_pass=False,robot_motion_executed=False,uses_free_body_poses=False,source=source.name,source_RGBD_capture_s=o['captured_wall_s'],measured_grasp=G.tolist(),planner=planner_result,recorded_rejections=rejections,unique_donor_endpoint_rejections=unique,left_standoff_solutions=len(left_solutions),adjusted_air_part_P=partP.tolist(),empty_LEFT_preposition_routes=preposition_routes,recomputed_future_context_handover=future_plans,future_context_comparisons=comparisons,execution_requirement='Any future-context success requires a separately checked actual empty LEFT reposition before replanning; no robot was placed at these prospective poses.')
    out=c['vision'].out/'double-handover-endpoint-cause.json';out.write_text(json.dumps(result,indent=2));print('DOUBLE_ENDPOINT_CAUSE_ADJUSTED',len(unique),'donor_rejections',len(left_solutions),'left_standoff_solutions',sum(v['right_endpoint_clear_with_future_LEFT']for v in comparisons),'future_context_admitted',flush=True);return result
