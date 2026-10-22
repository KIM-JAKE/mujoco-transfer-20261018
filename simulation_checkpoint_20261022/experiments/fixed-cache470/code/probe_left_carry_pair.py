"""Static LEFT carry proof using saved RGB-D + robot encoders, no motion."""
from pathlib import Path
import json,time,numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame
from grasp_transfer_feasibility import hinted_route

def execute(c):
    h=Path(__file__).resolve().parent;source=c['config'].get('left_carry_pair_probe_source_run','native-roas-table-scatter-stable-v235-broad-native-pad-all3-445');r=h.parents[1]/'runs'/source
    report=json.loads((r/'pipeline-results.json').read_text());raw=np.load(r/'executed-actuator-commands.npz');obs=json.loads((r/'rgb-observations.json').read_text());part=c['config'].get('pick_part','ring1');view=next(v for v in reversed(obs)if v.get('part')==part and v['mode']=='held_left'and v['result'].get('pose'))
    li=[report['dof_names'].index('left_arm_fr3v2_joint'+str(i))for i in range(1,8)];ri=[report['dof_names'].index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)];lf=[i for i,n in enumerate(report['dof_names'])if n.startswith('left_native_')];rf=[i for i,n in enumerate(report['dof_names'])if n.startswith('right_native_')]
    at=int(np.argmin(abs(raw['time_s']-view['captured_wall_s'])));initial=raw['command'][at,li];later=np.flatnonzero((raw['time_s']>view['captured_wall_s'])&(np.max(abs(raw['command'][:,li]-initial),axis=1)>1e-5));k=max(at,int(later[0])-1)if len(later)else at
    left_start=raw['command'][k,li].astype(float);left_actual=raw['actual_dof_position'][k,li].astype(float);right=raw['actual_dof_position'][k,ri].astype(float);lw=max(float(raw['actual_dof_position'][k,lf].sum()),float(raw['command'][k,lf].sum()));rw=float(raw['actual_dof_position'][k,rf].sum());p=view['result']['pose'];G=np.linalg.inv(np.asarray(view['hand_TCP_world']))@frame(p['center_m'],axis_frame(p['axis']))
    lc=c['checkers']['left'];rc=c['checkers']['right'];arm=c['left_arm'];row=c['handoff_reference'][part];goal=np.array(c['goals'][part]['P']);pre=frame(goal-[0,0,.080],np.eye(3))@np.linalg.inv(G);seeds=[row['approach_robot_seeds'][int(row['pre_index'])],row['left_insert_seed'],left_start]+list(np.random.default_rng(231035).uniform(arm.bounds[:,0],arm.bounds[:,1],(32,7)));counts={};examples={};started=time.perf_counter();deadline=started+45.;found=None
    def reject(label,why=None):
        counts[label]=counts.get(label,0)+1
        if why is not None and label not in examples:examples[label]=why
    saved=(lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative);lc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
    try:
        with lc.other_arm_at(rc,right,rw):
            start_ok,start_why=lc.clear(left_start,lw)
            if not start_ok:reject('current_start',start_why)
            solutions=[]
            for seed in seeds:
                if time.perf_counter()>deadline:break
                q=arm.solve(pre[:3,3],pre[:3,:3],seed)
                if q is not None and all(np.linalg.norm(q-old)>.05 for old in solutions):solutions.append(q)
            solutions.sort(key=lambda q:np.linalg.norm(q-left_start))
            for q in solutions:
                if time.perf_counter()>deadline:reject('budget');break
                ok,why=lc.clear(q,lw)
                if not ok:reject('payload_preinsert',why);continue
                route=hinted_route(c,part+'_left_carry',lc,left_start,q,lw,seconds=min(3.,max(.05,deadline-time.perf_counter())))
                if route is None:reject('carry_route');continue
                from fr3_path_timing import retime_safe_polyline,retime
                finger=np.array(raw['command'][k,lf]);knots=np.asarray(route);scale=max(1.,float(c['config'].get('stability_time_scale',1.)))
                if len(knots)>2 and not getattr(route,'verified_cubic',False):curve,_,_=retime_safe_polyline(knots,c['dt'],finger,finger,scale=scale)
                else:
                    try:
                        curve,_,timing=retime(knots,.05 if c['config'].get('optimized_motion_timing')else 1.4,c['dt'],finger,finger,optimize=bool(c['config'].get('optimized_motion_timing')))
                        if scale>1:curve,_,_=retime(knots,timing['duration_s']*scale,c['dt'],finger,finger,optimize=False)
                    except RuntimeError as error:
                        if 'Joint path leaves FR3 position limits'not in str(error):raise
                        curve,_,_=retime_safe_polyline(knots,c['dt'],finger,finger,scale=scale)
                oldres=getattr(lc,'resolution_rad',.035);lc.resolution_rad=min(oldres,.005)
                try:ok,why=lc.path(np.vstack([left_actual,curve.astype(np.float32).astype(float)]),lw)
                finally:lc.resolution_rad=oldres
                if not ok:reject('exact_retimed_payload_pair',why);continue
                found=dict(pre_q=q.tolist(),route=np.asarray(route).tolist(),exact_issued_curve_samples=len(curve));break
    finally:lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative=saved
    result=dict(state='static_current_left_carry_pair_probe',source_run=source,source_camera_wall_s=view['captured_wall_s'],source_encoder_wall_s=float(raw['time_s'][k]),part=part,inputs='Saved pre-carry RGB-D + actual robot encoders and fixed CAD; no workpiece truth',uses_free_body_poses=False,robot_motion_executed=False,full_task_pass=False,approved=found is not None,counts=counts,examples=examples,left_start=left_start.tolist(),right_actual=right.tolist(),left_opening_m=lw,right_opening_m=rw,measured_G=G.tolist(),solution=found,planning_s=time.perf_counter()-started)
    print('LEFT_CARRY_PAIR_PROBE',json.dumps(result),flush=True);return result
