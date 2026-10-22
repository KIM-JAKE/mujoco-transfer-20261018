"""Static small-ring insertion with a fully checked empty-left parking route.

A proposed route is not executed. Saved RGB-D measurements and robot encoder
samples provide the held-right configuration; no free-object state is read.
"""
from pathlib import Path
import json,time
import numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame
from grasp_transfer_feasibility import pad_aperture
from receiver_pad_contact import contact_state
from probe_native_right_ring import fixture_clear


def execute(c):
    began=time.perf_counter();part='ring2';here=Path(__file__).resolve().parent
    name=c['config'].get('insertion_probe_source_run','native-roas-table-scatter-stable-v191-right-physical-447-ring2')
    source=here.parents[1]/'runs'/name
    report=json.loads((source/'pipeline-results.json').read_text());trace=np.load(source/'physx-sequence-trace.npz');observations=json.loads((source/'rgb-observations.json').read_text())
    rows=[v for v in observations if v['part']==part and v['mode']=='held'and v['result'].get('pose')]
    if not rows:raise RuntimeError('No saved direct RGB-D small-ring hold')
    observation=rows[-1];index=int(np.argmin(abs(trace['time_s']-observation['captured_wall_s'])))
    joint_ids=[report['dof_names'].index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)]
    finger_ids=[i for i,n in enumerate(report['dof_names'])if n.startswith('right_native_')]
    if len(finger_ids)!=2:raise RuntimeError('Saved native right finger encoders ambiguous')
    actual_right=trace['q'][index,joint_ids];actual_width=float(trace['q'][index,finger_ids].sum());pose=observation['result']['pose']
    G=np.linalg.inv(c['arm'].fk(actual_right))@frame(pose['center_m'],axis_frame(pose['axis']))
    expected=pad_aperture(c,part,G);contact=contact_state(dict(c,config=dict(c['config'],receiver_axial_only=False)),part,G,actual_width)
    if expected is None or not contact['opposed_contact_compatible']:raise RuntimeError('Saved small pickup lacks native bilateral contact')
    width=max(expected,actual_width);left=c['left_arm'];lc=c['checkers']['left'];rc=c['checkers']['right'];left_start=c['art'].get_dof_positions()[0,c['left_ids']].copy();left_width=float(c['art'].get_dof_positions()[0,c['left_fi']].sum());H=left.fk(left_start)
    budget=float(c['config'].get('axial_receiver_fixture_budget_s',90.));deadline=began+budget
    counts={};examples={};trials=[];approved=None
    def reject(label,why=None):
        counts[label]=counts.get(label,0)+1
        if why is not None and label not in examples:examples[label]=why
    deltas=[[-.10,-.10,-.03],[-.20,-.10,-.03],[-.10,0.,-.03],[-.20,0.,-.03],[0.,-.10,-.03],[-.15,-.15,0.],[-.20,-.20,0.]]
    rng=np.random.default_rng(19621);seeds=[left_start,np.asarray(c['handoff_reference'][part]['left_receive_q'])]+list(rng.uniform(left.bounds[:,0],left.bounds[:,1],(12,7)))
    for delta in deltas:
        if time.perf_counter()>deadline:reject('park_budget');break
        target=H.copy();target[:3,3]+=delta;branches=[]
        for seed in seeds:
            q=left.solve(target[:3,3],target[:3,:3],seed)
            if q is not None and all(np.linalg.norm(q-old)>.05 for old in branches):branches.append(q)
        if not branches:reject('park_IK');continue
        for park_q in branches:
            if time.perf_counter()>deadline:reject('park_budget');break
            # Load only query geometry. The simulation's bodies and robot
            # actors remain untouched throughout this diagnostic.
            saved=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative)
            rc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
            try:
                with lc.other_arm_at(rc,actual_right,actual_width):
                    clear,why=lc.clear(park_q,left_width)
                    if not clear:reject('left_park_endpoint',why);continue
                    if lc.path([left_start,park_q],left_width)[0]:route=[left_start,park_q]
                    else:route=lc.route(left_start,park_q,left_width,seconds=.35)
                    if route is None:reject('left_park_route');continue
                    clear,why=lc.path(route,left_width)
                    if not clear:reject('left_park_exact_edges',why);continue
            finally:rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved
            # Collision caches for right fixture feasibility are dependent on
            # the stationary left pose. Each proposed park is a fresh context.
            ctx=dict(c,cmd=c['cmd'].copy(),_right_direct_measured_q=actual_right.copy(),_right_direct_fixture_cache={},_right_direct_fixture_counts={},_right_direct_fixture_examples={})
            ctx['cmd'][0,c['ids']]=actual_right;ctx['cmd'][0,c['left_ids']]=park_q
            # Bound one pose's search to leave time for alternate left parks.
            local_deadline=min(deadline,time.perf_counter()+15.)
            with rc.other_arm_at(lc,park_q,left_width):fixture=fixture_clear(ctx,part,G,width,deadline=local_deadline)
            row=dict(left_TCP_delta_m=delta,left_park_q=park_q.tolist(),left_park_route=np.asarray(route).tolist(),left_opening_m=left_width,left_route_executed=False,right_fixture=fixture,right_counts=ctx['_right_direct_fixture_counts'],right_examples=ctx['_right_direct_fixture_examples'])
            trials.append(row);print('SMALL_RIGHT_LEFT_PARK_CANDIDATE',json.dumps(row),flush=True)
            if fixture is not None:approved=row;break
        if approved is not None:break
    result=dict(state='static_small_right_with_left_park_probe_only',full_task_pass=False,uses_free_body_poses=False,robot_motion_executed=False,physical_insertion_validated=False,source_run=name,source_camera_capture_wall_s=observation['captured_wall_s'],source_encoder_wall_s=float(trace['time_s'][index]),measured_grasp=G.tolist(),held_right_q=actual_right.tolist(),held_right_actual_opening_m=actual_width,native_contact=contact,approved=approved,left_parking_counts=counts,left_parking_examples=examples,trials=trials,elapsed_s=time.perf_counter()-began)
    print('SMALL_RIGHT_LEFT_PARK_PROBE',json.dumps(result),flush=True)
    return result
