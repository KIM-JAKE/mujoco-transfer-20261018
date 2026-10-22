"""Carry-aware right insertion screen from camera and both arm encoders.

This diagnostic proposes paths only. It reads no free-object transforms and
never sends archived commands or writes robot/part poses to the simulator.
"""
from pathlib import Path
import json,time
import numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame
from receiver_pad_contact import contact_state
from grasp_transfer_feasibility import pad_aperture,hinted_route
from probe_native_right_ring import fixture_clear


def execute(c):
    began=time.perf_counter();part='ring2';here=Path(__file__).resolve().parent
    name=c['config'].get('insertion_probe_source_run','native-roas-table-scatter-stable-v196-small-right-park-physical-447-ring2')
    source=here.parents[1]/'runs'/name;report=json.loads((source/'pipeline-results.json').read_text());trace=np.load(source/'physx-sequence-trace.npz');observations=json.loads((source/'rgb-observations.json').read_text())
    rows=[r for r in observations if r['part']==part and r['mode']=='held'and r['result'].get('pose')]
    if not rows:raise RuntimeError('No saved directly measured held small ring')
    observation=rows[-1];index=int(np.argmin(abs(trace['time_s']-observation['captured_wall_s'])))
    def joints(side):return [report['dof_names'].index(side+'_arm_fr3v2_joint'+str(i))for i in range(1,8)]
    def fingers(side):return [i for i,n in enumerate(report['dof_names'])if n.startswith(side+'_native_')]
    right=trace['q'][index,joints('right')].copy();left=trace['q'][index,joints('left')].copy();right_width=float(trace['q'][index,fingers('right')].sum());left_width=float(trace['q'][index,fingers('left')].sum());pose=observation['result']['pose']
    G=np.linalg.inv(c['arm'].fk(right))@frame(pose['center_m'],axis_frame(pose['axis']));expected=pad_aperture(c,part,G)
    contact=contact_state(dict(c,config=dict(c['config'],receiver_axial_only=False)),part,G,right_width)
    if expected is None or not contact['opposed_contact_compatible']:raise RuntimeError('Saved camera/native encoder state lacks bilateral outer contacts')
    width=max(expected,right_width);rc=c['checkers']['right'];lc=c['checkers']['left'];ctx=dict(c,cmd=c['cmd'].copy(),_right_direct_measured_q=right,_right_direct_fixture_cache={},_right_direct_fixture_counts={},_right_direct_fixture_examples={})
    ctx['cmd'][0,c['ids']]=right;ctx['cmd'][0,c['left_ids']]=left;ctx['cmd'][0,c['fi']]=trace['q'][index,fingers('right')];ctx['cmd'][0,c['left_fi']]=trace['q'][index,fingers('left')]
    ctx['_right_direct_fixture_context_key']=(np.round(right,8).tobytes(),np.round(left,8).tobytes(),round(right_width,8),round(left_width,8))
    budget=float(c['config'].get('axial_receiver_fixture_budget_s',90.));deadline=time.perf_counter()+budget;route_counts=dict(attempts=0,accepted=0,vertical_corridors_checked=0,vertical_corridors_clear=0,escape_routes_attempted=0);route_examples=[];route_cache={};escape_cache={}
    def carry_candidate(pre_q,grasp,opening):
        key=np.round(pre_q,7).tobytes()
        if key in route_cache:return route_cache[key]
        if time.perf_counter()>deadline:return None
        route_counts['attempts']+=1;attempt_began=time.perf_counter();saved=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative)
        rc.detach_payload();rc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@grasp)
        try:
            with rc.other_arm_at(lc,left,left_width):
                route=hinted_route(ctx,part+'_direct_right_carry',rc,right,pre_q,opening,seconds=min(3.,max(.05,deadline-time.perf_counter())))
                escaped_distance=None
                if route is None and c['config'].get('right_carry_vertical_escape'):
                    start_H=c['arm'].fk(right)
                    for distance in [.080,.150,.250]:
                        if time.perf_counter()>deadline:break
                        if distance not in escape_cache:
                            route_counts['vertical_corridors_checked']+=1;corridor=[right.copy()];valid=True;why=None
                            for u in np.linspace(0,1,9)[1:]:
                                target=start_H[:3,3]+np.array([0.,0.,distance*u]);q=c['arm'].solve(target,start_H[:3,:3],corridor[-1])
                                if q is None:valid=False;why=dict(IK=False,progress=float(u));break
                                clear,why=rc.path([corridor[-1],q],opening)
                                if not clear:valid=False;break
                                corridor.append(q)
                            escape_cache[distance]=corridor if valid else None
                            route_examples.append(dict(vertical_escape_m=distance,corridor_clear=valid,rejection=None if valid else why))
                            if valid:route_counts['vertical_corridors_clear']+=1
                        corridor=escape_cache[distance]
                        if corridor is None:continue
                        route_counts['escape_routes_attempted']+=1
                        upper=hinted_route(ctx,part+'_escaped_right_carry',rc,corridor[-1],pre_q,opening,seconds=min(3.,max(.05,deadline-time.perf_counter())))
                        if upper is None:continue
                        combined=list(corridor)+list(upper)[1:]
                        clear,why=rc.path(combined,opening)
                        if not clear:route_examples.append(dict(vertical_escape_m=distance,joined_route_rejection=why));continue
                        route=combined;escaped_distance=distance;break
                if route is not None:
                    # Execution uses these exact edges with corner stops,
                    # including when an old hint originally described a cubic.
                    clear,why=rc.path(route,opening)
                    if not clear:route_examples.append(dict(pre_q=np.asarray(pre_q).tolist(),rejection=why));route=None
                if route is not None:route_counts['accepted']+=1
        finally:rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved
        route_examples.append(dict(pre_q=np.asarray(pre_q).tolist(),route_found=route is not None,vertical_escape_m=escaped_distance,search_wall_s=time.perf_counter()-attempt_began))
        route_cache[key]=route;return route
    ctx['_right_direct_carry_route_callback']=carry_candidate
    with rc.other_arm_at(lc,left,left_width):approved=fixture_clear(ctx,part,G,width,deadline=deadline)
    result=dict(state='static_small_right_carry_and_fixture_probe_only',full_task_pass=False,uses_free_body_poses=False,robot_motion_executed=False,physical_insertion_validated=False,source_run=name,camera_capture_wall_s=observation['captured_wall_s'],encoder_sample_wall_s=float(trace['time_s'][index]),held_right_q=right.tolist(),parked_left_q=left.tolist(),held_right_width_m=right_width,parked_left_width_m=left_width,measured_grasp=G.tolist(),native_contact=contact,approved=approved,carry_route_counts=route_counts,carry_route_examples=route_examples,fixture_counts=ctx['_right_direct_fixture_counts'],fixture_examples=ctx['_right_direct_fixture_examples'],elapsed_s=time.perf_counter()-began)
    print('SMALL_RIGHT_CARRY_AND_FIXTURE_PROBE',json.dumps(result),flush=True)
    return result
