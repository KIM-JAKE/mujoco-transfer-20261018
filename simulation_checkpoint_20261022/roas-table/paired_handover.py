"""Plan both sides of a handover before committing the donor motion."""
import time
import numpy as np
from scipy.interpolate import CubicSpline
from isaac_kinematics import frame, exp, log
from rgbd_geometry import axis_frame
from grasp_transfer_feasibility import receiving_poses, gripper_boxes, pad_aperture, sequential_transfer_clear, insertion_pose_options, SMALL_HANDOVER_HINT, hinted_route, robot_route_hint, shorten_checked_route, hint_path_clear, receiver_aperture, receiver_opening as native_receiver_opening, receiver_approach_route
from predictive_pair_clearance import shape_overlap



def empty_giver_route(checker,start,opening=.08):
    """Query short empty-hand exits; no physical actor is moved here."""
    arm=checker.arm;H=arm.fk(start)
    directions=[-H[:3,2],np.array([1.,0,0]),np.array([0.,-1.,0]),np.array([0.,0,1.]),np.array([0.,0,-1.]),np.array([-1.,0,0]),np.array([0.,1.,0])]
    old=getattr(checker,'resolution_rad',.035);checker.resolution_rad=.010
    try:
        for distance in [.035,.020,.055]:
            for direction in directions:
                q=arm.solve(H[:3,3]+distance*direction,H[:3,:3],start)
                if q is not None and checker.path([start,q],opening)[0]:return[start.copy(),q]
    finally:checker.resolution_rad=old
    return None


def plan(c, part, grasp,donor_release_width=.08):
    right, left = c['arm'], c['left_arm']
    rc, lc = c['checkers']['right'], c['checkers']['left']
    row = c['handoff_reference'][part]
    command = c['cmd'][0]
    rq0, lq0 = command[c['ids']], command[c['left_ids']]
    diameter = float(c['art'].get_dof_positions()[0,c['fi']].sum())
    receiver_opening=native_receiver_opening(c,part)
    reference = np.array(row['part_H'])
    receiver_grasp = np.linalg.inv(left.fk(row['left_receive_q'])) @ reference
    initial_rotation = right.fk(rq0)[:3, :3]
    a, b = initial_rotation @ grasp[:3, 2], reference[:3, 2]
    cross = np.cross(a, b)
    length = np.linalg.norm(cross)
    axis = cross / length if length > 1e-8 else axis_frame(a)[:, 0]
    aligned = exp(axis * np.arctan2(length, a @ b)) @ initial_rotation
    use_hint=part=='ring2' and c['config'].get('reliability_handover_hint')
    seeds = ([np.asarray(SMALL_HANDOVER_HINT['right_seed'])] if use_hint else [])+[rq0.copy(), np.array(row['right_seed'])]
    seeds += list(np.random.default_rng(9981).uniform(right.bounds[:, 0], right.bounds[:, 1], (8, 7)))
    tilts = [np.eye(3)] + [exp(np.array(axis) * angle)
        for angle in [np.pi/3, -np.pi/3, np.pi/2, -np.pi/2,np.pi,-np.pi*3/4,np.pi*3/4]
        for axis in [[1., 0, 0], [0, 1., 0]]]
    offsets = [[0, 0, 0], [0, 0, .15], [0, 0, -.10], [.10, 0, -.10],
               [.05, -.10, -.10], [0, -.10, -.15], [-.05, 0, -.10],
               [.10, -.10, -.15], [0, 0, .10]]
    if part=='ring2':offsets=[[.20,-.15,-.05],[.10,-.15,-.05],[.10,-.10,0.],[0,0,0.]]+offsets
    if part=='ring2' and c['config'].get('handover_offsets_override'):offsets=c['config']['handover_offsets_override']
    pair_rejections={};insertion_cache={}
    started = time.perf_counter()
    budget = float(c['config'].get('handover_plan_budget_s', 4.))
    counts = dict(receiver_ik=0, donor_ik=0, donor_collision=0,
                  pair_collision=0, receiver_route=0, donor_route=0, insertion_ik=0, intrinsic_pair_collision=0,donor_exit_route=0)
    def route_diagnostic(checker,start,goal,opening,order):
        def short(result):
            ok,why=result
            return dict(clear=ok,query=why.get('query','').split('/')[-1],actor=why.get('actor','').split('/')[-1]) if why else dict(clear=ok)
        value=dict(part=part,order=order,start=short(checker.clear(start,opening)),goal=short(checker.clear(goal,opening)),max_right_reference_encoder_difference_rad=float(np.max(abs(c['art'].get_dof_positions()[0,c['ids']]-rq0))))
        c.get('planning_diagnostics',[]).append(value);print('HANDOVER_ROUTE_DIAGNOSTIC',value,flush=True)
    via_pool=list(c.get('empty_return_waypoints',[]));prefix_cache={}
    retract=right.fk(rq0);retract[:3,3]=[.25,-.60,max(1.25,float(retract[2,3]))]
    retract_q=right.solve(retract[:3,3],retract[:3,:3],rq0)
    if retract_q is not None:via_pool.insert(0,retract_q)
    def transport_route(goal,tag):
        if not rc.clear(rq0,diameter)[0] or not rc.clear(goal,diameter)[0]:return None
        if rc.path([rq0,goal],diameter)[0]:return[rq0,goal]
        hint=robot_route_hint(c,part+'_donor_carry',rq0,goal)
        if hint is not None and hint_path_clear(rc,hint,diameter):return hint
        if getattr(rc,'fast_planner',None) is not None:
            accelerated=rc.route(rq0,goal,diameter,seconds=.35,skip_direct=True)
            if accelerated is not None:return accelerated
        for index,via in enumerate(via_pool):
            if time.perf_counter()-started>budget:break
            key=(tag,index)
            if key not in prefix_cache:prefix_cache[key]=rc.path([rq0,via],diameter)[0]
            if not prefix_cache[key] or not rc.path([via,goal],diameter)[0]:continue
            candidate=[rq0,via,goal];curve=CubicSpline([0,.5,1.],candidate)(np.linspace(0,1,33))
            if np.any(curve<right.bounds[:,0]) or np.any(curve>right.bounds[:,1]):continue
            if rc.path(curve,diameter)[0]:return candidate
        return rc.route(rq0,goal,diameter,seconds=.35,skip_direct=True,use_fast=False)
    rc.attach_payload(c['part_collision_templates'][part], frame([0, 0, .1034], np.eye(3)) @ grasp)
    try:
        for offset in offsets:
            for tilt in tilts:
                if time.perf_counter() - started > budget:
                    break
                incoming = reference.copy()
                incoming[:3, 3] += offset
                incoming[:3, :3] = tilt @ reference[:3, :3]
                for target_left, receiver_tilt in receiving_poses(c, part, incoming):
                    if time.perf_counter() - started > budget:
                        break
                    new_grasp=np.linalg.inv(target_left)@incoming;receiver_width=receiver_aperture(c,part,new_grasp)
                    if receiver_width is None:continue
                    donor_targets=[]
                    canonical=reference[:3,:3]@grasp[:3,:3].T
                    bases=[canonical,aligned] if part=='ring2' else [aligned,canonical]
                    rotations=[tilt@exp(b*yaw)@base for base in bases for yaw in [0,np.pi,np.pi/2,-np.pi/2,np.pi/4,-np.pi/4,3*np.pi/4,-3*np.pi/4]]
                    if c['config'].get('sort_donor_orientations'):rotations.sort(key=lambda rotation:np.linalg.norm(log(rotation@initial_rotation.T)))
                    for rotation in rotations:
                        position=incoming[:3,3]-rotation@grasp[:3,3]
                        if sequential_transfer_clear(rc,frame(position,rotation),diameter,lc,target_left,receiver_width,receiver_opening,donor_release_width=donor_release_width):donor_targets.append((rotation,position))
                        else:counts['intrinsic_pair_collision']+=1
                    if not donor_targets:continue
                    receiver_solutions=[]
                    receiver_seeds=[row['left_receive_q'],lq0]+[np.asarray(row['left_receive_q'])+np.eye(7)[2]*delta for delta in [-.6,.6,-1.2,1.2]]
                    if use_hint:receiver_seeds.insert(0,SMALL_HANDOVER_HINT['left_seed'])
                    for receiver_seed in receiver_seeds:
                        solution=left.solve(target_left[:3,3],target_left[:3,:3],receiver_seed)
                        if solution is not None and all(np.linalg.norm(solution-old)>.025 for old in receiver_solutions):receiver_solutions.append(solution)
                    receiver_solutions.sort(key=lambda q:np.linalg.norm(q-(np.asarray(SMALL_HANDOVER_HINT['left_seed']) if use_hint and not c['config'].get('prefer_near_handover_branch') else lq0)))
                    if not receiver_solutions:
                        counts['receiver_ik'] += 1
                        continue
                    if part=='ring1':
                        self_guard=getattr(lc,'same_arm_clearance',None)
                        if self_guard is None:raise RuntimeError('Large receiver requires carrier-payload self-clearance admission')
                        saved_payload=(lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative)
                        lc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@new_grasp)
                        try:
                            admitted=[]
                            for candidate in receiver_solutions:
                                clear,why=self_guard.clear(candidate,receiver_width)
                                if clear:admitted.append(candidate)
                                else:
                                    counts['receiver_payload_self_collision']=counts.get('receiver_payload_self_collision',0)+1
                            receiver_solutions=admitted
                        finally:
                            lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative=saved_payload
                        if not receiver_solutions:continue
                    lq=receiver_solutions[0]
                    insertion_hand = np.asarray(row['approach_ring_poses'])[-1] @ np.linalg.inv(new_grasp)
                    key=np.round(new_grasp,8).tobytes()
                    insertion_q=insertion_cache.get(key)
                    if key not in insertion_cache:
                        for insertion_hand,_ in insertion_pose_options(c,part,new_grasp):
                            insertion_seeds=[row['left_insert_seed'],lq,lq0]
                            insertion_seeds+=list(np.random.default_rng(36109).uniform(left.bounds[:,0],left.bounds[:,1],(4,7)))
                            for insertion_seed in insertion_seeds:
                                candidate_q=left.solve(insertion_hand[:3,3],insertion_hand[:3,:3],insertion_seed)
                                if candidate_q is None:continue
                                # Newly proposed depths have no nominal catalog
                                # fixture certificate. Check actual candidate
                                # full robot/fixed/self geometry before grasp.
                                if not lc.clear(candidate_q,max(receiver_width,receiver_opening))[0]:continue
                                insertion_q=candidate_q;break
                            if insertion_q is not None:break
                        insertion_cache[key]=insertion_q
                    if insertion_q is None:
                        counts['insertion_ik'] += 1
                        continue
                    for rotation,position in donor_targets:
                        donor_solutions=[]
                        for seed in seeds:
                            if time.perf_counter() - started > budget:
                                break
                            rq = right.solve(position, rotation, seed)
                            if rq is None:
                                counts['donor_ik'] += 1
                                continue
                            if any(np.linalg.norm(rq-old)<.025 for old in donor_solutions):continue
                            donor_solutions.append(rq.copy())
                        if not use_hint or c['config'].get('prefer_near_handover_branch'):donor_solutions.sort(key=lambda q:np.max(abs(q-rq0)/np.array([2.62,2.62,2.62,2.62,5.26,4.18,5.26])))
                        for rq in donor_solutions:
                            if not rc.clear(rq, diameter)[0]:
                                counts['donor_collision'] += 1
                                continue
                            for lq in receiver_solutions[:4]:
                                if time.perf_counter()-started>budget:break
                                with lc.other_arm_at(rc, rq, diameter):
                                    pair_ok,pair_why=lc.clear(lq, receiver_opening)
                                    if not pair_ok:
                                        counts['pair_collision'] += 1
                                        key=str((pair_why.get('query','').split('/')[-1],pair_why.get('actor','').split('/')[-1],tuple(round(v,3)for v in receiver_tilt)))
                                        pair_rejections[key]=pair_rejections.get(key,0)+1
                                        continue
                                    receiver_route = receiver_approach_route(c,part+'_receiver_approach',lc,lq0,lq,receiver_opening,seconds=.25)
                                # An empty giver must be able to leave after
                                # release. Its carried-object query is disabled
                                # only for this future empty-hand calculation.
                                exit_H=right.fk(rq)@frame([0,0,-.035],np.eye(3));exit_q=right.solve(exit_H[:3,3],exit_H[:3,:3],rq)
                                saved_payload=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative)
                                rc.payload_path=None;rc.payload_rows=[];rc.payload_boxes=[];rc.payload_relative=None
                                try:
                                    with rc.other_arm_at(lc,lq,receiver_width):exit_route=empty_giver_route(rc,rq,donor_release_width)
                                    if exit_route is None:
                                        # The executing skill can pull the received ring
                                        # clear before the open giver retreats. Include
                                        # that same physically queried sequence here.
                                        donor_root=right.fk(rq)@frame([0,0,-.1034],np.eye(3));away=incoming[:3,3]-donor_root[:3,3];away/=max(np.linalg.norm(away),1e-9)
                                        pull=left.fk(lq);pull[:3,3]+=away*.045
                                        pull_q=left.solve(pull[:3,3],pull[:3,:3],lq)
                                        lc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@new_grasp)
                                        try:
                                            with lc.other_arm_at(rc,rq,donor_release_width):pull_route=lc.route(lq,pull_q,receiver_width,seconds=.08) if pull_q is not None else None
                                            if pull_route is not None:
                                                with rc.other_arm_at(lc,pull_q,receiver_width):exit_route=empty_giver_route(rc,rq,donor_release_width)
                                        finally:lc.detach_payload()
                                finally:rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved_payload
                                if exit_route is None:counts['donor_exit_route']+=1;continue
                                order='donor_first'
                                if receiver_route is None:
                                    counts['receiver_route'] += 1
                                    with lc.other_arm_at(rc,rq0,diameter):receiver_route=receiver_approach_route(c,part+'_receiver_approach',lc,lq0,lq,receiver_opening,seconds=.30)
                                    if receiver_route is None:continue
                                    # Leave the receiver outside the part while the
                                    # donor arrives. The final axial approach is made
                                    # only after a fresh donor RGB-D observation.
                                    preposition_q=lq
                                    if hasattr(receiver_route,'standoff_index'):
                                        receiver_route=list(receiver_route[:receiver_route.standoff_index+1])
                                        preposition_q=np.asarray(receiver_route[-1])
                                    with rc.other_arm_at(lc,preposition_q,receiver_opening):
                                        donor_route=transport_route(rq,tuple(np.round(preposition_q,10)))
                                        if donor_route is None and counts['donor_route']==0:route_diagnostic(rc,rq0,rq,diameter,'receiver_first')
                                    order='receiver_first'
                                else:
                                    donor_route = transport_route(rq,'current_receiver')
                                    if donor_route is None and counts['donor_route']==0:route_diagnostic(rc,rq0,rq,diameter,'donor_first')
                                if donor_route is None:counts['donor_route'] += 1;continue
                                if c['config'].get('shortcut_checked_routes') and not getattr(donor_route,'verified_cubic',False):donor_route=shorten_checked_route(rc,donor_route,diameter)
                                elapsed = time.perf_counter() - started
                                print('LOOSE_PAIRED_HANDOVER_PLAN', part, elapsed, True, receiver_tilt, counts, flush=True)
                                c.get('planning_diagnostics',[]).append(dict(event='approved_handover',part=part,incoming=incoming.tolist(),receiver_parameters=[float(v) for v in receiver_tilt],right_q=rq.tolist(),left_q=lq.tolist(),approach_order=order,elapsed_s=elapsed,source='RGB-D grasp + robot model; no free-body pose'))
                                return dict(incoming=incoming, donor_route=donor_route,
                                            receiver_route=receiver_route, receiver_q=lq,donor_exit_route=exit_route,
                                            receiver_tilt_rad=receiver_tilt[0],receiver_yaw_rad=receiver_tilt[1],
                                            elapsed_s=elapsed, counts=counts,approach_order=order,
                                            future_pair_clearance='conservative mesh-enclosing OBBs + PhysX structure')
    finally:
        rc.detach_payload()
    elapsed = time.perf_counter() - started
    print('LOOSE_PAIRED_HANDOVER_PLAN', part, elapsed, False, counts,sorted(pair_rejections.items(),key=lambda x:-x[1])[:8],flush=True)
    raise RuntimeError('No jointly feasible handover within planning budget ' + str(counts))
