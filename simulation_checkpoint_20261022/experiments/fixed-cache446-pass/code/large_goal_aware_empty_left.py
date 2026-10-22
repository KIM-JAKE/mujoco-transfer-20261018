"""One empty LEFT park, admitted by a future complete native RIGHT stroke.

The unchanged CAD/robot fixture checker proves full nominal insertion,
release and withdrawal BEFORE moving LEFT. Actual current RIGHT plus its
fresh image-derived held CAD remain protected through every issued LEFT row.
The caller MUST then obtain fresh RIGHT RGB-D and plan its full actual carry.
"""
import time
import numpy as np
from isaac_kinematics import frame
from fr3_gpu_ik import propose_ik
from fr3_path_timing import retime,retime_safe_polyline
from probe_native_right_ring import fixture_clear
from dual_aperture_path_check import path_other_contexts


def execute(c,part,G,width,step):
    if part!='ring1' or c.get('left_hand_known_empty')is not True:
        raise RuntimeError('Goal-aware LARGE parking requires explicitly empty LEFT')
    left,right=c['left_arm'],c['arm'];lc,rc=c['checkers']['left'],c['checkers']['right']
    art,cmd,li,ri,lf,rf,dt=[c[k]for k in ['art','cmd','left_ids','ids','left_fi','fi','dt']]
    if lc.payload_path:raise RuntimeError('Logically empty LEFT still has planned payload')
    G=np.asarray(G,float)
    if G.shape!=(4,4)or not np.isfinite(G).all():raise ValueError('Fresh finite RIGHT image grasp required')
    measured=art.get_dof_positions()[0].copy();ql=measured[li].copy();qr=measured[ri].copy();cl=cmd[0,li].copy();cr=cmd[0,ri].copy()
    left_widths=np.unique(np.linspace(min(measured[lf].sum(),cmd[0,lf].sum()),max(measured[lf].sum(),cmd[0,lf].sum()),5))
    right_widths=np.unique(np.linspace(min(measured[rf].sum(),cmd[0,rf].sum()),max(measured[rf].sum(),cmd[0,rf].sum()),5))
    current=left.fk(ql).copy()
    began=time.perf_counter();deadline=began+float(c['config'].get('large_goal_aware_park_budget_s',35.))
    saved=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative)
    rc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
    proof_failures=[];chosen=None
    def terminal(park_q,branch=None,search_fallback=False):
        # A stored command/IK branch is a proposal only. Its exact native
        # preinsert+stroke+release+withdraw are rechecked against this future
        # LEFT pose, current held RGB-D CAD and ALL aperture combinations.
        import copy
        from fixed_saved_routes import DATA
        from aperture_path_check import _samples
        from gpu_pair_schedule_scope import certified_pair_scope
        old_resolution=getattr(rc,'resolution_rad',.035);rc.resolution_rad=min(old_resolution,.005)
        payload=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative)
        stage_stats=[]
        try:
            if search_fallback:
                search=dict(c,_right_direct_measured_q=qr.copy(),_right_direct_fixture_context_key=('future_empty_LEFT',tuple(park_q),float(left_widths[0])),_right_direct_fixture_cache={},_right_direct_fixture_counts={},_right_direct_fixture_examples={})
                search.pop('_right_direct_carry_route_callback',None)
                with rc.other_arm_at(lc,park_q,float(left_widths[0])):branch=fixture_clear(search,part,G,float(width),deadline=deadline)
                if branch is None:
                    proof_failures.append(dict(stage='one_original_terminal_search',counts=search['_right_direct_fixture_counts'],examples=search['_right_direct_fixture_examples']));return None
            if branch is None:branch=copy.deepcopy(DATA['fixture'])
            contexts=[(np.asarray(park_q,float),float(gap))for gap in left_widths]
            pre=np.asarray(branch['pre_q'],float);stroke=np.asarray(branch['stroke_q'],float);withdraw=np.asarray(branch['withdraw_q'],float)
            release=np.linspace(float(width),float(branch['release_width_m']),5)
            def guarded_group(groups):
                pairs=[]
                for qs,widths in groups:
                    for _,_,rq in _samples(rc,np.asarray(qs,float)):
                        for rgap in widths:
                            for lq,lgap in contexts:pairs.append((lq,lgap,rq,float(rgap)))
                with certified_pair_scope(c,lc,rc,pairs=pairs):
                    for qs,widths in groups:
                        stats={};ok,why=path_other_contexts(rc,lc,contexts,np.asarray(qs,float),np.asarray(widths,float),diagnostics=stats)
                        stage_stats.append(stats)
                        if not ok:return False,why
                return True,None
            # Free preinsert still carries the actual observed ring CAD.
            ok,why=guarded_group([(np.array([pre,pre]),[float(width)])])
            if not ok:
                proof_failures.append(dict(stage='saved_preinsert_current_payload',collision=why));return None
            # Exactly the original intended-contact policy: only insertion,
            # release and withdrawal omit payload/fixed contact; native robot
            # fixed/self and current opposite-arm guards remain complete.
            rc.detach_payload()
            ok,why=guarded_group([(stroke,[float(width)]),(np.array([stroke[-1],stroke[-1]]),release),(withdraw,[float(branch['release_width_m'])])])
            if not ok:
                proof_failures.append(dict(stage='saved_robot_terminal',collision=why));return None
            return [dict(full_nominal_terminal=branch,left_openings_m=left_widths.tolist(),right_release_openings_m=release.tolist(),original_sampling_rad=.005,all_aperture_contexts_checked=True,query_diagnostics=stage_stats,proposal='one_original_search'if search_fallback else 'stored432_branch')]
        finally:
            rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=payload
            rc.resolution_rad=old_resolution
    def current_LEFT_route(rows):
        from gpu_pair_path_scope import certified_path_scope
        contexts=[(np.asarray(rq,float),float(gap))for rq in[qr,cr]for gap in right_widths]
        curve=np.asarray(np.vstack([ql,cl,rows]),float);stats={}
        # Batch the full-own reference context. Other exact encoder/aperture
        # contexts retain the unchanged changed-body CPU pair checks, avoiding
        # a huge Cartesian-product RPC of duplicate whole-robot transforms.
        with lc.other_arm_at(rc,*contexts[0]):
            with certified_path_scope(c,lc,curve,left_widths):
                clear,why=path_other_contexts(lc,rc,contexts,curve,left_widths,diagnostics=stats)
        return clear,why,stats
    def command_candidate(q):
        # A float32 command rounded onto the wrong side of a hard joint
        # bound is not an admissible endpoint. Move only representational
        # roundoff inward, then prove the resulting EXACT command below.
        q=np.asarray(q,float)
        bounds=np.asarray(left.bounds,float)
        if q.shape!=(7,)or not np.isfinite(q).all()or np.any(q<bounds[:,0]-2e-6)or np.any(q>bounds[:,1]+2e-6):return None
        dtype=cmd.dtype
        low=np.asarray(bounds[:,0],dtype=dtype);high=np.asarray(bounds[:,1],dtype=dtype)
        low=np.nextafter(low,np.asarray(np.inf,dtype=dtype));high=np.nextafter(high,np.asarray(-np.inf,dtype=dtype))
        return np.maximum(low,np.minimum(high,np.asarray(q,dtype=dtype)))
    def candidates():
        if c['config'].get('fixed_supply_mode')and c['config'].get('fixed_saved_route_priors'):
            # Public actuator endpoint from physically accepted444 park.
            # Proposal only: ALL current held-RIGHT route, native fixed/self,
            # widths and future terminal guards below still run unchanged.
            prior=np.array([-.13557329773902893,-.14186294376850128,-.01783299632370472,-2.220045566558838,-2.794273853302002,1.8495267629623413,-1.3566551208496094])
            yield 'public444_accepted_empty_LEFT_park',left.fk(prior),prior
        # The startup standby is a robot joint prior captured before this
        # task, never a free-workpiece pose or a replayed trial command.
        # Try nearby withdrawal/down/away alternatives before a large move.
        offsets=[[-.10,-.10,-.08],[-.10,0.,-.10],[0.,-.12,-.08],[-.18,-.10,-.10],[-.20,0.,-.10],[0.,-.20,-.10]]
        for index,delta in enumerate(offsets):
            if time.perf_counter()>deadline:return
            target=current.copy();target[:3,3]+=delta
            proposed=propose_ik(c,left,target[:3,3],target[:3,:3],ql,'large_empty_LEFT_local_park_'+str(index))
            proposed+=[q for seed in [ql,cl]if(q:=left.solve(target[:3,3],target[:3,:3],seed))is not None]
            for q in sorted(proposed,key=lambda q:float(np.linalg.norm(q-cl)))[:3]:yield 'local_'+str(index),target,q
        standby=c.get('_fixed_supply_empty_left_standby_q')
        if standby is not None:yield 'startup_standby',left.fk(standby),standby
        target=current.copy();target[:3,3]=left.base[:3,3]+[.20,-.12,.45]
        proposed=propose_ik(c,left,target[:3,3],target[:3,:3],ql,'large_empty_LEFT_goal_aware_park')
        proposed+=[q for seed in [ql,cl]if(q:=left.solve(target[:3,3],target[:3,:3],seed))is not None]
        for q in proposed[:4]:yield 'original_base_relative',target,q
    try:
        seen=[];fallback_plan=None
        for candidate_label,target,proposal in candidates():
            if time.perf_counter()>deadline:break
            park_q=command_candidate(proposal)
            if park_q is None:continue
            if any(np.linalg.norm(park_q-old)<.025 for old in seen):continue
            seen.append(park_q)
            endpoint_clear=True
            # BOTH actual and commanded stationary RIGHT configurations and
            # all current aperture extrema are included, with its fresh CAD.
            for rq in [qr,cr]:
                for gap in right_widths:
                    with lc.other_arm_at(rc,rq,float(gap)):
                        if not all(lc.clear(park_q,float(lgap))[0]for lgap in left_widths):endpoint_clear=False;break
                if not endpoint_clear:break
            if not endpoint_clear:continue
            with lc.other_arm_at(rc,qr,float(max(right_widths))):
                route=[cl,park_q]if lc.path([ql,cl,park_q],float(max(left_widths)))[0]else lc.route(cl,park_q,float(max(left_widths)),seconds=.5)
            if route is None:continue
            start=cmd[0,lf].copy();knots=np.asarray(route,float).copy();knots[0]=cl
            if len(knots)>2:rows,_,timing=retime_safe_polyline(knots,dt,start,start,scale=max(1.,float(c['config'].get('stability_time_scale',1.))))
            else:rows,_,timing=retime(knots,.5,dt,start,start,optimize=bool(c['config'].get('optimized_motion_timing')))
            # Try the public saved terminal first; no per-width IK searches.
            future=terminal(np.asarray(rows[-1],float))
            if future is None:
                if fallback_plan is None:fallback_plan=(rows,timing,rows[-1].copy(),candidate_label,target.copy())
                continue
            clear,why,stats=current_LEFT_route(rows)
            if not clear:
                proof_failures.append(dict(stage='actual_issued_LEFT_route',collision=why));continue
            chosen=(rows,timing,future,stats,rows[-1].copy(),candidate_label,target.copy());break
        if chosen is None and fallback_plan is not None and time.perf_counter()<deadline:
            rows,timing,park_q,candidate_label,target=fallback_plan
            future=terminal(np.asarray(park_q,float),search_fallback=True)
            if future is not None:
                clear,why,stats=current_LEFT_route(rows)
                if clear:chosen=(rows,timing,future,stats,park_q,candidate_label,target)
                else:proof_failures.append(dict(stage='fallback_actual_issued_LEFT_route',collision=why))
        if chosen is None:raise RuntimeError('Goal-aware LARGE empty-LEFT park has no held-RIGHT-safe issued route and full future terminal proof '+str(proof_failures[-4:]))
        rows,timing,future,stats,park_q,candidate_label,target=chosen
        evidence=dict(event='large_goal_aware_empty_LEFT_park_admitted',part=part,candidate_label=candidate_label,geometric_target_world_m=target[:3,3].tolist(),candidate_count=len(seen),future_left_q=np.asarray(park_q).tolist(),future_terminal_proofs=future,actual_RIGHT_q=qr.tolist(),command_RIGHT_q=cr.tolist(),right_openings_m=right_widths.tolist(),left_openings_m=left_widths.tolist(),fresh_RGBD_held_RIGHT_G=G.tolist(),exact_issued_LEFT_rows=len(rows)-1,issued_guard=stats,timing=timing,uses_free_body_poses=False,full_actual_RIGHT_carry_replan_required=True)
        c['planning_diagnostics'].append(evidence)
        for row in rows[1:]:cmd[0,li]=row;step('goal_aware_empty_left_park')
        c.pop('_right_direct_fixture_cache',None);c.pop('_right_direct_fixture_counts',None);c.pop('_right_direct_fixture_examples',None)
        return dict(empty_left_parked=True,goal_aware=True,physical_duration_s=(len(rows)-1)*dt,elapsed_s=time.perf_counter()-began,model_terminal_proof_only=True,actual_RIGHT_carry_replan_required=True,uses_free_body_poses=False)
    finally:
        rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved
