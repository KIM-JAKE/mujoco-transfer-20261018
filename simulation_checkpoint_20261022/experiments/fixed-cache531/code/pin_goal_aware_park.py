"""One goal-aware empty-LEFT park, with current RIGHT image-derived PIN.

Only model queries before admission. All executed rows use the serial
actual/cmd-other-arm/aperture/payload guard. No body-pose reads or pose writes.
"""
from pathlib import Path
import json,time
import numpy as np
from isaac_kinematics import frame
from fr3_gpu_ik import propose_ik
from pin_terminal_geometry import check_terminal


def _candidates(c,arm,H,current,seeds,label):
    seen=[]
    for q in propose_ik(c,arm,H[:3,3],H[:3,:3],current,label):
        q=np.asarray(q,float)
        if all(np.linalg.norm(q-old)>.025 for old in seen):seen.append(q);yield q
        if len(seen)>=12:return
    for seed in seeds:
        if seed is None:continue
        q=arm.solve(H[:3,3],H[:3,:3],seed)
        if q is not None and all(np.linalg.norm(q-old)>.025 for old in seen):seen.append(q);yield q
        if len(seen)>=16:return


def execute(c,pin_grasp,move,close_empty_left,standby_q):
    if c.get('left_hand_known_empty')is not True:raise RuntimeError('Goal-aware pin parking requires confirmed empty LEFT')
    left,right=c['left_arm'],c['arm'];lc,rc=c['checkers']['left'],c['checkers']['right'];art,cmd=c['art'],c['cmd'];li,ri,lf,rf=c['left_ids'],c['ids'],c['left_fi'],c['fi'];here=Path(__file__).resolve().parent
    G=np.asarray(pin_grasp,float)
    if G.shape!=(4,4)or not np.isfinite(G).all():raise RuntimeError('Fresh image-derived body-origin PIN grasp required')
    pinctx=c['pin_context'];selected=pinctx['paths']['workpiece_pin'];candidate=pinctx['candidate'];goal=np.asarray(pinctx['goal'],float)
    # This callback receives body-origin G from direct_pin_skill.payload_grasp.
    # Its matching body-origin jig goal avoids a second symmetry-axis shift.
    pre=frame(goal-[0,0,.045],np.asarray(candidate['goal_part_R'],float))@np.linalg.inv(G)
    seeds=json.loads((here/'bin-picking/direct-pin-robot-seeds.json').read_text())['seeds']
    closure=close_empty_left(G)
    measured=art.get_dof_positions()[0].copy();start=cmd[0,li].copy();actual_left=measured[li].copy();actual_right=measured[ri].copy();own_width=float(measured[lf].sum());right_width=float(measured[rf].sum())
    widths=np.unique(np.linspace(min(own_width,float(cmd[0,lf].sum())),max(own_width,float(cmd[0,lf].sum())),5))
    H=left.fk(actual_left);target=H.copy();target[:3,3]=left.base[:3,3]+[.20,-.12,.45]
    evidence=dict(target_TCP_world_m=target[:3,3].tolist(),target_source='One deterministic LEFT-base outward/robotward offset [+.20,-.12,+.45]m; same actual Hand orientation',source='Calibrated robot base/FK, image-derived held PIN body grasp, calibrated jig goal; no actor poses',model_only_before_move=True,closure=closure)
    saved=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative);had_callback=hasattr(rc,'other_configuration');callback=getattr(rc,'other_configuration',None);began=time.perf_counter();terminal_examples=[]
    def terminal(left_q,openings):
        current=art.get_dof_positions()[0,ri].copy();opening=max(float(art.get_dof_positions()[0,rf].sum()),float(cmd[0,rf].sum()))
        for q in _candidates(c,right,pre,current,[seeds.get('pin_direct_preinsert'),seeds.get('pin_tail_carry_to_preinsert'),candidate.get('insert_waypoints',[None])[0],current],'pin_goal_aware_park_terminal'):
            approved=True;checks=[]
            for width in openings:
                # check_terminal takes an explicit current-other callback;
                # bind it to this MODEL future park for this no-motion proof.
                rc.other_configuration=lambda ql=np.asarray(left_q).copy(),w=float(width):(lc,ql,w)
                with rc.other_arm_at(lc,left_q,float(width)):
                    ok,why=check_terminal(pinctx,right,rc,q,.045,opening,selected,margin=.03)
                checks.append(dict(left_opening_m=float(width),clear=bool(ok),evidence=why))
                if not ok:approved=False;break
            terminal_examples.append(dict(right_preinsert_q=q.tolist(),accepted=approved,checks=checks))
            if approved:return dict(right_preinsert_q=q.tolist(),checks=checks)
        return None
    rc.attach_payload(selected,frame([0,0,.1034],np.eye(3))@G)
    try:
        # A completed SMALL insertion may already leave the empty LEFT clear.
        # Avoid a physical parking round trip only after the unchanged full PIN
        # stroke/release/withdraw proof passes at BOTH actual and command LEFT
        # configurations and every current aperture bound. No route is skipped
        # when either context has no original terminal certificate.
        if c.get('config',{}).get('pin_skip_park_when_current_terminal_clear'):
            actual_proof=terminal(actual_left,widths)
            command_proof=terminal(start,widths) if actual_proof is not None else None
            if actual_proof is not None and command_proof is not None:
                result=dict(empty_left_arm_parked=True,goal_aware=True,
                    already_current_terminal_clear=True,physical_parking_motion_performed=False,
                    actual_left_q=actual_left.tolist(),command_left_q=start.tolist(),
                    actual_left_openings_m=widths.tolist(),terminal_proof=actual_proof,
                    command_terminal_proof=command_proof,current_right_and_held_pin_protected=True,
                    model_and_actual_terminal_proof=True,uses_free_body_poses=False,
                    elapsed_s=time.perf_counter()-began)
                c['planning_diagnostics'].append(dict(event='pin_current_empty_left_terminal_clear_no_park',**result))
                return result
        route=None;proof=None;park_q=None
        for q in _candidates(c,left,target,actual_left,[start,np.asarray(standby_q)],'pin_empty_LEFT_far_park'):
            with lc.other_arm_at(rc,actual_right,right_width):
                if not all(lc.clear(q,float(width))[0]for width in widths):continue
                candidate_route=[start,q]if lc.path([actual_left,start,q],max(widths))[0]else lc.route(start,q,max(widths),seconds=.8)
            if candidate_route is None:continue
            proof=terminal(q,widths)
            if proof is None:continue
            route=candidate_route;park_q=q;break
        if route is None:raise RuntimeError('One goal-aware empty LEFT park has no current-pin-safe route and original PIN terminal proof '+str(terminal_examples[-3:]))
        # Restore the actual-context callback before every physical command.
        if had_callback:rc.other_configuration=callback
        else:del rc.other_configuration
        c['planning_diagnostics'].append(dict(event='pin_goal_aware_empty_left_park_admitted',park_q=park_q.tolist(),terminal_proof=proof,**evidence))
        move('left',route,.8,'pin_goal_aware_empty_left_far_park',other_carried_part='pin',other_carried_grasp=G)
        measured=art.get_dof_positions()[0];actual_left=measured[li].copy();widths=np.unique(np.linspace(min(float(measured[lf].sum()),float(cmd[0,lf].sum())),max(float(measured[lf].sum()),float(cmd[0,lf].sum())),5))
        # Actual attained q/width, not the proposed endpoint, must still leave
        # an original full terminal stroke/release/withdraw branch available.
        actual_proof=terminal(actual_left,widths)
        if actual_proof is None:raise RuntimeError('Actual LEFT park leaves no original PIN terminal branch; retain RIGHT PIN and abort')
        result=dict(empty_left_arm_parked=True,goal_aware=True,one_geometric_parking_target=True,actual_left_q=actual_left.tolist(),actual_left_openings_m=widths.tolist(),terminal_proof=actual_proof,current_right_and_held_pin_protected=True,model_and_actual_terminal_proof=True,uses_free_body_poses=False,elapsed_s=time.perf_counter()-began)
        c['planning_diagnostics'].append(dict(event='pin_goal_aware_empty_left_park_completed',**result));return result
    finally:
        rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved
        if had_callback:rc.other_configuration=callback
        elif hasattr(rc,'other_configuration'):del rc.other_configuration
