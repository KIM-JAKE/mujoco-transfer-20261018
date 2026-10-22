"""Checked empty RIGHT retreat after assembly; encoders and calibrated goals only."""
from contextlib import ExitStack
import time,numpy as np
from isaac_kinematics import frame
from fr3_gpu_ik import propose_ik


def execute(c,trajectory,step):
    art=c['art'];arm=c['arm'];cmd=c['cmd'];ri,rf=c['ids'],c['fi'];li,lf=c['left_ids'],c['left_fi'];rc,lc=c['checkers']['right'],c['checkers']['left']
    for checker in [rc,lc]:
        binding=getattr(checker,'loaded_contact_aperture',None)
        if binding is not None and binding.active:raise RuntimeError('Final empty retreat requires all loaded contact bindings released')
    actual=art.get_dof_positions()[0].copy();right_width=float(actual[rf].sum());left_width=float(actual[lf].sum())
    if right_width<.014:raise RuntimeError('Final RIGHT opening does not establish clearance around released10.5mm pin head')
    # These are measured fixture/class goal coordinates, not actor transforms.
    goals=np.asarray([c['goals'][p]['P'] for p in ['ring1','ring2','pin']],float)
    def native_root(q):return (arm.fk(q)@frame([0,0,-.1034],np.eye(3)))[:3,3]
    before=np.linalg.norm(goals-native_root(actual[ri]),axis=1)
    if before.min()>=.170:return dict(performed=False,calibrated_goal_clearance_m=before.tolist(),uses_free_body_poses=False)
    H=arm.fk(cmd[0,ri]);openings=np.unique(np.linspace(min(right_width,float(cmd[0,rf].sum())),max(right_width,float(cmd[0,rf].sum())),5));left_openings=np.unique(np.linspace(min(left_width,float(cmd[0,lf].sum())),max(left_width,float(cmd[0,lf].sum())),5))
    saved={s:(v.payload_path,v.payload_rows,v.payload_boxes,v.payload_relative)for s,v in [('right',rc),('left',lc)]};rejections=[];chosen=None
    rc.detach_payload();lc.detach_payload()
    try:
        # Lateral alternatives avoid the known unsafe extra downward pin stroke.
        for delta in [[.025,-.040,0.],[0.,-.070,0.],[-.030,-.055,0.],[.035,-.045,0.]]:
            target=H.copy();target[:3,3]+=delta
            root=(target@frame([0,0,-.1034],np.eye(3)))[:3,3]
            if np.linalg.norm(goals-root,axis=1).min()<.170:continue
            solutions=propose_ik(c,arm,target[:3,3],target[:3,:3],actual[ri],'final_empty_RIGHT_retreat')
            q=arm.solve(target[:3,3],target[:3,:3],cmd[0,ri])
            if q is not None and all(np.linalg.norm(q-v)>.01 for v in solutions):solutions.append(q)
            solutions.sort(key=lambda q:np.linalg.norm(q-cmd[0,ri]))
            for q in solutions:
                with rc.other_arm_at(lc,actual[li],left_width):
                    path=[cmd[0,ri].copy(),q]
                    if not rc.path(np.vstack([actual[ri],path]),max(openings))[0]:
                        path=rc.route(cmd[0,ri],q,max(openings),seconds=.25)
                    if path is None:continue
                rows=trajectory('right',path,.25,checked_edges=True);accepted=True
                qs=np.vstack([actual[ri],cmd[0,ri],rows[:,:7]])
                if c['config'].get('reuse_actual_other_context_checks'):
                    # Identical immutable Cartesian product to the legacy
                    # loop below: every actual/cmd/issued q at every current
                    # RIGHT width, against every current LEFT width. Reuse
                    # only the unchanged own fixed/self/installed-part proof;
                    # every changed LEFT finger retains exact pair checks.
                    from dual_aperture_path_check import rows_other_contexts
                    exact_qs=np.tile(qs,(len(openings),1))
                    exact_widths=np.repeat(openings,len(qs))
                    contexts=[(actual[li].copy(),float(lw))for lw in left_openings]
                    stats={}
                    accepted,why=rows_other_contexts(rc,lc,contexts,exact_qs,exact_widths,diagnostics=stats)
                    c['planning_diagnostics'].append(dict(event='final_empty_retreat_immutable_row_proof_reuse',original_issued_and_start_rows=len(qs),original_RIGHT_widths=len(openings),original_LEFT_widths=len(left_openings),exact_original_combinations=len(qs)*len(openings)*len(left_openings),clear=bool(accepted),**stats))
                    if not accepted:rejections.append(dict(collision=why))
                else:
                    for lw in left_openings:
                        with rc.other_arm_at(lc,actual[li],float(lw)):
                            for opening in openings:
                                for index,pose in enumerate(qs):
                                    clear,why=rc.clear(pose,float(opening))
                                    if not clear:rejections.append(dict(row=index,collision=why));accepted=False;break
                                if not accepted:break
                        if not accepted:break
                if accepted:chosen=(rows,delta);break
            if chosen is not None:break
        if chosen is None:raise RuntimeError('No full-CAD/pair/installed-part clear final empty RIGHT retreat '+str(rejections[:3]))
        rows,delta=chosen
        for row in rows:
            cmd[0,ri]=row[:7];cmd[0,rf]=row[7:];step('final_empty_RIGHT_clearance_retreat')
        after=np.linalg.norm(goals-native_root(art.get_dof_positions()[0,ri]),axis=1)
        if after.min()<.165:raise RuntimeError('Actual encoder RIGHT retreat did not establish165mm calibrated-goal clearance')
        result=dict(performed=True,world_delta_m=delta,physical_motion_s=len(rows)*c['dt'],exact_issued_rows=len(rows),right_openings_checked_m=openings.tolist(),left_openings_checked_m=left_openings.tolist(),calibrated_goal_clearance_before_m=before.tolist(),calibrated_goal_clearance_after_m=after.tolist(),installed_parts_checked=True,uses_free_body_poses=False)
        c['planning_diagnostics'].append(dict(event='final_empty_RIGHT_clearance',**result));print('FINAL_EMPTY_RIGHT_CLEARANCE',result,flush=True);return result
    finally:
        for side,checker in [('right',rc),('left',lc)]:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved[side]
