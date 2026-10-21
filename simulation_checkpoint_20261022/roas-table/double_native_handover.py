"""SIM component: stationary axial-face holder -> positive-axis RIGHT regrasp.

Only saved public RGB-D/encoder grasp priors, current RGB-D and robot encoders
enter targets. No object actor poses, attachments, welds, mass or force changes.
"""
from pathlib import Path
from contextlib import contextmanager
import json,time
import numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame
from grasp_transfer_feasibility import sequential_transfer_clear,receiver_approach_route
from receiver_pad_contact import contact_state
from loaded_contact_aperture import LoadedContactAperture

def execute(c,part,left_G):
    if part!='ring1':raise RuntimeError('Double-handover component currently requires LARGE class')
    art=c['art'];cmd=c['cmd'];dt=c['dt'];arms={'left':c['left_arm'],'right':c['arm']};ids={'left':c['left_ids'],'right':c['ids']};fi={'left':c['left_fi'],'right':c['fi']};checkers=c['checkers'];template=c['part_collision_templates'][part];events=[]
    move=c['component_move'];observe=c['component_observe'];trajectory=c['component_trajectory'];step=c['step'];bindings=c['component_bindings']
    def q(side):return art.get_dof_positions()[0,ids[side]].copy()
    def w(side):return float(art.get_dof_positions()[0,fi[side]].sum())
    def wait(duration,label):
        for _ in range(int(np.ceil(duration/dt))):step(label)
    def event(name,**data):
        row=dict(event=name,wall_s=time.perf_counter()-c['started'],uses_free_body_poses=False,**data);events.append(row);c['planning_diagnostics'].append(row);print('DOUBLE_HANDOVER',row,flush=True)
    @contextmanager
    def payload(checker,G):
        saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
        checker.attach_payload(template,frame([0,0,.1034],np.eye(3))@G)
        try:yield
        finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
    @contextmanager
    def no_payload(checker):
        saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative);checker.detach_payload()
        try:yield
        finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
    def issue(side,path,width,label,*,holder=None,Gholder=None):
        """Check every exact retimed command with real fixed/pair/payload scope."""
        rows=trajectory(side,path,.4,width,checked_edges=True);other='left'if side=='right'else'right';checker=checkers[side];otherchecker=checkers[other]
        context=payload(otherchecker,Gholder)if holder==other else no_payload(otherchecker)
        with context:
            other_actual=w(other);other_command=float(cmd[0,fi[other]].sum())
            for other_opening in np.linspace(min(other_actual,other_command),max(other_actual,other_command),5):
                with checker.other_arm_at(otherchecker,q(other),float(other_opening)):
                    for i,r in enumerate(rows):
                        ok,why=checker.clear(r[:7],float(r[7:].sum()))
                        if not ok:raise RuntimeError('Double-handover exact issued robot/pair/held-CAD blocked '+str(dict(label=label,row=i,other_opening_m=float(other_opening),collision=why)))
        for r in rows:cmd[0,ids[side]]=r[:7];cmd[0,fi[side]]=r[7:];step(label)
        event(label,control_steps=len(rows),physical_motion_s=len(rows)*dt)
    def shared_stationary_sweep(P,widths):
        """Selected-ring distal contact only; BOTH full arms/palms/proximal strict.

        The duplicated held CAD is omitted only from the cross-arm set; each
        actual arm independently checks it with its original distal-pad-only
        OwnPayloadClearance. Robot-vs-robot still checks ALL complete fingers.
        """
        qs={s:q(s)for s in arms}
        for wl,wr in widths:
            with no_payload(checkers['left']),no_payload(checkers['right']):
                with checkers['right'].other_arm_at(checkers['left'],qs['left'],wl):
                    ok,why=checkers['right'].clear(qs['right'],wr)
                    if not ok:raise RuntimeError('Double-handover stationary full robot/pair/fixed sweep blocked '+str(why))
            for side,width in [('left',wl),('right',wr)]:
                G=np.linalg.inv(arms[side].fk(qs[side]))@P
                with payload(checkers[side],G):
                    old_binding=getattr(checkers[side],'loaded_contact_aperture',None);checkers[side].loaded_contact_aperture=None
                    try:ok,why=checkers[side].clear(qs[side],width)
                    finally:checkers[side].loaded_contact_aperture=old_binding
                    if not ok:raise RuntimeError('Double-handover selected-CAD vs full arm/palm/proximal/fixed blocked '+str(dict(side=side,collision=why)))
    left_G,left_pose=observe('left',part,left_G,'double_holder_fresh_RGBD');P=arms['left'].fk(q('left'))@left_G
    if not contact_state(dict(c,config=dict(c['config'],receiver_axial_only=True)),part,left_G,w('left'))['opposed_contact_compatible']:
        raise RuntimeError('Temporary LEFT axial holder lost fresh bilateral geometry')
    # Prior is a successful public RGB-D + Hand encoder grasp, not an actor pose.
    here=Path(__file__).resolve().parent;source=here.parents[1]/'runs/native-roas-table-scatter-stable-v274-gpu-route-pin-yaw-all3-446/rgb-observations.json';prior=json.loads(source.read_text())[49]
    right_G=np.linalg.inv(np.asarray(prior['hand_TCP_world']))@frame(prior['result']['pose']['center_m'],axis_frame(prior['result']['pose']['axis']))
    from grasp_transfer_feasibility import donor_aperture
    clean=dict(c,config=dict(c['config'],receiver_axial_only=False))
    expected=donor_aperture(clean,part,right_G)
    if expected is None or right_G[2,2]<=.1:raise RuntimeError('Saved positive-axis RIGHT prior lacks current native class support')
    right_target=P@np.linalg.inv(right_G);left_target=arms['left'].fk(q('left'))
    if not sequential_transfer_clear(checkers['left'],left_target,w('left'),checkers['right'],right_target,expected,.080,donor_release_width=.0235):
        raise RuntimeError('Fresh temporary-holder/RIGHT regrasp sequential native pair is blocked')
    seeds=[q('right'),np.asarray(c['handoff_reference'][part]['right_seed'])]+list(np.random.default_rng(317).uniform(arms['right'].bounds[:,0],arms['right'].bounds[:,1],(12,7)))
    route=None
    for seed in seeds:
        target_q=arms['right'].solve(right_target[:3,3],right_target[:3,:3],seed)
        if target_q is None:continue
        with payload(checkers['left'],left_G),checkers['right'].other_arm_at(checkers['left'],q('left'),w('left')):
            route=receiver_approach_route(c,'double_RIGHT_positive_receiver',checkers['right'],cmd[0,ids['right']],target_q,.080,seconds=.5)
        if route is not None:break
    if route is None:raise RuntimeError('No full fixed/pair/selected-CAD clear RIGHT positive receiver approach')
    if c.get('set_empty_donor_contact_guard'):c['set_empty_donor_contact_guard']('right',False)
    issue('right',route,.080,'double_RIGHT_positive_approach',holder='left',Gholder=left_G)
    left_G,left_pose=observe('left',part,left_G,'double_holder_before_RIGHT_close_RGBD');P=arms['left'].fk(q('left'))@left_G
    fresh_right_G=np.linalg.inv(arms['right'].fk(q('right')))@P;expected=donor_aperture(clean,part,fresh_right_G)
    if expected is None:raise RuntimeError('Fresh camera does not confirm proposed RIGHT opposing pad geometry')
    closing=max(.001,expected-.003)
    shared_stationary_sweep(P,[(w('left'),gap)for gap in np.linspace(.080,closing,9)])
    c['set_grip_hold']('right',True)
    # The arm is stationary; only a finite native single-width motor closes.
    close_rows=trajectory('right',[cmd[0,ids['right']].copy()]*2,.15,closing,checked_edges=True)
    for r in close_rows:cmd[0,ids['right']]=r[:7];cmd[0,fi['right']]=r[7:];step('double_RIGHT_close')
    window=[];times=[];settled=False
    for _ in range(120):
        window.append(art.get_dof_positions()[0,fi['right']].copy());times.append(time.perf_counter());window=window[-5:]
        gap=w('right');stationary=len(window)==5 and np.max(np.ptp(window,axis=0))<=.00015 and np.ptp(np.asarray(window).sum(1))<=.00015
        if stationary and closing+.0005<gap<expected+.002:settled=True;break
        wait(.025,'double_RIGHT_contact_plateau')
    if not settled:raise RuntimeError('Positive RIGHT grasp failed bounded native encoder plateau; LEFT stays closed')
    right_G,right_pose=observe('right',part,fresh_right_G,'double_RIGHT_bilateral_contact_RGBD')
    contact=contact_state(clean,part,right_G,w('right'));event('double_RIGHT_bilateral_contact',contact=contact,measured_grasp=right_G.tolist())
    if not contact['opposed_contact_compatible']:raise RuntimeError('Fresh RIGHT bilateral pads not confirmed; LEFT stays closed')
    c['set_grip_hold']('right',True,preload_m_s=float(c['config'].get('native_loaded_preload_after_contact_m_s',.015)))
    wait(float(c['config'].get('native_loaded_preload_ramp_s',0))+.05,'double_RIGHT_existing_force_ramp')
    right_G,right_pose=observe('right',part,right_G,'double_RIGHT_force_confirmed_RGBD');P=arms['right'].fk(q('right'))@right_G
    contact=contact_state(clean,part,right_G,w('right'))
    if not contact['opposed_contact_compatible']:raise RuntimeError('RIGHT opposing contact lost during original force ramp')
    shared_stationary_sweep(P,[(gap,wr)for gap in np.linspace(w('left'),.0235,9)for wr in np.linspace(min(float(cmd[0,fi['right']].sum()),w('right')),max(float(cmd[0,fi['right']].sum()),w('right')),5)])
    binding=bindings.pop('left',None)
    if binding is not None:binding.release()
    checkers['left'].loaded_contact_aperture=None;c['set_grip_hold']('left',False)
    open_rows=trajectory('left',[cmd[0,ids['left']].copy()]*2,.15,.0235,checked_edges=True)
    for r in open_rows:cmd[0,ids['left']]=r[:7];cmd[0,fi['left']]=r[7:];step('double_LEFT_release')
    released=False;release_window=[]
    for _ in range(120):
        release_window.append(art.get_dof_positions()[0,fi['left']].copy());release_window=release_window[-5:]
        stationary=len(release_window)==5 and np.max(np.ptp(release_window,axis=0))<=.00015 and np.ptp(np.asarray(release_window).sum(1))<=.00015
        if stationary and w('left')>=.0235-.0005:released=True;break
        wait(.025,'double_LEFT_actual_release_plateau')
    event('double_LEFT_actual_release',confirmed=released,actual_aperture_m=w('left'),stationary_encoder_samples=[r.tolist()for r in release_window])
    if not released:raise RuntimeError('LEFT actual opening did not establish23mm stationary release; no withdrawal issued')
    previous_right_G=right_G.copy();right_G,right_pose=observe('right',part,right_G,'double_RIGHT_after_LEFT_actual_release_RGBD')
    contact=contact_state(clean,part,right_G,w('right'))
    release_translation=float(np.linalg.norm(right_G[:3,3]-previous_right_G[:3,3]));release_angle=float(np.arccos(np.clip(right_G[:3,2]@previous_right_G[:3,2],-1,1)))
    event('double_RIGHT_holds_before_LEFT_withdraw',contact=contact,translation_change_m=release_translation,axis_change_rad=release_angle)
    if not contact['opposed_contact_compatible']or release_translation>.0015 or release_angle>.025:
        raise RuntimeError('RIGHT fresh hold not confirmed after actual LEFT release; no withdrawal issued')
    from paired_handover import empty_giver_route
    with payload(checkers['right'],right_G),checkers['left'].other_arm_at(checkers['right'],q('right'),w('right')):
        route=empty_giver_route(checkers['left'],cmd[0,ids['left']],.0235)
    if route is None:raise RuntimeError('No LEFT empty withdrawal with received RIGHT held-CAD protection')
    issue('left',route,.0235,'double_LEFT_empty_withdraw',holder='right',Gholder=right_G);c['left_hand_known_empty']=True
    # Stationary physical encoder window first, then a fresh image collected
    # while the same actual encoder callback remains live. Original .25s TTL.
    samples=[]
    def collect(t):samples.append((t,art.get_dof_positions()[0,fi['right']].copy()))
    callbacks=c['component_encoder_capture_callbacks'];callbacks.append(collect)
    try:
        wait(.105,'double_RIGHT_encoder_window');right_G,right_pose=observe('right',part,right_G,'double_RIGHT_final_fresh_RGBD')
    finally:callbacks.remove(collect)
    contact=contact_state(clean,part,right_G,w('right'))
    if donor_aperture(clean,part,right_G)is None:raise RuntimeError('Final positive RIGHT camera grip lacks stable class/native geometry')
    capture_age=float(c['component_image_age_steps']('right',part))*dt
    binding=LoadedContactAperture(side='right',part=part,payload_path=template,encoder_samples=[r[1]for r in samples],sample_times_s=[r[0]for r in samples],contact=contact,measured_grasp=right_G,control_dt_s=dt,capture_age_physics_s=capture_age)
    checkers['right'].loaded_contact_aperture=binding;bindings['right']=binding
    with payload(checkers['right'],right_G):
        ok,why=checkers['right'].clear(q('right'),max(w('right'),float(cmd[0,fi['right']].sum())))
        if not ok:raise RuntimeError('Received RIGHT actual interval/full own arm or fixed clearance rejected '+str(why))
    prior_G=right_G.copy();wait(.15,'double_RIGHT_camera_retention_hold');right_G,retained=observe('right',part,right_G,'double_RIGHT_retention_RGBD');translation=float(np.linalg.norm(right_G[:3,3]-prior_G[:3,3]));angle=float(np.arccos(np.clip(right_G[:3,2]@prior_G[:3,2],-1,1)))
    if translation>.0015 or angle>.025:raise RuntimeError('Positive RIGHT regrasp did not retain stable camera-measured grasp')
    with payload(checkers['right'],right_G):
        ok,why=checkers['right'].clear(q('right'),max(w('right'),float(cmd[0,fi['right']].sum())))
        if not ok:raise RuntimeError('Retained RIGHT fresh-G own/full clearance failed '+str(why))
    event('double_RIGHT_retained',translation_change_m=translation,axis_change_rad=angle,measured_grasp=right_G.tolist(),actual_aperture_m=w('right'),loaded_geometry_binding=binding.record)
    return dict(state='physical_double_handover_RIGHT_regrasp_camera_confirmed',part=part,measured_grasp=right_G.tolist(),grasp_type='outer',full_task_pass=False,physical_insertion_validated=False,uses_free_body_poses=False,events=events,retention_RGBD=retained)
