"""Static right-arm direct insertion from a measured external closing grasp.

Inputs are saved RGB-D ring poses and robot encoders. No robot motion, free
workpiece transforms, physical contact bypass or additional fixture is used.
"""
from pathlib import Path
import hashlib,json,time
import numpy as np
from isaac_kinematics import frame,exp
from rgbd_geometry import axis_frame
from grasp_transfer_feasibility import pad_aperture
from receiver_pad_contact import contact_state

def _key(G):return np.round(G,8).tobytes()

def robot_ik_hint(c,part):
    """A fixed robot/model prior, retargeted and fully rechecked for this G."""
    cache=c.setdefault('_right_direct_robot_hint_cache',{})
    if part in cache:return cache[part]
    from fixed_saved_routes import robot_hint as fixed_robot_hint
    saved=fixed_robot_hint(c,part)
    if saved is not None:cache[part]=saved;return saved
    here=Path(__file__).resolve().parent;path=here/'bin-picking/right-native-insertion-ik-hints.json'
    if not path.exists():cache[part]=None;return None
    saved=json.loads(path.read_text());arm=c['arm']
    geometry={k:np.round(np.asarray(getattr(arm,k)),9).tolist()for k in ['base','tip','frames','axes','bounds']}
    valid=hashlib.sha256(json.dumps(geometry,sort_keys=True).encode()).hexdigest()==saved.get('robot_geometry_sha256')
    valid=valid and all(hashlib.sha256((here/name).read_bytes()).hexdigest()==digest for name,digest in saved.get('native_geometry_sha256',{}).items())
    # A nearby class may share an IK seed/hand heading, not a motion or grasp.
    row=saved.get('parts',{}).get(part,saved.get('parts',{}).get('ring1')) if valid else None
    cache[part]=row;return row

def insertion_pose_options(c,part,G):
    angles=[]
    if c.get('_right_direct_current_heading_first'):
        # Preserve the CURRENT hand heading, then try nearby continuous ring
        # yaws to recover reserved-joint margin after a measured grasp shift.
        orientation=c['arm'].fk(np.asarray(c['_right_direct_measured_q']))[:3,:3]@G[:3,:3]
        current_yaw=float(np.arctan2(orientation[1,0],orientation[0,0]))
        angles.extend(current_yaw+np.radians(offset)for offset in [10,-10,15,-15,5,-5,30,-30,0])
    hint=robot_ik_hint(c,part)
    if hint is not None:
        # Camera axis_frame chooses an arbitrary in-plane basis for this
        # rotationally symmetric ring. Preserve a proven HAND heading while
        # deriving the required ring yaw from the CURRENT measured grasp.
        orientation=c['arm'].fk(np.asarray(hint['final_q']))[:3,:3]@G[:3,:3]
        yaw=float(np.arctan2(orientation[1,0],orientation[0,0]))
        angles.extend(yaw+np.radians(offset)for offset in [0,5,-5,10,-10,15,-15])
    angles.extend(np.radians([0,30,-30,60,-60,90,-90,120,-120,150,-150,180]))
    angles.extend(np.radians(np.arange(-180,180,5)))
    seen=set()
    for angle in angles:
        angle=float((angle+np.pi)%(2*np.pi)-np.pi);key=round(angle,8)
        if key in seen:continue
        seen.add(key)
        ring=frame(c['goals'][part]['P'],exp(np.array([0.,0.,angle])))
        yield ring@np.linalg.inv(G),float(angle)

def fixture_clear(c,part,G,width,deadline=None):
    """Query full robot stroke, small release and world-down withdrawal.

    Payload collision is required at the free-space preinsert endpoint.
    Robot-only insertion allows the intended ring/jig contact; that contact
    still needs a later physical insertion test, and is never disabled there.
    """
    if deadline is None:deadline=time.perf_counter()+float(c.get('config',{}).get('axial_receiver_fixture_budget_s',45.))
    carry_callback=c.get('_right_direct_carry_route_callback')
    context_key=c.get('_right_direct_fixture_context_key')
    if carry_callback is not None and context_key is None:raise ValueError('Carry-aware fixture search requires a current robot/other-arm context cache key')
    minimum_margin=float(c.get('_right_direct_minimum_joint_margin_rad',0.))
    cache=c.setdefault('_right_direct_fixture_cache',{});key=(part,_key(G),round(float(width),8),context_key,minimum_margin)
    if key in cache:return cache[key]
    arm=c['arm'];checker=c['checkers']['right'];row=c['handoff_reference'][part]
    opening=.08 if part=='ring1' else min(.08,width+.002)
    counts=c.setdefault('_right_direct_fixture_counts',{})
    examples=c.setdefault('_right_direct_fixture_examples',{})
    def reject(label,why=None):
        counts[label]=counts.get(label,0)+1
        if why is not None and label not in examples:examples[label]=why
    def has_joint_margin(q):
        return bool(np.all(np.asarray(q)>=arm.bounds[:,0]+minimum_margin) and np.all(np.asarray(q)<=arm.bounds[:,1]-minimum_margin)) if minimum_margin>0 else True
    def blocked(result,label):
        ok,why=result
        if not ok:reject(label,why)
        return not ok
    old_resolution=getattr(checker,'resolution_rad',.035);checker.resolution_rad=min(old_resolution,.005)
    saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
    checker.detach_payload()
    try:
        for final,azimuth in insertion_pose_options(c,part,G):
            if time.perf_counter()>deadline:reject('fixture_budget');return None
            # Check final opening first; changing only IK cannot fix a palm
            # or fingertip intersection at this exact end-effector pose.
            if blocked(checker.hand_clear(final,width),'final_hand'):continue
            if any(blocked(checker.hand_clear(final,float(gap)),'release_hand')for gap in np.linspace(width,opening,5)):continue
            withdraw=final.copy();withdraw[2,3]-=.080
            if any(blocked(checker.hand_clear(frame(final[:3,3]*(1-u)+withdraw[:3,3]*u,final[:3,:3]),opening),'withdraw_hand')for u in np.linspace(0,1,17)):continue
            for clearance in [.045,.080,.060]:
                if time.perf_counter()>deadline:reject('fixture_budget');return None
                pre=final.copy();pre[2,3]-=clearance
                if any(blocked(checker.hand_clear(frame(pre[:3,3]*(1-u)+final[:3,3]*u,final[:3,:3]),width),'stroke_hand')for u in np.linspace(0,1,17)):continue
                seeds=[c['_right_direct_measured_q'],row['right_seed'],c['cmd'][0,c['ids']]]
                if c.get('pin_context',{}).get('candidate',{}).get('insert_waypoints'):seeds.insert(0,np.asarray(c['pin_context']['candidate']['insert_waypoints'][0]))
                hint=robot_ik_hint(c,part)
                if hint is not None:seeds=[np.asarray(hint['pre_q']),np.asarray(hint['final_q'])]+seeds
                if c.get('_right_direct_current_heading_first'):seeds=[np.asarray(c['_right_direct_measured_q']),np.asarray(c['cmd'][0,c['ids']])]+seeds
                seeds+=list(np.random.default_rng(61021).uniform(arm.bounds[:,0],arm.bounds[:,1],(48,7)))
                tried=[]
                for seed in seeds:
                    if time.perf_counter()>deadline:reject('fixture_budget');return None
                    q=arm.solve(pre[:3,3],pre[:3,:3],seed)
                    if q is None:reject('pre_IK');continue
                    if not has_joint_margin(q):reject('pre_joint_margin');continue
                    if any(np.linalg.norm(q-old)<.05 for old in tried):continue
                    tried.append(q)
                    checker.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
                    try:carry_clear=not blocked(checker.clear(q,width),'payload_preinsert')
                    finally:checker.detach_payload()
                    if not carry_clear:continue
                    stroke=[q];valid=True
                    for u in np.linspace(0,1,17)[1:]:
                        nq=arm.solve(pre[:3,3]*(1-u)+final[:3,3]*u,final[:3,:3],stroke[-1])
                        if nq is None:reject('stroke_IK');valid=False;break
                        if not has_joint_margin(nq):reject('stroke_joint_margin');valid=False;break
                        if blocked(checker.path([stroke[-1],nq],width),'stroke_robot'):valid=False;break
                        stroke.append(nq)
                    if not valid:continue
                    if any(blocked(checker.clear(stroke[-1],float(gap)),'release_robot')for gap in np.linspace(width,opening,5)):continue
                    retreat=[stroke[-1]]
                    for u in np.linspace(0,1,17)[1:]:
                        nq=arm.solve(final[:3,3]*(1-u)+withdraw[:3,3]*u,final[:3,:3],retreat[-1])
                        if nq is None:reject('withdraw_IK');valid=False;break
                        if not has_joint_margin(nq):reject('withdraw_joint_margin');valid=False;break
                        if blocked(checker.path([retreat[-1],nq],opening),'withdraw_robot'):valid=False;break
                        retreat.append(nq)
                    if not valid:continue
                    carry_route=None
                    if carry_callback is not None:
                        carry_route=carry_callback(q,G,width)
                        if carry_route is None:reject('payload_carry_route');continue
                    approved=dict(pre_q=np.asarray(q).tolist(),stroke_q=np.asarray(stroke).tolist(),withdraw_q=np.asarray(retreat).tolist(),held_width_m=float(width),release_width_m=float(opening),preinsert_clearance_m=clearance,ring_azimuth_rad=float(azimuth),grasp=G.tolist(),source='Fixed class CAD, native robot IK and live fixed-scene PhysX queries; no free-body transforms',physical_insertion_validated=False,additional_nominal_joint_margin_rad=minimum_margin)
                    if carry_route is not None:
                        approved['carry_route']=np.asarray(carry_route).tolist()
                        approved['carry_route_verified_cubic']=bool(getattr(carry_route,'verified_cubic',False))
                    cache[key]=approved;return approved
        cache[key]=None;return None
    finally:
        checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
        checker.resolution_rad=old_resolution



def execute(c):
    started=time.perf_counter();here=Path(__file__).resolve().parent
    name=c.get('config',{}).get('insertion_probe_source_run','native-roas-table-scatter-stable-v156-slow-446')
    source=here.parents[1]/'runs'/name
    report=json.loads((source/'pipeline-results.json').read_text());trace=np.load(source/'physx-sequence-trace.npz');observations=json.loads((source/'rgb-observations.json').read_text())
    part=c.get('config',{}).get('pick_part','ring1')
    rows=[v for v in observations if v['part']==part and v['mode']=='held'and v['result'].get('pose')]
    # Before receiver approach: donor carries its own payload, so this is not
    # a pose disturbed by closing the second hand or an inferred attachment.
    before=[v for v in rows if v['captured_wall_s']<35.]
    observation=min(before or rows,key=lambda v:abs(v['captured_wall_s']-33.716))
    index=int(np.argmin(abs(trace['time_s']-observation['captured_wall_s'])))
    joint_ids=[report['dof_names'].index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)]
    finger_ids=[i for i,n in enumerate(report['dof_names'])if n.startswith('right_native_')]
    if len(finger_ids)!=2:raise RuntimeError('Saved right native finger encoder names ambiguous')
    actual_q=trace['q'][index,joint_ids];actual_width=float(trace['q'][index,finger_ids].sum());pose=observation['result']['pose']
    G=np.linalg.inv(c['arm'].fk(actual_q))@frame(pose['center_m'],axis_frame(pose['axis']))
    expected=pad_aperture(c,part,G)
    if expected is None:raise RuntimeError('Measured donor has no native diameter contact geometry')
    contact=contact_state(c,part,G,actual_width)
    if not contact['opposed_contact_compatible']:raise RuntimeError('Measured donor fails bilateral native finger geometry '+str(contact))
    c['_right_direct_measured_q']=actual_q
    with c['checkers']['right'].other_arm_at(c['checkers']['left'],c['cmd'][0,c['left_ids']],.08):
        approved=fixture_clear(c,part,G,max(expected,actual_width))
    result=dict(state='right_direct_static_fixed_CAD_probe_only',part=part,full_task_pass=False,uses_free_body_poses=False,robot_motion_executed=False,physical_insertion_validated=False,source_run=name,camera_capture_wall_s=observation['captured_wall_s'],actual_encoder_wall_s=float(trace['time_s'][index]),measured_grasp=G.tolist(),actual_opening_m=actual_width,expected_contact_opening_m=expected,native_contact=contact,approved=approved,counts=c.get('_right_direct_fixture_counts',{}),rejection_examples=c.get('_right_direct_fixture_examples',{}),elapsed_s=time.perf_counter()-started)
    print('RIGHT_DIRECT_FIXED_CAD_PROBE',json.dumps(result),flush=True)
    return result
