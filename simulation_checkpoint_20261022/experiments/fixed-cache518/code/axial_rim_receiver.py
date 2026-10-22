"""Receive a ring across its axial cap faces with the native closing hand.

Class CAD and calibrated jig/robot geometry only. These poses are a distinct
family from the taught diameter receiver: closing Y is parallel to ring axis,
while TCP Z approaches a local rim radially. No free-body state is accessed.
"""
import time
import numpy as np
from isaac_kinematics import frame
from receiver_pad_contact import contact_state


def _key(G):return np.round(G,8).tobytes()


def geometric_candidates(c,part,incoming):
    from grasp_transfer_feasibility import receiver_aperture
    radius=.0375 if part=='ring1'else .031
    # Start with the full flat wall under the middle of the short finger pad.
    radii=[radius-.003,radius,radius-.005,radius+.002,radius-.008]
    for yaw in np.radians([-90,-60,-120,0,180,90,-30,30,-150,150,60,120,-75,-105,-45,-135]):
        radial=np.array([np.cos(yaw),np.sin(yaw),0.])
        for sign in [1.,-1.]:
            y=np.array([0.,0.,sign]);z=-radial;rotation=np.column_stack([np.cross(y,z),y,z])
            for rho in radii:
                local_tcp=frame(radial*rho,rotation);G=np.linalg.inv(local_tcp)
                width=receiver_aperture(c,part,G)
                if width is None:continue
                contact=contact_state(c,part,G,width)
                if not contact['opposed_contact_compatible']:continue
                yield incoming@local_tcp,G,width,(float(yaw),float(sign),float(rho),0.)


def fixture_clear(c,part,G,width,deadline=None):
    """Query full robot stroke, small release and world-down withdrawal.

    Payload collision is required at the free-space preinsert endpoint.
    Robot-only insertion allows the intended ring/jig contact; that contact
    still needs a later physical insertion test, and is never disabled there.
    """
    from grasp_transfer_feasibility import insertion_pose_options
    if deadline is None:deadline=time.perf_counter()+float(c.get('config',{}).get('axial_receiver_fixture_budget_s',45.))
    cache=c.setdefault('_axial_receiver_fixture_cache',{});key=(part,_key(G),round(float(width),8))
    if key in cache:return cache[key]
    arm=c['left_arm'];checker=c['checkers']['left'];row=c['handoff_reference'][part]
    opening=min(.08,width+.002)
    counts=c.setdefault('_axial_receiver_fixture_counts',{})
    examples=c.setdefault('_axial_receiver_fixture_examples',{})
    def reject(label,why=None):
        counts[label]=counts.get(label,0)+1
        if why is not None and label not in examples:examples[label]=why
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
                seeds=[row['left_insert_seed'],row['left_receive_q'],c['cmd'][0,c['left_ids']]]
                seeds+=list(np.random.default_rng(61021).uniform(arm.bounds[:,0],arm.bounds[:,1],(20,7)))
                tried=[]
                for seed in seeds:
                    if time.perf_counter()>deadline:reject('fixture_budget');return None
                    q=arm.solve(pre[:3,3],pre[:3,:3],seed)
                    if q is None:reject('pre_IK');continue
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
                        if blocked(checker.path([stroke[-1],nq],width),'stroke_robot'):valid=False;break
                        stroke.append(nq)
                    if not valid:continue
                    if any(blocked(checker.clear(stroke[-1],float(gap)),'release_robot')for gap in np.linspace(width,opening,5)):continue
                    retreat=[stroke[-1]]
                    for u in np.linspace(0,1,17)[1:]:
                        nq=arm.solve(final[:3,3]*(1-u)+withdraw[:3,3]*u,final[:3,:3],retreat[-1])
                        if nq is None:reject('withdraw_IK');valid=False;break
                        if blocked(checker.path([retreat[-1],nq],opening),'withdraw_robot'):valid=False;break
                        retreat.append(nq)
                    if not valid:continue
                    approved=dict(pre_q=np.asarray(q).tolist(),stroke_q=np.asarray(stroke).tolist(),withdraw_q=np.asarray(retreat).tolist(),held_width_m=float(width),release_width_m=float(opening),preinsert_clearance_m=clearance,ring_azimuth_rad=float(azimuth),grasp=G.tolist(),source='Fixed class CAD, native robot IK and live fixed-scene PhysX queries; no free-body transforms',physical_insertion_validated=False)
                    cache[key]=approved;return approved
        cache[key]=None;return None
    finally:
        checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
        checker.resolution_rad=old_resolution


def receiving_poses(c,part,incoming):
    started=time.perf_counter();budget=float(c.get('config',{}).get('axial_receiver_fixture_budget_s',45.))
    for target,G,width,metadata in geometric_candidates(c,part,incoming):
        if time.perf_counter()-started>budget:break
        if fixture_clear(c,part,G,width,deadline=started+budget) is not None:yield target,metadata
