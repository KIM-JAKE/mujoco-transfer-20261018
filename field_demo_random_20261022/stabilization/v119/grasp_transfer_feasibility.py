"""Reject grasps that cannot fit the two native hands at transfer.

Only image-estimated ring pose, commanded gripper geometry and a calibrated
transfer frame are used. No free-body state or scene seed is an input.
This is a cheap necessary check, not an arm reachability or path guarantee.
"""
import numpy as np
from isaac_kinematics import frame, exp
from rgbd_geometry import axis_frame
from predictive_pair_clearance import shape_overlap, transform_proxy, proxy


# Robot commands from a camera-confirmed physical handover; class-only hint.
SMALL_HANDOVER_HINT={'right_seed': [-2.159945249557495, -0.0637764185667038, -2.6602888107299805, -1.5165488719940186, 0.33451780676841736, 0.5093748569488525, -1.6139613389968872], 'left_seed': [-0.14601506292819977, 1.8138461112976074, -1.06662118434906, -1.8581074476242065, -2.2154061794281006, 1.5448558330535889, -0.1740092635154724]}

def robot_route_hint(c,key,start,goal):
    """Retarget a robot-joint prior, never a free-body pose or actor ID."""
    value=c.get('robot_route_hints',{}).get('routes',{}).get(key)
    if value is None:return None
    q=np.asarray(value,float).copy()
    if q.ndim!=2 or q.shape[1]!=7 or len(q)<2:return None
    if max(np.max(abs(q[0]-start)),np.max(abs(q[-1]-goal)))>.75:return None
    q[0]=start;q[-1]=goal
    return q


class VerifiedCubicRoute(list):
    """Original knots whose executed cubic has passed live payload queries."""
    verified_cubic=True


def shorten_checked_route(checker,route,opening,ignore=None):
    """Greedy long-edge shortcuts; every new edge gets the live checker."""
    points=np.asarray(route,float)
    if len(points)<=2:return points
    out=[points[0]];i=0
    while i<len(points)-1:
        for j in range(len(points)-1,i,-1):
            if j==i+1 or checker.path([points[i],points[j]],opening,ignore)[0]:
                out.append(points[j]);i=j;break
    out=np.asarray(out)
    if len(out)>2:
        from scipy.interpolate import CubicSpline
        curve=CubicSpline(np.linspace(0,1,len(out)),out)(np.linspace(0,1,max(65,32*len(out))))
        if np.all(curve>=checker.arm.bounds[:,0]) and np.all(curve<=checker.arm.bounds[:,1]):
            old=getattr(checker,'resolution_rad',.035);checker.resolution_rad=.010
            try:clear=checker.path(curve,opening,ignore)[0]
            finally:checker.resolution_rad=old
            if clear:return VerifiedCubicRoute(out)
    return out


def hinted_route(c,key,checker,start,goal,opening,seconds=.5):
    if checker.path([start,goal],opening)[0]:return [np.asarray(start).copy(),np.asarray(goal).copy()]
    hint=robot_route_hint(c,key,start,goal)
    route=hint if hint is not None and checker.path(hint,opening)[0] else checker.route(start,goal,opening,seconds=seconds,skip_direct=True)
    if route is not None and c.get('config',{}).get('shortcut_checked_routes'):
        return shorten_checked_route(checker,route,opening)
    return route


def pad_aperture(c,part,grasp):
    """Expected two-pad contact using class CAD, never a live object state."""
    cache=c.setdefault('_pad_contact_cache',{});key=(part,bool(c.get('config',{}).get('stable_small_grasp')),np.round(grasp,8).tobytes())
    if key in cache:return cache[key]
    estimator=c['observed_grasp_clearance'].estimator
    points=estimator.models[part]@grasp[:3,:3].T+grasp[:3,3]
    normals=estimator.normals[part]@grasp[:3,:3].T
    usable=(abs(points[:,0])<.0087)&(points[:,2]>-.009)&(points[:,2]<.0088)&(abs(normals[:,1])>.6)&(points[:,1]*normals[:,1]>0)
    y=points[usable,1]
    result=None
    if len(y)>=60:
        low,high=np.quantile(y,[.005,.995])
        if low<-.005 and high>.005 and abs(low+high)<.003:
            width=float(high-low)
            if .010<width<=.080:result=width
    if result is not None and part=='ring2' and c.get('config',{}).get('stable_small_grasp'):
        # A rim can contact both tips while the ring centre already hangs
        # within 1 mm of their ends. That is not a transportable grasp.
        axial_rim_pinch=abs(grasp[1,2])>.95 and .020<grasp[2,3]<.040
        if grasp[2,3]>.0045 and not axial_rim_pinch:
            cache[key]=None
            return None
        # Circular chord contacts can squeeze a ring out sideways even when
        # both pads geometrically reach it. Demand an antipodal friction-cone
        # reserve and material away from the native pad's sharp edges.
        patches=[]
        for sign in [-1,1]:
            mask=usable&(points[:,1]*sign>0);subset=points[mask];ns=normals[mask]
            if len(subset)<15:result=None;break
            edge=np.quantile(subset[:,1]*sign,.995);active=subset[:,1]*sign>edge-.0003
            patch=subset[active];normal=np.mean(ns[active],axis=0);normal/=max(np.linalg.norm(normal),1e-12)
            center=np.median(patch,axis=0)
            if len(patch)<6 or abs(center[0])>.0072 or abs(center[2])>(.0072 if axial_rim_pinch else .0065):result=None;break
            patches.append((center,normal))
        if result is not None:
            line=patches[1][0]-patches[0][0];line/=max(np.linalg.norm(line),1e-12)
            minimum_cosine=1/np.sqrt(1+.4**2)
            if patches[1][1]@line<minimum_cosine or -patches[0][1]@line<minimum_cosine:result=None
    cache[key]=result
    return result


def swept_gripper_boxes(checker,tcp,start,end):
    cache=getattr(checker,'_transfer_sweep_cache',None)
    if cache is None:cache={};checker._transfer_sweep_cache=cache
    key=(round(float(start),9),round(float(end),9))
    if key not in cache:
        a=gripper_boxes(checker,np.eye(4),start);b=gripper_boxes(checker,np.eye(4),end)
        cache[key]=[proxy(np.r_[left[3],right[3]]) for left,right in zip(a,b)]
    return [transform_proxy(box,tcp) for box in cache[key]]


def sequential_transfer_clear(rc,donor_tcp,held_width,lc,receiver_tcp,receiver_width,receiver_opening=.08,donor_release_width=.08):
    # Receiver closes first; giver opens afterwards. Requiring both hands to
    # be fully open simultaneously incorrectly excluded valid transfers.
    relative=np.linalg.inv(receiver_tcp)@donor_tcp;canonical=np.round(relative,12);canonical[canonical==0]=0.
    key=(id(lc),round(float(donor_release_width),12),round(float(held_width),12),round(float(receiver_width),12),round(float(receiver_opening),12),canonical.tobytes())
    cache=getattr(rc,'_sequential_transfer_clear_cache',None)
    if cache is None:cache={};rc._sequential_transfer_clear_cache=cache
    if key in cache:return cache[key]
    # Work in the receiver frame: rigidly moving the whole pair cannot change
    # its mutual clearance. Cache only this invariant geometry calculation.
    receiver_target=max(.001,receiver_width-.003)
    receiver_sweep=swept_gripper_boxes(lc,np.eye(4),receiver_opening,receiver_target)
    donor_held=gripper_boxes(rc,relative,held_width)
    if any(shape_overlap(a,b,margin=.0005) for a in receiver_sweep for b in donor_held):cache[key]=False;return False
    receiver_closed=swept_gripper_boxes(lc,np.eye(4),receiver_target,min(receiver_opening,receiver_width+.002))
    donor_sweep=swept_gripper_boxes(rc,relative,held_width,donor_release_width)
    result=not any(shape_overlap(a,b,margin=.0005) for a in receiver_closed for b in donor_sweep)
    cache[key]=result
    return result


def gripper_boxes(checker, tcp, opening):
    root = tcp @ frame([0, 0, -.1034], np.eye(3))
    side = checker.side
    transforms = {checker.arm.hand_path: root}
    for sign, label in [(1, 'left'), (-1, 'right')]:
        transforms[checker.arm.hand_path + '/' + side + '_' + label + '_finger'] = (
            root @ frame([0, sign * opening / 2, .0584],
                         np.eye(3) if sign == 1 else np.diag([-1., -1., 1.])))
    result = []
    for body, _, box in checker.pair_boxes:
        if body not in transforms:
            continue
        transform = transforms[body]
        result.append(transform_proxy(box, transform))
    if not result:
        raise RuntimeError('Missing native hand collision boxes for transfer check')
    return result



def insertion_pose_options(c,part,grasp):
    base=np.asarray(c['handoff_reference'][part]['approach_ring_poses'])[-1]
    angles=np.radians([30,0,-30,60,-60,90,-90,120,-120,150,-150,180]) if part=='ring2' and c.get('config',{}).get('ring_yaw_free') else [0.]
    for angle in angles:
        target=base.copy();target[:3,:3]=exp(np.array([0.,0.,angle]))@base[:3,:3]
        yield target@np.linalg.inv(grasp),float(angle)


def receiving_poses(c, part, incoming):
    reference = c['handoff_reference'][part]
    taught_tcp = c['left_arm'].fk(reference['left_receive_q'])
    taught_grasp = np.linalg.inv(taught_tcp) @ np.asarray(reference['part_H'])
    base = incoming @ np.linalg.inv(taught_grasp)
    # Tilt about the closing axis: fingers still oppose the ring's diameter,
    # while the palm approaches from another side of the held part.
    cache=c.setdefault('_receiver_angle_cache',{})
    if part not in cache:
        allowed=[]
        for yaw in np.radians([0,180,90,-90,135,-135,45,-45,30,-30,150,-150]):
            for angle in np.radians([0,90,-90,75,-75,60,-60,45,-45,105,-105,120,-120,150,-150,180]):
                R=exp(np.asarray(reference['part_H'])[:3,2]*yaw)@taught_tcp[:3,:3]@exp(np.array([0.,angle,0.]))
                for chord in [0.,.012,-.012,.020,-.020]:
                    for depth in [0.,.004,.008]:
                        H=frame(np.asarray(reference['part_H'])[:3,3]-R@taught_grasp[:3,3]+R[:,0]*chord+R[:,2]*depth,R)
                        G=np.linalg.inv(H)@np.asarray(reference['part_H'])
                        if pad_aperture(c,part,G) is None:continue
                        found=False
                        for final,_ in insertion_pose_options(c,part,G):
                            for seed in [reference['left_insert_seed'],reference['left_receive_q']]:
                                q=c['left_arm'].solve(final[:3,3],final[:3,:3],seed)
                                if q is not None and c['checkers']['left'].clear(q,.075 if part=='ring1' else .062)[0]:found=True;break
                            if found:break
                        if found:allowed.append((angle,yaw,chord,depth))
        cache[part]=allowed
        print('INSERTION_COMPATIBLE_RECEIVER_ANGLES',part,allowed,flush=True)
    rows=cache[part]
    if part=='ring2' and c.get('config',{}).get('reliability_handover_hint'):
        rows=sorted(rows,key=lambda r:0 if np.allclose(r,[0.,0.,0.,.004],atol=1e-9) else 1 if np.allclose(r,[np.pi/2,np.pi,.012,0],atol=1e-9) else 2)
    for angle,yaw,chord,depth in rows:
        rotation=exp(incoming[:3,2]*yaw)@base[:3,:3]@exp(np.array([0.,angle,0.]))
        yield frame(incoming[:3,3]-rotation@taught_grasp[:3,3]+rotation[:,0]*chord+rotation[:,2]*depth,rotation),(angle,yaw,chord,depth)



def compatible(c, part, pose, pickup_tcp):
    grasp = np.linalg.inv(pickup_tcp) @ frame(pose['center_m'], axis_frame(pose['axis']))
    if pose.get('grasp_type')!='inner' and pad_aperture(c,part,grasp) is None:return False
    compatibility_cache=c.setdefault('_grasp_transfer_cache',{})
    cache_key=(part,pose.get('grasp_type'),pose.get('planned_grip_width_m'),np.round(grasp,9).tobytes())
    if cache_key in compatibility_cache:return compatibility_cache[cache_key]
    reference = c['handoff_reference'][part]
    incoming = np.asarray(reference['part_H'])
    receivers = [(tcp,pad_aperture(c,part,np.linalg.inv(tcp)@incoming))
                 for tcp, _ in receiving_poses(c, part, incoming)]
    initial = c['arm'].fk(reference['right_seed'])[:3, :3]
    a, b = initial @ grasp[:3, 2], incoming[:3, 2]
    cross = np.cross(a, b)
    length = np.linalg.norm(cross)
    axis = cross / length if length > 1e-8 else axis_frame(a)[:, 0]
    aligned = exp(axis * np.arctan2(length, a @ b)) @ initial
    canonical=incoming[:3,:3]@grasp[:3,:3].T
    bases=[canonical,aligned] if part=='ring2' else [aligned,canonical]
    rotations=[exp(b*yaw)@base for base in bases for yaw in [0,np.pi,np.pi/2,-np.pi/2,np.pi/4,-np.pi/4,3*np.pi/4,-3*np.pi/4]]
    for rotation in rotations:
        tcp = frame(incoming[:3, 3] - rotation @ grasp[:3, 3], rotation)
        for receiver,width in receivers:
            if sequential_transfer_clear(c['checkers']['right'],tcp,pose.get('planned_grip_width_m',.075 if part == 'ring1' else .062),c['checkers']['left'],receiver,width,min(.08,(.075 if part=='ring1' else .062)+.006),donor_release_width=.001 if pose.get('grasp_type')=='inner' else .08):
                compatibility_cache[cache_key]=True
                return True
    compatibility_cache[cache_key]=False
    return False
