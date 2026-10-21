"""Reject grasps that cannot fit the two native hands at transfer.

Only image-estimated ring pose, commanded gripper geometry and a calibrated
transfer frame are used. No free-body state or scene seed is an input.
This is a cheap necessary check, not an arm reachability or path guarantee.
"""
import numpy as np
from isaac_kinematics import frame, exp, log
from rgbd_geometry import axis_frame
from predictive_pair_clearance import shape_overlap, transform_proxy, proxy


# Robot commands from a camera-confirmed physical handover; class-only hint.
SMALL_HANDOVER_HINT={'right_seed': [-2.159945249557495, -0.0637764185667038, -2.6602888107299805, -1.5165488719940186, 0.33451780676841736, 0.5093748569488525, -1.6139613389968872], 'left_seed': [-0.14601506292819977, 1.8138461112976074, -1.06662118434906, -1.8581074476242065, -2.2154061794281006, 1.5448558330535889, -0.1740092635154724]}

def robot_route_hint(c,key,start,goal):
    """Retarget a robot-joint prior, never a free-body pose or actor ID."""
    spline=c.get('config',{}).get('warm_spline_routes') and key in c.get('robot_route_hints',{}).get('spline_routes',{})
    value=c.get('robot_route_hints',{}).get('spline_routes' if spline else 'routes',{}).get(key)
    if value is None:return None
    q=np.asarray(value,float).copy()
    if q.ndim!=2 or q.shape[1]!=7 or len(q)<2:return None
    if max(np.max(abs(q[0]-start)),np.max(abs(q[-1]-goal)))>.75:return None
    if spline:
        u=np.linspace(0,1,len(q));q+=(np.asarray(start)-q[0])*(1-u[:,None])+(np.asarray(goal)-q[-1])*u[:,None]
        return VerifiedCubicRoute(q)
    q[0]=start;q[-1]=goal
    return q


class VerifiedCubicRoute(list):
    """Original knots whose executed cubic has passed live payload queries."""
    verified_cubic=True


def hint_path_clear(checker,route,opening,ignore=None):
    if getattr(route,'verified_cubic',False):
        from scipy.interpolate import CubicSpline
        curve=CubicSpline(np.linspace(0,1,len(route)),np.asarray(route))(np.linspace(0,1,max(129,4*len(route))))
        if np.any(curve<checker.arm.bounds[:,0]) or np.any(curve>checker.arm.bounds[:,1]):return False
        old=getattr(checker,'resolution_rad',.035);checker.resolution_rad=.010
        try:return checker.path(curve,opening,ignore)[0]
        finally:checker.resolution_rad=old
    return checker.path(route,opening,ignore)[0]


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
        from scipy.interpolate import CubicSpline,BSpline
        candidates=[]
        if len(out)>3:
            for repeats in [1,2]:
                controls=np.vstack([out[:1],np.repeat(out[1:-1],repeats,axis=0),out[-1:]])
                degree=min(3,len(controls)-1);knots=np.r_[np.zeros(degree+1),np.linspace(0,1,len(controls)-degree+1)[1:-1],np.ones(degree+1)]
                candidates.append(BSpline(knots,controls,degree)(np.linspace(0,1,33)))
        candidates.append(out)
        for points in candidates:
            curve=CubicSpline(np.linspace(0,1,len(points)),points)(np.linspace(0,1,max(65,4*len(points))))
            if np.all(curve>=checker.arm.bounds[:,0]) and np.all(curve<=checker.arm.bounds[:,1]):
                old=getattr(checker,'resolution_rad',.035);checker.resolution_rad=.010
                try:clear=checker.path(curve,opening,ignore)[0]
                finally:checker.resolution_rad=old
                if clear:return VerifiedCubicRoute(points)

    return out


def hinted_route(c,key,checker,start,goal,opening,seconds=.5):
    if checker.path([start,goal],opening)[0]:return [np.asarray(start).copy(),np.asarray(goal).copy()]
    hint=robot_route_hint(c,key,start,goal)
    if hint is not None and getattr(hint,'verified_cubic',False) and hint_path_clear(checker,hint,opening):return hint
    route=hint if hint is not None and not getattr(hint,'verified_cubic',False) and checker.path(hint,opening)[0] else checker.route(start,goal,opening,seconds=seconds,skip_direct=True)
    if route is not None and c.get('config',{}).get('shortcut_checked_routes'):
        return shorten_checked_route(checker,route,opening)
    return route


def receiver_opening(c,part):
    """One native-hand opening used by compatibility, planning and execution."""
    if c.get('config',{}).get('receiver_axial_only'):return (.0155 if part=='ring1' else .014)+.008
    nominal=.068 if part=='ring2' else .08
    value=float(c.get('config',{}).get('small_receiver_opening_m',nominal)) if part=='ring2' else nominal
    if not np.isfinite(value) or not .01<=value<=.08:
        raise ValueError('Receiver opening must be within the native 10--80 mm range')
    return value


class StagedReceiverRoute(list):
    """Exact checked joint edges; stop at stand-off before terminal approach."""
    def __init__(self,points,standoff_index):
        super().__init__(points)
        self.standoff_index=int(standoff_index)


def receiver_approach_route(c,key,checker,start,goal,opening,seconds=.5):
    """Approach along the receiving TCP axis, with held stock in query context.

    The caller must install the donor arm AND its image-estimated payload as
    obstacles. No scene actor is moved. Returned edges require corner stops;
    they must not be replaced with an unchecked global interpolating cubic.
    """
    config=c.get('config',{});distance=float(config.get('receiver_approach_standoff_m',0.))
    if distance<=0:return hinted_route(c,key,checker,start,goal,opening,seconds=seconds)
    if not np.isfinite(distance) or distance>.08:raise ValueError('Invalid receiver stand-off')
    spacing=float(config.get('receiver_approach_step_m',.002))
    resolution=float(config.get('receiver_approach_resolution_rad',.002))
    if not .0005<=spacing<=.005 or not .0002<=resolution<=.01:raise ValueError('Invalid receiver approach resolution')
    arm=checker.arm;start=np.asarray(start,float);goal=np.asarray(goal,float)
    target=arm.fk(goal);stand=target.copy();stand[:3,3]-=target[:3,2]*distance
    pre=arm.solve(stand[:3,3],stand[:3,:3],goal)
    if pre is None:return None
    old=getattr(checker,'resolution_rad',.035);checker.resolution_rad=min(old,resolution)
    try:
        prefix=hinted_route(c,key+'_standoff',checker,start,pre,opening,seconds=seconds)
        if prefix is None:return None
        # Prefix hints can carry cubic semantics. Recheck the actual joint
        # chords before joining them to the terminal stop-and-go edges.
        prefix=[np.asarray(q,float).copy() for q in prefix]
        if not checker.path(prefix,opening)[0]:return None
        terminal=[pre.copy()];count=max(2,int(np.ceil(distance/spacing)))
        for index in range(1,count+1):
            u=index/count;position=stand[:3,3]*(1-u)+target[:3,3]*u
            q=goal.copy() if index==count else arm.solve(position,target[:3,:3],terminal[-1])
            if q is None or np.max(abs(q-terminal[-1]))>.15:return None
            if not checker.path([terminal[-1],q],opening)[0]:return None
            terminal.append(np.asarray(q,float).copy())
        return StagedReceiverRoute(prefix+terminal[1:],len(prefix)-1)
    finally:checker.resolution_rad=old


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
        if grasp[2,3]>.006 and not axial_rim_pinch:
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
        if result is not None and .0045<grasp[2,3]<=.006 and not axial_rim_pinch:
            # The COM alone can cross the old 4.5 mm heuristic while both
            # native pads still have broad, opposed CAD contact. Admit only
            # this narrow band with an independent actual finger-hull proof;
            # pickup must still confirm the fresh image/encoder contacts.
            from receiver_pad_contact import contact_state
            if not contact_state(c,part,grasp,result)['opposed_contact_compatible']:result=None
    cache[key]=result
    return result


def receiver_aperture(c,part,grasp):
    """Receiver-only opposed-pad stability, independently of donor grasp mode."""
    width=pad_aperture(c,part,grasp)
    if width is None or not c.get('config',{}).get('stable_receiver_grasp'):return width
    cache=c.setdefault('_receiver_stable_contact_cache',{});key=(part,np.round(grasp,8).tobytes())
    if key in cache:return cache[key]
    estimator=c['observed_grasp_clearance'].estimator
    points=estimator.models[part]@grasp[:3,:3].T+grasp[:3,3]
    normals=estimator.normals[part]@grasp[:3,:3].T
    usable=(abs(points[:,0])<.0087)&(points[:,2]>-.009)&(points[:,2]<.0088)&(abs(normals[:,1])>.6)&(points[:,1]*normals[:,1]>0)
    axial_rim_pinch=abs(grasp[1,2])>.95 and .020<grasp[2,3]<.040
    # The accepted large-ring receiver has COM Z=4.64 mm while both native
    # pad patches remain supported. Keep the existing edge/antipodal checks,
    # and admit the extra 1.5 mm only with an independent native-hull proof.
    tip_limit=.006 if part=='ring1' else .0045
    valid=not(grasp[2,3]>tip_limit and not axial_rim_pinch);patches=[]
    if valid:
        for sign in [-1,1]:
            mask=usable&(points[:,1]*sign>0);subset=points[mask];ns=normals[mask]
            if len(subset)<15:valid=False;break
            edge=np.quantile(subset[:,1]*sign,.995);active=subset[:,1]*sign>edge-.0003
            patch=subset[active];normal=np.mean(ns[active],axis=0);normal/=max(np.linalg.norm(normal),1e-12)
            center=np.median(patch,axis=0)
            if len(patch)<6 or abs(center[0])>.0072 or abs(center[2])>(.0072 if axial_rim_pinch else .0065):valid=False;break
            patches.append((center,normal))
    if valid:
        line=patches[1][0]-patches[0][0];line/=max(np.linalg.norm(line),1e-12)
        minimum_cosine=1/np.sqrt(1+.4**2)
        valid=bool(patches[1][1]@line>=minimum_cosine and -patches[0][1]@line>=minimum_cosine)
    if valid and part=='ring1' and .0045<grasp[2,3]<=.006 and not axial_rim_pinch:
        from receiver_pad_contact import contact_state
        valid=contact_state(c,part,grasp,width)['opposed_contact_compatible']
    cache[key]=width if valid else None
    return cache[key]



def large_donor_pad_support(c,grasp,actual_opening_m):
    """Conservative finite-pad support reserve from image G/native CAD.

    This is a geometry admission condition, not measured contact force or a
    dynamic retention guarantee. A 0.3 mm first-contact band must support at
    least 5 mm vertically on each native flat pad. Existing proximity,
    antipodal, finger-edge and actual encoder completion gates still apply.
    """
    from pathlib import Path
    from scipy.spatial import ConvexHull
    G=np.asarray(grasp,float);opening=float(actual_opening_m)
    cache=c.setdefault('_large_donor_support_geometry',{})
    if 'planes' not in cache:
        vertices=np.load(Path(__file__).resolve().parent/'native-official-finger.npz')['vertices']
        cache['planes']=ConvexHull(vertices).equations
    estimator=c['observed_grasp_clearance'].estimator;planes=cache['planes']
    points=estimator.models['ring1']@G[:3,:3].T+G[:3,3];jaws=[]
    for sign in [-1,1]:
        flip=np.array([sign,sign,1.]);local=points*flip;local[:,1]-=opening/2;local[:,2]+=.045
        halfspace=local@planes[:,:3].T+planes[:,3];distance=halfspace.max(axis=1)
        distal=(abs(points[:,0])<.011)&(abs(points[:,2])<.009)
        first=float(distance[distal].min()) if distal.any() else float('inf')
        active=distal&(distance<=first+.0003)&(distance<=.0008)
        count=int(active.sum());facets=planes[np.argmax(halfspace[active],axis=1),:3]*flip
        flat=np.abs(facets[:,1])>=.999;flat_count=int(flat.sum())
        flat_points=points[active][flat]
        span=float(np.ptp(flat_points[:,2])) if flat_count else 0.;flat_fraction=float(flat.mean()) if count else 0.
        accepted=bool(flat_count>=5 and span>=.005 and flat_fraction>=.9)
        jaws.append(dict(sign=sign,active_samples=count,active_flat_samples=flat_count,active_vertical_span_m=span,flat_native_facet_fraction=flat_fraction,minimum_hull_clearance_m=first if np.isfinite(first)else None,supported=accepted))
    return dict(broad_opposed_flat_pad_support=bool(all(j['supported']for j in jaws)),jaws=jaws,first_contact_band_m=.0003,minimum_vertical_support_m=.005,minimum_flat_facet_fraction=.9,actual_opening_m=opening,uses_free_body_poses=False,force_contact_measured=False)


def donor_aperture(c,part,grasp):
    """Opt-in large donor antipodal and native-hull contact admission.

    Uses class CAD at an image-derived candidate pose. Actual encoder contact
    and a fresh RGB-D observation are still required before lifting.
    """
    width=pad_aperture(c,part,grasp)
    if width is None or part!='ring1' or not (c.get('config',{}).get('stable_outer_donor_grasp') or c.get('config',{}).get('require_broad_large_donor_support')):
        return width
    cache=c.setdefault('_outer_donor_contact_cache',{})
    key=(bool(c.get('config',{}).get('receiver_axial_only')),bool(c.get('config',{}).get('require_broad_large_donor_support')),np.round(grasp,8).tobytes())
    if key in cache:return cache[key]
    checked=dict(c,config=dict(c.get('config',{}),stable_receiver_grasp=True))
    width=receiver_aperture(checked,part,grasp)
    if width is not None:
        from receiver_pad_contact import contact_state
        if not contact_state(c,part,grasp,width)['opposed_contact_compatible']:width=None
    if width is not None and c.get('config',{}).get('require_broad_large_donor_support'):
        if not large_donor_pad_support(c,grasp,width)['broad_opposed_flat_pad_support']:width=None
    cache[key]=width
    return width


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
    angles=np.radians([30,0,-30,60,-60,90,-90,120,-120,150,-150,180]) if (part=='ring2' or c.get('config',{}).get('all_rings_yaw_free')) and c.get('config',{}).get('ring_yaw_free') else [0.]
    for angle in angles:
        target=base.copy();target[:3,:3]=exp(np.array([0.,0.,angle]))@base[:3,:3]
        yield target@np.linalg.inv(grasp),float(angle)


def _receiver_depth_options(c,part):
    """Physical receiver TCP approach depths; never modify measured G."""
    option='receiver_depth_options_m' if part=='ring2' else 'large_receiver_depth_options_m' if part=='ring1' else None
    configured=c.get('config',{}).get(option) if option else None
    if configured is None:return [0.,.004,.008],part
    depths=np.asarray(configured,dtype=float)
    if depths.ndim!=1 or not 1<=len(depths)<=8 or not np.isfinite(depths).all() or np.any(depths<0.) or np.any(depths>.008):
        raise ValueError('Native ring receiver depths must contain 1--8 finite values within 0--8 mm')
    # Isolate custom catalogs from both the default and other configured
    # depths, including when an existing precompiled catalog is supplied.
    depths=list(dict.fromkeys(float(v) for v in depths))
    key=part+'|depths='+','.join(format(v,'.17g') for v in depths)
    return depths,key


def receiving_poses(c, part, incoming):
    if part=='ring1' and c.get('config',{}).get('temporary_axial_double_handover_component'):
        # Temporary holder NEVER inserts: use actual bilateral axial caps,
        # not the irrelevant final-jig receiver catalog.
        from axial_rim_receiver import geometric_candidates
        for target,G,width,metadata in geometric_candidates(c,part,incoming):
            yaw,sign,rho,_=metadata
            if abs(yaw)<1e-8 and sign==1. and abs(rho-.0345)<1e-8:
                yield target,metadata
        return
    if c.get('config',{}).get('receiver_axial_only'):
        from axial_rim_receiver import receiving_poses as axial_poses
        yield from axial_poses(c,part,incoming)
        return
    reference = c['handoff_reference'][part]
    taught_tcp = c['left_arm'].fk(reference['left_receive_q'])
    taught_grasp = np.linalg.inv(taught_tcp) @ np.asarray(reference['part_H'])
    base = incoming @ np.linalg.inv(taught_grasp)
    # Tilt about the closing axis: fingers still oppose the ring's diameter,
    # while the palm approaches from another side of the held part.
    depths,cache_key=_receiver_depth_options(c,part)
    exact_axis=bool(part=='ring1' and c.get('config',{}).get('exact_axis_large_receiver_grasp'))
    balanced=bool(part=='ring1' and (c.get('config',{}).get('balanced_large_receiver_grasp') or exact_axis))
    exact_angle=float(np.arctan2(taught_grasp[0,2],taught_grasp[2,2]))
    centered_chord=float(taught_grasp[0,3])
    chords=[0.,.012,-.012,.020,-.020]
    if balanced:
        # The taught receiver COM lies about5 mm along tool X. Chord0
        # preserves that offset; physically translate the TCP along X so
        # nominal ring COM is centered between the finite native pad widths.
        chords=list(dict.fromkeys([centered_chord]+chords))
        cache_key=str(cache_key)+'|balanced-X='+format(centered_chord,'.17g')
    angles=list(np.radians([0,90,-90,75,-75,60,-60,45,-45,105,-105,120,-120,150,-150,180]))
    if exact_axis:
        # Rotate physically about closing Y until the ring axis is parallel
        # to tool Z. This exact class-derived angle is absent from the grid.
        angles=list(dict.fromkeys([exact_angle]+angles))
        cache_key=str(cache_key)+'|exact-axis-Y='+format(exact_angle,'.17g')
    cache=c.setdefault('_receiver_angle_cache',{})
    if cache_key not in cache:
        allowed=[]
        for yaw in np.radians([0,180,90,-90,135,-135,45,-45,30,-30,150,-150]):
            for angle in angles:
                R=exp(np.asarray(reference['part_H'])[:3,2]*yaw)@taught_tcp[:3,:3]@exp(np.array([0.,angle,0.]))
                for chord in chords:
                    for depth in depths:
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
        cache[cache_key]=allowed
        print('INSERTION_COMPATIBLE_RECEIVER_ANGLES',cache_key,allowed,flush=True)
    rows=cache[cache_key]
    if part=='ring1' and (balanced or c.get('config',{}).get('center_large_receiver_grasp')):
        preferred_depth=depths[0] if c.get('config',{}).get('large_receiver_depth_options_m') is not None else .004
        preferred_chord=centered_chord if balanced else 0.
        preferred_angle=exact_angle if exact_axis else 0.
        rows=sorted(rows,key=lambda r:0 if np.allclose(r,[preferred_angle,0.,preferred_chord,preferred_depth],atol=1e-9) else 1 if exact_axis and abs(r[0]-exact_angle)<1e-9 and abs(r[2]-centered_chord)<1e-9 else 2 if balanced and abs(r[2]-centered_chord)<1e-9 else 3)
    if part=='ring2' and c.get('config',{}).get('reliability_handover_hint'):
        preferred_depth=depths[0] if c.get('config',{}).get('receiver_depth_options_m') is not None else .004
        rows=sorted(rows,key=lambda r:0 if np.allclose(r,[0.,0.,0.,preferred_depth],atol=1e-9) else 1 if np.allclose(r,[np.pi/2,np.pi,.012,0],atol=1e-9) else 2)
    if part=='ring1' and balanced and c.get('config',{}).get('large_receiver_depth_options_m') is not None:
        # The nominal two-seed catalog only proposes candidates. Retain a
        # bounded centered depth bank for live RGB-D/model revalidation; a
        # nominal IK miss must not suppress physically admissible new branches.
        centered=[(0.,0.,centered_chord,float(depth)) for depth in depths]
        rows=centered+[r for r in rows if not any(np.allclose(r,p,atol=1e-9) for p in centered)]
    if part=='ring1' and c.get('config',{}).get('native_large_receiver_tilt25_only'):
        # Two bounded native-specific proposals. Their 3.5 mm COM offset
        # keeps the finite native fingers clear of the original 0540 fixture;
        # 25 degrees is reachable where the straight axial family was not.
        # This diagnostic mode has no legacy tilted-posture fallback: every
        # actual contact/IK/fixed/payload/pair/issued-path gate must still pass.
        tilt=np.radians(25.)
        for yaw in np.radians([-75.,-90.]):
            rotation=exp(np.array([0.,0.,yaw]))@exp(np.array([0.,-tilt,0.]))
            grasp=frame([0,0,.0035],rotation.T)
            target=incoming@np.linalg.inv(grasp)
            if receiver_aperture(c,part,grasp) is not None:
                yield target,(float(tilt),float(yaw),0.,.0035)
        return
    if part=='ring1' and c.get('config',{}).get('large_receiver_tool_axis_aligned'):
        # A native Hand diameter grip with tool Z parallel to the measured
        # ring axis avoids inheriting the tilted taught gripper posture.
        # The offsets below are actual proposed ring COM in TCP coordinates,
        # not an assignment of a physical part or an assumed contact.
        for axial_com in [.002,.0015]:
            for yaw in [0.,np.pi/4,np.pi/2]:
                local_tcp=frame([0,0,-axial_com],exp(np.array([0.,0.,yaw])))
                target=incoming@local_tcp
                if receiver_aperture(c,part,np.linalg.inv(target)@incoming) is not None:
                    yield target,(0.,float(yaw),0.,float(axial_com))
    if part=='ring1' and balanced and c.get('config',{}).get('project_large_receiver_closing_axis'):
        # The real native pads oppose a diameter only when their closing
        # direction lies in the observed ring plane. Correct the taught
        # frame's small axial closing component by physically rotating the
        # receiver target, never by altering the measured object transform.
        axis=np.asarray(incoming[:3,2],float).copy();axis/=np.linalg.norm(axis)
        y=base[:3,1]-axis*(base[:3,1]@axis)
        if np.linalg.norm(y)>1e-6:
            y/=np.linalg.norm(y);z=base[:3,2]-y*(base[:3,2]@y);z/=np.linalg.norm(z)
            rotation=np.column_stack([np.cross(y,z),y,z])
            # This is a small alignment proposal, not another whole-hand
            # orientation family. Original candidates remain available below.
            correction=float(np.linalg.norm(log(rotation@base[:3,:3].T)))
            if correction<=.035:
                for depth in depths:
                    target=frame(incoming[:3,3]-rotation@taught_grasp[:3,3]+rotation[:,0]*centered_chord+rotation[:,2]*depth,rotation)
                    if receiver_aperture(c,part,np.linalg.inv(target)@incoming) is not None:
                        yield target,(0.,0.,centered_chord,float(depth))
    for angle,yaw,chord,depth in rows:
        rotation=exp(incoming[:3,2]*yaw)@base[:3,:3]@exp(np.array([0.,angle,0.]))
        target=frame(incoming[:3,3]-rotation@taught_grasp[:3,3]+rotation[:,0]*chord+rotation[:,2]*depth,rotation)
        # Apply even to a cached fixed-angle catalog: changing a runtime
        # reliability flag must never reuse unfiltered receiver geometry.
        if receiver_aperture(c,part,np.linalg.inv(target)@incoming) is None:continue
        yield target,(angle,yaw,chord,depth)



def compatible(c, part, pose, pickup_tcp):
    grasp = np.linalg.inv(pickup_tcp) @ frame(pose['center_m'], axis_frame(pose['axis']))
    if pose.get('grasp_type')=='rim':
        from local_rim_grasp import rim_aperture
        if rim_aperture(c,part,grasp) is None:return False
    elif pose.get('grasp_type')!='inner' and pad_aperture(c,part,grasp) is None:return False
    donor_release=float(pose.get('release_width_m',.001 if pose.get('grasp_type')=='inner' else .08))
    compatibility_cache=c.setdefault('_grasp_transfer_cache',{})
    cache_key=(bool(c.get('config',{}).get('temporary_axial_double_handover_component')),bool(c.get('config',{}).get('native_large_receiver_tilt25_only')),bool(c.get('config',{}).get('large_receiver_tool_axis_aligned')),bool(c.get('config',{}).get('project_large_receiver_closing_axis')),part,pose.get('grasp_type'),pose.get('planned_grip_width_m'),donor_release,receiver_opening(c,part),bool(c.get('config',{}).get('balanced_large_receiver_grasp')),bool(c.get('config',{}).get('exact_axis_large_receiver_grasp')),np.round(grasp,9).tobytes())
    if cache_key in compatibility_cache:return compatibility_cache[cache_key]
    reference = c['handoff_reference'][part]
    incoming = np.asarray(reference['part_H'])
    receivers = [(tcp,receiver_aperture(c,part,np.linalg.inv(tcp)@incoming))
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
            if width is None:continue
            if sequential_transfer_clear(c['checkers']['right'],tcp,pose.get('planned_grip_width_m',.075 if part == 'ring1' else .062),c['checkers']['left'],receiver,width,receiver_opening(c,part),donor_release_width=donor_release):
                compatibility_cache[cache_key]=True
                return True
    compatibility_cache[cache_key]=False
    return False
