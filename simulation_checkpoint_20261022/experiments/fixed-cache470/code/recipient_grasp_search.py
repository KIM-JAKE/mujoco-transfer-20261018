"""Native-hand regrasp candidates checked at handover and insertion.

The selected grasp offset is carried through the entire insertion path. Only
the empty-hand retreat blends back to a safe parked joint configuration.
"""
import math
import numpy as np
from isaac_kinematics import exp,frame
from basket_pipeline import hermite


def plan(arm,checker,tape,indices,center,axis,selected,endpoint,handover_offset=None):
    from pxr import UsdGeom
    for _,source_path,op,_ in checker.rows:
        if 'NativeHand'not in source_path:continue
        mesh=UsdGeom.Mesh(op.GetAttr().GetPrim());v=np.asarray(mesh.GetPointsAttr().Get());C=(v.max(0)+v.min(0))/2;half=np.maximum((v.max(0)-v.min(0))/2,.001);mesh.GetPointsAttr().Set((C+(v-C)*(1+.001/half)).astype(np.float32))
    offset=np.zeros(3)if handover_offset is None else np.array(handover_offset,float)
    def shifted(T,t):
        T=T.copy();u=np.clip((t-67.)/7.,0,1);T[:3,3]+=offset*(1-u**3*(10-15*u+6*u*u));return T
    times=tape['time_s'];baseline=tape['command'].copy();contact_t=63.1229166667;insert_t=80.5541666667;withdraw_t=84.2854166667;park_t=89.38125
    original_contact=arm.fk(hermite(tape,contact_t)[indices]);original_insert=arm.fk(hermite(tape,insert_t)[indices]);checks=[]
    angles=[math.pi/4,0.,math.pi/12,-math.pi/12,math.pi/6,-math.pi/6,-math.pi/4,math.pi/3,-math.pi/3,math.pi/2,-math.pi/2,2*math.pi/3,-2*math.pi/3,math.pi]
    for tilt in [math.pi/6,math.pi/12,math.pi/4,0.,-math.pi/6,-math.pi/12,-math.pi/4,-math.pi/3]:
        for angle in angles:
            world_tilt=original_contact[:3,:3]@exp(np.array([0.,tilt,0.]))@original_contact[:3,:3].T
            R=exp(axis*angle)@world_tilt;H=frame(center+R@(original_contact[:3,3]-center),R@original_contact[:3,:3]);D=np.linalg.inv(original_contact)@H
            candidate_path=[];clear=True;reason=None;seed=hermite(tape,endpoint)[indices]
            for t in np.linspace(endpoint,contact_t,8):
                target=shifted(arm.fk(hermite(tape,t)[indices])@D,t);q=arm.solve(target[:3,3],target[:3,:3],seed)
                if q is None:clear=False;reason='handover IK';break
                seed=q;candidate_path.append(q);clear,reason=checker.clear(q,.08,selected)
                if not clear:break
            if clear:
                for width in [.078,.073,.068,.062]:
                    clear,reason=checker.clear(candidate_path[-1],width,selected)
                    if not clear:break
            if clear:
                target=original_insert@D;insert=arm.solve(target[:3,3],target[:3,:3],hermite(tape,insert_t)[indices])
                if insert is None:clear=False;reason='insertion IK'
                else:clear,reason=checker.clear(insert,.062,selected)
            if not clear:
                checks.append(dict(yaw_rad=angle,tilt_rad=tilt,clear=False,reason=reason));continue
            print('RECIPIENT_STATIC_CLEAR',float(tilt),float(angle),flush=True)
            mapped={};seed=candidate_path[0];carry_start=68.3958333333;carry_end=76.58125
            # Reusing the old joint-space transit with a different grasp can
            # pass through unreachable Cartesian poses. Replan that transit
            # instead of forcing IK through the old arm's null-space motion.
            for i in np.where((times>=endpoint-.025)&(times<=carry_start))[0]:
                target=shifted(arm.fk(baseline[i,indices])@D,times[i]);q=arm.solve(target[:3,3],target[:3,:3],seed)
                if q is None:clear=False;reason='carry path IK';break
                seed=q;mapped[int(i)]=q
            if not clear:
                checks.append(dict(yaw_rad=angle,tilt_rad=tilt,clear=False,reason=reason));continue
            start_q=seed.copy();seed=insert.copy();entry={}
            for i in np.where((times>=carry_end)&(times<=withdraw_t))[0]:
                target=arm.fk(baseline[i,indices])@D;q=arm.solve(target[:3,3],target[:3,:3],seed)
                if q is None:clear=False;reason='entry path IK';break
                seed=q;entry[int(i)]=q
            if not clear or not entry:
                checks.append(dict(yaw_rad=angle,tilt_rad=tilt,clear=False,reason=reason));continue
            old_ignored=getattr(checker,'ignored_future_actors',set())
            checker.ignored_future_actors={str(p.GetPath())for p in checker.stage.Traverse()if 'right_arm_base'in str(p.GetPath())}
            # At this future interval the donor has withdrawn and is fetching
            # the pin. Cross-arm dynamics are still checked during execution.
            from isaac_kinematics import world
            hand_at_contact=shifted(original_contact@D,contact_t)@frame([0,0,-.1034],np.eye(3))
            checker.attach_payload(selected,np.linalg.inv(hand_at_contact)@world(checker.stage,selected))
            route=checker.route(start_q,entry[min(entry)],.0574,selected,seconds=8)
            checker.detach_payload()
            checker.ignored_future_actors=old_ignored
            if route is None:
                checks.append(dict(yaw_rad=angle,tilt_rad=tilt,clear=False,reason='carry route blocked'));continue
            from isaac_motion import duration
            required=np.array([duration([a,b],.35)for a,b in zip(route[:-1],route[1:])]);available=carry_end-carry_start
            if sum(required)>available:
                checks.append(dict(yaw_rad=angle,tilt_rad=tilt,clear=False,reason='carry route exceeds allocated time',required_s=float(sum(required))));continue
            durations=required*available/sum(required);knots=np.r_[0,np.cumsum(durations)]
            for i in np.where((times>carry_start)&(times<carry_end))[0]:
                elapsed=times[i]-carry_start;k=min(len(durations)-1,int(np.searchsorted(knots,elapsed,side='right')-1));u=np.clip((elapsed-knots[k])/durations[k],0,1);h=u**3*(10-15*u+6*u*u);mapped[int(i)]=route[k]+(route[k+1]-route[k])*h
            mapped.update(entry)
            # The source's original ready pose lies inside a PBV zero-speed
            # band. Returning to its exact joints would violate the limits.
            # Retarget the empty-arm park to the interior of those bands.
            last_index=max(mapped);delta=mapped[last_index]-baseline[last_index,indices]
            old_ready=hermite(tape,park_t)[indices];safe_ready=np.clip(old_ready,arm.bounds[:,0]+.002,arm.bounds[:,1]-.002)
            for i,q in mapped.items():tape['command'][i,indices]=q
            for i in np.where(times>times[last_index])[0]:
                u=np.clip((times[i]-times[last_index])/(park_t-times[last_index]),0,1);h=u**3*(10-15*u+6*u*u)
                tape['command'][i,indices]=baseline[i,indices]+delta*(1-h)+(safe_ready-old_ready)*h
            checks.append(dict(yaw_rad=angle,tilt_rad=tilt,clear=True))
            return dict(yaw_rad=angle,tilt_rad=tilt,handover_offset_m=offset.tolist(),grasp_offset=D.tolist(),safe_ready_q=safe_ready.tolist(),carry_route_waypoints=len(route),carry_duration_s=available,carry_required_duration_s=float(sum(required)),checks=checks,planning_margin_m=.001,future_donor_ignored_for_carry_plan=True)
    checker.recipient_checks=checks
    raise RuntimeError('No native recipient grasp clears handover and insertion: '+str(checks))
