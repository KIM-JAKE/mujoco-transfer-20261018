"""Retain the checked joint path; stop both arms smoothly at each task boundary."""
import numpy as np
from scipy.interpolate import CubicSpline
from fr3_motion_control import position_velocity_bounds

class ReferenceProfile:
    def __init__(self,times,q,names,schedule):
        self.indices=np.array([names.index(f'{side}_arm_fr3v2_joint{j}')for side in ['left','right']for j in range(1,8)])
        self.path=CubicSpline(times,q[:,self.indices],axis=0);self.full_path=CubicSpline(times,q,axis=0)
        drivers=[names.index(side+'_grip_left_driver_joint')for side in ['left','right']];self.driver_indices=drivers
        grip=CubicSpline(times,q[:,drivers],axis=0)
        boundaries=[float(times[0]),float(times[-1])]
        for side,lane in schedule['lanes'].items():
            for row in lane:boundaries.extend([row['start_s']+schedule['delays'][side],row['end_s']+schedule['delays'][side]])
        b=np.unique(np.clip(boundaries,times[0],times[-1]));self.bounds=b[np.r_[True,np.diff(b)>1e-7]]
        self.durations=[];self.records=[]
        ring_grasps=[r for side in ['left','right']for r in schedule['lanes'][side]if r['stage']['label']=='grasp_two_parts']
        pin_approach=next(r for r in schedule['lanes']['left']if r['stage']['label']=='insert_last_pin_approach')
        pin_fine=next(r for r in schedule['lanes']['left']if r['stage']['label']=='insert_last_pin_last5mm')
        u=np.linspace(0,1,1001)
        h=u**3*(10+u*(-15+6*u));h1=30*u*u*(1-u)**2;h2=60*u*(1-u)*(1-2*u);h3=60*(1-6*u+6*u*u)
        for begin,end in zip(self.bounds[:-1],self.bounds[1:]):
            span=end-begin;s=begin+span*h;d1=span*h1;d2=span*h2;d3=span*h3
            p=self.path(s);qs=self.path(s,1);qss=self.path(s,2);qsss=self.path(s,3)
            v=qs*d1[:,None];a=qss*d1[:,None]**2+qs*d2[:,None];j=qsss*d1[:,None]**3+3*qss*d1[:,None]*d2[:,None]+qs*d3[:,None]
            lo,_=position_velocity_bounds((p-.015).reshape(-1,2,7));_,hi=position_velocity_bounds((p+.015).reshape(-1,2,7))
            cap=.70*np.where(v>=0,hi.reshape(-1,14),-lo.reshape(-1,14))
            if np.any((cap<1e-6)&(abs(v)>1e-7)):raise RuntimeError('Path reaches a forbidden FR3 velocity region')
            duration=max(float(np.max(abs(v)/np.maximum(cap,1e-6))),float(np.sqrt(np.max(abs(a))/4.)),float(np.cbrt(np.max(abs(j))/800.)),float(np.max(abs(grip(s,1)*d1[:,None]))/.6),1.5*span,.04)*1.03
            if any(begin>=r['start_s']-1e-7 and end<=r['end_s']+1e-7 for r in ring_grasps):duration=max(duration,float(np.max(abs(grip(s,1)*d1[:,None]))/.15)*1.03)
            if abs(begin-pin_approach['start_s'])<1e-7 and abs(end-pin_approach['end_s'])<1e-7:duration=max(duration,4.)
            if abs(begin-pin_fine['start_s'])<1e-7 and abs(end-pin_fine['end_s'])<1e-7:duration=max(duration,5.)
            self.durations.append(duration);self.records.append(dict(reference_begin_s=float(begin),reference_end_s=float(end),duration_s=duration,peak_acceleration_rad_s2=float(np.max(abs(a))/duration**2),peak_jerk_rad_s3=float(np.max(abs(j))/duration**3)))
        self.durations=np.array(self.durations);self.ends=np.cumsum(self.durations);self.elapsed=0.;self.held=False;self.started=False;self.last_reference=float(times[0]);self.last_rate=0.
    def reference_at(self,time):
        if time>=self.ends[-1]:return float(self.bounds[-1]),0.,0.,0.
        i=int(np.searchsorted(self.ends,time,side='right'));start=0. if i==0 else self.ends[i-1];T=self.durations[i];u=float(np.clip((time-start)/T,0,1));span=self.bounds[i+1]-self.bounds[i]
        return (float(self.bounds[i]+span*u**3*(10+u*(-15+6*u))),float(span*30*u*u*(1-u)**2/T),float(span*60*u*(1-u)*(1-2*u)/T**2),float(span*60*(1-6*u+6*u*u)/T**3))
    def peek(self,dt):return self.reference_at(self.elapsed+dt)[0]
    def gate(self,reference,hold,started):
        self.started=started;self.held=bool(hold)
        if hold:
            i=int(np.argmin(abs(self.bounds-reference)))
            if abs(self.bounds[i]-reference)>2e-6:raise RuntimeError('Unexpected stop inside a planned motion segment: '+str(hold))
            self.elapsed=0. if i==0 else float(self.ends[i-1])
    def advance(self,dt):
        if self.started and not self.held:self.elapsed=min(self.elapsed+dt,float(self.ends[-1]))
        s,d1,d2,d3=self.reference_at(self.elapsed);p=self.path(s);qs=self.path(s,1);qss=self.path(s,2)
        if not self.started or self.held:d1=d2=d3=0.
        self.last_reference=s;self.last_rate=d1
        return p,qs*d1,qss*d1*d1+qs*d2
    def report(self):return dict(total_nominal_s=float(self.ends[-1]),segments=self.records,semantics='C2 spatial spline with minimum-jerk common phase; both arms stop at every task boundary',velocity_fraction=.70,acceleration_design_cap_rad_s2=4.,jerk_design_cap_rad_s3=800.,active_gripper_driver_velocity_cap_rad_s=.6)


class EfficientReferenceProfile(ReferenceProfile):
    """Keep the collision-checked path, retime local curvature, stop at interlocks.

    95% is a planning ceiling, not a guarantee that every axis uses 95% at once.
    A small part of this budget is reserved for the live object-pose correction.
    Contact approach/closure can impose a lower limit than the arm envelope.
    """
    def __init__(self,times,q,names,schedule):
        from scipy.ndimage import gaussian_filter1d
        from scipy.integrate import cumulative_trapezoid
        self.indices=np.array([names.index(f'{side}_arm_fr3v2_joint{j}')for side in ['left','right']for j in range(1,8)])
        self.path=CubicSpline(times,q[:,self.indices],axis=0);self.full_path=CubicSpline(times,q,axis=0)
        self.driver_indices=[names.index(side+'_grip_left_driver_joint')for side in ['left','right']]
        grip=CubicSpline(times,q[:,self.driver_indices],axis=0)
        boundaries=[float(times[0]),float(times[-1])];ring_grasps=[]
        for side,lane in schedule['lanes'].items():
            delay=schedule['delays'][side]
            for row in lane:
                label=row['stage']['label'];begin=row['start_s']+delay;end=row['end_s']+delay
                if label in ['grasp_two_parts','grasp_last_pin','release_two_parts','release_last_pin']:
                    boundaries.extend([begin,end])
                if label=='grasp_two_parts':ring_grasps.append((begin,end))
                if label=='insert_last_pin_approach':boundaries.append(begin)
        self.bounds=np.unique(np.clip(boundaries,times[0],times[-1]));self.warps=[];self.durations=[];self.records=[]
        # Retiming is scalar: both arms retain the checked simultaneous path.
        # Local arc weighting avoids slowing a whole transfer for one tight turn.
        for begin,end in zip(self.bounds[:-1],self.bounds[1:]):
            span=end-begin;s=np.linspace(begin,end,max(501,int(span/.0007)+1))
            qs=self.path(s,1);qss=self.path(s,2);qsss=self.path(s,3)
            w=np.maximum.reduce([np.max(abs(qs),axis=1)/1.8,.35*np.sqrt(np.max(abs(qss),axis=1)/9.1),.15*np.cbrt(np.max(abs(qsss),axis=1)/4700),np.full(len(s),.20)])
            w=gaussian_filter1d(w,max(1.,.020/(s[1]-s[0])),mode='nearest')
            x=cumulative_trapezoid(w,s,initial=0);x/=x[-1]
            warp=CubicSpline(x,s);self.warps.append(warp)
            u=np.linspace(0,1,max(5001,int(span/.0002)+1));h=u**3*(10+u*(-15+6*u));h1=30*u*u*(1-u)**2;h2=60*u*(1-u)*(1-2*u);h3=60*(1-6*u+6*u*u)
            phase=warp(h);w1=warp(h,1);w2=warp(h,2);w3=warp(h,3)
            d1=w1*h1;d2=w2*h1*h1+w1*h2;d3=w3*h1**3+3*w2*h1*h2+w1*h3
            p=self.path(phase);ps=self.path(phase,1);pss=self.path(phase,2);psss=self.path(phase,3)
            v=ps*d1[:,None];acc=pss*d1[:,None]**2+ps*d2[:,None];jerk=psss*d1[:,None]**3+3*pss*d1[:,None]*d2[:,None]+ps*d3[:,None]
            lower,_=position_velocity_bounds((p-.015).reshape(-1,2,7));_,upper=position_velocity_bounds((p+.015).reshape(-1,2,7))
            vcap=.95*np.where(v>=0,upper.reshape(-1,14),-lower.reshape(-1,14))-.025
            if np.any((vcap<1e-6)&(abs(v)>1e-7)):raise RuntimeError('Path reaches a forbidden FR3 velocity region')
            driver_cap=np.full((len(phase),2),.6)
            for b,e in ring_grasps:driver_cap[(phase>=b)&(phase<=e),:]=.15
            constraints=dict(velocity=float(np.max(abs(v)/np.maximum(vcap,1e-6))),acceleration=float(np.sqrt(np.max(abs(acc))/9.1)),jerk=float(np.cbrt(np.max(abs(jerk))/4700)),gripper=float(np.max(abs(grip(phase,1)*d1[:,None])/driver_cap)))
            duration=max(*constraints.values(),.04)*1.015
            # Preserve deliberately slow insertion, but remove the 4+5 s fixed
            # delays. Measured alignment and clamp gates still control release.
            labels=[row['stage']['label']for row in schedule['lanes']['left']if row['start_s']>=begin-1e-7 and row['end_s']<=end+1e-7]
            if 'insert_last_pin_last5mm'in labels:duration=max(duration,1.5)
            self.durations.append(duration);self.records.append(dict(reference_begin_s=float(begin),reference_end_s=float(end),duration_s=duration,limiting_constraint=max(constraints,key=constraints.get),unit_duration_constraints=constraints,peak_acceleration_rad_s2=float(np.max(abs(acc))/duration**2),peak_jerk_rad_s3=float(np.max(abs(jerk))/duration**3)))
        self.durations=np.array(self.durations);self.ends=np.cumsum(self.durations);self.elapsed=0.;self.held=False;self.started=False;self.last_reference=float(times[0]);self.last_rate=0.

    def reference_at(self,time):
        if time>=self.ends[-1]:return float(self.bounds[-1]),0.,0.,0.
        i=int(np.searchsorted(self.ends,time,side='right'));start=0. if i==0 else self.ends[i-1];T=self.durations[i];u=float(np.clip((time-start)/T,0,1));warp=self.warps[i]
        h=u**3*(10+u*(-15+6*u));d1=30*u*u*(1-u)**2/T;d2=60*u*(1-u)*(1-2*u)/T**2;d3=60*(1-6*u+6*u*u)/T**3
        p=float(warp(h));w1=float(warp(h,1));w2=float(warp(h,2));w3=float(warp(h,3))
        return p,w1*d1,w2*d1*d1+w1*d2,w3*d1**3+3*w2*d1*d2+w1*d3

    def report(self):return dict(total_nominal_s=float(self.ends[-1]),segments=self.records,semantics='C2 collision-checked spatial path with local arc weighting; stop only at measured-contact interlocks',planning_fraction=.95,velocity_fraction=.95,acceleration_planning_ceiling_rad_s2=9.5,nominal_acceleration_budget_rad_s2=9.1,jerk_planning_ceiling_rad_s3=4750.,nominal_jerk_budget_rad_s3=4700.,live_correction_budget=dict(velocity_rad_s=.025,acceleration_rad_s2=.4,jerk_rad_s3=50.),active_gripper_driver_velocity_cap_rad_s=.6,ring_grasp_driver_velocity_cap_rad_s=.15)


def reference_fingerprint(times,q,names,schedule):
    import hashlib,json
    h=hashlib.sha256()
    for a in [times,q]:h.update(np.ascontiguousarray(a,dtype='<f8').tobytes())
    h.update(json.dumps(names).encode());h.update(json.dumps(schedule,sort_keys=True).encode())
    return h.hexdigest()


class PreparedReferenceProfile(ReferenceProfile):
    """Execute the frozen offline timing without loading a solver into Kit."""
    def __init__(self,times,q,names,schedule,archive):
        import json
        from scipy.interpolate import PPoly
        self.indices=np.array([names.index(f'{side}_arm_fr3v2_joint{j}')for side in ['left','right']for j in range(1,8)])
        self.driver_indices=[names.index(side+'_grip_left_driver_joint')for side in ['left','right']]
        self.path=CubicSpline(times,q[:,self.indices],axis=0);self.full_path=CubicSpline(times,q,axis=0)
        with np.load(archive)as data:
            if str(data['fingerprint'])!=reference_fingerprint(times,q,names,schedule):raise RuntimeError('Prepared timing does not match frozen reference')
            self.bounds=data['bounds'].copy();self.durations=data['durations'].copy();self.clocks=[];self.scales=[]
            for i in range(len(self.durations)):
                self.clocks.append(PPoly(data[f'clock_{i}_c'].copy(),data[f'clock_{i}_x'].copy()))
                self.scales.append(float(data[f'clock_{i}_scale']))
        self.metadata=json.loads(archive.with_suffix('.json').read_text());self.ends=np.cumsum(self.durations)
        self.elapsed=0.;self.held=False;self.started=False;self.last_reference=float(times[0]);self.last_rate=0.

    def reference_at(self,time):
        if time>=self.ends[-1]:return float(self.bounds[-1]),0.,0.,0.
        i=int(np.searchsorted(self.ends,time,side='right'));start=0. if i==0 else self.ends[i-1];scale=self.scales[i];clock=self.clocks[i];t=float(np.clip((time-start)/scale,clock.x[0],clock.x[-1]))
        s=float(np.clip(clock(t),self.bounds[i],self.bounds[i+1]))
        if s<=self.bounds[i]+1e-13 or s>=self.bounds[i+1]-1e-13:return s,0.,0.,0.
        return s,float(clock(t,1)/scale),float(clock(t,2)/scale**2),float(clock(t,3)/scale**3)

    def report(self):return self.metadata|dict(state='executing_prepared_timing',semantics='Frozen C2 phase clock; simultaneous collision-checked path; stops only at measured-contact interlocks',velocity_fraction=.95)
