"""Offline path-preserving retiming; keep the HiGHS runtime outside Isaac Kit."""
from pathlib import Path
import argparse, hashlib, json
import numpy as np
from scipy.interpolate import CubicSpline, PchipInterpolator
from scipy.ndimage import gaussian_filter1d
from scipy.optimize import linprog, minimize_scalar
from scipy.sparse import coo_matrix
from fr3_motion_control import position_velocity_bounds
from fr3_reference_profile import EfficientReferenceProfile as ArcProfile, reference_fingerprint

CONTACT_MINIMUMS={'insert_last_pin_approach':.85,'insert_last_pin_last5mm':.65,'press_button':.5}
GRIPPER_TRAVEL_CAP=.6
PIN_EARLY_CLOSE_CAP=.25
MAX_PHASE_RATE=3.
PATH_ACCELERATION_LIMIT=9.1
MINIMUM_TIME_OBJECTIVE=False


def minimize_traversal_time(initial,ds,matrix,rhs,bounds,iterations=60):
    """Convex time objective on the same acceleration/speed polytope.

    Maximizing sum(sdot²) is a useful seed but can sacrifice a slow section to
    gain speed elsewhere. Frank-Wolfe uses sparse LPs plus exact line searches
    to minimize the actual trapezoidal traversal time instead.
    """
    tiny=np.full(len(initial),1e-8);tiny[0]=tiny[-1]=0
    tiny[1:-1]=np.minimum(tiny[1:-1],np.array([b[1]for b in bounds[1:-1]]))
    x=.99999*initial+.00001*tiny
    def objective(value):
        v=np.sqrt(np.maximum(value,0.))
        return float(np.sum(2*ds/np.maximum(v[:-1]+v[1:],1e-20)))
    before=objective(x)
    for k in range(iterations):
        v=np.sqrt(np.maximum(x,1e-30));pair=(v[:-1]+v[1:])**2
        g=np.zeros(len(x));g[:-1]-=ds/(v[:-1]*pair);g[1:]-=ds/(v[1:]*pair)
        g[0]=g[-1]=0
        result=linprog(g/max(1.,float(abs(g).max())),A_ub=matrix,b_ub=rhs,bounds=bounds,method='highs')
        if not result.success:raise RuntimeError(result.message)
        direction=result.x-x
        line=minimize_scalar(lambda alpha:objective(x+alpha*direction),bounds=(0.,1.),method='bounded',options={'xatol':1e-9})
        previous=objective(x)
        if line.fun>=previous-1e-8:break
        x=x+line.x*direction
        if (previous-line.fun)/max(previous,1e-12)<1e-6:break
    return x,dict(iterations=k+1,seed_time_s=before,optimized_time_s=objective(x))


def driver_caps(s,schedule):
    cap=np.full((len(s),2),GRIPPER_TRAVEL_CAP)
    for side,lane in schedule['lanes'].items():
        for row in lane:
            if row['stage']['label']in ['grasp_two_parts','grasp_last_pin']:
                a=row['start_s']+schedule['delays'][side];b=row['end_s']+schedule['delays'][side]
                active=(s>=a)&(s<=b)
                if row['stage']['label']=='grasp_two_parts':cap[active,:]=.14
                else:
                    u=np.clip((s[active]-a)/(b-a),0,1)
                    w=np.clip((u-.5)/.25,0,1);w=w*w*(3-2*w)
                    cap[active,:]=((1-w)*PIN_EARLY_CLOSE_CAP+w*.25)[:,None]
    return cap


def fine_rate_cap(s,schedule,target_15=False):
    cap=np.full(len(s),MAX_PHASE_RATE)
    for side,lane in schedule['lanes'].items():
        for row in lane:
            label=row['stage']['label']
            if label not in ['insert_two_parts_last5mm','insert_last_pin_last5mm']:continue
            a=row['start_s']+schedule['delays'][side];b=row['end_s']+schedule['delays'][side]
            cap[(s>=a)&(s<=b)]=np.minimum(cap[(s>=a)&(s<=b)],(b-a)/((CONTACT_MINIMUMS['insert_last_pin_last5mm'] if target_15 else 1.)if label=='insert_last_pin_last5mm'else .5))
    return cap


def retime_segment(profile,begin,end,schedule,sigma,target_15=False):
    s=np.linspace(begin,end,max(101,int((end-begin)/.002)+1));ds=np.diff(s);mid=(s[:-1]+s[1:])/2;n=len(s)
    ps=profile.path(mid,1);pss=profile.path(mid,2);qs=profile.path(s,1);p=profile.path(s)
    lo,_=position_velocity_bounds((p-.015).reshape(-1,2,7));_,hi=position_velocity_bounds((p+.015).reshape(-1,2,7))
    cap=.95*np.where(qs>=0,hi.reshape(-1,14),-lo.reshape(-1,14))-.025
    if np.any((cap<0)&(abs(qs)>1e-7)):raise RuntimeError('No reserved FR3 velocity envelope')
    vmax=np.min(np.where(abs(qs)>1e-8,np.maximum(cap,.0001)/np.maximum(abs(qs),1e-8),MAX_PHASE_RATE),axis=1)
    gs=profile.full_path(s,1)[:,profile.driver_indices]
    vmax=np.minimum(vmax,np.min(driver_caps(s,schedule)/np.maximum(abs(gs),1e-8),axis=1))
    vmax=np.minimum(vmax,fine_rate_cap(s,schedule,target_15))
    rows=[];cols=[];values=[];rhs=[]
    for k in range(n-1):
        a=.5*pss[k]-.5*ps[k]/ds[k];b=.5*pss[k]+.5*ps[k]/ds[k]
        for sign in [1,-1]:
            for j in range(14):
                row=len(rhs);rows.extend([row,row]);cols.extend([k,k+1]);values.extend([sign*a[j],sign*b[j]]);rhs.append(PATH_ACCELERATION_LIMIT)
    bounds=[(1e-10,float(max(v*v,1e-10)))for v in vmax];bounds[0]=bounds[-1]=(0,0)
    matrix=coo_matrix((values,(rows,cols)),shape=(len(rhs),n)).tocsr()
    result=linprog(-np.ones(n),A_ub=matrix,b_ub=rhs,bounds=bounds,method='highs')
    if not result.success:raise RuntimeError(result.message)
    if MINIMUM_TIME_OBJECTIVE:
        result.x,_=minimize_traversal_time(result.x,ds,matrix,rhs,bounds)
    # Tiny derivatives in exactly stationary reference intervals can leave an
    # arbitrary LP basis. Give these points finite phase speed; the full analytic
    # derivative check below still certifies every candidate after smoothing.
    x=np.maximum(result.x,np.minimum(vmax*vmax,.01));x[0]=x[-1]=0
    v=np.sqrt(x);t=np.r_[0,np.cumsum(2*ds/np.maximum(v[:-1]+v[1:],1e-8))]
    if t[-1]>30:raise RuntimeError('Ill-conditioned phase clock')
    pad=5*sigma;tt=np.linspace(0,t[-1]+2*pad,int((t[-1]+2*pad)*960)+2);tick=tt[1]-tt[0]
    raw=PchipInterpolator(t,s)(np.clip(tt-pad,0,t[-1]))
    smooth=gaussian_filter1d(raw,sigma/tick,mode='nearest',truncate=4);smooth[:3]=begin;smooth[-3:]=end
    clock=CubicSpline(tt,smooth,bc_type=((1,0.),(1,0.)))
    # Dense planning check is separate from the actual 480 Hz PhysX audit.
    ts=np.unique(np.r_[np.linspace(0,tt[-1],max(16001,int(tt[-1]*4000))),tt])
    phase=np.clip(clock(ts),begin,end);d1=clock(ts,1);d2=clock(ts,2);d3=clock(ts,3)
    ps=profile.path(phase,1);pss=profile.path(phase,2);psss=profile.path(phase,3)
    qv=ps*d1[:,None];qa=pss*d1[:,None]**2+ps*d2[:,None]
    qj=psss*d1[:,None]**3+3*pss*d1[:,None]*d2[:,None]+ps*d3[:,None]
    p=profile.path(phase);lo,_=position_velocity_bounds((p-.015).reshape(-1,2,7));_,hi=position_velocity_bounds((p+.015).reshape(-1,2,7))
    cap=.95*np.where(qv>=0,hi.reshape(-1,14),-lo.reshape(-1,14))-.025
    gv=profile.full_path(phase,1)[:,profile.driver_indices]*d1[:,None]
    ratios=dict(velocity=float(np.max(abs(qv)/np.maximum(cap,.001))),acceleration=float(np.sqrt(np.max(abs(qa))/PATH_ACCELERATION_LIMIT)),jerk=float(np.cbrt(np.max(abs(qj))/4700)),gripper=float(np.max(abs(gv)/driver_caps(phase,schedule))),fine_insertion=float(np.max(abs(d1)/fine_rate_cap(phase,schedule,target_15))))
    if getattr(profile,'model_torque_screen',None) is not None:
        ratios['model_torque_rate']=float(np.max(profile.model_torque_screen.ratios(phase,d1,d2,d3)))
    factor=max(1.,*ratios.values())*1.015
    # The final precision insertion has its own measured 25um alignment gate.
    # Budget time separately for pin approach, fine insertion and button contact.
    pin_approach=next(row['start_s']+schedule['delays']['left']for row in schedule['lanes']['left']if row['stage']['label']=='insert_last_pin_approach')
    pin_fine=next(row['start_s']+schedule['delays']['left']for row in schedule['lanes']['left']if row['stage']['label']=='insert_last_pin_last5mm')
    if abs(begin-pin_approach)<1e-7:factor=max(factor,(CONTACT_MINIMUMS['insert_last_pin_approach'] if target_15 else 4.)/tt[-1])
    if abs(begin-pin_fine)<1e-7:factor=max(factor,(CONTACT_MINIMUMS['insert_last_pin_last5mm'] if target_15 else 2.)/tt[-1])
    button_press=next(row for row in schedule['lanes']['right']if row['stage']['label']=='press_button')
    if abs(begin-button_press['start_s']-schedule['delays']['right'])<1e-7:factor=max(factor,(CONTACT_MINIMUMS['press_button'] if target_15 else 1.2)/tt[-1])
    return clock, factor, dict(reference_begin_s=float(begin),reference_end_s=float(end),duration_s=float(tt[-1]*factor),raw_duration_s=float(tt[-1]),time_scale=factor,smoothing_s=sigma,limiting_constraint=max(ratios,key=ratios.get),peak_acceleration_rad_s2=float(np.max(abs(qa))/factor**2),peak_jerk_rad_s3=float(np.max(abs(qj))/factor**3),minimum_phase_rate=float(d1.min()/factor))


def planning_ratios(profile,clock,t,begin,end,schedule):
    s=np.clip(clock(t),begin,end);d1=clock(t,1);d2=clock(t,2);d3=clock(t,3)
    ps=profile.path(s,1);pss=profile.path(s,2);psss=profile.path(s,3)
    v=ps*d1[:,None];a=pss*d1[:,None]**2+ps*d2[:,None]
    j=psss*d1[:,None]**3+3*pss*d1[:,None]*d2[:,None]+ps*d3[:,None]
    p=profile.path(s);lo,_=position_velocity_bounds((p-.015).reshape(-1,2,7));_,hi=position_velocity_bounds((p+.015).reshape(-1,2,7))
    cap=.95*np.where(v>=0,hi.reshape(-1,14),-lo.reshape(-1,14))-.025
    ratios=np.maximum.reduce([np.max(abs(v)/np.maximum(cap,.001),axis=1),np.sqrt(np.max(abs(a),axis=1)/PATH_ACCELERATION_LIMIT),np.cbrt(np.max(abs(j),axis=1)/4700),np.max(abs(profile.full_path(s,1)[:,profile.driver_indices]*d1[:,None])/driver_caps(s,schedule),axis=1),abs(d1)/fine_rate_cap(s,schedule,True),np.ones(len(t))])
    if getattr(profile,'model_torque_screen',None) is not None:
        ratios=np.maximum(ratios,profile.model_torque_screen.ratios(s,d1,d2,d3))
    return ratios,a,j


def contact_minimum(begin,schedule):
    for side,label in [('left','insert_last_pin_approach'),('left','insert_last_pin_last5mm'),('right','press_button')]:
        row=next(r for r in schedule['lanes'][side]if r['stage']['label']==label)
        if abs(begin-row['start_s']-schedule['delays'][side])<1e-7:return CONTACT_MINIMUMS[label]
    return 0.


def local_time_allocation(profile,clock,begin,end,schedule,sigma):
    from scipy.integrate import cumulative_trapezoid
    from scipy.ndimage import maximum_filter1d
    old=np.linspace(0,clock.x[-1],int(clock.x[-1]*1920)+2);phase=clock(old);weights=np.ones(len(old));tick=old[1]-old[0]
    for iteration in range(9):
        times=cumulative_trapezoid(weights,old,initial=0);candidate=CubicSpline(times,phase,bc_type=((1,0.),(1,0.)))
        ratios,_,_=planning_ratios(profile,candidate,times,begin,end,schedule)
        if ratios.max()<1.005:break
        dilation=maximum_filter1d(ratios,size=max(3,int(sigma/tick)),mode='nearest')
        dilation=gaussian_filter1d(dilation,max(1,sigma/tick/2),mode='nearest')
        weights*=np.maximum(1.,dilation)
    times=cumulative_trapezoid(weights,old,initial=0);candidate=CubicSpline(times,phase,bc_type=((1,0.),(1,0.)))
    check=np.unique(np.r_[times,np.linspace(0,times[-1],int(times[-1]*6000)+1)])
    ratios,acceleration,jerk=planning_ratios(profile,candidate,check,begin,end,schedule)
    factor=max(float(ratios.max())*1.015,contact_minimum(begin,schedule)/times[-1])
    report=dict(reference_begin_s=float(begin),reference_end_s=float(end),duration_s=float(times[-1]*factor),raw_duration_s=float(times[-1]),time_scale=factor,smoothing_s=sigma,limiting_constraint='local_envelope',local_iterations=iteration+1,peak_acceleration_rad_s2=float(np.max(abs(acceleration))/factor**2),peak_jerk_rad_s3=float(np.max(abs(jerk))/factor**3),minimum_phase_rate=float(candidate(check,1).min()/factor))
    return candidate,factor,report


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--base',type=Path,default=Path(__file__).parent/'runs/contact-fr3-09');parser.add_argument('--output',type=Path);parser.add_argument('--target-15',action='store_true');args=parser.parse_args()
    if args.output is None:args.output=Path(__file__).parent/'generated/contact_task'/('fr3-under15-profile.npz'if args.target_15 else'fr3-efficient-profile.npz')
    root=Path(__file__).resolve().parent;base=args.base
    names=json.loads((base/'dof-names.json').read_text());source=json.loads((root/'generated/contact_task/contact-source.json').read_text());lookup={j['name']:j['qpos_address']for j in source['joints']}
    z=np.load(base/'recording/trajectory.npz');q=z['qpos'][:,[lookup[n]for n in names]].copy();plan=np.load(base/'arm-posture-fr3.npz')
    for i,name in enumerate(names):
        if '_arm_'in name:q[:,i]=plan[name.split('_')[0]+'_qpos'][:,int(name[-1])-1]
    schedule=json.loads((base/'recording/independent_schedule.json').read_text());profile=ArcProfile(z['time_s'],q,names,schedule)
    pin_fine=next(row['start_s']+schedule['delays']['left']for row in schedule['lanes']['left']if row['stage']['label']=='insert_last_pin_last5mm')
    button_press=next(row for row in schedule['lanes']['right']if row['stage']['label']=='press_button')
    profile.bounds=np.unique(np.r_[profile.bounds,pin_fine,button_press['start_s']+schedule['delays']['right'],button_press['end_s']+schedule['delays']['right']])
    data=dict(bounds=profile.bounds,fingerprint=np.array(reference_fingerprint(z['time_s'],q,names,schedule)));records=[]
    for i,(begin,end)in enumerate(zip(profile.bounds[:-1],profile.bounds[1:])):
        candidates=[retime_segment(profile,begin,end,schedule,sigma,args.target_15)for sigma in [.002,.006,.012]]
        clock,factor,report=min(candidates,key=lambda candidate:candidate[2]['duration_s'])
        if args.target_15 and end-begin>.15:
            for sigma in [.015,.04,.07]:
                candidate=local_time_allocation(profile,clock,begin,end,schedule,sigma)
                if candidate[2]['duration_s']<report['duration_s']:clock,factor,report=candidate
        # The empty right-arm return needs extra tracking margin in native PhysX.
        if args.target_15 and 3.89<begin<3.90:
            factor*=1.08;report['duration_s']*=1.08;report['time_scale']=factor
            report['peak_acceleration_rad_s2']/=1.08**2;report['peak_jerk_rad_s3']/=1.08**3
            report['native_tracking_margin_time_scale']=1.08
        if report['minimum_phase_rate']<-.0001:raise RuntimeError('Nonmonotonic retiming')
        data[f'clock_{i}_x']=clock.x;data[f'clock_{i}_c']=clock.c;data[f'clock_{i}_scale']=np.array(factor);records.append(report)
        print(i,round(begin,3),round(end,3),round(report['duration_s'],3),report['limiting_constraint'],flush=True)
    data['durations']=np.array([row['duration_s']for row in records]);np.savez_compressed(args.output,**data)
    report=dict(state='planned_unvalidated_in_PhysX',target_15=args.target_15,cycle_target_s=15. if args.target_15 else None,planning_fraction=.95,total_nominal_s=float(data['durations'].sum()),segments=records,input_fingerprint=str(data['fingerprint']),archive_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),solver='offline scalar path retiming + smooth phase clock; geometry and two-arm coordination preserved',acceleration_planning_ceiling_rad_s2=9.5,nominal_acceleration_budget_rad_s2=PATH_ACCELERATION_LIMIT,jerk_planning_ceiling_rad_s3=4750.,nominal_jerk_budget_rad_s3=4700.,live_correction_budget=dict(velocity_rad_s=.025,acceleration_rad_s2=9.5-PATH_ACCELERATION_LIMIT,jerk_rad_s3=50.))
    args.output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items()if k!='segments'},indent=2))
if __name__=='__main__':main()
