"""Offline local time allocation for native-hand command paths.

Only actuator targets and finite clamp commands are retimed. Workpiece states
are never replayed. The generated clock still requires fresh Isaac dynamics.
"""
from pathlib import Path
import sys,json,argparse
import numpy as np
from scipy.interpolate import BSpline,CubicSpline,PchipInterpolator
from scipy.optimize import linprog
from scipy.sparse import coo_matrix
from scipy.integrate import cumulative_trapezoid
from scipy.ndimage import gaussian_filter1d,maximum_filter1d
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parents[1]))
from fr3_motion_control import position_velocity_bounds
from isaac_motion import Audit


def minimum(label,duration):
    if label=='initial':return 0.
    if 'planar_drag'in label:return min(duration,.85)
    if 'installed_hold'in label:return min(duration,.8)
    if any(x in label for x in ['grasp_hold','close_hold']):return min(duration,.22)
    if any(x in label for x in ['recipient_retention','partial_retention','giver_release_hold']):return min(duration,.1)
    if any(x in label for x in ['settled','_hold','clamp']):return min(duration,.06)
    if 'insert'in label and 'preinsert'not in label and 'orient'not in label:return min(duration,.30)
    if any(x in label for x in ['descend','_contact']):return min(duration,.25)
    if label=='settle':return .12
    return 0.


def prepare(source,output,names,contact_scale=1.,method='local',stops_override=None,loaded_right_accel=2.,stop_hold_s=.18):
    z=np.load(source);source_old=z['time_s'];old=source_old.copy();cmd=z['command'];ph=z['phase'];control=z['clamp'];dt=1/480
    ai=[names.index(f'{s}_arm_fr3v2_joint{j}')for s in ['left','right']for j in range(1,8)];fi=[i for i,n in enumerate(names)if '_native_'in n]
    # Arc coordinates remove the slow quintic tails of the original commands.
    # Include finger/clamp travel and finite stationary length in the metric.
    delta=np.column_stack([np.diff(cmd[:,ai].astype(float),axis=0),30*np.diff(cmd[:,fi].astype(float),axis=0),.05*np.diff(control,axis=0)])
    old=np.r_[0,np.cumsum(np.maximum(np.linalg.norm(delta,axis=1),np.diff(source_old)*.02))];duration=float(old[-1])
    tick=.0015;ts=np.arange(0,duration+tick/2,tick);q=np.column_stack([np.interp(ts,old,cmd[:,i])for i in ai]);coeff=np.pad(q,((1,2),(0,0)),mode='edge');knots=(np.arange(len(coeff)+4)-3)*tick;path=BSpline(knots,coeff,3,axis=0)
    s=np.arange(0,duration,.002);s=np.r_[s,duration];ds=s[1]-s[0];p=path(s);ps=path(s,1);pss=path(s,2);psss=path(s,3)
    lo,hi=position_velocity_bounds(p.reshape(-1,2,7));cap=.95*np.where(ps.reshape(-1,2,7)>=0,hi,-lo).reshape(-1,14)
    rate=np.minimum(np.min(np.maximum(cap,1e-8)/np.maximum(abs(ps),1e-12),axis=1),100.)
    fg=np.column_stack([np.interp(s,old,cmd[:,i])for i in fi]);cc=np.column_stack([np.interp(s,old,control[:,i])for i in range(3)]);fgs=np.gradient(fg,s,axis=0);cs=np.gradient(cc,s,axis=0)
    rate=np.minimum(rate,.046/np.maximum(abs(fgs).max(1),1e-12));rate=np.minimum(rate,9./np.maximum(abs(cs).max(1),1e-12))
    # Individual lane labels, rather than arbitrary intersections of labels.
    for lane in range(2):
        labels=np.array([str(x).split(' || ')[min(lane,len(str(x).split(' || '))-1)]for x in ph]);cuts=np.r_[0,np.flatnonzero(labels[1:]!=labels[:-1])+1,len(old)-1]
        for a,b in zip(cuts[:-1],cuts[1:]):
            length=float(old[b]-old[a]);keep=minimum(str(labels[a]),float(source_old[b]-source_old[a]))*(1. if any(x in str(labels[a])for x in ['grasp_hold','close_hold','retention','giver_release_hold'])else contact_scale)
            if keep>0:rate[(s>=old[a])&(s<=old[b])]=np.minimum(rate[(s>=old[a])&(s<=old[b])],length/keep)
    # Always retain an actual final observation window.
    final_start=np.interp(source_old[-1]-1,source_old,old);rate[s>final_start]=np.minimum(rate[s>final_start],.02)
    # Solve q''(s)*sdot^2 + q'(s)*sddot acceleration constraints.
    rr=[];co=[];vv=[];rhs=[]
    for u in [0.,.5,1.]:
        loc=s[:-1]+np.diff(s)*u;pm=path(loc,1);pmm=path(loc,2)
        aa=(1-u)*pmm-.5*pm/np.diff(s)[:,None];bb=u*pmm+.5*pm/np.diff(s)[:,None]
        for sign in [1,-1]:
            for joint in range(14):
                budget=np.full(len(s)-1,8.6)
                if joint>=7:
                    source_loc=np.interp(loc,old,source_old)
                    for begin,end in [(11.08,17.335),(37.67,41.60),(60.76,63.06)]:budget[(source_loc>=begin)&(source_loc<=end)]=float(loaded_right_accel)
                rows=np.arange(len(s)-1)+len(rhs);rr.extend(np.repeat(rows,2));co.extend(np.column_stack([np.arange(len(s)-1),np.arange(1,len(s))]).ravel());vv.extend(np.column_stack([sign*aa[:,joint],sign*bb[:,joint]]).ravel());rhs.extend(budget)
    stops=[]
    for part,point in (stops_override if stops_override is not None else [('ring1',20.95),('ring2',46.4896),('pin',64.858)]):
        if source_old[0]<point<source_old[-1]:
            pos=np.interp(point,source_old,old);ix=int(np.argmin(abs(s-pos)));stops.append((part,ix,float(np.interp(s[ix],old,source_old))))
    bounds=[(1e-10,float(max(x*x,1e-10)))for x in rate];bounds[0]=bounds[-1]=(0,0)
    if method=='filtered-lp':
        for _,ix,_ in stops:bounds[ix]=(0,0)
    result=linprog(-np.ones(len(s)),A_ub=coo_matrix((vv,(rr,co)),shape=(len(rhs),len(s))).tocsr(),b_ub=rhs,bounds=bounds,method='highs')
    if not result.success:raise RuntimeError(result.message)
    solved=np.maximum(result.x,np.minimum(rate*rate,.01));solved[0]=solved[-1]=0
    if method=='filtered-lp':
        for _,ix,_ in stops:solved[ix]=0.
    v=np.sqrt(solved);raw_t=np.r_[0,np.cumsum(2*np.diff(s)/np.maximum(v[:-1]+v[1:],1e-8))]
    print('RAW_COST',float(raw_t[-1]),[(float(np.interp(s[i],old,source_old)),float(np.diff(raw_t)[i]),float(rate[i]))for i in np.argsort(np.diff(raw_t))[-12:]],flush=True)
    if method=='filtered-lp':
        # Smooth the actual target curve in physical time, rather than creating
        # additional clock curvature. This rounds the path slightly and therefore
        # remains an unvalidated physical candidate until the full contact run.
        raw_s=s.copy()
        for _,ix,_ in reversed(stops):
            raw_t=np.insert(raw_t,ix+1,raw_t[ix]+stop_hold_s);raw_t[ix+2:]+=stop_hold_s;raw_s=np.insert(raw_s,ix+1,raw_s[ix])
        pad=.12;total=raw_t[-1]+2*pad;factor=1.;history=[]
        for iteration in range(81):
            factor=1.+iteration*.005
            ticks=np.arange(int(np.ceil(total*factor/dt))+1)*dt
            reference=PchipInterpolator(raw_t,raw_s)(np.clip(ticks/factor-pad,0,raw_t[-1]));full=np.column_stack([np.interp(reference,old,cmd[:,i])for i in range(cmd.shape[1])]);raw=path(reference);full[:,ai]=gaussian_filter1d(raw,.018/dt,axis=0,mode='nearest');full=full.astype(np.float32)
            v=np.diff(np.vstack([full[:1,ai],full[:,ai]]).astype(float),axis=0)/dt;a=np.diff(np.vstack([np.zeros((1,14)),v]),axis=0)/dt;j=np.diff(np.vstack([np.zeros((1,14)),a]),axis=0)/dt
            lo,hi=position_velocity_bounds(full[:,ai].reshape(-1,2,7));cap=.95*np.where(v.reshape(-1,2,7)>=0,hi,-lo).reshape(-1,14)
            grip=float(abs(np.diff(full[:,fi],axis=0)).max()/dt);ratio=max(float((abs(v)/np.maximum(cap,1e-8)).max()),float(np.sqrt(abs(a).max()/9.1)),float(np.cbrt(abs(j).max()/4400)),grip/.046)
            history.append(dict(iteration=iteration,duration_s=float(ticks[-1]),ratio=ratio));print('FILTERED_LP',history[-1],flush=True)
            if ratio<=1.:break
        audit=Audit(full[:1],dict(left=ai[:7],right=ai[7:]),dt)
        for row in full[1:]:audit.update(row[None,:])
        clamps=np.column_stack([np.interp(reference,old,control[:,i])for i in range(3)]);output.parent.mkdir(parents=True,exist_ok=True);metadata={}
        if 'right_source_time_s'in z:metadata['right_source_time_s']=np.interp(reference,old,z['right_source_time_s'])
        metadata['normalization_parts']=np.array([p for p,_,_ in stops]);metadata['normalization_source_time_s']=np.array([v for _,_,v in stops])
        np.savez_compressed(output,time_s=ticks,command=full,clamp=clamps,source_time_s=np.interp(reference,old,source_old),**metadata)
        report=dict(source=str(source),duration_s=float(ticks[-1]),physical_cycle_pass=False,command_limit_audit=audit.report(),finger_target_peak_speed_m_s=grip,contact_scale=contact_scale,method=method,smoothing_sigma_s=.018,maximum_joint_path_change_rad=float(abs(full[:,ai]-raw).max()),history=history,workpiece_state_replay=False);output.with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));return
    # Smooth in physical time before allocating extra time locally for jerk.
    sigma=.016;pad=5*sigma;x=np.linspace(0,raw_t[-1]+2*pad,int((raw_t[-1]+2*pad)*960)+2);tick=x[1]-x[0]
    raw=PchipInterpolator(raw_t,s)(np.clip(x-pad,0,raw_t[-1]));smooth=gaussian_filter1d(raw,sigma/tick,mode='nearest',truncate=4);smooth[:3]=0;smooth[-3:]=duration
    base=CubicSpline(x,smooth,bc_type=((1,0.),(1,0.)))
    # Allocation coordinates now follow the smooth LP solution in time.
    allocation=x.copy();phase=base(allocation);weights=np.ones(len(x))
    history=[]
    for iteration in range(24):
        t=np.r_[0,cumulative_trapezoid(weights,allocation)];clock=CubicSpline(t,phase,bc_type=((1,0.),(1,0.)))
        # Rescreen in physical time; include all allocation nodes.
        x=np.unique(np.r_[t,np.arange(0,t[-1],1/960),t[-1]]);ss=np.clip(clock(x),0,duration);v=clock(x,1);a=clock(x,2);j=clock(x,3);qs=path(ss,1);qss=path(ss,2);qsss=path(ss,3);pp=path(ss)
        qv=qs*v[:,None];qa=qss*v[:,None]**2+qs*a[:,None];qj=qsss*v[:,None]**3+3*qss*v[:,None]*a[:,None]+qs*j[:,None]
        lo,hi=position_velocity_bounds(pp.reshape(-1,2,7));vc=.95*np.where(qv.reshape(-1,2,7)>=0,hi,-lo).reshape(-1,14)
        grip_s=np.column_stack([np.interp(ss,s,fgs[:,i])for i in range(len(fi))]);clamp_s=np.column_stack([np.interp(ss,s,cs[:,i])for i in range(3)])
        components=np.array([np.max(np.where(abs(qv)<1e-6,0.,abs(qv))/np.maximum(vc,1e-8),axis=1),np.sqrt(abs(qa).max(1)/8.8),np.cbrt(abs(qj).max(1)/4200),abs(grip_s*v[:,None]).max(1)/.046,abs(clamp_s*v[:,None]).max(1)/9.])
        ratios=np.maximum(components.max(0),1.)
        worst=np.unravel_index(np.argmax(components),components.shape)
        history.append(dict(iteration=iteration,duration_s=float(t[-1]),ratio=float(ratios.max()),minimum_clock_rate=float(v.min())))
        print('LOCAL_RETIME',history[-1],'worst',worst[0],float(ss[worst[1]]),flush=True)
        if ratios.max()<1.007 and v.min()>-1e-4:break
        dilation=np.interp(t,x,ratios);dilation=maximum_filter1d(dilation,size=max(3,int(.20/tick)));dilation=gaussian_filter1d(dilation,.035/tick);weights*=np.maximum(1.,dilation)*1.001
    factor=max(1.025,float(ratios.max())*1.02)
    total=float(t[-1]*factor);ticks=np.arange(0,total,dt);ticks=np.r_[ticks,round(np.ceil(total/dt))*dt];reference=np.clip(clock(np.minimum(ticks/factor,t[-1])),0,duration)
    full=np.column_stack([np.interp(reference,old,cmd[:,i])for i in range(cmd.shape[1])]);full[:,ai]=path(reference);clamps=np.column_stack([np.interp(reference,old,control[:,i])for i in range(3)])
    # Actual execution also uses float32 targets; audit exactly those values.
    full=full.astype(np.float32);audit=Audit(full[:1],dict(left=ai[:7],right=ai[7:]),dt)
    for row in full[1:]:audit.update(row[None,:])
    grip_peak=float(np.max(abs(np.diff(full[:,fi],axis=0)))/dt)
    if grip_peak>.0476:raise RuntimeError('Retimed gripper target exceeds speed limit '+str(grip_peak))
    output.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(output,time_s=ticks,command=full,clamp=clamps,source_time_s=np.interp(reference,old,source_old))
    report=dict(source=str(source),duration_s=float(ticks[-1]),physical_cycle_pass=False,command_limit_audit=audit.report(),finger_target_peak_speed_m_s=grip_peak,contact_scale=contact_scale,history=history,workpiece_state_replay=False)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);p.add_argument('--contact-scale',type=float,default=1.);p.add_argument('--method',choices=['local','filtered-lp'],default='local');a=p.parse_args();r=json.load(open(a.source.parent/'pipeline-results.json'));prepare(a.source,a.output,r['dof_names'],a.contact_scale,a.method)
