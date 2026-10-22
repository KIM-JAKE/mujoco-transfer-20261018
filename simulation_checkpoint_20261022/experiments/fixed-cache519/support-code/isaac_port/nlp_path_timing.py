"""Experimental scalar jerk-control NLP with dense independent rescreening.

Preserves q(s). Imported-model torque rate excludes contacts/payload/friction.
No clock produced here is a physical assembly or hardware success claim.
"""
from pathlib import Path
from types import SimpleNamespace
import argparse,copy,json,shutil,time
import numpy as np
import casadi as ca
from scipy.interpolate import CubicSpline,PPoly,make_interp_spline
import prepare_fr3_timing as timing
from model_torque_screen import ModelTorqueScreen
from fr3_motion_control import LOWER,UPPER,VMAX
from optimization_clock import require_optimization_window,install_cutoff_alarm


def projected_clock(begin,end,duration,jerk):
    jerk=np.asarray(jerk,float).copy();nodes=len(jerk);ticks=np.linspace(0,duration,nodes+1);step=duration/nodes
    remaining=duration-ticks
    endpoint=np.vstack([np.full(nodes,step),(remaining[:-1]**2-remaining[1:]**2)/2,(remaining[:-1]**3-remaining[1:]**3)/6])
    correction=endpoint.T@np.linalg.solve(endpoint@endpoint.T,np.array([0.,0.,end-begin])-endpoint@jerk);jerk+=correction
    position,velocity,acceleration=begin,0.,0.;columns=[]
    for j in jerk:
        columns.append([j/6,acceleration/2,velocity,position]);position+=velocity*step+.5*acceleration*step**2+j*step**3/6;velocity+=acceleration*step+.5*j*step**2;acceleration+=j*step
    return PPoly(np.asarray(columns).T,ticks),correction


def optimize(profile,old,scale,begin,end,schedule,rate_limit,nodes=50):
    started=time.monotonic();screen=profile.model_torque_screen
    grid=np.linspace(begin,end,max(301,int((end-begin)/.0015)+1))
    def lut(name,values):return ca.interpolant(name,'bspline',[grid.tolist()],np.asarray(values).ravel())
    f={name:lut(name,values)for name,values in dict(q=profile.path(grid),qs=profile.path(grid,1),qss=profile.path(grid,2),qsss=profile.path(grid,3),A=screen.a(grid),B=screen.b(grid),Ap=screen.a(grid,1),Bp=screen.b(grid,1),grip=profile.full_path(grid,1)[:,profile.driver_indices]).items()}
    grip_cap=ca.interpolant('grip_cap','linear',[grid.tolist()],timing.driver_caps(grid,schedule).ravel())
    phase_cap=ca.interpolant('phase_cap','linear',[grid.tolist()],timing.fine_rate_cap(grid,schedule,True))
    opt=ca.Opti();s=opt.variable(nodes+1);v=opt.variable(nodes+1);a=opt.variable(nodes+1);u=opt.variable(nodes);total=opt.variable();h=total/nodes
    old_total=float(old.x[-1]*scale);minimum=max(.005,timing.contact_minimum(begin,schedule))
    opt.subject_to(opt.bounded(minimum,total,max(old_total*1.5,minimum*1.1)))
    opt.subject_to(opt.bounded(begin,s,end));opt.subject_to(opt.bounded(0.,v,timing.MAX_PHASE_RATE));opt.subject_to(opt.bounded(-20000.,u,20000.))
    opt.subject_to(s[0]==begin);opt.subject_to(s[-1]==end)
    for z in [v,a]:opt.subject_to(z[0]==0);opt.subject_to(z[-1]==0)
    lower=ca.DM(np.tile(LOWER,2));upper=ca.DM(np.tile(UPPER,2));vmax=ca.DM(np.tile(VMAX,2))
    offset=ca.DM(np.tile([.6599,.2517,.2,.3533,.5757,.4878,.4628],2));decel=ca.DM(np.tile([6.,2.585,3.5,4.,17.,5.5,17.],2))
    seen=set()
    def constrain(i,fraction):
        key=(i,round(float(fraction),5))
        if key in seen:return
        seen.add(key)
        tick=h*fraction;ss=s[i]+v[i]*tick+.5*a[i]*tick*tick+u[i]*tick**3/6;vv=v[i]+a[i]*tick+.5*u[i]*tick*tick;aa=a[i]+u[i]*tick
        q,qs,qss,qsss=f['q'](ss),f['qs'](ss),f['qss'](ss),f['qsss'](ss)
        qv=qs*vv;qa=qss*vv**2+qs*aa;qj=qsss*vv**3+3*qss*vv*aa+qs*u[i]
        # Same 15mrad position reserve / 0.025rad/s margin as existing planner.
        hi=ca.fmin(vmax,ca.fmax(0.,-offset+ca.sqrt(ca.fmax(1e-12,2*decel*(upper-q-.015)))))
        lo=ca.fmax(-vmax,ca.fmin(0.,offset-ca.sqrt(ca.fmax(1e-12,2*decel*(q-.015-lower)))))
        velocity_cap=ca.fmax(.001,.95*ca.if_else(qv>=0,hi,-lo)-.025)
        opt.subject_to(opt.bounded(-1.,qv/velocity_cap,1.))
        opt.subject_to(opt.bounded(-1.,qa/9.1,1.));opt.subject_to(opt.bounded(-1.,qj/4700.,1.))
        tr=f['A'](ss)*u[i]+(f['Ap'](ss)+2*f['B'](ss))*vv*aa+f['Bp'](ss)*vv**3
        opt.subject_to(opt.bounded(-1.,tr/rate_limit,1.))
        opt.subject_to(opt.bounded(-1.,f['grip'](ss)*vv/grip_cap(ss),1.))
        opt.subject_to(vv<=phase_cap(ss))

    for i in range(nodes):
        opt.subject_to(s[i+1]==s[i]+v[i]*h+.5*a[i]*h*h+u[i]*h**3/6)
        opt.subject_to(v[i+1]==v[i]+a[i]*h+.5*u[i]*h*h)
        opt.subject_to(a[i+1]==a[i]+u[i]*h)
        # Bernstein middle control point certifies nonnegative quadratic speed.
        opt.subject_to(v[i]+a[i]*h/2>=0)
        for fraction in [0.,.5,1.]:constrain(i,fraction)
    guess=np.linspace(0,old_total,nodes+1);raw=guess/scale
    opt.set_initial(total,old_total);opt.set_initial(s,np.clip(old(raw),begin,end));opt.set_initial(v,np.maximum(0,old(raw,1)/scale));opt.set_initial(a,old(raw,2)/scale**2);opt.set_initial(u,old((raw[:-1]+raw[1:])/2,3)/scale**3)
    opt.minimize(total+1e-11*ca.sumsqr(u))
    opt.solver('ipopt',{'expand':False,'print_time':False},{'print_level':0,'max_iter':1800,'tol':1e-8,'acceptable_tol':1e-6,'max_cpu_time':90.,'hessian_approximation':'limited-memory'})
    refinement_stopped=None
    for iteration in range(5):
        try:solution=opt.solve()
        except RuntimeError as exc:
            if iteration==0:raise
            refinement_stopped=str(exc).splitlines()[-1]
            break
        duration=float(solution.value(total))
        clock,correction=projected_clock(begin,end,duration,np.asarray(solution.value(u)).reshape(-1))
        check=np.unique(np.r_[clock.x,np.linspace(0,duration,max(16001,int(duration*6000)))])
        if float(clock(check,1).min())< -1e-7:raise RuntimeError('NLP clock is not monotone')
        ratios,acc,j=timing.planning_ratios(profile,clock,check,begin,end,schedule)
        print('NLP_REFINEMENT',iteration,'duration',duration,'dense_ratio',float(ratios.max()),'points',len(seen),flush=True)
        if ratios.max()<=1.02 or iteration==4:break
        chosen={}
        for k in np.argsort(ratios)[::-1]:
            if ratios[k]<=1.02 or len(chosen)>=24:break
            position=float(check[k]/duration*nodes);i=min(nodes-1,int(position))
            if i not in chosen:chosen[i]=position-i
        before=len(seen)
        for i,fraction in chosen.items():
            for extra in [-.04,0.,.04]:constrain(i,float(np.clip(fraction+extra,0,1)))
        if len(seen)==before:break
        opt.set_initial(opt.x,solution.value(opt.x))
    factor=max(1.,float(ratios.max()))*1.015
    factor=max(factor,minimum/duration)
    return clock,factor,dict(reference_begin_s=float(begin),reference_end_s=float(end),duration_s=duration*factor,raw_duration_s=duration,time_scale=factor,minimum_phase_rate=float(clock(check,1).min()/factor),peak_acceleration_rad_s2=float(abs(acc).max()/factor**2),peak_jerk_rad_s3=float(abs(j).max()/factor**3),optimizer='scalar jerk-controlled NLP, adaptive collocation, dense original-path/model rescreen',nodes=nodes,collocation_points=len(seen),refinement_iterations=iteration+1,refinement_stopped=refinement_stopped,rate_limit_nm_s=rate_limit,original_duration_s=old_total,endpoint_projection_peak_phase_jerk=float(abs(correction).max()),solver_status=solution.stats()['return_status'],planning_seconds=time.monotonic()-started)


def refine(profile,old,scale,begin,end,schedule,rate_limit):
    candidates=[]
    for sigma in [.008,.02,.04]:
        candidate,_,_=timing.local_time_allocation(profile,old,begin,end,schedule,sigma)
        x=np.unique(candidate.x);s=candidate(x)
        clock=PPoly.from_spline(make_interp_spline(x,s,k=5,bc_type=([(1,0.),(2,0.)],[(1,0.),(2,0.)])))
        check=np.unique(np.r_[x,np.linspace(0,x[-1],max(16001,int(x[-1]*6000)))])
        if float(clock(check,1).min())< -1e-7:continue
        ratios,acc,j=timing.planning_ratios(profile,clock,check,begin,end,schedule)
        factor=max(1.,float(ratios.max()))*1.015
        report=dict(reference_begin_s=float(begin),reference_end_s=float(end),duration_s=float(x[-1]*factor),raw_duration_s=float(x[-1]),time_scale=factor,minimum_phase_rate=float(clock(check,1).min()/factor),peak_acceleration_rad_s2=float(abs(acc).max()/factor**2),peak_jerk_rad_s3=float(abs(j).max()/factor**3),optimizer='NLP seed with local time allocation and C2 quintic boundary fit',rate_limit_nm_s=rate_limit,original_duration_s=float(old.x[-1]*scale),smoothing_s=sigma)
        candidates.append((clock,factor,report))
    if not candidates:raise RuntimeError('No monotone refined clock')
    return min(candidates,key=lambda row:row[2]['duration_s'])


def prepare(base,plan_root,output,side,segment,rate_limit,nodes,respect_cutoff=False,refine_only=False):
    if output.parent.resolve()==plan_root.resolve():raise ValueError('Use a fresh candidate directory')
    if respect_cutoff:require_optimization_window(360);install_cutoff_alarm()
    root=Path(__file__).parent;names=json.loads((base/'dof-names.json').read_text());source=json.loads((root/'generated/contact_task/contact-source.json').read_text());lookup={j['name']:j['qpos_address']for j in source['joints']}
    z=np.load(base/'recording/trajectory.npz');plan=np.load(plan_root/'arm-posture-fr3.npz');q=z['qpos'][:,[lookup[n]for n in names]].copy()
    indices=[names.index(f'{s}_arm_fr3v2_joint{j}')for s in ['left','right']for j in range(1,8)]
    for i,n in enumerate(names):
        if '_arm_'in n:q[:,i]=plan[n.split('_')[0]+'_qpos'][:,int(n[-1])-1]
        if n.startswith(('right_'if side=='left'else'left_')):q[:,i]=q[0,i]
    schedule=json.loads((base/'recording/independent_schedule.json').read_text());metadata=json.loads((plan_root/'fr3-concurrent-profile.json').read_text())
    schedule=copy.deepcopy(schedule)
    for row in schedule['lanes']['right'if side=='left'else'left']:
        row['start_s']+=1000;row['end_s']+=1000
    timing.CONTACT_MINIMUMS.update(metadata['contact_minimum_s']);timing.MAX_PHASE_RATE=metadata['maximum_source_phase_rate'];timing.GRIPPER_TRAVEL_CAP=metadata['gripper_travel_driver_cap_rad_s'];timing.PIN_EARLY_CLOSE_CAP=metadata['pin_early_close_driver_cap_rad_s']
    with np.load(plan_root/'fr3-concurrent-profile.npz')as archive:data={n:archive[n].copy()for n in archive.files}
    begin,end=data[side+'_bounds'][segment:segment+2];prefix=f'{side}_clock_{segment}_';old=PPoly(data[prefix+'c'],data[prefix+'x']);scale=float(data[prefix+'scale'])
    profile=SimpleNamespace(path=CubicSpline(z['time_s'],q[:,indices],axis=0),full_path=CubicSpline(z['time_s'],q,axis=0),driver_indices=[names.index(s+'_grip_left_driver_joint')for s in ['left','right']])
    profile.model_torque_screen=ModelTorqueScreen(profile,names,begin,end,rate_limit=rate_limit)
    clock,factor,report=refine(profile,old,scale,begin,end,schedule,rate_limit)if refine_only else optimize(profile,old,scale,begin,end,schedule,rate_limit,nodes)
    data[prefix+'c']=clock.c;data[prefix+'x']=clock.x;data[prefix+'scale']=np.array(factor);data[side+'_durations'][segment]=report['duration_s']
    output.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(output,**data)
    metadata['segments'][side][segment]=report;metadata['side_nominal_s']={s:float(data[s+'_durations'].sum())for s in ['left','right']};metadata['state']='NLP_retimed_requires_new_PhysX_validation';metadata['nlp_updates']=metadata.get('nlp_updates',[])+[dict(side=side,segment=segment,casadi_version=ca.__version__,**report)];metadata['hardware_ready']=False
    previous_limit=metadata.get('model_torque_rate_planning_limit_nm_s')
    metadata['model_torque_rate_planning_limit_nm_s']=max(previous_limit,rate_limit)if previous_limit is not None else None
    output.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n')
    for n in ['arm-posture-fr3.npz','arm-posture-fr3.json']:shutil.copy2(plan_root/n,output.parent/n)
    for n in ['trajectory.npz','config.json','independent_schedule.json','retiming.json']:shutil.copy2(base/'recording'/n,output.parent/n)
    shutil.copytree(base,output.parent/'planning-base',dirs_exist_ok=True)
    print(json.dumps(dict(side=side,segment=segment,report=report,side_nominal_s=metadata['side_nominal_s']),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,required=True);p.add_argument('--plan-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--side',choices=['left','right'],required=True);p.add_argument('--segment',type=int,required=True);p.add_argument('--rate-limit',type=float,default=700.);p.add_argument('--nodes',type=int,default=50);p.add_argument('--respect-cutoff',action='store_true');p.add_argument('--refine-only',action='store_true');a=p.parse_args();prepare(a.base,a.plan_root,a.output,a.side,a.segment,a.rate_limit,a.nodes,a.respect_cutoff,a.refine_only)
