"""Compile a taught robot skill and image/encoder grasp references.

Only robot commands, recorded encoders and camera estimates are read. No
recorded workpiece transforms are read or accepted by the runtime recipe.
Compilation and fixed camera BVH setup happen before the first capture.
"""
import json,numpy as np
from isaac_kinematics import frame,log,exp
from rgbd_geometry import axis_frame

def fk_tables(arm,rows):
    inverse=[];rotation=[]
    for q in rows:
        T=arm.base.copy();origins=[];axes=[]
        for x,axis,(a,b)in zip(q,arm.axes,arm.frames):
            joint=T@a;origins.append(joint[:3,3].copy());axes.append(joint[:3,:3]@axis);T=joint@frame([0,0,0],exp(axis*x))@b
        T=T@arm.tip;A=np.array(axes);J=np.vstack([np.cross(A,T[:3,3]-np.array(origins)).T,A.T]);inverse.append(J.T@np.linalg.inv(J@J.T+np.eye(6)*1e-7));rotation.append(T[:3,:3])
    return np.array(inverse),np.array(rotation)

def compile_recipe(here,arm,indices,dt=1/240,left_arm=None,left_indices=None):
    root=here.parents[1]/'runs/native-roas-table-rgbd-control05';z=np.load(root/'executed-actuator-commands.npz');t=z['time_s'];keep=np.ones(len(t),bool)
    cuts=[(4.61,4.895),(15.20,15.375),(24.49,24.67),(25.355,25.44),(28.825,28.895)]
    for a,b in cuts:keep&=~((t>=a)&(t<b))
    stride=int(round(dt*480));assert abs(stride/480-dt)<1e-10
    kept=t[keep];clock=np.arange(len(kept))/480
    # Local time dilation gives image-driven floor corrections headroom while
    # retaining the original 95% position-dependent velocity cap.
    smooth=lambda x:np.clip(x,0,1)**3*(10-15*np.clip(x,0,1)+6*np.clip(x,0,1)**2)
    dilation=1+.06*smooth(kept-6)*(1-smooth(kept-14.2))+.015*smooth((kept-7)/.5)*(1-smooth((kept-9.5)/.5))
    from scipy.ndimage import maximum_filter1d,gaussian_filter1d
    from fr3_motion_control import position_velocity_bounds
    robot_indices=left_indices+indices;native_indices=[i for i in range(z['command'].shape[1])if i not in robot_indices];raw=z['command'][keep].astype(float);qa=raw[:,robot_indices];v=np.gradient(qa,1/480,axis=0);a=np.gradient(v,1/480,axis=0);j=np.gradient(a,1/480,axis=0);lo,hi=position_velocity_bounds(qa.reshape(-1,2,7));ratio=np.max(abs(v.reshape(-1,2,7))/np.maximum(.95*np.where(v.reshape(-1,2,7)>=0,hi,-lo),1e-8),axis=(1,2));stress=np.maximum.reduce([ratio,np.sqrt(np.max(abs(a),axis=1)/9.5),np.cbrt(np.max(abs(j),axis=1)/4750)]);fv=np.gradient(raw[:,native_indices],1/480,axis=0);stress=np.maximum(stress,np.max(abs(fv),axis=1)/.0475);guard=gaussian_filter1d(maximum_filter1d(np.max(abs(fv),axis=1)/.04735,size=193),24);shrink=gaussian_filter1d(np.clip(.99-maximum_filter1d(stress,size=97),0,.5),48);base_dilation=dilation.copy();blendmask=np.zeros(len(kept));retiming_windows=[]
    # Preserve pickup/contact timing; shorten only travel and withdrawal.
    # Quantized savings preserve the original physics/actuator sample grid.
    for start,end,strength in [(5.5,7.3,.7),(16.2,18.8,.8),(21.3,23.6,.8),(25.4,28.3,.6)]:
        w=smooth((kept-start)/.2)*(1-smooth((kept-(end-.2))/.2));delta=w*(np.maximum(base_dilation-strength*shrink,guard)-base_dilation);gain=-float(delta.sum())/480;saved=float(np.floor(max(gain,0)*60)/60);fraction=saved/gain if gain>0 else 0.;dilation+=delta*fraction;blendmask=np.maximum(blendmask,w);retiming_windows.append(dict(start_source_s=start,end_source_s=end,saved_s=saved))
    import os,json
    cfgpath=here.parents[1]/'runs'/os.environ.get('M80_ROAS_RUN','none')/'trial-config.json';cfg=json.loads(cfgpath.read_text())if cfgpath.exists()else{}
    warped=np.cumsum(dilation)/480;warped-=warped[0]
    phase=np.arange(0,warped[-1]+dt/2,dt);source_index=np.interp(phase,warped,np.arange(len(kept)));old=np.interp(source_index,np.arange(len(kept)),kept)
    from scipy.interpolate import CubicSpline
    q=CubicSpline(warped,z['command'][keep].astype(float))(phase);pm=np.interp(phase,warped,blendmask);linear=np.stack([np.interp(phase,warped,raw[:,i])for i in native_indices],axis=1);q[:,native_indices]=q[:,native_indices]*(1-pm[:,None])+linear*pm[:,None];controls=np.stack([np.interp(phase,warped,z['clamp'][keep,j])for j in range(3)],axis=1);rc=np.interp(phase,warped,z['right_source_time_s'][keep]);lc=np.interp(phase,warped,z['source_time_s'][keep])
    def at_old(value):return float(np.interp(value,old,phase))
    # Precompute differential IK; only small observation-driven corrections
    # are applied at runtime. The stored path is always a robot motion skill.
    import os,json
    cfgpath=here.parents[1]/'runs'/os.environ.get('M80_ROAS_RUN','none')/'trial-config.json';cfg=json.loads(cfgpath.read_text())if cfgpath.exists()else{};joint_weight=np.diag([1,1,1,1,1,1,float(cfg.get('correction_joint7_weight',1.))])
    if cfg.get('ring_seat_hold_s'):
        hold=float(cfg['ring_seat_hold_s']);lclock=phase.copy();rclock=phase.copy();nclock=phase.copy()
        for center in [at_old(8.315),at_old(19.80)]:
            lclock+=hold*smooth((phase-(center-.015))/.030);rclock+=hold*smooth((phase-(center-.6))/1.2);nclock+=hold*smooth((phase-(center+.6))/.2)
        original_phase=phase.copy();phase=np.arange(0,max(lclock[-1],rclock[-1])+dt/2,dt);li=np.minimum(phase,lclock[-1]);ri=np.minimum(phase,rclock[-1])
        # DOFs are interleaved arm joints, then left fingers, then right fingers.
        left_native=native_indices[:2];right_native=native_indices[2:];newq=np.empty((len(phase),q.shape[1]));newq[:,left_indices]=CubicSpline(lclock,q[:,left_indices])(li);newq[:,left_native]=CubicSpline(nclock,q[:,left_native])(np.minimum(phase,nclock[-1]));newq[:,indices+right_native]=CubicSpline(rclock,q[:,indices+right_native])(ri);q=newq
        controls=np.column_stack([np.interp(li,lclock,controls[:,0]),np.interp(li,lclock,controls[:,1]),np.interp(ri,rclock,controls[:,2])]);lc=np.interp(li,lclock,lc);rc=np.interp(ri,rclock,rc);old=np.interp(ri,rclock,old)
    scale=float(cfg.get('motion_time_scale',1.))
    if scale!=1.:
        old_phase=phase.copy();phase=np.arange(0,old_phase[-1]*scale+dt/2,dt);mapped=np.minimum(phase/scale,old_phase[-1]);q=CubicSpline(old_phase,q)(mapped);controls=np.stack([np.interp(mapped,old_phase,controls[:,j])for j in range(3)],axis=1);rc=np.interp(mapped,old_phase,rc);lc=np.interp(mapped,old_phase,lc);old=np.interp(mapped,old_phase,old)
    lower=float(cfg.get('left_grasp_lower_m',0.))
    if lower:
        lji,lr=fk_tables(left_arm,q[:,left_indices]);weight=smooth((phase-.1)/1.)*(1-smooth((phase-at_old(20.))/1.));translation=np.einsum('nij,j->ni',lr,np.array([0.,0.,-lower]));twist=np.c_[translation,np.zeros_like(translation)];q[:,left_indices]+=np.einsum('nij,nj->ni',lji,twist)*weight[:,None]
    small_raise=float(cfg.get('left_small_grasp_raise_m',0.))
    if small_raise:
        lji,lr=fk_tables(left_arm,q[:,left_indices]);weight=smooth((phase-at_old(13.0))/.7)*(1-smooth((phase-at_old(21.0))/.7));translation=np.einsum('nij,j->ni',lr,np.array([0.,0.,small_raise]));q[:,left_indices]+=np.einsum('nij,nj->ni',lji,np.c_[translation,np.zeros_like(translation)])*weight[:,None]
    receiver_clearance=float(cfg.get('small_handoff_receiver_retract_m',0.))
    if receiver_clearance:
        lji,lr=fk_tables(left_arm,q[:,left_indices]);weight=smooth((phase-at_old(12.0))/1.0)*(1-smooth((phase-at_old(14.35))/.5));translation=np.einsum('nij,j->ni',lr,np.array([0.,0.,-receiver_clearance]));q[:,left_indices]+=np.einsum('nij,nj->ni',lji,np.c_[translation,np.zeros_like(translation)])*weight[:,None]
    Jinv=[];held_Jinv=[];rot=[]
    for row in q:
        T=arm.base.copy();origins=[];axes=[]
        for x,axis,(a,b)in zip(row[indices],arm.axes,arm.frames):
            joint=T@a;origins.append(joint[:3,3].copy());axes.append(joint[:3,:3]@axis);T=joint@frame([0,0,0],exp(axis*x))@b
        T=T@arm.tip;A=np.array(axes);J=np.vstack([np.cross(A,T[:3,3]-np.array(origins)).T,A.T]);Jinv.append(J.T@np.linalg.inv(J@J.T+np.eye(6)*1e-7));held_Jinv.append(joint_weight@J.T@np.linalg.inv(J@joint_weight@J.T+np.eye(6)*1e-7));rot.append(T[:3,:3])
    # This archive also has validation bodies, which are deliberately NOT read.
    with np.load(root/'physx-sequence-trace.npz')as enc:tt=enc['time_s'];encoder_q=enc['q']
    observations=json.load(open(root/'rgb-observations.json'));events=[]
    for row in observations:
        source_time=float(row['scene_sim_time_s']);event=dict(mode=row['mode'],part=row['part'],deadline=at_old(source_time),request_at=max(0,at_old(source_time)-.30),reference=row['result'],source_time_s=source_time)
        if row['mode']=='held':
            qenc=np.array([np.interp(source_time,tt,encoder_q[:,j])for j in indices]);H=arm.fk(qenc);p=row['result']['pose'];event['reference_grasp']=np.linalg.inv(H)@frame(p['center_m'],axis_frame(p['axis']))
        else:event['reference_grasp']=None
        if event['mode']=='floor'and source_time<9:event['request_at']=at_old(4.9);event['deadline']=at_old(5.25)
        elif event['mode']=='floor':event['request_at']=event['deadline']-.30
        if event['part']=='pin'and event['mode']=='held':event['request_at']=event['deadline'];event['deadline']+=.35
        if event['mode']=='held' and event['part']!='pin':
            advance=float(cfg.get('held_observation_advance_s_by_part',{}).get(event['part'],0.));event['request_at']=max(0,event['request_at']-advance);event['deadline']-=advance
        events.append(event)
    left_Jinv,left_rot=fk_tables(left_arm,q[:,left_indices]);goals=json.load(open(root/'arm-plan.json'))['goals']
    for part,at,release in [('ring1',5.6,39.99),('ring2',16.7,67.7)]:
        goal_index=min(len(q)-1,int(np.searchsorted(lc,release-.15)));H=left_arm.fk(q[goal_index,left_indices]);G=np.linalg.inv(H)@frame(goals[part]['P'],np.eye(3))
        events.append(dict(mode='held_left',part=part,request_at=at_old(at),deadline=at_old(at)+.4,reference={},source_time_s=at,reference_grasp=G))
    recipe=dict(left_Jinv=left_Jinv,left_rot=left_rot,command=q,base=q.copy(),controls=controls,phase=phase,source=lc,right_source=rc,Jinv=np.array(Jinv),held_Jinv=np.array(held_Jinv),rot=np.array(rot),events=events,at_old=at_old,dt=dt,reference_image_initial=observations[0]['result'],reference_pin_initial=observations[1]['result']['pose'],trimmed_stationary_s=float((len(t)-keep.sum())/480),joint_headroom_retiming_saved_s=sum(row['saved_s']for row in retiming_windows),retimed_free_motion_windows=retiming_windows,source_run=root.name)
    return recipe
