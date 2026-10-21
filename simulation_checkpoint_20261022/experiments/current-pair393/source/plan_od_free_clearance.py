"""Plan revised OD grasps in Isaac queries. This file supplies no task success."""
from pathlib import Path
import os,sys,json,traceback
import numpy as np
from pxr import Usd,UsdUtils,UsdPhysics
import omni.kit.app,omni.physx,omni.physics.tensors as tensors
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[1])]
from isaac_kinematics import Arm,frame,exp
from isaac_collision_queries import Checker
from isaac_motion import duration
from native_pickup_probe import rotation
from bin_grasp_physx import quaternion
RUNS=HERE.parents[1]/'runs';OUT=RUNS/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
try:
    source=RUNS/'native-roas-table-bin-retimed14';stage=Usd.Stage.Open(str(source/'scene-authored.usda'));paths=json.load(open(source/'body-paths.json'));capture=RUNS/'native-roas-table-bin-under30-capture01';ref=np.load(capture/'full-command-trace.npz');meta=json.load(open(capture/'pipeline-results.json'));t=ref['time_s'];q=ref['command'].copy();controls=ref['clamp'].copy();names=meta['dof_names'];ai={s:[names.index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']};right=ai['right'];drives={p.GetName():UsdPhysics.DriveAPI(p,'angular')for p in stage.Traverse()if p.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in p.GetName()}
    ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);sim.simulate(1/480,0);sim.fetch_results();view=tensors.create_simulation_view('numpy',ident);art=view.create_articulation_view('/World/RobotMountJoint');idx=np.array([0],np.uint32);cmd=q[np.searchsorted(t,6.3):np.searchsorted(t,6.3)+1].copy();cmd[0,-4:]=.04;art.set_dof_positions(cmd,idx);art.set_dof_velocities(np.zeros_like(cmd),idx);art.set_dof_position_targets(cmd,idx);sim.simulate(1/480,1/480);sim.fetch_results();arm=Arm(stage,'right',drives,art.get_dof_positions()[0,right]);left_arm=Arm(stage,'left',drives,art.get_dof_positions()[0,ai['left']]);left_checker=Checker(stage,left_arm,'left');left_overrides={};checker=Checker(stage,arm,'right',allow_loose_fingertips=True);parts={p:view.create_rigid_body_view(paths['workpiece_'+p])for p in ['ring1','ring2']};dt=1/480;plans={};routes=[];clock=.01
    def left_pose(source_time,opening=.08):
        global clock
        x=art.get_dof_positions();x[0,ai['left']]=left_overrides.get(round(source_time,3),ref['command'][np.searchsorted(t,source_time),ai['left']]);x[0,-4:-2]=opening/2;art.set_dof_positions(x,idx);art.set_dof_velocities(np.zeros_like(x),idx);art.set_dof_position_targets(x,idx);sim.simulate(dt,clock);sim.fetch_results();clock+=dt
    def line(A,B,seed,steps=12):
        result=[]
        for u in np.linspace(0,1,steps):
            R=A[:3,:3] if np.linalg.norm(A[:3,:3]-B[:3,:3])<1e-6 else None
            if R is None:
                from isaac_kinematics import log
                R=exp(log(B[:3,:3]@A[:3,:3].T)*u)@A[:3,:3]
            target=A[:3,3]*(1-u)+B[:3,3]*u;sol=arm.solve(target,R,seed)
            if sol is None:raise RuntimeError('Cartesian planning IK failed '+str(target))
            result.append(sol);seed=sol
        return result
    def assign(a,b,path,label):
        path=np.array(path);weights=np.array([max(.05,np.linalg.norm(y-x))for x,y in zip(path[:-1],path[1:])]);knots=np.r_[0,np.cumsum(weights)];knots=knots/knots[-1]*(b-a);mask=np.flatnonzero((t>=a)&(t<=b));elapsed=t[mask]-a;seg=np.clip(np.searchsorted(knots,elapsed,side='right')-1,0,len(path)-2);u=np.clip((elapsed-knots[seg])/(knots[seg+1]-knots[seg]),0,1);h=u**3*(10-15*u+6*u*u);q[np.ix_(mask,right)]=path[seg]+(path[seg+1]-path[seg])*h[:,None];q[np.ix_(np.flatnonzero(t>b),right)]=path[-1];routes.append(dict(label=label,source_start_s=a,source_end_s=b,waypoints=path.tolist()));print('OD_PLANNED',label,len(path),flush=True)
    def route(a,b,opening,part,label,payload=None):
        if payload is not None:checker.attach_payload(paths['workpiece_'+part],frame([0,0,.1034],np.eye(3))@payload)
        found=checker.route(a,b,opening,paths['workpiece_'+part],seconds=12)
        checker.detach_payload()
        if found is None:raise RuntimeError('No route '+label)
        return found
    def width(a,b,w0,w1):
        u=np.clip((t-a)/max(b-a,1e-6),0,1);mask=t>=a;q[mask,-2:]=(w0+(w1-w0)*u[mask,None])/2
    # Both arms start in empty, prepared poses. No part is pre-installed.
    # Recipient pose is prepared below from the free-space search.
    # The original insertion and withdrawal paths are preserved after entry.
    large=json.load(open(RUNS/'native-roas-table-od-handover-probe03/od-handover-probe.json'))['poses']['ring1'];small=json.load(open(RUNS/'native-roas-table-od-handover-probe04/od-handover-probe.json'))['poses']['ring2'];plans={'ring1':large,'ring2':small}
    measured=np.load(capture/'physx-sequence-trace.npz')
    for part,record,when in [('ring1',large,27.85),('ring2',small,52.2)]:
        from isaac_kinematics import log
        k=int(np.argmin(abs(measured['time_s']-when)));j=measured['part_names'].tolist().index(part);value=measured['parts'][k,j];Tbase=frame(value[:3],rotation(value[3:]));grip=np.array(record['grasp_TCP_relative']);pickupR=np.array(record['pickup_TCP'])[:3,:3]
        angles=[record['donor_part_yaw_rad'],record['donor_part_yaw_rad']+np.pi]
        choices=[]
        for angle in angles:
            target=Tbase@frame([0,0,0],exp(np.array([0.,0.,angle])))@np.linalg.inv(grip);choices.append((float(np.linalg.norm(log(target[:3,:3]@pickupR.T))),angle,target))
        _,angle,D=min(choices,key=lambda item:item[0]);record['donor_part_yaw_rad']=angle
        source_time=20.95 if part=='ring1'else 46.49
        seeds=[ref['command'][np.searchsorted(t,source_time),right],np.array(record['pickup_q']),np.array(record['donor_q'])]
        candidates=[]
        for seed in seeds:
            candidate=arm.solve(D[:3,3],D[:3,:3],seed)
            if candidate is not None:candidates.append(candidate)
        if not candidates:
            rng=np.random.default_rng(20)
            for _ in range(16):
                candidate=arm.solve(D[:3,3],D[:3,:3],rng.uniform(arm.bounds[:,0],arm.bounds[:,1]))
                if candidate is not None:candidates.append(candidate)
        if not candidates:raise RuntimeError('No recipient-consistent OD donor IK '+part)
        solution=min(candidates,key=lambda x:np.linalg.norm(x-record['pickup_q']))
        record['donor_TCP']=D.tolist();record['donor_q']=solution.tolist();record['recipient_reference_time_s']=when

    search=json.load(open(RUNS/'native-roas-table-handover-search01/handover-search.json'))['parts'];free={};left_data={}
    right_actors={str(p.GetPath())for p in stage.Traverse()if p.HasAPI(UsdPhysics.RigidBodyAPI)and('right_arm_'in str(p.GetPath())or'right_NativeHand'in str(p.GetPath()))};left_actors={str(p.GetPath())for p in stage.Traverse()if p.HasAPI(UsdPhysics.RigidBodyAPI)and('left_arm_'in str(p.GetPath())or'left_NativeHand'in str(p.GetPath()))}
    for part,record,when,pre_time,entry_time in [('ring1',large,27.85,25.,35.542),('ring2',small,52.2,49.2,63.265)]:
        candidates=[row for row in search[part]['candidates']if min(row['min_velocity_caps_rad_s'].values())>.8];chosen=min(candidates,key=lambda row:row['score']);free[part]=chosen;record['donor_TCP']=chosen['right_TCP'];record['donor_q']=chosen['right_q'];record['handover_offset_m']=chosen['offset_m'];offset=np.array(chosen['offset_m']);contact=np.array(chosen['left_q']);contactH=np.array(chosen['left_TCP']);pre_old=left_arm.fk(ref['command'][np.searchsorted(t,pre_time),ai['left']]);preH=pre_old.copy();preH[:3,3]+=offset;pre=left_arm.solve(preH[:3,3],preH[:3,:3],contact)
        if pre is None:raise RuntimeError('No free-space recipient precontact '+part)
        left_overrides[pre_time]=pre;left_overrides[30.14 if part=='ring1'else 54.7]=contact;left_data[part]=dict(contact=contact,contactH=contactH,pre=pre,preH=preH,entry_time=entry_time)
    def left_line(A,B,seed,steps=10):
        from isaac_kinematics import log
        path=[]
        for u in np.linspace(0,1,steps):
            R=exp(log(B[:3,:3]@A[:3,:3].T)*u)@A[:3,:3];P=A[:3,3]*(1-u)+B[:3,3]*u;sol=left_arm.solve(P,R,seed)
            if sol is None:raise RuntimeError('Left recipient line IK')
            path.append(sol);seed=sol
        return path
    def assign_left(a,b,path,label):
        path=np.array(path);weights=np.array([max(.05,np.linalg.norm(y-x))for x,y in zip(path[:-1],path[1:])]);knots=np.r_[0,np.cumsum(weights)];knots=knots/knots[-1]*(b-a);mask=np.flatnonzero((t>=a)&(t<=b));elapsed=t[mask]-a;seg=np.clip(np.searchsorted(knots,elapsed,side='right')-1,0,len(path)-2);u=np.clip((elapsed-knots[seg])/(knots[seg+1]-knots[seg]),0,1);h=u**3*(10-15*u+6*u*u);q[np.ix_(mask,ai['left'])]=path[seg]+(path[seg+1]-path[seg])*h[:,None];routes.append(dict(label=label,source_start_s=a,source_end_s=b,waypoints=path.tolist()));print('OD_PLANNED',label,len(path),flush=True)
    for part,when,start,entry in [('ring1',27.85,31.944,35.542),('ring2',52.2,54.794,63.265)]:
        row=left_data[part];contact=row['contact'];pre=row['pre'];a,b=(25.,26.492)if part=='ring1'else(49.202,50.696)
        if part=='ring1':
            for j,col in enumerate(ai['left']):q[t<=25.,col]=pre[j]
        else:
            left_checker.ignored_future_actors=right_actors;startq=ref['command'][np.searchsorted(t,46.492),ai['left']];path=left_checker.route(startq,pre,.08,paths['workpiece_'+part],seconds=10)
            if path is None:raise RuntimeError('Free-space recipient approach RRT')
            assign_left(46.492,49.202,path,'small_recipient_precontact')
        assign_left(a,b,left_line(row['preH'],row['contactH'],pre),'recipient_contact_'+part)
        mask=(t>b)&(t<start)
        for j,col in enumerate(ai['left']):q[mask,col]=contact[j]
        k=int(np.argmin(abs(measured['time_s']-when)));pid=measured['part_names'].tolist().index(part);oldp=measured['parts'][k,pid];oldh=measured['links'][k,meta['link_names'].index('left_NativeHand')];Gleft=np.linalg.inv(frame(oldh[:3],rotation(oldh[3:]))@frame([0,0,.1034],np.eye(3)))@frame(oldp[:3],rotation(oldp[3:]));left_checker.ignored_future_actors=right_actors;left_checker.attach_payload(paths['workpiece_'+part],frame([0,0,.1034],np.eye(3))@Gleft);goalq=ref['command'][np.searchsorted(t,entry),ai['left']];path=left_checker.route(contact,goalq,.074895 if part=='ring1'else.061913,paths['workpiece_'+part],seconds=12);left_checker.detach_payload()
        if path is None:raise RuntimeError('Free-space loaded recipient carry '+part)
        assign_left(start,entry,path,'left_loaded_carry_'+part)
    (OUT/'free-handover-choices.json').write_text(json.dumps(free,indent=2))
    # Large ring: whole OD, lifted centrally, then transferred from above.
    left_pose(25.);B=np.array(large['pickup_TCP']);G=np.array(large['grasp_TCP_relative']);D=np.array(large['donor_TCP']);pre=B.copy();pre[:3,3]+=[0,0,.13];lift=B.copy();lift[:3,3]+=[0,0,.15];pick=np.array(large['pickup_q']);preq=arm.solve(pre[:3,3],pre[:3,:3],pick);assert preq is not None;q[:,right]=preq;q[:,-2:]=.04
    assign(6.3,10.3,line(pre,B,preq),'large_OD_descend');assign(11.0833,16.0833,line(B,lift,pick),'large_OD_lift');liftq=q[np.searchsorted(t,16.1),right];donor=np.array(large['donor_q']);assign(17.335,20.6521,route(liftq,donor,.074895,'ring1','large_OD_carry',G),'large_OD_carry');width(10.55,10.6833,.08,.074895-.008);width(27.3083,27.442,.074895-.008,.08)
    up=D.copy();up[:3,3]+=[0,0,.03];assign(27.842,29.842,line(D,up,donor),'large_donor_clear');large_clear=q[np.searchsorted(t,29.85),right].copy()
    # Empty trip to the original small-ring rim. Plan against future placed ring.
    left_pose(30.14,.074895);checker.ignored_future_actors=left_actors|{paths['workpiece_ring1']};rim_pre=ref['command'][np.searchsorted(t,33.889),right];assign(30.142,33.889,route(large_clear,rim_pre,.08,'ring2','small_rim_pregrasp'),'small_rim_pregrasp');width(31.944,32.388,.08,.016)
    rim_pick=ref['command'][np.searchsorted(t,36.9),right];assign(34.188,36.838,[rim_pre,rim_pick],'small_rim_descend');width(37.138,37.271,.016,.0035)
    H=arm.fk(rim_pick);drag=H.copy();drag[:3,3]+=[0,-.08,0];dragpath=line(H,drag,rim_pick);assign(37.671,40.304,dragpath,'small_planar_drag');dragq=dragpath[-1];width(40.304,40.604,.0035,.024)
    clear=drag.copy();clear[:3,3]+=[0,0,.035];assign(40.704,41.604,line(drag,clear,dragq),'small_drag_clear');clearq=q[np.searchsorted(t,41.61),right].copy();width(41.604,42.6,.024,.08)
    B=np.array(small['pickup_TCP']);G=np.array(small['grasp_TCP_relative']);D=np.array(small['donor_TCP']);above=B.copy();above[:3,3]+=[0,0,.035];aboveq=arm.solve(above[:3,3],above[:3,:3],clearq);assert aboveq is not None;assign(41.604,42.6,[clearq,aboveq],'small_OD_approach');assign(42.6,43.404,line(above,B,aboveq),'small_OD_descend');width(43.404,43.704,.08,.061913-.008)
    lift=B.copy();lift[:3,3]+=[0,0,.15];pick=np.array(small['pickup_q']);assign(44.004,45.604,line(B,lift,pick),'small_OD_lift');liftq=q[np.searchsorted(t,45.61),right].copy();left_pose(49.2);checker.ignored_future_actors={paths['workpiece_ring1']};donor=np.array(small['donor_q']);assign(45.604,46.482,route(liftq,donor,.061913,'ring2','small_OD_carry',G),'small_OD_carry');width(51.629,51.762,.061913-.008,.08)
    up=D.copy();up[:3,3]+=[0,0,.03];assign(52.162,53.2646,line(D,up,donor),'small_donor_clear');clearq=q[np.searchsorted(t,53.27),right].copy();left_pose(54.7,.061913);checker.ignored_future_actors=left_actors|{paths['workpiece_ring1'],paths['workpiece_ring2']};pinpre=ref['command'][np.searchsorted(t,58.089),right];assign(53.2646,58.089,route(clearq,pinpre,.022,'ring2','pin_pregrasp'),'pin_pregrasp');width(54.794,55.242,.08,.022)
    mask=t>=58.089;q[np.ix_(np.flatnonzero(mask),right)]=ref['command'][mask][:,right];q[mask,-2:]=ref['command'][mask,-2:]
    phases=[]
    for raw in ref['phase']:
        pair=str(raw).split(' || ');phases.append([pair[0],pair[-1]])
    for a,b,label in [(37.271,37.671,'ring2_drag_grasp_hold'),(37.671,40.304,'ring2_planar_drag'),(40.304,40.604,'ring2_drag_release'),(40.604,40.704,'ring2_giver_release_hold'),(40.704,41.604,'ring2_drag_clear'),(41.604,42.6,'ring2_OD_preshape'),(42.6,43.404,'ring2_OD_descend'),(43.404,43.704,'ring2_OD_close'),(43.704,44.004,'ring2_OD_grasp_hold'),(44.004,45.604,'ring2_OD_lift'),(45.604,46.482,'ring2_OD_carry')]:
        for j in np.flatnonzero((t>=a)&(t<b)):phases[j][1]=label
    output=HERE/'bin-picking/od-pipeline-source.npz';np.savez_compressed(output,time_s=t,command=q,clamp=controls,phase=[' || '.join(p)for p in phases]);(OUT/'od-plan.json').write_text(json.dumps(dict(full_task_pass=False,planning_only=True,poses=plans,routes=routes,source_file=str(output)),indent=2));print('OD_PIPELINE_SOURCE',output,flush=True);sim.detach_stage();omni.kit.app.get_app().post_quit(0)
except Exception:
    (OUT/'error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
