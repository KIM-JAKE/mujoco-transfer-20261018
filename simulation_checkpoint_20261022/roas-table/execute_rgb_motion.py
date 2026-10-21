"""Fresh PhysX execution of retimed actuator commands; objects stay dynamic."""
import json,traceback
import numpy as np
import omni.physx
import omni.physics.tensors as tensors
from pxr import UsdUtils,UsdPhysics,PhysicsSchemaTools,PhysxSchema
from isaac_motion import Audit

def execute(stage,paths,hands,drives,initial,report,out,here):
    options=json.load(open(out/'pipeline-options.json'if(out/'pipeline-options.json').exists()else here/'bin-picking/pipeline-options.json'))
    motion=out/'motion-commands.npz'if(out/'motion-commands.npz').exists()else here/options['motion_file'];z=np.load(motion)
    commands=z['command'].copy();original_commands=commands.copy();controls=z['clamp'];source_times=z['source_time_s'];right_times=z['right_source_time_s']if 'right_source_time_s'in z else source_times;dt=1/480
    if options.get('native_finger_mimic'):
        for side in ['left','right']:
            master=stage.GetPrimAtPath('/World/NativeJoints/'+side+'_native_left');follower=stage.GetPrimAtPath('/World/NativeJoints/'+side+'_native_right')
            assert master and follower
            gear=PhysxSchema.PhysxMimicJointAPI.Apply(follower,'rotX');gear.CreateReferenceJointRel().SetTargets([master.GetPath()]);gear.CreateReferenceJointAxisAttr('rotX');gear.CreateGearingAttr(-1.);gear.CreateOffsetAttr(0.);gear.CreateNaturalFrequencyAttr(float(options.get('mimic_natural_frequency',2000.)));gear.CreateDampingRatioAttr(1.2)
        report['native_parallel_finger_mimic']=dict(enabled=True,equation='q_left - q_right = 0',natural_frequency=float(options.get('mimic_natural_frequency',2000.)),damping_ratio=1.2,workpiece_constraints=False)
    ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
    sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident)
    sim.simulate(dt,0);sim.fetch_results()
    view=tensors.create_simulation_view('numpy',ident);art=view.create_articulation_view('/World/RobotMountJoint')
    names=list(art.shared_metatype.dof_names);links=list(art.shared_metatype.link_names);index=np.array([0],np.uint32)
    aidx={s:[names.index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']}
    fi=[i for i,n in enumerate(names)if '_native_'in n]
    cmd=commands[:1].copy();art.set_dof_positions(cmd,index);art.set_dof_velocities(np.zeros_like(cmd),index)
    finger_stiffness=float(options.get('finger_stiffness_n_m',5000.))
    stiffness=np.array([[12000 if n in drives else finger_stiffness for n in names]],np.float32);damping=np.array([[600 if n in drives else 100 for n in names]],np.float32)
    art.set_dof_stiffnesses(stiffness,index);art.set_dof_dampings(damping,index)
    maxforces=np.array([[(87 if int(n[-1])<=4 else 12)*.95 if n in drives else 30 for n in names]],np.float32);art.set_dof_max_forces(maxforces,index)
    truth_parts={p:view.create_rigid_body_view(paths['workpiece_'+p])for p in ['ring1','ring2','pin']}
    joints={p:[UsdPhysics.PrismaticJoint(pr)for pr in stage.Traverse()if str(pr.GetPath()).startswith('/World/Clamps/'+p+'/')and pr.IsA(UsdPhysics.PrismaticJoint)]for p in truth_parts}
    tape=[];contacts=[];peak=0.;t=0.;audit=Audit(cmd,aidx,dt);finger_peak=0.;last=np.full(3,-1.)
    def callback(headers,data):
        nonlocal peak
        for h in headers:
            actors=[str(PhysicsSchemaTools.intToSdfPath(x))for x in [h.actor0,h.actor1]]
            if not(any('left_arm_'in a or 'left_NativeHand'in a for a in actors)and any('right_arm_'in a or 'right_NativeHand'in a for a in actors)):continue
            force=sum(float(np.linalg.norm(data[i].impulse))/dt for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data));peak=max(peak,force)
            if force>2:contacts.append(dict(time_s=t,actors=actors,force_n=force))
    subscription=sim.subscribe_contact_report_events(callback)
    report.update(state='running_retimed_commands',dof_names=names,link_names=links,physics_hz=480,original_finger_mesh=True,motion_file=str(motion),perception='SAM3.1 RGB masks + aligned depth + CAD; no body poses for control',timing=dict(timing_is_measured_success=False,planned_s=float(z['time_s'][-1])),gripper_model=dict(max_force_per_finger_n=30.,stiffness_n_m=finger_stiffness,force_control_approximation=True,hardware_calibrated=False))
    report['gripper_model']['mode']='force_limited_closing_velocity_servo'if options.get('native_force_grasp')else'position_servo'
    if options.get('effort_grasp_after_lift'):report['gripper_model'].update(mode='effort_preload_after_liftoff',commanded_hold_force_per_finger_n=float(options.get('effort_force_per_finger_n',25.)),combined_drive_and_effort_cap_per_finger_n=30.,activation_lift_m=.004)
    extra=0.;aligned=set();pending_retreat={};skip_until=0
    norm_points=dict(zip(z['normalization_parts'].tolist(),z['normalization_source_time_s']))if 'normalization_parts'in z else{}
    force_state={'left':False,'right':False};active_clock=[float(source_times[0]),float(right_times[0])]
    assert not options.get('effort_grasp_after_lift'), 'RGB prototype does not support oracle lift-triggered effort mode'
    effort_blend=0.;effort_part=None;initial_part_z={}
    from isaac_kinematics import Arm
    from rgbd_observer import RGBDObserver
    # Measured encoder FK is permitted; workpiece transforms are validation only.
    sim.simulate(dt,0);sim.fetch_results()
    rgb_arm=Arm(stage,'right',drives,art.get_dof_positions()[0,aidx['right']])
    vision=RGBDObserver(stage,out,here,lambda:rgb_arm.fk(art.get_dof_positions()[0,aidx['right']]),sim_clock=lambda:t)
    observed_initial=vision.initialize()
    report['size_classification']=[observed_initial[p]for p in ['ring1','ring2']]
    report['RGBD_camera_control']=True;report['depth_input']=True
    report['perception_body_pose_reads']=0
    from rgb_pickup_adaptation import adapt
    commands=adapt(rgb_arm,commands,right_times,aidx['right'],observed_initial,out,here)
    cmd[:]=commands[0]
    preview=Audit(cmd,aidx,dt)
    for row in commands[1:]:preview.update(row[None,:])
    report['initial_adaptation_command_preview']=preview.report()
    def apply_targets(new,old):
        nonlocal effort_blend,effort_part
        """Native grasp(speed, force): velocity servo capped at 30 N/finger.

        Unlike a stopped position target, this continues squeezing a held part.
        Opening and pushing retain position control. No attachment is created.
        """
        velocity=(new-old)/dt;changed=False
        if options.get('native_force_grasp'):
            l,r=active_clock
            # First close in position control so both pads make contact;
            # apply holding preload after the commanded close has finished.
            wanted=dict(left=(26.91<=l<39.99 or 51.23<=l<67.7),right=(10.684<=r<27.31 or 37.274<=r<51.63 or 60.36<=r<81.82))
            for side in wanted:
                ids=[i for i,n in enumerate(names)if n.startswith(side+'_native')]
                if wanted[side]!=force_state[side]:
                    force_state[side]=wanted[side];stiffness[0,ids]=finger_stiffness;damping[0,ids]=1000. if wanted[side]else 100.;changed=True
                if wanted[side]:velocity[0,ids]=-.0475
            if changed:art.set_dof_stiffnesses(stiffness,index);art.set_dof_dampings(damping,index)
        art.set_dof_position_targets(new,index);art.set_dof_velocity_targets(velocity,index)
    executed=[dict(t=0.,command=cmd[0].copy(),clamp=controls[0].copy(),source=float(source_times[0]),right=float(right_times[0]))]
    def advance():
        sim.simulate(dt,t);sim.fetch_results()
        executed.append(dict(t=t,command=cmd[0].copy(),clamp=last.copy(),source=active_clock[0],right=active_clock[1]))
    nominal=json.load(open(out/'arm-plan.json'))['scene_parts']
    initial_part_positions={part:np.array(row['source_P'])for part,row in nominal.items()}
    relocalized=set();small_floor_offset=np.zeros(3)
    def relocalize_small(k,label,expected):
        nonlocal small_floor_offset
        from isaac_kinematics import Arm
        observed=vision.observe('ring2',mode='floor',seed=expected)
        delta=observed[:3].astype(float)-expected
        if np.linalg.norm(delta)>.04:raise RuntimeError('Small ring moved outside local pickup correction range')
        small_floor_offset+=delta
        arm=Arm(stage,'right',drives,art.get_dof_positions()[0,aidx['right']]);idx=aidx['right'];end=int(np.searchsorted(right_times,norm_points.get('ring2',46.4896)));fade_start=int(np.searchsorted(right_times,44.404));start_time=k*dt;fade_time=fade_start*dt;end_time=end*dt
        seed=commands[k,idx].copy();previous=seed.copy()
        for n in range(k,end+1):
            base=commands[n,idx].copy();ramp=np.clip((n*dt-start_time)/.35,0,1);ramp=ramp**3*(10-15*ramp+6*ramp*ramp);u=np.clip((n*dt-fade_time)/max(end_time-fade_time,dt),0,1);weight=ramp*(1-u**3*(10-15*u+6*u*u));H=arm.fk(base);H[:3,3]+=delta*weight
            solution=arm.solve(H[:3,3],H[:3,:3],seed+base-previous,position_tolerance=1e-8,rotation_tolerance=1e-7)
            if solution is None:raise RuntimeError('Live small-ring pickup correction IK failed '+label)
            commands[n,idx]=base+(solution-base)*weight;seed=solution;previous=base
        report.setdefault('small_ring_pose_updates',[]).append(dict(label=label,source_s=float(right_times[k]),observed_m=observed[:3].tolist(),delta_m=delta.tolist(),robot_targets_only=True))
        print('SMALL_RING_RELOCALIZED',report['small_ring_pose_updates'][-1],flush=True)
    def normalize(part,k,release_end,attempt=0):
        """Correct a changed friction grasp by moving the robot, never the part."""
        nonlocal cmd,t,extra,skip_until
        from isaac_kinematics import Arm,frame,exp,log
        from native_pickup_probe import rotation
        from isaac_motion import duration
        from isaac_collision_queries import Checker
        ref=np.load(here.parents[1]/'runs/native-roas-table-bin-under30-capture01/physx-sequence-trace.npz')
        lane_time=right_times
        st=float(lane_time[k]);reference_time=({'ring1':27.85,'ring2':52.2}.get(part,st)if options.get('od_grasp_pipeline')else st);ix=int(np.argmin(abs(ref['time_s']-reference_time)));pidx=ref['part_names'].tolist().index(part)
        target=ref['parts'][ix,pidx].copy();target[:3]+=np.array(options.get('handover_offsets_m',{}).get(part,[0,0,0]));observed=vision.observe(part)
        if np.linalg.norm(observed[:3]-target[:3])>.06:raise RuntimeError('Part was lost before handover: '+part)
        axis_distance=float(np.arccos(np.clip(rotation(observed[3:])[:,2]@rotation(target[3:])[:,2],-1,1)))
        if options.get('od_grasp_pipeline')and np.linalg.norm(observed[:3]-target[:3])<(.0001 if part=='pin'else.0003)and axis_distance<(.003 if part=='pin'else.005):
            report.setdefault('live_grasp_normalizations',[]).append(dict(part=part,source_time_s=st,target_reference_time_s=reference_time,already_aligned=True,position_error_m=float(np.linalg.norm(observed[:3]-target[:3])),axis_error_rad=axis_distance,extra_duration_s=0.));return

        arm=Arm(stage,'right',drives,art.get_dof_positions()[0,aidx['right']]);H=arm.fk(art.get_dof_positions()[0,aidx['right']]);G=np.linalg.inv(H)@frame(observed[:3],rotation(observed[3:]))
        targetR=rotation(target[3:])
        if part in ['ring1','ring2','pin']:
            axis=rotation(observed[3:])[:,2];desired=targetR[:,2];cross=np.cross(axis,desired);norm=np.linalg.norm(cross);targetR=exp(cross/max(norm,1e-12)*np.arctan2(norm,float(axis@desired)))@rotation(observed[3:])
        checker=Checker(stage,arm,'right');route=None;saved_for_pin=commands.copy()if part=='pin'else None
        if part=='pin'and options.get('pin_in_motion_alignment'):checker.ignored_future_actors={str(pr.GetPath())for pr in stage.Traverse()if pr.HasAPI(UsdPhysics.RigidBodyAPI)and('left_arm_'in str(pr.GetPath())or'left_NativeHand'in str(pr.GetPath()))}
        for yaw in ([]if part=='pin'and(options.get('pin_axis_ik')or options.get('pin_replan_sequence'))else([0.,.15,-.15,.3,-.3,.6,-.6,1.,-1.]if part.startswith('ring')else[0.,.3,-.3,.6,-.6,1.,-1.,1.5,-1.5,2.,-2.])):
            if part=='pin'and yaw in options.get('pin_excluded_alignment_yaws',[]):continue
            desiredH=frame(target[:3],targetR@exp(np.array([0.,0.,yaw])))@np.linalg.inv(G);goal=arm.solve(desiredH[:3,3],desiredH[:3,:3],cmd[0,aidx['right']])
            if goal is None:continue
            if part=='pin':
                candidateD=np.linalg.inv(arm.fk(commands[k,aidx['right']]))@arm.fk(goal);future_seed=goal.copy();future_ok=True;pin_stop=int(np.searchsorted(lane_time,release_end))
                for n in np.unique(np.linspace(k,pin_stop,36).astype(int)):
                    expectedH=arm.fk(commands[n,aidx['right']])@candidateD;future_seed=arm.solve(expectedH[:3,3],expectedH[:3,:3],future_seed)
                    if future_seed is None:future_ok=False;break
                if not future_ok:continue
            route=checker.route(cmd[0,aidx['right']],goal,float(cmd[0,-2:].sum()),paths['workpiece_'+part],seconds=4)
            if route is not None:break
        pin_axis_path=None
        pin_complete_plan=None
        if part=='pin'and options.get('pin_replan_sequence'):
            from plan_live_pin import plan as plan_pin
            pin_complete_plan,evidence=plan_pin(stage,arm,commands,lane_time,aidx['right'],k,G,paths,float(art.get_dof_positions()[0,-2:].sum()),ref,pidx);report['pin_live_plan']=evidence;print('PIN_LIVE_PLAN',evidence,flush=True);pin_axis_path=pin_complete_plan[k:int(np.searchsorted(lane_time,release_end))+1];goal=pin_axis_path[0];route=[goal,goal]
        elif part=='pin'and options.get('pin_axis_ik'):
            from pin_axis_ik import path as axis_path
            pin_axis_path=axis_path(arm,commands[:,aidx['right']].copy(),lane_time,k,int(np.searchsorted(lane_time,release_end)),G,ref,pidx);goal=pin_axis_path[0];route=[commands[k,aidx['right']].copy(),goal]
        if route is None:raise RuntimeError('Live grasp normalization IK/route failed '+part+' initial error='+str(np.linalg.norm(observed[:3]-target[:3])))
        baseH=arm.fk(commands[k,aidx['right']]);D=np.linalg.inv(baseH)@arm.fk(goal);rv=log(D[:3,:3]);dp=D[:3,3]
        # Map the held-part portion to the observed grasp. Fade only after release.
        stop=int(np.searchsorted(lane_time,release_end))
        previous_base=commands[k,aidx['right']].copy();previous_solution=goal.copy()
        for n in range(k,stop+1):
            base=commands[n,aidx['right']].copy();B=arm.fk(base)@D;seed=previous_solution+base-previous_base;sol=(pin_axis_path[n-k]if pin_axis_path is not None else(goal.copy()if n==k else arm.solve(B[:3,3],B[:3,:3],seed,position_tolerance=1e-8,rotation_tolerance=1e-7)))
            if sol is None:raise RuntimeError('Live held trajectory IK failed '+part+' source='+str(float(z['source_time_s'][n])))
            weight=1.
            if part=='pin'and options.get('pin_in_motion_alignment')and not options.get('pin_replan_sequence'):
                align_end=int(np.searchsorted(lane_time,79.20));u=float(np.clip((n-k)/max(align_end-k,1),0,1));weight=u**3*(10-15*u+6*u*u)
            commands[n,aidx['right']]=base+(sol-base)*weight;previous_base=base;previous_solution=sol
        elapsed=0.
        for a,b in ([]if part=='pin'and options.get('pin_in_motion_alignment')else zip(route[:-1],route[1:])):
            count=int(np.ceil(duration([a,b],.2 if part=='pin'else.4)/dt))
            for j in range(count):
                oldcmd=cmd.copy();u=(j+1)/count;h=u**3*(10-15*u+6*u*u);cmd[0,aidx['right']]=a+(b-a)*h;audit.update(cmd);apply_targets(cmd,oldcmd);t+=dt;elapsed+=dt;advance()
                if peak>12:raise RuntimeError('Cross-arm contact during normalization')
                if j%8==0:tape.append(dict(t=t,q=art.get_dof_positions()[0].copy(),links=art.get_link_transforms()[0].copy(),parts=np.array([v.get_transforms()[0]for v in truth_parts.values()])))
        extra+=elapsed
        if options.get('smooth_donor_blend')and(part!='ring2'or options.get('od_grasp_pipeline')):
            # Release is complete in this stationary interval. Blend the small
            # joint correction back over physical time while following the
            # collision-checked empty-hand path. No workpiece is constrained.
            import copy
            idx=aidx['right'];base=original_commands
            offset=commands[stop,idx].astype(float)-base[stop,idx]
            vel=((commands[stop,idx]-commands[stop-1,idx])-(base[stop,idx]-base[stop-1,idx])).astype(float)/dt
            acc=((commands[stop,idx]-2*commands[stop-1,idx]+commands[stop-2,idx])-(base[stop,idx]-2*base[stop-1,idx]+base[stop-2,idx])).astype(float)/dt**2
            candidates=[];fallback=None
            for requested in np.arange(float(options.get('donor_blend_s',1.8)),3.61,.2):
                count=min(int(round(requested/dt)),len(commands)-1-stop);T=count*dt;commands[stop+1:,idx]=base[stop+1:,idx]
                M=np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,2,0,0,0],[1,T,T*T,T**3,T**4,T**5],[0,1,2*T,3*T*T,4*T**3,5*T**4],[0,0,2,6*T,12*T*T,20*T**3]])
                coef=np.linalg.solve(M,np.array([offset,vel,acc,np.zeros(7),np.zeros(7),np.zeros(7)]))
                for n in range(stop+1,stop+count+1):
                    x=(n-stop)*dt;commands[n,idx]=base[n,idx]+np.array([x**j for j in range(6)])@coef
                if fallback is None:fallback=commands.copy()
                if pin_complete_plan is not None:commands[k:,idx]=pin_complete_plan[k:];fallback=commands.copy()
                preview=copy.deepcopy(audit)
                try:
                    for row in commands[k:]:preview.update(row[None,:])
                    candidates.append(dict(duration_s=T,command_limits_pass=True));break
                except RuntimeError as error:candidates.append(dict(duration_s=T,command_limits_pass=False,error=str(error)))
            else:
                if part=='pin'and options.get('pin_in_motion_alignment'):
                    from retime_live_continuation import prepare as retime_future
                    commands[:]=fallback
                    continuation=np.vstack([cmd,commands[k:]]);cc=np.vstack([last,controls[k:]]);lc=np.r_[source_times[k-1],source_times[k:]];rc=np.r_[right_times[k-1],right_times[k:]]
                    np.savez_compressed(out/'pin-before-local-retiming.npz',command=continuation,clamp=cc,source_time_s=lc,right_source_time_s=rc,time_s=np.arange(len(continuation))*dt)
                    continuation,cc,lc,rc,evidence=retime_future(continuation,cc,lc,rc,audit,aidx,dt)
                    report['pin_live_retiming']=evidence;print('PIN_CONTINUATION_RETIMED',evidence,flush=True)
                    np.savez_compressed(out/'pin-retimed-continuation.npz',command=continuation,clamp=cc,source_time_s=lc,right_source_time_s=rc,time_s=np.arange(len(continuation))*dt)
                    for n in range(1,len(continuation)):
                        oldcmd=cmd.copy();cmd[:]=continuation[n];audit.update(cmd);active_clock[:]=[float(lc[n]),float(rc[n])];apply_targets(cmd,oldcmd);t+=dt
                        apply_clamps(cc[n]);advance()
                        if peak>12:raise RuntimeError('Cross-arm contact in retimed pin continuation')
                        if n%8==0 or n==len(continuation)-1:tape.append(dict(t=t,q=art.get_dof_positions()[0].copy(),links=art.get_link_transforms()[0].copy(),parts=np.array([v.get_transforms()[0]for v in truth_parts.values()])))
                    skip_until=len(commands);report.setdefault('live_grasp_normalizations',[]).append(dict(part=part,in_motion=True,locally_retimed=True,extra_duration_s=evidence['added_s']));return
                raise RuntimeError('No bounded donor correction blend '+part+' '+str(candidates))
            report.setdefault('empty_donor_blends',[]).append(dict(part=part,duration_s=T,max_joint_offset_rad=float(abs(offset).max()),release_source_time_s=release_end,preview_command_audit=preview.report(),candidates=candidates))
        else:pending_retreat[part]=stop
        report.setdefault('live_grasp_normalizations',[]).append(dict(part=part,source_time_s=st,initial_position_error_m=float(np.linalg.norm(observed[:3]-target[:3])),final_position_error_m=None,in_motion=bool(part=='pin'and options.get('pin_in_motion_alignment')),extra_duration_s=elapsed))
        print('LIVE_NORMALIZED',report['live_grasp_normalizations'][-1],flush=True)
        if options.get('normalization_refine')and not(part=='pin'and options.get('pin_in_motion_alignment')):
            final=vision.observe(part);error=float(np.linalg.norm(final[:3]-target[:3]));axis_error=float(np.arccos(np.clip(rotation(final[3:])[:,2]@rotation(target[3:])[:,2],-1,1)))
            report['live_grasp_normalizations'][-1]['final_position_error_m']=error
            if error>(.00015 if part=='pin'else.0015)or axis_error>(.005 if part=='pin'else.035):
                if attempt>=2:raise RuntimeError('Handover alignment failed to converge '+part+' error='+str(error)+' axis='+str(axis_error))
                normalize(part,k,release_end,attempt+1)

    def retreat(part,k):
        """Plan withdrawal from the corrected handover, replacing old retreat."""
        nonlocal cmd,t,extra,skip_until
        from isaac_kinematics import Arm
        from isaac_collision_queries import Checker
        from isaac_motion import duration
        endpoint={'ring1':30.14,'ring2':54.85119657796458,'pin':84.18}[part]
        end=int(np.searchsorted(right_times,endpoint));goal=original_commands[end,aidx['right']]
        if np.max(abs(original_commands[end,aidx['left']]-cmd[0,aidx['left']]))>1e-4:raise RuntimeError('Cannot skip a moving recipient during donor retreat')
        arm=Arm(stage,'right',drives,art.get_dof_positions()[0,aidx['right']]);checker=Checker(stage,arm,'right');route=None;saved_for_pin=commands.copy()if part=='pin'else None
        if part=='pin'and options.get('pin_in_motion_alignment'):checker.ignored_future_actors={str(pr.GetPath())for pr in stage.Traverse()if pr.HasAPI(UsdPhysics.RigidBodyAPI)and('left_arm_'in str(pr.GetPath())or'left_NativeHand'in str(pr.GetPath()))} if part=='ring2'else checker.route(cmd[0,aidx['right']],goal,float(cmd[0,-2:].sum()),paths['workpiece_'+part],seconds=12)
        if route is None and part!='ring2':raise RuntimeError('Corrected donor retreat blocked '+part)
        if part=='ring2':
            H=arm.fk(cmd[0,aidx['right']]);up=arm.solve(H[:3,3]+np.array([0.,0.,.035]),H[:3,:3],cmd[0,aidx['right']])
            if up is None:raise RuntimeError('No vertical donor release clearance IK')
            clear_route=checker.route(cmd[0,aidx['right']],up,float(cmd[0,-2:].sum()),paths['workpiece_'+part],seconds=6)
            if clear_route is None:raise RuntimeError('Vertical donor release clearance blocked')
            park_route=checker.route(up,goal,float(original_commands[end,-2:].sum()),paths['workpiece_'+part],seconds=8)
            if park_route is None:raise RuntimeError('Donor park after clearance blocked')
            route=clear_route+park_route[1:]
        elapsed=0.
        for a,b in ([]if part=='pin'and options.get('pin_in_motion_alignment')else zip(route[:-1],route[1:])):
            count=int(np.ceil(duration([a,b],.25)/dt))
            for j in range(count):
                oldcmd=cmd.copy();u=(j+1)/count;h=u**3*(10-15*u+6*u*u);cmd[0,aidx['right']]=a+(b-a)*h;cmd[0,-2:]+=np.clip(original_commands[end,-2:]-cmd[0,-2:],-.045*dt,.045*dt);audit.update(cmd);apply_targets(cmd,oldcmd);t+=dt;elapsed+=dt;advance()
                if peak>12:raise RuntimeError('Cross-arm contact in corrected donor retreat')
                if j%8==0:tape.append(dict(t=t,q=art.get_dof_positions()[0].copy(),links=art.get_link_transforms()[0].copy(),parts=np.array([v.get_transforms()[0]for v in truth_parts.values()])))
        extra+=elapsed-(end-k)*dt;skip_until=end
        report.setdefault('live_donor_retreats',[]).append(dict(part=part,duration_s=elapsed,replaced_duration_s=(end-k)*dt,waypoints=len(route)))
        print('LIVE_RETREAT',report['live_donor_retreats'][-1],flush=True)
    def apply_clamps(values):
        changed=False
        for j,p in enumerate(truth_parts):
            if abs(values[j]-last[j])<1e-7:continue
            last[j]=values[j];changed=True
            for joint in joints[p]:
                limit=joint.GetLowerLimitAttr().Get()if p=='pin'else joint.GetUpperLimitAttr().Get()
                UsdPhysics.DriveAPI(joint.GetPrim(),'linear').GetTargetPositionAttr().Set(float(limit*last[j]))
        if changed:sim.flush_changes()
    try:
        for k in range(1,len(commands)):
            if k<skip_until:continue
            active_clock[:]=[float(source_times[k]),float(right_times[k])]
            for part,deadline in list(pending_retreat.items()):
                if k>=deadline:retreat(part,k);del pending_retreat[part]
            if k<skip_until:continue
            if options.get('od_grasp_pipeline'):
                for label,checkpoint,offset in [('before_planar_drag',33.889,np.zeros(3)),('after_planar_drag',42.6,np.array([0.,-.08,0.]))]:
                    if label not in relocalized and right_times[k]>=checkpoint:
                        relocalize_small(k,label,initial_part_positions['ring2']+offset+small_floor_offset);relocalized.add(label)
            if options.get('live_grasp_normalization'):
                for part,checkpoint,release_end in [('ring1',20.95,27.90 if options.get('smooth_donor_blend')else 28.33),('ring2',46.4896,52.20 if options.get('smooth_donor_blend')else 52.66),('pin',64.858,84.18 if options.get('smooth_donor_blend')else 82.39)]:
                    if part not in options.get('normalization_parts',['ring1','ring2']):continue
                    checkpoint=norm_points.get(part,checkpoint)
                    if part not in aligned and right_times[k]>checkpoint+.5:raise RuntimeError('No stationary normalization window '+part)
                    quiet=slice(7,14)if part=='pin'and options.get('pin_in_motion_alignment')else slice(None)
                    if part not in aligned and right_times[k]>=checkpoint-1e-6 and max(abs(audit.v[quiet]))<.0002 and max(abs(audit.a[quiet]))<.05:
                        normalize(part,k,release_end);aligned.add(part)
            if k<skip_until:continue
            t=k*dt+extra;old=cmd.copy();cmd[:]=commands[k];audit.update(cmd)
            finger_peak=max(finger_peak,float(abs(cmd[0,fi]-old[0,fi]).max()/dt))
            if finger_peak>.0476:raise RuntimeError('Finger speed limit exceeded')
            apply_targets(cmd,old)
            apply_clamps(controls[k])
            advance()
            if peak>12:raise RuntimeError('Cross-arm contact exceeded 12 N: '+str(peak))
            if k%8==0 or k==len(commands)-1:
                tape.append(dict(t=t,q=art.get_dof_positions()[0].copy(),links=art.get_link_transforms()[0].copy(),parts=np.array([v.get_transforms()[0]for v in truth_parts.values()])))
            if k%960==0:print('RETIMED_STEP',round(t,2),'source',round(float(source_times[k]),2),{p:v.get_transforms()[0][:3].tolist()for p,v in truth_parts.items()},flush=True)
        from native_pickup_probe import rotation
        goals=json.load(open(out/'arm-plan.json'))['goals'];checks={}
        for p,v in truth_parts.items():
            tr=v.get_transforms()[0];checks[p]=dict(position_error_m=float(np.linalg.norm(tr[:3]-goals[p]['P'])),axis_error_rad=float(np.arccos(np.clip(rotation(tr[3:])[2,2],-1,1))))
        passed=all(x['position_error_m']<.0015 and x['axis_error_rad']<.05 for x in checks.values())
        # Time task completion separately from the final observation window.
        # All truth_parts must stay seated and both hands must be clear throughout
        # the remaining trace, for at least half a second.
        valid=[];roots=[links.index(s+'_NativeHand')for s in ['left','right']]
        for row in tape:
            good=True
            for j,p in enumerate(truth_parts):
                tr=row['parts'][j];axis=float(np.arccos(np.clip(rotation(tr[3:])[2,2],-1,1)))
                good&=np.linalg.norm(tr[:3]-goals[p]['P'])<.0015 and axis<.05
                good&=min(np.linalg.norm(row['links'][h,:3]-tr[:3])for h in roots)>.16
            valid.append(good)
        bad=np.flatnonzero(~np.array(valid));begin=int(bad[-1]+1)if len(bad)else 0
        complete=float(tape[begin]['t'])if begin<len(tape)else None
        retained=complete is not None and t-complete>=.5;passed=bool(passed and retained)
        report.update(state='pipeline_pass'if passed else'pipeline_failed_acceptance',full_task_pass=passed,continuous_cycle_pass=passed,final_checks=checks,cycle_time_s=complete if passed else None,verified_elapsed_s=t,final_retention_observation_s=(t-complete)if complete is not None else 0.,hand_clearance_gate_m=.16)
        report['timing']['timing_is_measured_success']=passed
    except Exception:
        report.update(state='pipeline_failed',error=traceback.format_exc(),cycle_time_s=None);print(report['error'],flush=True)
    finally:
        report.update(vision_end_to_end_pass=bool(report.get('full_task_pass')),perception_wall_s=vision.total_wall_s,perception_observations=len(vision.history),simulated_elapsed_s=t,cross_arm_peak_contact_n=peak,cross_arm_contacts=contacts,FR3_command_limit_audit=audit.report(),finger_target_peak_speed_m_s=finger_peak,part_attachments=False,workpiece_teleports_after_initialization=0)
        if executed:np.savez_compressed(out/'executed-actuator-commands.npz',time_s=[v['t']for v in executed],command=[v['command']for v in executed],clamp=[v['clamp']for v in executed],source_time_s=[v['source']for v in executed],right_source_time_s=[v['right']for v in executed])
        if tape:report['measured_finger_symmetry_peak_m']={side:float(max(abs(row['q'][[i for i,n in enumerate(names)if n.startswith(side+'_native')][0]]-row['q'][[i for i,n in enumerate(names)if n.startswith(side+'_native')][1]])for row in tape))for side in ['left','right']}
        if tape:np.savez_compressed(out/'physx-sequence-trace.npz',time_s=[v['t']for v in tape],q=[v['q']for v in tape],links=[v['links']for v in tape],parts=[v['parts']for v in tape],part_names=list(truth_parts))
        (out/'pipeline-results.json').write_text(json.dumps(report,indent=2));sim.detach_stage()
    print('PIPELINE_RESULT',report['state'],flush=True)
