"""Actual dual-arm PhysX execution, with no workpiece pose replay/attachments."""
import json, math, traceback
import numpy as np
import omni.physx
import omni.physics.tensors as tensors
from pxr import UsdUtils, UsdPhysics, PhysicsSchemaTools
from basket_pipeline import Schedule
from isaac_motion import Audit


def execute(stage, paths, hands, drives, initial, report, out, here):
    root = here.parents[1]/'runs'
    rings = np.load(root/'native-roas-table-bin-quick07/physx-pickup-trace.npz')
    pin = np.load(root/'native-roas-table-bin-pin-simple02/physx-pickup-trace.npz')
    source_parts=np.load(root/'native-roas-table-bin-quick07/physx-sequence-trace.npz')
    options_file=out/'pipeline-options.json'if(out/'pipeline-options.json').exists()else here/'bin-picking/pipeline-options.json'
    options=json.load(open(options_file))if options_file.exists()else{}
    schedule = Schedule(rings, pin,ring2_carry_s=float(options.get('ring2_carry_s',8.1854166667)))
    sequential = Schedule(rings, pin, overlap=False)
    speed=float(options.get('clock_speed',1.))
    if not 1<=speed<=1.12:raise ValueError('Unsupported command clock speed')
    ident = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
    sim = omni.physx.get_physx_simulation_interface(); sim.attach_stage(ident)
    dt = 1/480; sim.simulate(dt, 0); sim.fetch_results()
    view = tensors.create_simulation_view('numpy', ident)
    art = view.create_articulation_view('/World/RobotMountJoint')
    names = list(art.shared_metatype.dof_names)
    links = list(art.shared_metatype.link_names)
    index = np.array([0], np.uint32)
    aidx = {s:[names.index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']}
    allidx = {s:[i for i,n in enumerate(names)if n.startswith(s+'_')]for s in aidx}
    cmd = schedule.command(0, allidx['left'], allidx['right'])[None,:].astype(np.float32)
    art.set_dof_positions(cmd,index); art.set_dof_velocities(np.zeros_like(cmd),index)
    art.set_dof_stiffnesses(np.array([[12000 if n in drives else 5000 for n in names]],np.float32),index)
    art.set_dof_dampings(np.array([[600 if n in drives else 100 for n in names]],np.float32),index)
    art.set_dof_max_forces(np.array([[(87 if int(n[-1])<=4 else 12)*.95 if n in drives else 30 for n in names]],np.float32),index)
    parts={p:view.create_rigid_body_view(paths['workpiece_'+p])for p in ['ring1','ring2','pin']}
    tape=[]; contacts=[]; peak_cross=0.; t=0.; audit=Audit(cmd,aidx,dt)
    if options.get('record_full_commands'):
        from record_motion import RecordingAudit
        audit=RecordingAudit(cmd,aidx,dt)
    ring_phases=rings['phase'];pin_phases=pin['phase'];ring_times=rings['time_s'];pin_times=pin['time_s']
    def callback(headers, data):
        nonlocal peak_cross
        for h in headers:
            actors=[str(PhysicsSchemaTools.intToSdfPath(x))for x in [h.actor0,h.actor1]]
            cross=(any('left_arm_'in a or 'left_NativeHand'in a for a in actors)
                   and any('right_arm_'in a or 'right_NativeHand'in a for a in actors))
            if not cross:continue
            force=sum(float(np.linalg.norm(data[i].impulse))/dt for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
            peak_cross=max(peak_cross,force)
            if force>2:contacts.append(dict(time_s=t,actors=actors,force_n=force))
    subscription=sim.subscribe_contact_report_events(callback)
    joint_rows={}
    for part in parts:
        joint_rows[part]=[UsdPhysics.PrismaticJoint(p)for p in stage.Traverse()if str(p.GetPath()).startswith('/World/Clamps/'+part+'/')and p.IsA(UsdPhysics.PrismaticJoint)]
    report.update(state='running_pipeline',dof_names=names,link_names=links,
                  timing=dict(sequential_command_schedule_s=sequential.duration,
                              overlap_command_schedule_s=schedule.duration,
                              scheduled_saving_s=sequential.duration-schedule.duration,
                              ring2_carry_duration_s=schedule.ring2_carry_s,
                              timing_is_measured_success=False),
                  perception='Ideal USD object geometry and poses; not camera/SAM validation',
                  original_finger_mesh=True,physics_hz=480)
    reroutes=[];skip_to=0.;time_offset=0.;last_clamp={};rejoin_offset=None;rejoin_begin=0.
    def route_handoff(clock,endpoint,part):
        nonlocal cmd,time_offset,t
        from isaac_kinematics import Arm,exp,log
        from isaac_collision_queries import Checker
        from isaac_motion import duration
        from basket_pipeline import hermite
        arm=Arm(stage,'left',drives,art.get_dof_positions()[0,aidx['left']])
        checker=Checker(stage,arm,'left');checker.resolution_rad=.012
        if part=='ring2':
            if hasattr(audit,'phase'):audit.phase='donor_ring2_normalize'
            # Normalize the donor's held ring before the recipient enters the
            # tight Duo workspace. Ring yaw remains unconstrained.
            from native_pickup_probe import rotation
            from isaac_kinematics import frame
            sj=min(len(source_parts['time_s'])-1,int(np.searchsorted(source_parts['time_s'],endpoint-4.46875)))
            original=source_parts['parts'][sj,source_parts['part_names'].tolist().index(part)].copy()
            handover_offset=np.asarray(options.get('ring2_handover_offset_m',[0.,0.,0.]),float)
            reference_center=original[:3].copy();original[:3]+=handover_offset
            observed=parts[part].get_transforms()[0].copy();old_axis=rotation(original[3:])[:,2];new_axis=rotation(observed[3:])[:,2];cross=np.cross(new_axis,old_axis);norm=np.linalg.norm(cross);rv=cross/max(norm,1e-12)*math.atan2(norm,float(old_axis@new_axis));part_goal=frame(original[:3],exp(rv)@rotation(observed[3:]));giver=Arm(stage,'right',drives,art.get_dof_positions()[0,aidx['right']]);actual_hand=giver.fk(art.get_dof_positions()[0,aidx['right']]);grip=np.linalg.inv(actual_hand)@frame(observed[:3],rotation(observed[3:]));target_hand=part_goal@np.linalg.inv(grip);goal_right=giver.solve(target_hand[:3,3],target_hand[:3,:3],cmd[0,aidx['right']])
            if goal_right is None:raise RuntimeError('Donor alignment IK failed')
            donor_checker=Checker(stage,giver,'right');opening=float(cmd[0,allidx['right'][-2:]].sum());right_route=donor_checker.route(cmd[0,aidx['right']],goal_right,opening,paths['workpiece_'+part],seconds=20)
            if right_route is None:raise RuntimeError('Donor alignment route blocked')
            elapsed=0.
            for aa,bb in zip(right_route[:-1],right_route[1:]):
                n=math.ceil(duration([aa,bb],.8)/dt)
                for j in range(n):
                    old=cmd.copy();u=(j+1)/n;h=u**3*(10-15*u+6*u*u);cmd[0,aidx['right']]=aa+(bb-aa)*h;audit.update(cmd);art.set_dof_position_targets(cmd,index);art.set_dof_velocity_targets((cmd-old)/dt,index);elapsed+=dt;t=clock/speed+time_offset+elapsed;sim.simulate(dt,t);sim.fetch_results()
                    if peak_cross>12:raise RuntimeError('Donor alignment cross-arm contact')
                    if j%8==0:tape.append(dict(t=t,q=art.get_dof_positions()[0].copy(),links=art.get_link_transforms()[0].copy(),parts=np.array([v.get_transforms()[0]for v in parts.values()])))
            time_offset+=elapsed
            source_clock=endpoint-4.46875;reference=hermite(schedule.rings,source_clock)[aidx['right']];D=np.linalg.inv(giver.fk(reference))@giver.fk(goal_right);times=schedule.rings['time_s'];values=schedule.rings['command'];rotation_offset=log(D[:3,:3]);translation_offset=D[:3,3]
            original_values=values.copy();original_tape={'time_s':times,'command':original_values}
            for i in np.where((times>=source_clock-.025)&(times<=68.3958333333))[0]:
                # Keep the new handover center until the fingers have lifted
                # clear. Removing it during release sweeps through the receiver.
                source_t=times[i]
                if 64.7958333333<=source_t<=66.5958333333:source_t=64.7958333333+(source_t-64.7958333333)*(1.5/1.8)
                u=np.clip((times[i]-66.5958333333)/1.5,0,1);w=1-u**3*(10-15*u+6*u*u);base=hermite(original_tape,source_t)[aidx['right']]if 64.7958333333<=times[i]<=66.5958333333 else original_values[i,aidx['right']].astype(float);H=giver.fk(base)@frame(w*translation_offset,exp(w*rotation_offset));solved=giver.solve(H[:3,3],H[:3,:3],base)
                if solved is None:raise RuntimeError('Donor withdrawal correction IK failed')
                values[i,aidx['right']]=solved
            report['ring2_donor_alignment']=dict(initial_position_error_m=float(np.linalg.norm(observed[:3]-original[:3])),final_position_error_m=float(np.linalg.norm(parts[part].get_transforms()[0,:3]-original[:3])),duration_s=elapsed,axial_yaw_unconstrained=True)
            print('DONOR_RING_ALIGNED',report['ring2_donor_alignment'],flush=True)
            from recipient_grasp_search import plan as search_recipient
            report['recipient_grasp_search']=search_recipient(arm,checker,schedule.rings,aidx['left'],reference_center,old_axis,paths['workpiece_'+part],endpoint,handover_offset)
            print('RECEIVING_GRASP',report['recipient_grasp_search'],flush=True)
        goal=hermite(schedule.rings,endpoint)[aidx['left']]
        opening=float(cmd[0,allidx['left'][-2:]].sum())
        print('HANDOFF_ROUTE_QUERY',part,checker.clear(goal,opening,paths['workpiece_'+part]),flush=True)
        route=checker.route(cmd[0,aidx['left']],goal,opening,paths['workpiece_'+part],seconds=40)
        if route is None:raise RuntimeError('No collision-free handoff approach '+part)
        elapsed=0.
        if hasattr(audit,'phase'):audit.phase='recipient_route_'+part
        for a,b in zip(route[:-1],route[1:]):
            T=duration([a,b],.35);n=math.ceil(T/dt)
            for j in range(n):
                old=cmd.copy();u=(j+1)/n;h=u**3*(10-15*u+6*u*u);cmd[0,aidx['left']]=a+(b-a)*h;audit.update(cmd)
                art.set_dof_position_targets(cmd,index);art.set_dof_velocity_targets((cmd-old)/dt,index)
                elapsed+=dt;t=clock/speed+time_offset+elapsed;sim.simulate(dt,t);sim.fetch_results()
                if peak_cross>12:raise RuntimeError('Rerouted handoff cross-arm contact '+str(peak_cross))
                if j%8==0:tape.append(dict(t=t,q=art.get_dof_positions()[0].copy(),links=art.get_link_transforms()[0].copy(),parts=np.array([v.get_transforms()[0]for v in parts.values()])))
        nominal_endpoint=endpoint-(schedule.saved1 if part=='ring2'else 0.)
        time_offset+=elapsed-(nominal_endpoint-clock)/speed
        reroutes.append(dict(part=part,waypoints=len(route),duration_s=elapsed))
    def clear_donor(clock,endpoint):
        nonlocal cmd,time_offset,t
        from isaac_kinematics import Arm
        from isaac_collision_queries import Checker
        from isaac_motion import duration
        from basket_pipeline import hermite
        arm=Arm(stage,'right',drives,art.get_dof_positions()[0,aidx['right']]);checker=Checker(stage,arm,'right');checker.resolution_rad=.012;elapsed=0.;records=[]
        for label,source_t in [('lift_clear',66.5958333333),('park',68.3958333333)]:
            if hasattr(audit,'phase'):audit.phase='empty_donor_'+label
            goal=hermite(schedule.rings,source_t)[aidx['right']]
            route=checker.route(cmd[0,aidx['right']],goal,.009,paths['workpiece_ring2'],seconds=20)
            if route is None:raise RuntimeError('No empty donor retreat route '+label)
            begin=elapsed
            for aa,bb in zip(route[:-1],route[1:]):
                n=math.ceil(duration([aa,bb],.4)/dt)
                for j in range(n):
                    old=cmd.copy();u=(j+1)/n;h=u**3*(10-15*u+6*u*u);cmd[0,aidx['right']]=aa+(bb-aa)*h;audit.update(cmd);art.set_dof_position_targets(cmd,index);art.set_dof_velocity_targets((cmd-old)/dt,index);elapsed+=dt;t=clock/speed+time_offset+elapsed;sim.simulate(dt,t);sim.fetch_results()
                    if peak_cross>12:raise RuntimeError('Empty donor retreat cross-arm contact')
                    if j%8==0:tape.append(dict(t=t,q=art.get_dof_positions()[0].copy(),links=art.get_link_transforms()[0].copy(),parts=np.array([v.get_transforms()[0]for v in parts.values()])))
            records.append(dict(phase=label,waypoints=len(route),duration_s=elapsed-begin))
        time_offset+=elapsed-(endpoint-clock)/speed;report['empty_donor_retreat']=records;print('DONOR_RETREAT_COMPLETE',records,flush=True)
    try:
        for k in range(math.ceil(schedule.duration/(dt*speed))):
            clock=(k+1)*dt*speed
            if clock<skip_to:continue
            for start,end,part in [(21.415,25.13125,'ring1'),(56.8541666667-schedule.saved1,61.3229166667-schedule.saved1,'ring2')]:
                if start<=clock<start+dt*speed:
                    # The recipient approaches the actual stationary donor;
                    # do not reuse an unchecked straight joint interpolation.
                    # endpoint is in ring-tape time for the second handoff.
                    source_endpoint=end+(schedule.saved1 if part=='ring2'else 0.)
                    route_handoff(clock,source_endpoint,part)
                    skip_to=end
                    rejoin_begin=end
                    rejoin_offset=cmd[0].astype(float)-schedule.command(end,allidx['left'],allidx['right'])
            start=64.7958333333-schedule.saved1;end=68.3958333333-schedule.saved1
            if start<=clock<start+dt*speed:
                clear_donor(clock,end);skip_to=end;rejoin_begin=end;rejoin_offset=cmd[0].astype(float)-schedule.command(end,allidx['left'],allidx['right'])
            if clock<skip_to:continue
            t=clock/speed+time_offset; old=cmd.copy();cmd[:]=schedule.command(clock,allidx['left'],allidx['right'])
            # A narrow rim grasp needs only 9 mm clearance to release. The
            # old 16 mm opening swept a donor finger into the receiving hand.
            _,right_source,_=schedule.clocks(clock)
            if right_source>=63.7666666667 and clock<schedule.pin_begin:
                u=np.clip((right_source-66.3)/.3,0,1);cap=.0045+(.008-.0045)*(u**3*(10-15*u+6*u*u))
                fi=[i for i,n in enumerate(names)if n.startswith('right_native')];cmd[0,fi]=np.minimum(cmd[0,fi],cap)
            if rejoin_offset is not None:
                u=np.clip((clock-rejoin_begin)/.35,0,1);weight=1-u**3*(10-15*u+6*u*u)
                cmd[0]+=rejoin_offset*weight
            for side in allidx:
                fi=[i for i,n in enumerate(names)if n.startswith(side+'_native')]
                cmd[0,fi]=np.clip(cmd[0,fi],old[0,fi]-.0475*dt,old[0,fi]+.0475*dt)
            if hasattr(audit,'phase'):
                l,r,p=schedule.clocks(clock)
                lp=ring_phases[min(len(ring_phases)-1,int(np.searchsorted(ring_times,l)))];rp=(pin_phases[min(len(pin_phases)-1,int(np.searchsorted(pin_times,p)))]if clock>=schedule.pin_begin else ring_phases[min(len(ring_phases)-1,int(np.searchsorted(ring_times,r)))])
                audit.phase=str(lp)+' || '+str(rp)
            audit.update(cmd)
            art.set_dof_position_targets(cmd,index);art.set_dof_velocity_targets((cmd-old)/dt,index)
            lc,rc,pc=schedule.clocks(clock)
            changed=False
            for p,part_clock,start in [('ring1',lc,39.3729166667),('ring2',lc,80.5541666667),('pin',pc,17.275)]:
                close=np.clip((part_clock-start)/.25,0,1)
                if p=='pin':close=min(close,1-np.clip((part_clock-31.7)/.25,0,1))+np.clip((part_clock-34.65)/.25,0,1)
                if last_clamp.get(p)==close:continue
                last_clamp[p]=close;changed=True
                if hasattr(audit,'control'):audit.control(p,close)
                for j in joint_rows[p]:
                    limit=j.GetLowerLimitAttr().Get()if p=='pin'else j.GetUpperLimitAttr().Get()
                    UsdPhysics.DriveAPI(j.GetPrim(),'linear').GetTargetPositionAttr().Set(float(limit*close))
            if changed:sim.flush_changes()
            sim.simulate(dt,t);sim.fetch_results()
            if peak_cross>12:raise RuntimeError('Cross-arm contact exceeded 12 N: '+str(peak_cross))
            if k%8==0:
                tape.append(dict(t=t,q=art.get_dof_positions()[0].copy(),links=art.get_link_transforms()[0].copy(),parts=np.array([v.get_transforms()[0]for v in parts.values()])))
            if k%2400==0:
                print('PIPELINE_STEP',round(t,2),{p:v.get_transforms()[0][:3].tolist()for p,v in parts.items()},flush=True)
            # Reject a dropped part promptly, without a long irrelevant run.
            if clock>45 and parts['ring1'].get_transforms()[0][2]<1.4:raise RuntimeError('Large ring was not retained')
            if clock>53 and parts['ring2'].get_transforms()[0][2]<.8:raise RuntimeError('Small ring dropped after handoff')
        goals=json.load(open(out/'arm-plan.json'))['goals'];checks={}
        for p,v in parts.items():
            tr=v.get_transforms()[0];error=float(np.linalg.norm(tr[:3]-goals[p]['P']))
            from native_pickup_probe import rotation
            axis=float(np.arccos(np.clip(rotation(tr[3:])[2,2],-1,1)))
            checks[p]=dict(position_error_m=error,axis_error_rad=axis)
        passed=all(x['position_error_m']<.0015 and x['axis_error_rad']<.05 for x in checks.values())
        report.update(state='pipeline_pass'if passed else'pipeline_failed_acceptance',full_task_pass=passed,continuous_cycle_pass=passed,final_checks=checks,cycle_time_s=t if passed else None)
        report['timing']['timing_is_measured_success']=passed
    except Exception:
        report.update(state='pipeline_failed',error=traceback.format_exc(),cycle_time_s=None)
        print(report['error'],flush=True)
    finally:
        if hasattr(audit,'save'):audit.save(out/'full-command-trace.npz')
        np.savez_compressed(out/'retargeted-command-primitives.npz',time_s=schedule.rings['time_s'],command=schedule.rings['command'])
        report.update(simulated_elapsed_s=t,clock_speed=speed,handoff_reroutes=reroutes,cross_arm_peak_contact_n=peak_cross,cross_arm_contacts=contacts,FR3_command_limit_audit=audit.report(),part_attachments=False,workpiece_teleports_after_initialization=0)
        if tape:np.savez_compressed(out/'physx-sequence-trace.npz',time_s=[v['t']for v in tape],q=[v['q']for v in tape],links=[v['links']for v in tape],parts=[v['parts']for v in tape],part_names=list(parts))
        (out/'pipeline-results.json').write_text(json.dumps(report,indent=2))
        sim.detach_stage()
    print('PIPELINE_RESULT',report['state'],flush=True)
