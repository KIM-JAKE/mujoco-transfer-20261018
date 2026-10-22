"""Physical contact diagnostic. Never attaches or kinematically moves a part."""
import json,math,traceback,hashlib
import numpy as np
import omni.physx
import omni.physics.tensors as tensors
from pxr import UsdUtils,PhysicsSchemaTools

def rotation(q):
    x,y,z,w=map(float,q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def execute(stage,paths,hands,drives,qcurrent,initial_pose,report,OUT,HERE,plan_path=None):
    plan_path=plan_path or HERE/'pickup-probe-plan.json'
    plan=json.load(open(plan_path))
    (OUT/'executed-pickup-plan.json').write_text(json.dumps(plan,indent=2))
    (OUT/'executed-initial-robot-pose.json').write_text(json.dumps(initial_pose,indent=2))
    report['executed_plan_sha256']=hashlib.sha256(plan_path.read_bytes()).hexdigest()
    report.update(scope=plan['scope'],state='running',pickup_pass=False,full_task_pass=False,cycle_time_s=None,gripper_force_limit_per_finger_n=30,gripper_total_aperture_speed_limit_m_s=.095)
    report['servo_control']='Position and command-velocity targets; finite effort and velocity limits'
    cache=UsdUtils.StageCache.Get();ident=cache.Insert(stage).ToLongInt()
    sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);dt=1/plan.get('simulation_hz',480);t=0.;report['simulation_timestep_s']=dt
    sim.simulate(dt,t);sim.fetch_results();t+=dt
    view=tensors.create_simulation_view('numpy',ident);art=view.create_articulation_view('/World/RobotMountJoint');idx=np.array([0],np.uint32)
    names=list(art.shared_metatype.dof_names);links=list(art.shared_metatype.link_names)
    cmd=np.array([[qcurrent[n]if n in qcurrent else plan['initial_apertures_m'][n.split('_')[0]]/2 for n in names]],np.float32)
    art.set_dof_positions(cmd,idx);art.set_dof_velocities(np.zeros_like(cmd),idx);art.set_dof_position_targets(cmd,idx)
    art.set_dof_stiffnesses(np.array([[12000 if n in drives else 5000 for n in names]],np.float32),idx)
    art.set_dof_dampings(np.array([[600 if n in drives else 100 for n in names]],np.float32),idx)
    art.set_dof_max_forces(np.array([[(87 if int(n[-1])<=4 else 12)*.95 if n in drives else 30 for n in names]],np.float32),idx)
    aidx={s:[names.index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']}
    fidx={s:[i for i,n in enumerate(names)if n.startswith(s+'_native_')]for s in ['left','right']}
    hidx={s:links.index(s+'_NativeHand')for s in ['left','right']}
    part_order=list(plan['parts'])
    parts={p:view.create_rigid_body_view(paths['workpiece_'+p])for p in part_order}
    initial_parts={p:v.get_transforms()[0].copy()for p,v in parts.items()}
    contacts={};current_contacts={};trace=[];phase='settle';events=[]
    def callback(headers,data):
        for h in headers:
            if not h.num_contact_data:continue
            actors=tuple(str(PhysicsSchemaTools.intToSdfPath(v))for v in [h.actor0,h.actor1])
            if not any('official_FR3_Duo'in a for a in actors):continue
            pair=tuple(sorted(str(PhysicsSchemaTools.intToSdfPath(v))for v in [h.collider0,h.collider1]))
            force=sum(float(np.linalg.norm(data[i].impulse))/dt for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
            if force<.05:continue
            old=contacts.setdefault(pair,dict(colliders=list(pair),actors=list(actors),first_time_s=t,peak_force_n=0.,phases=[]))
            old['peak_force_n']=max(old['peak_force_n'],force)
            if phase not in old['phases']:old['phases'].append(phase)
            current_contacts[pair]=force
    subscription=sim.subscribe_contact_report_events(callback)
    def snapshot():
        return dict(time_s=t,phase=phase,q=art.get_dof_positions()[0].copy(),qv=art.get_dof_velocities()[0].copy(),links=art.get_link_transforms()[0].copy(),parts=np.array([parts[p].get_transforms()[0]for p in part_order]),command=cmd[0].copy(),contact_pairs=list(current_contacts))
    previous_command=cmd.copy()
    from isaac_motion import duration as motion_duration,Audit
    motion_audit=Audit(cmd,aidx,dt)
    def step():
        nonlocal t
        motion_audit.update(cmd)
        current_contacts.clear();art.set_dof_position_targets(cmd,idx);art.set_dof_velocity_targets((cmd-previous_command)/dt,idx);previous_command[:]=cmd;sim.simulate(dt,t);sim.fetch_results();t+=dt
        root=art.get_root_transforms()[0]
        if not np.isfinite(art.get_dof_positions()).all()or np.linalg.norm(root[:3]-np.array(report['placement']['duo_world_position']))>.02:raise RuntimeError('Invalid articulated state/base drift')
        if round(t/dt)%8==0:trace.append(snapshot())
    def hold(duration,label):
        nonlocal phase
        phase=label
        for _ in range(round(duration/dt)):step()
        events.append(dict(time_s=t,event=label,parts={p:v.get_transforms()[0].tolist()for p,v in parts.items()}))
        print('NATIVE_PICKUP_PHASE',label,round(t,3),flush=True)
    def move(up,duration,label):
        nonlocal phase
        phase=label
        duration=max(motion_duration(r['cartesian_lift_q'],duration)for r in plan['parts'].values())
        for k in range(round(duration/dt)):
            u=min(1.,(k+1)*dt/duration);h=u**3*(10-15*u+6*u*u)
            for part,r in plan['parts'].items():
                path=np.array(r['cartesian_lift_q']);fraction=h if up else 1-h
                knots=np.linspace(0.,1.,len(path))
                j=min(len(path)-2,max(0,int(np.searchsorted(knots,fraction,side='right')-1)))
                w=float(np.clip((fraction-knots[j])/(knots[j+1]-knots[j]),0,1))
                cmd[0,aidx[r['side']]]=path[j]*(1-w)+path[j+1]*w
            step()
        hold(.25,label+'_settled')
    def relative(part):
        side=plan['parts'][part]['side'];hand=art.get_link_transforms()[0,hidx[side]];body=parts[part].get_transforms()[0]
        return rotation(hand[3:]).T@(body[:3]-hand[:3])
    def relative_rotation(part):
        side=plan['parts'][part]['side'];hand=art.get_link_transforms()[0,hidx[side]];body=parts[part].get_transforms()[0]
        return rotation(hand[3:]).T@rotation(body[3:])
    try:
        hold(2.0 if plan.get('clutter_items')else .5,'settle');settled={p:v.get_transforms()[0].copy()for p,v in parts.items()}
        if plan.get('capture_camera'):
            from isaac_cpu_camera import capture
            report['camera']=capture(stage,OUT)
        if plan.get('pin_approach_requires_query'):
            from isaac_kinematics import Arm
            from isaac_collision_queries import Checker
            row=plan['parts']['pin'];side=row['side'];current=art.get_dof_positions()[0,aidx[side]];kin=Arm(stage,side,drives,current);checker=Checker(stage,kin,side)
            route=checker.route(current,row['cartesian_lift_q'][-1],plan['initial_apertures_m'][side],paths['workpiece_pin'])
            if route is None:raise RuntimeError('No structure-clear pin approach found in live Isaac PhysX scene')
            ok,why=checker.path(list(reversed(row['cartesian_lift_q'])),plan['initial_apertures_m'][side],paths['workpiece_pin'])
            if not ok:raise RuntimeError('Pin descent structural collision '+str(why))
            plan['initial_transition_q']=[q.tolist()for q in route]
            (OUT/'executed-pickup-plan.json').write_text(json.dumps(plan,indent=2))
        if plan.get('clutter_items'):
            if plan.get('fixed_demo_grasp'):
                mapping=json.load(open(OUT/'clutter-body-paths.json'));report['clutter_settled_objects']=[dict(id=row['id'],part=row['part'],path=mapping[row['id']],transform=view.create_rigid_body_view(mapping[row['id']]).get_transforms()[0].tolist())for row in plan['clutter_items']];selected={p:dict(path=paths['workpiece_'+p])for p in plan['parts']};report.update(perception_mode='Ideal Isaac body poses; curated one-scene demonstration',vision_end_to_end_pass=False,clutter_counts=plan['clutter_counts'],clutter_count=len(mapping))
            else:
                from bin_arm_task import select_clutter_grasp
                selected=select_clutter_grasp(stage,paths,view,art,aidx,drives,plan,report,OUT)
            for p,row in selected.items():
                paths['workpiece_'+p]=row['path'];parts[p]=view.create_rigid_body_view(row['path']);initial_parts[p]=parts[p].get_transforms()[0].copy();settled[p]=initial_parts[p].copy()
            starts=cmd.copy();T=max(abs(float(cmd[0,fidx[r['side']]].sum())-plan['initial_apertures_m'][r['side']])/.095 for r in plan['parts'].values())
            phase='clutter_grasp_preshape'
            for k in range(max(1,math.ceil(T/dt))):
                u=min(1.,(k+1)*dt/max(T,dt))
                for r in plan['parts'].values():
                    side=r['side'];cmd[0,fidx[side]]=starts[0,fidx[side]]*(1-u)+plan['initial_apertures_m'][side]/2*u
                step()
            (OUT/'executed-pickup-plan.json').write_text(json.dumps(plan,indent=2))
        if plan.get('initial_arm_transition'):
            phase='approach_to_pregrasp';begin=cmd.copy();T=4.
            if plan.get('initial_transition_q'):
                side=next(iter(plan['parts'].values()))['side'];plan['initial_transition_q'][0]=begin[0,aidx[side]].tolist()
            if plan.get('initial_transition_q'):T=8.
            if plan.get('pin_approach_requires_query'):
                transition=np.array(plan['initial_transition_q']);phase='approach_to_pregrasp'
                for a,b in zip(transition[:-1],transition[1:]):
                    segment_T=motion_duration([a,b],max(.35,T/(len(transition)-1)));count=max(1,math.ceil(segment_T/dt))
                    for k in range(count):
                        u=(k+1)/count;h=u**3*(10-15*u+6*u*u);cmd[0,aidx[side]]=a+(b-a)*h;step()
                plan['initial_transition_q']=[];begin=cmd.copy();T=.1
            T=max(motion_duration(plan.get('initial_transition_q')or[begin[0,aidx[r['side']]].tolist(),r['cartesian_lift_q'][-1]],T)for r in plan['parts'].values())
            for k in range(round(T/dt)):
                u=(k+1)*dt/T;h=u**3*(10-15*u+6*u*u)
                for r in plan['parts'].values():
                    if plan.get('initial_transition_q'):
                        path=np.array(plan['initial_transition_q']);x=h*(len(path)-1);j=min(len(path)-2,int(x));w=x-j;cmd[0,aidx[r['side']]]=path[j]*(1-w)+path[j+1]*w
                    else:cmd[0,aidx[r['side']]]=begin[0,aidx[r['side']]]*(1-h)+np.array(r['cartesian_lift_q'][-1])*h
                step()
            hold(.3,'pregrasp_ready')
        move(False,plan.get('diagnostic_descend_duration_s',2.),'descend')
        phase='close';starts=cmd.copy()
        compression=plan.get('grasp_command_compression_m',.004)
        close_duration=max(abs(plan['initial_apertures_m'][r['side']]-(r['recipe']['gap']+(compression if r['recipe'].get('grasp_mode')=='internal_expansion'else-compression)))/.095 for r in plan['parts'].values())
        for k in range(math.ceil(close_duration/dt)):
            for r in plan['parts'].values():
                side=r['side'];target=max(0.,r['recipe']['gap']+(compression if r['recipe'].get('grasp_mode')=='internal_expansion'else-compression))/2
                cmd[0,fidx[side]]=starts[0,fidx[side]]+np.clip(target-starts[0,fidx[side]],-.0475*(k+1)*dt,.0475*(k+1)*dt)
            step()
        hold(.4,'grasp_hold');grasp_relative={p:relative(p).copy()for p in plan['parts']};grasp_rotations={p:relative_rotation(p).copy()for p in plan['parts']};grasp_positions={p:parts[p].get_transforms()[0].copy()for p in plan['parts']}
        move(True,plan.get('diagnostic_lift_duration_s',3.),'lift');hold(1.,'lift_hold')
        results={}
        for part,r in plan['parts'].items():
            final=parts[part].get_transforms()[0];rise=float(final[2]-settled[part][2]);slip=float(np.linalg.norm(relative(part)-grasp_relative[part]));side=r['side']
            pair_contacts=[c for c in contacts.values()if any(paths['workpiece_'+part]in a for a in c['actors'])and any(side+'_NativeHand'in a for a in c['actors'])]
            finger_actors={a for c in pair_contacts for a in c['actors']if a.endswith('_finger')}
            actual_width=float(np.sum(art.get_dof_positions()[0,fidx[side]]))
            rr=relative_rotation(part)@grasp_rotations[part].T;angular_slip=math.degrees(math.acos(np.clip((np.trace(rr)-1)/2,-1,1)))
            normal_slip=math.degrees(math.acos(np.clip(relative_rotation(part)[:,2]@grasp_rotations[part][:,2],-1,1)))
            relevant_angle=normal_slip if part.startswith('ring')else angular_slip
            stable=[x for x in trace if x['phase']=='lift_hold'and x['time_s']>=t-.5];body_index=part_order.index(part);rel_samples=[];rotation_samples=[]
            for sample in stable:
                h=sample['links'][hidx[side]];p=sample['parts'][body_index];rel_samples.append(rotation(h[3:]).T@(p[:3]-h[:3]));rotation_samples.append(rotation(h[3:]).T@rotation(p[3:]))
            stable_drift=float(max(np.linalg.norm(v-rel_samples[-1])for v in rel_samples))if rel_samples else float('inf')
            stable_tilt=float(max(math.degrees(math.acos(np.clip(v[:,2]@rotation_samples[-1][:,2],-1,1)))for v in rotation_samples))if rotation_samples else float('inf')
            final_bilateral=[]
            for sample in stable:
                names_now={side+'_'+label+'_finger'for label in ['left','right']if any(paths['workpiece_'+part]in a and side+'_'+label+'_finger'in b for pair in sample['contact_pairs']for a,b in [pair,pair[::-1]])}
                final_bilateral.append(len(names_now)==2)
            bilateral_fraction=float(np.mean(final_bilateral))if final_bilateral else 0.
            fixed_recipe_pass=rise>r.get('lift_m',plan['lift_m'])-.004 and slip<.003 and relevant_angle<2 and len(finger_actors)>=2
            relocalized_bin_pass=rise>plan.get('minimum_lift_m',.12)and stable_drift<.0005 and stable_tilt<.5 and len(finger_actors)>=2
            relocalized_bin_pass=relocalized_bin_pass and bilateral_fraction>.9
            bin_clearance=None
            if plan.get('clutter_items')and part.startswith('ring'):
                archive=np.load(HERE.parents[1]/'generated/contact_task/precision-cad.npz');key='instance_0542'if part=='ring1'else'instance_0549';vertices=archive[key+'_vertices'];low=float(np.min(vertices@rotation(final[3:]).T+final[:3],axis=0)[2]);bin_clearance=low-1.210918206667893
                if not plan.get('fixed_demo_grasp'):relocalized_bin_pass=relocalized_bin_pass and bin_clearance>.003
            pickup_ok=relocalized_bin_pass if plan.get('acceptance_mode')=='bin_pick_relocalize_after_lift'else fixed_recipe_pass
            results[part]=dict(lift_m=rise,grasp_relative_translation_slip_m=slip,grasp_relative_rotation_slip_deg=angular_slip,grasp_relative_axis_tilt_slip_deg=normal_slip,rotation_acceptance='Ring normal tilt; axial yaw is rotationally symmetric and is reported separately'if part.startswith('ring')else'Full rigid orientation',bilateral_finger_contact=len(finger_actors)>=2,actual_final_aperture_m=actual_width,final_transform=final.tolist(),pass_pickup=pickup_ok,fixed_recipe_pose_retention_pass=fixed_recipe_pass,late_hold_relative_drift_m=stable_drift,late_hold_axis_tilt_variation_deg=stable_tilt,pose_update_required_before_transport=not fixed_recipe_pass,acceptance_mode=plan.get('acceptance_mode','fixed_recipe_pose_retention'),nominal_loaded_CAD_screen_pass=r['nominal_loaded_clear'])
            results[part]['minimum_part_to_highest_basket_rim_vertical_clearance_m']=bin_clearance
            results[part]['late_hold_bilateral_contact_fraction']=bilateral_fraction
        def intended(c):
            if plan.get('clutter_items')and any(a.startswith('/World/BinStock/')or a in ['/World/Cell/Geometry/workpiece_ring1','/World/Cell/Geometry/workpiece_ring2']for a in c['actors'])and any(a.endswith('_finger')for a in c['actors']):return True
            return any(any(paths['workpiece_'+p]==a for a in c['actors'])and any(a.startswith(hands[r['side']]+'/')and a.endswith('_finger')for a in c['actors'])for p,r in plan['parts'].items())
        unexpected=[c for c in contacts.values()if not intended(c)and c['peak_force_n']>5]
        if plan.get('clutter_items'):
            stock_final=[];extra_lifts=[];selected_paths={paths['workpiece_'+p]for p in plan['parts']}
            for row in report['clutter_settled_objects']:
                final=view.create_rigid_body_view(row['path']).get_transforms()[0];rise_other=float(final[2]-row['transform'][2]);stock_final.append(dict(id=row['id'],part=row['part'],path=row['path'],transform=final.tolist(),rise_m=rise_other,selected=row['path']in selected_paths))
                if row['path']not in selected_paths and rise_other>.025:extra_lifts.append(row['id'])
            report.update(clutter_final_objects=stock_final,unselected_objects_lifted=extra_lifts,single_object_pick_verified=not extra_lifts)
            if extra_lifts:
                for row in results.values():row['pass_pickup']=False
        grasp_pass=all(r['pass_pickup']for r in results.values());contact_gate=not unexpected or plan.get('quick_dirty_demo',False)
        report.update(state='pickup_pass_full_task_pending'if grasp_pass and contact_gate else'pickup_diagnostic_failed',pickup_pass=grasp_pass and contact_gate,parts=results,unexpected_robot_contacts=unexpected,incidental_contacts_are_diagnostic_only=plan.get('quick_dirty_demo',False),events=events,robot_contacts=list(contacts.values()),dof_names=names,link_names=links,diagnostic_duration_s=t,initial_part_transforms={p:v.tolist()for p,v in initial_parts.items()})
        if report['pickup_pass']and plan.get('continue_sequence'):
            from bin_sequence_physx import execute as sequence
            def set_phase(label):
                nonlocal phase
                phase=label
            context=dict(stage=stage,paths=paths,hands=hands,drives=drives,qcurrent=qcurrent,initial_pose=initial_pose,report=report,OUT=OUT,HERE=HERE,plan=plan,view=view,art=art,idx=idx,names=names,links=links,aidx=aidx,fidx=fidx,hidx=hidx,cmd=cmd,step=step,hold=hold,set_phase=set_phase,now=lambda:t,current_contacts=current_contacts,contacts=contacts,dt=dt)
            report['sequence']=sequence(context);report['full_task_pass']=report['sequence'].get('full_task_pass',False);report['state']=report['sequence']['state']
    except Exception:
        report.update(state='invalid_or_aborted_diagnostic',pickup_pass=False,error=traceback.format_exc(),robot_contacts=list(contacts.values()))
        raise
    finally:
        if trace:
            np.savez_compressed(OUT/'physx-pickup-trace.npz',time_s=[r['time_s']for r in trace],phase=[r['phase']for r in trace],q=[r['q']for r in trace],qv=[r['qv']for r in trace],links=[r['links']for r in trace],parts=[r['parts']for r in trace],command=[r['command']for r in trace])
        (OUT/'physx-pickup-results.json').write_text(json.dumps(report,indent=2));sim.detach_stage()
        report['FR3_command_limit_audit']=motion_audit.report();(OUT/'physx-pickup-results.json').write_text(json.dumps(report,indent=2))
    print('NATIVE_PICKUP_DONE',report['state'],results,flush=True)
