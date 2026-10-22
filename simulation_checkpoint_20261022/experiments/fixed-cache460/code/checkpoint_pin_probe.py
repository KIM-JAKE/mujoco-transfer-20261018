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
    try:
        report['dof_names']=names;report['link_names']=links;report['parts']={'pin':{}};hold(.5,'checkpoint_settle')
        def set_phase(label):
            nonlocal phase
            phase=label
        from pin_finish_physx import execute as sequence
        context=dict(stage=stage,paths=paths,hands=hands,drives=drives,qcurrent=qcurrent,initial_pose=initial_pose,report=report,OUT=OUT,HERE=HERE,plan=plan,view=view,art=art,idx=idx,names=names,links=links,aidx=aidx,fidx=fidx,hidx=hidx,cmd=cmd,step=step,hold=hold,set_phase=set_phase,now=lambda:t,current_contacts=current_contacts,contacts=contacts,dt=dt)
        report['sequence']=sequence(context);report['state']=report['sequence']['state'];report['full_task_pass']=False;report['final_assembly_pass']=report['sequence'].get('final_assembly_pass',False)
    finally:
        if trace:np.savez_compressed(OUT/'physx-pickup-trace.npz',time_s=[r['time_s']for r in trace],phase=[r['phase']for r in trace],q=[r['q']for r in trace],qv=[r['qv']for r in trace],links=[r['links']for r in trace],parts=[r['parts']for r in trace],command=[r['command']for r in trace])
        report['FR3_command_limit_audit']=motion_audit.report();(OUT/'physx-pickup-results.json').write_text(json.dumps(report,indent=2));sim.detach_stage()
