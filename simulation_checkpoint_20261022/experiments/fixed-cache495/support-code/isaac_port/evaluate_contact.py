"""Fail-closed acceptance checks on measured PhysX states and contact impulses."""
from pathlib import Path
import argparse,json,sys
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
SOURCE=HERE.parent/'keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/cap_button_parallel_alt_20261016'

def evaluate(out):
    trace=json.loads((out/'trace.json').read_text());contacts=json.loads((out/'contacts.json').read_text())
    status=json.loads((out/'status.json').read_text());manifest=json.loads((out/'run-manifest.json').read_text());recording=HERE/'generated/contact_task/fast'if manifest['arguments'].get('fast')else HERE/'generated/contact_task/retimed'if manifest['arguments'].get('retimed')else SOURCE;recording=out/'recording'if(out/'recording/trajectory.npz').exists()else recording;source=json.loads((recording/'config.json').read_text())
    schedule=json.loads((recording/'independent_schedule.json').read_text());recording=np.load(recording/'trajectory.npz')
    times=np.array([r['t']for r in trace]);ref=np.array([r['source_t']for r in trace]);pos=np.array([r['part_positions']for r in trace]);q=np.array([r['part_quaternions']for r in trace]);vel=np.array([r['part_velocities']for r in trace])
    parts={}
    roles={'ring1':('right','lift_two_parts_30mm','transfer_two_parts','release_two_parts'),'ring2':('left','lift_two_parts_30mm','transfer_two_parts','release_two_parts'),'pin':('left','lift_last_pin_30mm','transfer_last_pin','release_last_pin')}
    for i,name in enumerate(['ring1','ring2','pin']):
        side,lift,transfer,release=roles[name];lane=schedule['lanes'][side];delay=schedule['delays'][side]
        ref=np.array([r.get('source_times',{}).get(side,r['source_t'])for r in trace])
        lift_row=next(r for r in lane if r['stage']['label']==lift);transfer_row=next(r for r in lane if r['stage']['label']==transfer);release_row=next(r for r in lane if r['stage']['label']==release)
        released=np.flatnonzero(ref>=release_row['end_s']+delay-1e-6)
        release_time=float(times[released[0]]) if len(released) else None
        target=np.array(next(p['goal_origin_m']for p in source['parts']if p['name']==name))
        distance=np.linalg.norm(pos[:,i]-target,axis=1)
        expected=Rotation.from_quat(recording['mocap_quat'][-1,i,[1,2,3,0]])
        actual=Rotation.from_quat(q[:,i][:,[1,2,3,0]])
        angle=(expected*actual.inv()).magnitude()
        if name!='pin':angle=np.arccos(np.clip(actual.apply([0,0,1])@expected.apply([0,0,1]),-1,1))
        carrying=(ref>=lift_row['end_s']+delay+.03)&(ref<=transfer_row['end_s']+delay-.02)
        lift_height=float(np.max(pos[carrying,i,2]-pos[0,i,2]))if carrying.any()else 0
        per_hand={jaw:[]for jaw in ['left','right']};fixture=[];clamp_jaws={}
        for row in contacts:
            aa,bb=row['actor0'],row['actor1']
            if not (aa.endswith('/workpiece_'+name) or bb.endswith('/workpiece_'+name)):continue
            other=bb if aa.endswith('/workpiece_'+name)else aa
            for jaw in per_hand:
                if other.endswith('/'+side+'_grip_'+jaw+'_pad') and row['force_n']>.1:per_hand[jaw].append(row['time_s'])
            if ('/Environment/m80/' in other or '/World/Clamps/' in other) and row['force_n']>.02:fixture.append(row['time_s'])
            if other.startswith('/World/Clamps/'+name+'/')and row['force_n']>.5:clamp_jaws.setdefault(other,[]).append(row['time_s'])
        coverage=[]
        for t in times[carrying]:
            coverage.append(all(np.any(abs(np.array(v)-t)<.019)for v in per_hand.values()))
        bilateral_coverage=float(np.mean(coverage))if coverage else 0
        retention=(times>=release_time+.5)if release_time is not None else np.zeros(len(times),bool)
        if retention.any():
            max_pos=float(distance[retention].max());max_angle=float(angle[retention].max());max_speed=float(np.linalg.norm(vel[retention,i],axis=1).max())
            min_duration=float(times[-1]-(release_time+.5))
            fingers_clear=all(not np.any(np.array(v)>times[-1]-.5) for v in per_hand.values())
            fixture_support=any(t>times[-1]-.5 for t in fixture)
        else:max_pos=max_angle=max_speed=None;min_duration=0;fingers_clear=fixture_support=False
        final_clamp_count=sum(any(t>times[-1]-.5 for t in ts)for ts in clamp_jaws.values())
        sleeping_jaws=set()
        for row in trace[-1].get('active_clamp_contacts',[]):
            actors=[row['actor0'],row['actor1']]
            if not any(actor.endswith('/workpiece_'+name)for actor in actors):continue
            jaw=next((actor for actor in actors if actor.startswith('/World/Clamps/'+name+'/')),None)
            if jaw and row['last_force_n']>.5 and trace[-1].get('sleeping',{}).get(name) and trace[-1].get('sleeping',{}).get(jaw):sleeping_jaws.add(jaw)
        if len(sleeping_jaws)>=2:fixture_support=True;final_clamp_count=max(final_clamp_count,len(sleeping_jaws))
        clamp_verified=bool(trace[-1].get('clamp_verified',{}).get(name,False))
        passed=(clamp_verified and final_clamp_count>=2 and lift_height>.015 and bilateral_coverage>.8 and release_time is not None and min_duration>=1 and max_pos is not None and max_pos<.0008 and max_angle<.04 and max_speed<.005 and fingers_clear and fixture_support)
        parts[name]=dict(passed=bool(passed),lift_height_m=lift_height,bilateral_contact_fraction=bilateral_coverage,release_time_s=release_time,retention_s=min_duration,max_retained_position_error_m=max_pos,max_retained_rotation_error_rad=max_angle,max_retained_speed_m_s=max_speed,final_fingers_clear=bool(fingers_clear),final_fixture_contact=bool(fixture_support),final_contacting_clamp_jaws=final_clamp_count,clamp_sequence_verified=clamp_verified,persistent_sleeping_contact_jaws=len(sleeping_jaws),contact_semantics='Fresh impulses, or actual sleeping bodies with retained contact pairs and prior measured preload; LOST removes pair',final_position_error_m=float(distance[-1]))
    highrate=json.loads((out/'button-trace.json').read_text())if(out/'button-trace.json').exists()else None
    button_times=np.array([r['t']for r in highrate])if highrate else times
    travel=np.array([r['travel_m']for r in highrate])if highrate else np.array([r['button_travel_m']for r in trace]);pressed=travel>=.0018;dwell=0.;dwell_start=None;actuation=None
    for j,active in enumerate(pressed):
        if active:
            if dwell_start is None:dwell_start=button_times[j]
            dwell=button_times[j]-dwell_start
            # 100ms is exactly 48 simulation steps. Avoid a spurious extra
            # 30Hz frame when binary floating point represents 0.1 just below.
            if round(dwell*manifest['physics_hz'])>=round(.1*manifest['physics_hz']) and actuation is None:actuation=float(button_times[j])
        else:dwell_start=None
    button_force=[r for r in contacts if '/World/Button' in [r['actor0'],r['actor1']] and r['force_n']>.1]
    last_release=max((r['release_time_s'] or float('inf'))for r in parts.values())
    button_pass=(actuation is not None and actuation>last_release and len(button_force)>0 and float(travel.max())<.0025)
    button=dict(passed=bool(button_pass),max_travel_m=float(travel.max()),actuation_time_s=actuation,contact_samples=len(button_force),peak_contact_force_n=max((r['force_n']for r in button_force),default=0),no_early_actuation=bool(actuation is not None and actuation>last_release))
    unexpected=[]
    for row in contacts:
        aa,bb=row['actor0'],row['actor1'];other=bb if 'official_FR3_Duo'in aa else aa if 'official_FR3_Duo'in bb else ''
        cross_arms=('left_arm_base/'in aa and 'right_arm_base/'in bb)or('right_arm_base/'in aa and 'left_arm_base/'in bb)
        touched=next((name for name in ['ring1','ring2','pin']if aa.endswith('/workpiece_'+name)or bb.endswith('/workpiece_'+name)),None)
        robot_actor=aa if 'official_FR3_Duo'in aa else bb if 'official_FR3_Duo'in bb else ''
        intended_side='right'if touched=='ring1'else'left'
        wrong_workpiece_contact=bool(touched and robot_actor and not any(robot_actor.endswith('/'+intended_side+'_grip_'+jaw+'_pad')for jaw in ['left','right']))
        if row['time_s']>=.5 and (other.startswith(('/World/Environment/','/World/Clamps/'))or cross_arms or wrong_workpiece_contact):unexpected.append(row)
    peak_unexpected=max((r['force_n']for r in unexpected),default=0)
    contact_audit=dict(passed=peak_unexpected<=20,peak_unintended_robot_contact_n=peak_unexpected,limit_n=20,contact_samples=len(unexpected),peak_pair=max(unexpected,key=lambda r:r['force_n'])if unexpected else None)
    manifest=json.loads((out/'run-manifest.json').read_text());args=manifest['arguments'];full_scope=not(args.get('only_pin')or args.get('partial_env')or args.get('start_source',0)!=0) and args.get('until',float(recording['time_s'][-1]))>=float(recording['time_s'][-1])-1e-8
    cycle_time=actuation-manifest['arguments']['settle']if actuation is not None else None
    target_15=bool(args.get('target_15'));target_10=bool(args.get('target_10'));cycle_target=10. if target_10 else 15. if target_15 else 8.0603333334
    cycle_pass=(cycle_time is not None and cycle_time<cycle_target)if target_15 or target_10 else not args.get('fast')or(cycle_time is not None and cycle_time<=cycle_target)
    fr3_required=bool(args.get('fr3_limits'))
    fr3_report=json.loads((out/'fr3-limits.json').read_text())if(out/'fr3-limits.json').exists()else None
    fr3_pass=bool(fr3_report and fr3_report['fr3_kinematic_limits_pass'])if fr3_required else None
    passed=(cycle_pass or(fr3_required and not(target_15 or target_10))) and (not fr3_required or fr3_pass) and full_scope and contact_audit['passed'] and status['state']=='completed_probe' and all(r['passed']for r in parts.values()) and button_pass
    result=dict(cycle_time_s=cycle_time,cycle_target_s=cycle_target if args.get('fast')else None,cycle_time_pass=bool(cycle_pass),full_task_scope=full_scope,state='passed'if passed else'failed',mode='physx_dynamic_contact_task',robot_contact_audit=contact_audit,parts=parts,button=button,physical_simulation_pass=bool(passed),hardware_assembly_certified=False,source_cad_nominal_fit_certified=False,physical_duration_s=float(times[-1]),sample_count=len(trace))
    result.update(fr3_limits_required=fr3_required,fr3_kinematic_limits_pass=fr3_pass,cycle_time_required=target_15 or target_10 or not fr3_required,cycle_time_policy=f'Strictly below {cycle_target} seconds and FR3 limits both required'if target_15 or target_10 else'FR3 limits take priority; original 8.060s target reported separately'if fr3_required else'Original timing target required')
    (out/'acceptance.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);a=p.parse_args();result=evaluate(a.run);print(json.dumps(result,indent=2,allow_nan=False));sys.exit(0 if result['physical_simulation_pass']else 1)
