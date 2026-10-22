"""Trim selected clock durations while preserving every other segment exactly.

New contact-time heuristics are experimental. FR3/PBV/a/jerk and gripper caps
are densely rechecked; actual physical assembly still requires a new test.
"""
from pathlib import Path
from types import SimpleNamespace
import argparse,copy,hashlib,json,shutil
import numpy as np
from scipy.interpolate import CubicSpline,PPoly
import prepare_fr3_timing as timing
from optimization_clock import require_optimization_window,install_cutoff_alarm


def prepare(base,output,changes,minimums,respect_cutoff=False):
    if respect_cutoff:require_optimization_window(120);install_cutoff_alarm()
    if output.resolve()==base.resolve()or output.exists():raise ValueError('Use a fresh candidate directory')
    root=Path(__file__).parent;recording=base if(base/'trajectory.npz').exists()else base/'recording'
    z=np.load(recording/'trajectory.npz');plan=np.load(base/'arm-posture-fr3.npz')
    names=json.loads((base/'dof-names.json'if(base/'dof-names.json').exists()else root/'assets/reference-dof-names.json').read_text())
    source=json.loads((root/'generated/contact_task/contact-source.json').read_text());lookup={j['name']:j['qpos_address']for j in source['joints']};q=z['qpos'][:,[lookup[n]for n in names]].copy()
    for i,n in enumerate(names):
        if '_arm_'in n:q[:,i]=plan[n.split('_')[0]+'_qpos'][:,int(n[-1])-1]
    metadata=json.loads((base/'fr3-concurrent-profile.json').read_text());timing.CONTACT_MINIMUMS.update(metadata['contact_minimum_s']);timing.PATH_ACCELERATION_LIMIT=metadata.get('path_acceleration_limit_rad_s2',9.1)
    for key,value in minimums.items():
        if value is not None:
            if not np.isfinite(value)or value<=0:raise ValueError('Positive finite contact duration required')
            timing.CONTACT_MINIMUMS[key]=value
    timing.MAX_PHASE_RATE=metadata['maximum_source_phase_rate'];timing.GRIPPER_TRAVEL_CAP=metadata['gripper_travel_driver_cap_rad_s'];timing.PIN_EARLY_CLOSE_CAP=metadata['pin_early_close_driver_cap_rad_s']
    with np.load(base/'fr3-concurrent-profile.npz')as archive:data={n:archive[n].copy()for n in archive.files}
    schedule=json.loads((recording/'independent_schedule.json').read_text());indices=[names.index(f'{s}_arm_fr3v2_joint{j}')for s in ['left','right']for j in range(1,8)];updates=[]
    for side,segment,duration in changes:
        if not np.isfinite(duration)or duration<=0:raise ValueError('Positive finite clock duration required')
        masked=q.copy();other='right'if side=='left'else'left'
        for i,n in enumerate(names):
            if n.startswith(other+'_'):masked[:,i]=masked[0,i]
        local=copy.deepcopy(schedule)
        for row in local['lanes'][other]:row['start_s']+=1000;row['end_s']+=1000
        profile=SimpleNamespace(path=CubicSpline(z['time_s'],masked[:,indices],axis=0),full_path=CubicSpline(z['time_s'],masked,axis=0),driver_indices=[names.index(s+'_grip_left_driver_joint')for s in ['left','right']])
        begin,end=data[side+'_bounds'][segment:segment+2];prefix=f'{side}_clock_{segment}_';clock=PPoly(data[prefix+'c'],data[prefix+'x']);old_scale=float(data[prefix+'scale']);old_duration=float(data[side+'_durations'][segment])
        check=np.unique(np.r_[clock.x,np.linspace(clock.x[0],clock.x[-1],max(16001,int(clock.x[-1]*6000)))])
        ratios,acc,jerk=timing.planning_ratios(profile,clock,check,begin,end,local)
        required=max(float(ratios.max())*1.015,timing.contact_minimum(begin,local)/clock.x[-1])
        scale=max(duration/clock.x[-1],required)
        data[prefix+'scale']=np.array(scale);data[side+'_durations'][segment]=clock.x[-1]*scale
        report=metadata['segments'][side][segment];report.update(duration_s=float(clock.x[-1]*scale),time_scale=scale,peak_acceleration_rad_s2=float(abs(acc).max()/scale**2),peak_jerk_rad_s3=float(abs(jerk).max()/scale**3),minimum_phase_rate=float(clock(check,1).min()/scale))
        update=dict(side=side,segment=segment,requested_duration_s=duration,previous_duration_s=old_duration,duration_s=report['duration_s'],scale_ratio=scale/old_scale,FR3_dense_motion_screen_pass=True)
        updates.append(update);print(json.dumps(update),flush=True)
    output.mkdir(parents=True)
    np.savez_compressed(output/'fr3-concurrent-profile.npz',**data)
    metadata.update(state='selected_clocks_trimmed_requires_new_PhysX',hardware_ready=False,archive_sha256=hashlib.sha256((output/'fr3-concurrent-profile.npz').read_bytes()).hexdigest(),clock_trims=updates,contact_minimum_s=dict(timing.CONTACT_MINIMUMS),side_nominal_s={s:float(data[s+'_durations'].sum())for s in ['left','right']},generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (output/'fr3-concurrent-profile.json').write_text(json.dumps(metadata,indent=2)+'\n')
    for n in ['arm-posture-fr3.npz','arm-posture-fr3.json']:shutil.copy2(base/n,output/n)
    for n in ['trajectory.npz','config.json','independent_schedule.json','retiming.json']:shutil.copy2(recording/n,output/n)
    stage=output/'planning-base';(stage/'recording').mkdir(parents=True)
    for n in ['arm-posture-fr3.npz','arm-posture-fr3.json']:shutil.copy2(output/n,stage/n)
    for n in ['trajectory.npz','config.json','independent_schedule.json','retiming.json']:shutil.copy2(output/n,stage/'recording'/n)
    shutil.copy2(root/'assets/reference-dof-names.json',stage/'dof-names.json')
    print(json.dumps(metadata['side_nominal_s']))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--left',action='append',default=[]);p.add_argument('--right',action='append',default=[]);p.add_argument('--pin-approach-min',type=float);p.add_argument('--pin-fine-min',type=float);p.add_argument('--button-min',type=float);p.add_argument('--respect-cutoff',action='store_true');a=p.parse_args()
    changes=[]
    for side,values in [('left',a.left),('right',a.right)]:
        for value in values:index,duration=value.split(':');changes.append((side,int(index),float(duration)))
    prepare(a.base,a.output,changes,dict(insert_last_pin_approach=a.pin_approach_min,insert_last_pin_last5mm=a.pin_fine_min,press_button=a.button_min),a.respect_cutoff)
