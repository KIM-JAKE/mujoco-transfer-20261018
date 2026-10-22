"""Keep validated M80 contact clocks and retime only two free-motion intervals.

Input is a restored model600-overlap measured run. A new output is offline
planning data, never hardware authorization or a physical acceptance result.
"""
from pathlib import Path
import argparse, copy, hashlib, json, math, shutil, subprocess, sys
import numpy as np

PORT = Path(__file__).resolve().parent


def prepare(baseline, output, *, model_budget=800., free_time_scale=1., respect_cutoff=False):
    baseline, output = Path(baseline).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('Use a fresh profile directory')
    if not math.isfinite(model_budget) or not 600 <= model_budget <= 1000:
        raise ValueError('Free-motion model budget must be finite and in [600,1000]')
    if not math.isfinite(free_time_scale) or free_time_scale < 1:
        raise ValueError('Free-motion time scaling may only slow the generated clocks')
    source_meta = json.loads((baseline/'fr3-concurrent-profile.json').read_text())
    if source_meta.get('model_torque_rate_planning_limit_nm_s') != 600 or source_meta.get('conservative_ring_handoff'):
        raise ValueError('Expected the original-layout model600-overlap profile')
    recording = baseline/'recording' if (baseline/'recording').exists() else baseline
    with np.load(baseline/'fr3-concurrent-profile.npz') as z:
        source = {n:z[n].copy() for n in z.files}
    output.mkdir(parents=True)
    stage = output/'planning-base';(stage/'recording').mkdir(parents=True)
    for name in ['trajectory.npz','config.json','independent_schedule.json','retiming.json']:
        shutil.copy2(recording/name,output/name);shutil.copy2(recording/name,stage/'recording'/name)
    for name in ['arm-posture-fr3.npz','arm-posture-fr3.json']:
        shutil.copy2(baseline/name,output/name);shutil.copy2(baseline/name,stage/name)
    names = baseline/'dof-names.json'
    shutil.copy2(names if names.exists() else PORT/'assets/reference-dof-names.json',stage/'dof-names.json')
    args = ['--base',stage,'--output',output/'fr3-concurrent-profile.npz',
            '--aggressive-contact-timing','--coordinated','--left-ring-clear','1.96',
            '--right-ring-clear','3.4','--left-pin-time-reserve','1.04',
            '--left-pin-lift-time-reserve','1.15','--right-return-time-reserve','1.1',
            '--pin-prealignment','--strict-clamp-alignment','--pin-approach-min','.4',
            '--pin-fine-min','.35','--button-min','.2','--model-torque-rate',str(model_budget)]
    if respect_cutoff:args.append('--respect-cutoff')
    with (output/'planning.log').open('w') as stream:
        subprocess.run([sys.executable,str(PORT/'prepare_concurrent_timing.py'),*map(str,args)],
                       stdout=stream,stderr=subprocess.STDOUT,check=True)
    profile = output/'fr3-concurrent-profile.npz'
    with np.load(profile) as z:data = {n:z[n].copy() for n in z.files}
    meta = json.loads(profile.with_suffix('.json').read_text())
    if not np.array_equal(source['fingerprint'],data['fingerprint']):
        raise RuntimeError('Source and target reference geometry differ')
    for key in ['planning_fraction','coordination_rules','strict_clamp_alignment',
                'pin_prealignment_reference','contact_minimum_s',
                'gripper_travel_driver_cap_rad_s','pin_early_close_driver_cap_rad_s']:
        if source_meta[key] != meta[key]:raise RuntimeError('Clock constraint changed: '+key)
    changes=[];budgets={}
    for side in ['left','right']:
        if not np.array_equal(source[side+'_bounds'],data[side+'_bounds']):
            raise RuntimeError('Clock boundaries differ')
        bounds=data[side+'_bounds'];budgets[side]=[]
        for i,(begin,end) in enumerate(zip(bounds[:-1],bounds[1:])):
            free=(side=='left' and abs(begin-3.357916666666667)<1e-8 and abs(end-5.107916666666667)<1e-8) or (
                  side=='right' and abs(begin-3.897916666666667)<1e-8 and abs(end-6.447975146198832)<1e-8)
            budgets[side].append(model_budget if free else 600.)
            if not free:
                for key in ['x','c','scale']:data[f'{side}_clock_{i}_{key}']=source[f'{side}_clock_{i}_{key}'].copy()
                data[side+'_durations'][i]=source[side+'_durations'][i]
                meta['segments'][side][i]=copy.deepcopy(source_meta['segments'][side][i])
            else:
                data[f'{side}_clock_{i}_scale']*=free_time_scale;data[side+'_durations'][i]*=free_time_scale
                row=meta['segments'][side][i]
                for key in ['duration_s','time_scale']:row[key]*=free_time_scale
                row['peak_acceleration_rad_s2']/=free_time_scale**2;row['peak_jerk_rad_s3']/=free_time_scale**3
                row['minimum_phase_rate']/=free_time_scale
                changes.append(dict(side=side,segment=i,begin=float(begin),end=float(end)))
            meta['segments'][side][i]['planning_model_budget_nm_s']=budgets[side][-1]
    if len(changes)!=2:raise RuntimeError('Expected exactly two free-motion intervals')
    np.savez_compressed(profile,**data)
    meta.update(state='offline_free_motion_plan_requires_nominal_and_PhysX_checks',hardware_ready=False,
                side_nominal_s={s:float(data[s+'_durations'].sum()) for s in ['left','right']},
                archive_sha256=hashlib.sha256(profile.read_bytes()).hexdigest(),
                planning_model_budget_by_segment_nm_s=budgets,free_time_scale=free_time_scale,
                free_segments=changes,generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    profile.with_suffix('.json').write_text(json.dumps(meta,indent=2)+'\n')
    return meta


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--model-budget',type=float,default=800.)
    p.add_argument('--free-time-scale',type=float,default=1.);p.add_argument('--respect-cutoff',action='store_true')
    a=p.parse_args();r=prepare(a.baseline,a.output,model_budget=a.model_budget,free_time_scale=a.free_time_scale,respect_cutoff=a.respect_cutoff)
    print(json.dumps(dict(state=r['state'],side_nominal_s=r['side_nominal_s'],hardware_ready=False),indent=2))
