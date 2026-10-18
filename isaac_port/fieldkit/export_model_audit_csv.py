"""Export exact C2 mathematical waypoints. Never authorizes robot motion.

The SDK audit samples the resulting trajectory at 1kHz. Preserve the original
knots: resampling and fitting again can introduce extra interpolation extrema.
"""
from pathlib import Path
import argparse,csv,hashlib,json,math


def export(output,*,state=None,run=None,side='left',seconds=1.,time_scale=1.):
    output=Path(output)
    if output.exists():raise FileExistsError('Use a new CSV output path')
    if not math.isfinite(time_scale)or time_scale<1:raise ValueError('Audit time scaling may only slow a trajectory')
    header=['time_s']+[f'{prefix}{j}'for prefix in ['q','dq','ddq']for j in range(1,8)]
    if state is not None:
        raw=Path(state).read_bytes();data=json.loads(raw)
        if 'q_d'not in data:raise ValueError('Use the updated m80_read_state output including q_d')
        q=data['q_d']
        if len(q)!=7 or not all(math.isfinite(x)for x in q):raise ValueError('Seven finite desired joint positions required')
        count=round(seconds*1000)
        if not 1<=count<=60000 or abs(count/1000-seconds)>1e-9:raise ValueError('Stationary audit duration must be 1ms to 60s, in whole milliseconds')
        rows=([t,*q,*([0.]*14)]for t in [0.,seconds*time_scale]);count=1
        provenance=dict(kind='stationary_SDK_model_probe',source_sha256=hashlib.sha256(raw).hexdigest(),reported_tool_mass_kg=data.get('m_ee_kg'),reported_load_mass_kg=data.get('m_load_kg'))
    else:
        import numpy as np
        path=Path(run)/'joint-motion.npz'
        with np.load(path)as z:
            t=z['time_s'].copy();values=np.stack([z['command_position'],z['command_velocity'],z['command_acceleration']],axis=1)
        indices=slice(0,7)if side=='left'else slice(7,14)
        arrays=[values[:,k,indices]/time_scale**k for k in range(3)]
        if not all(np.isfinite(a).all()for a in arrays):raise ValueError('Nonfinite reconstructed commands')
        matrix=np.column_stack([(t-t[0])*time_scale,*arrays]);rows=matrix.tolist();count=len(rows)-1
        provenance=dict(kind='simulation_commands_for_math_only',side=side,source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),interpolation='Original q/dq/ddq knot locations retained; C2 quintic Hermite, SDK model sampled internally at 1kHz',field_calibrated=False)
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x',newline='')as stream:
        writer=csv.writer(stream);writer.writerow(header)
        for row in rows:writer.writerow([format(float(value),'.17g')for value in row])
    report=dict(**provenance,waypoints=count+1,SDK_model_sampling_hz=1000,time_scale=time_scale,csv_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),hardware_ready=False,motion_execution_authorized=False,scope='Input for read-only model analysis. Not a calibrated hardware motion program.')
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);group=p.add_mutually_exclusive_group(required=True);group.add_argument('--state',type=Path);group.add_argument('--run',type=Path);p.add_argument('--side',choices=['left','right'],default='left');p.add_argument('--seconds',type=float,default=1.);p.add_argument('--time-scale',type=float,default=1.);p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(json.dumps(export(a.output,state=a.state,run=a.run,side=a.side,seconds=a.seconds,time_scale=a.time_scale),indent=2))
