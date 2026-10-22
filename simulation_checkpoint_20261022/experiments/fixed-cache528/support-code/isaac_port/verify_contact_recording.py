"""Independent recording checks, including the previously found aliased-joint bug."""
from pathlib import Path
import argparse,json,hashlib
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from source_model import load_model
p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();run=args.run
tr=json.loads((run/'trace.json').read_text());tape=np.load(run/'physics-tape.npz');names=json.loads((run/'tape-bodies.json').read_text());dofs=json.loads((run/'dof-names.json').read_text());manifest=json.loads((run/'run-manifest.json').read_text())
assert len(tr)==len(tape['time_s'])==len(tape['joint_positions'])
part_indices=[names.index('workpiece_'+n)for n in ['ring1','ring2','pin']]
max_part_error=float(np.max(np.abs(tape['body_positions'][:,part_indices]-np.array([r['part_positions']for r in tr]))))
assert max_part_error<1e-8,max_part_error
assert np.all(np.diff(tape['time_s'])>0)
assert np.all(np.diff([r['source_t']for r in tr])>=-1e-9)
q=tape['joint_positions'];assert np.isfinite(q).all();unique=len(np.unique(q,axis=0));assert unique>min(100,len(q)*.8),unique
max_norm_error=float(abs(np.linalg.norm(tape['body_quaternions'],axis=2)-1).max());assert max_norm_error<1e-5,max_norm_error
m=load_model();d=mujoco.MjData(m);adr=[int(m.joint(n).qposadr[0])for n in dofs];max_fk=0.;max_angle=0.
for f in np.unique(np.linspace(0,len(tr)-1,min(100,len(tr))).astype(int)):
 d.qpos[adr]=q[f];mujoco.mj_kinematics(m,d)
 for side in ['left','right']:
  name=side+'_grip_base';i=names.index(name);b=m.body(name).id;max_fk=max(max_fk,float(np.linalg.norm(d.xpos[b]-tape['body_positions'][f,i])))
  actual=Rotation.from_quat(tape['body_quaternions'][f,i,[1,2,3,0]]);expected=Rotation.from_matrix(d.xmat[b].reshape(3,3));max_angle=max(max_angle,float((actual*expected.inv()).magnitude()))
assert max_fk<5e-5,(max_fk,max_angle)
assert max_angle<1e-4,(max_fk,max_angle)
reference_hashes={name:hashlib.sha256((run/'recording'/name).read_bytes()).hexdigest()==digest for name,digest in manifest['recording_sha256'].items()}if(run/'recording').exists()else {}
assert all(reference_hashes.values()),reference_hashes
snapshot={n:hashlib.sha256((run/'implementation'/n).read_bytes()).hexdigest()==sha for n,sha in manifest['code_sha256'].items()};assert all(snapshot.values()),snapshot
frozen_input_hashes={name:hashlib.sha256((run/name).read_bytes()).hexdigest()==digest for name,digest in manifest['input_sha256'].items()if(run/name).exists()}
if manifest['arguments'].get('efficient_timing',False):
 prefix='fr3-concurrent-profile'if'concurrent_profile.py'in manifest['code_sha256']else'fr3-under15-profile'if manifest['arguments'].get('target_15')else'fr3-efficient-profile'
 for name in [prefix+suffix for suffix in ['.npz','.json']]:assert frozen_input_hashes.get(name,False),name
if tr and 'source_times'in tr[0]:
 for side in ['left','right']:assert np.all(np.diff([r['source_times'][side]for r in tr])>=-1e-9)
assert all(frozen_input_hashes.values()),frozen_input_hashes
native_timing=None
if(run/'joint-motion.npz').exists():
 high=np.load(run/'joint-motion.npz');ht=high['time_s'];hv=high['actual_velocity'].astype(float);hq=high['actual_position'].astype(float)
 assert np.allclose(np.diff(ht),1/manifest['physics_hz'],atol=1e-9)
 fd=np.diff(hq,axis=0)/np.diff(ht)[:,None];mask=abs(hv[1:])>.05;assert mask.any()
 ratio=float(np.median(fd[mask]/hv[1:][mask]));assert .98<ratio<1.02,ratio
 native_timing=dict(checked=True,median_position_derivative_to_native_velocity_ratio=ratio,rms_velocity_difference_rad_s=float(np.sqrt(np.mean((fd-hv[1:])**2))),semantics='Independent consistency of declared timestep with native joint position and velocity; finite-step integration adds a small discretization error')
report=dict(state='passed',samples=len(tr),unique_joint_rows=unique,max_part_tape_trace_error_m=max_part_error,max_wrist_fk_error_m=max_fk,max_wrist_fk_angle_error_rad=max_angle,max_quaternion_norm_error=max_norm_error,implementation_hashes_match=snapshot,reference_hashes_match=reference_hashes,tape_sha256=hashlib.sha256((run/'physics-tape.npz').read_bytes()).hexdigest(),scope='Recording integrity only; physical task acceptance is independent')
report['native_timestep_consistency']=native_timing
report['frozen_input_hashes_match']=frozen_input_hashes
(run/'recording-audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
