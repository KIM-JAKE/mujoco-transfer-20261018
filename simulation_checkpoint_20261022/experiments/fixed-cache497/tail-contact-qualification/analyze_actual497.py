import json,ast,numpy as np
from pathlib import Path
r=Path('/home/gpuadmin/workspace/jake/mujoco/.local/warm-worker-gpu2/snapshot/isaac_port/runs/native-roas-table-fixed-front-v497-tail-flat-contact-minus');p=json.loads((r/'pipeline-results.json').read_text());obs=[o for o in json.loads((r/'rgb-observations.json').read_text())if o['part']=='pin'and o.get('result',{}).get('pose')];rows=[]
for o in obs:
 H=np.asarray(o['hand_TCP_world']);pose=o['result']['pose'];P=np.asarray(pose['center_m']);axis=np.asarray(pose['axis']);g=np.linalg.inv(H)@np.r_[P,1.];n=H[:3,:3].T@axis;row=dict(mode=o['mode'],work=Path(o['work']).name,t=o['captured_wall_s'],center_world=P.tolist(),axis_world=axis.tolist(),G_center_m=g[:3].tolist(),G_axis=n.tolist(),cap_plane_rms_m=pose.get('plane_rms_m'),cap_edge_rms_m=pose.get('cap_circular_edge_rms_m'),edge_coverage_deg=pose.get('cap_edge_coverage_deg'),ready_s=o['ready_wall_s']-o['captured_wall_s']);
 if rows and rows[-1]['mode']=='held'and o['mode']=='held':
  prev=rows[-1];row.update(G_center_step_m=float(np.linalg.norm(g[:3]-prev['G_center_m'])),G_axis_step_rad=float(np.arccos(np.clip(n@np.asarray(prev['G_axis']),-1,1))))
 rows.append(row)
t=Path('/home/gpuadmin/workspace/jake/mujoco/.local/warm-worker-gpu2/daemon.log').read_text();t=t[t.rfind('PIN_TAIL_FLAT_CONTACT_TARGET '):];events=[]
for l in t.splitlines():
 if l.startswith(('PIN_TAIL_FLAT_CONTACT_TARGET ','PIN_HOLDER_PRECLOSE ','PIN_SDK_POSTCLOSE_LIFT_ONLY ','PIN_NATIVE_POSTLIFT ','PIN_RGBD_SDK_POSTLIFT ','PIN_QUALIFIED_POSTLIFT_PRELOAD ','PIN_RGBD_SDK_POSTCARRY ')):
  try:events.append(dict(type=l.split(' ',1)[0],data=ast.literal_eval(l.split(' ',1)[1])))
  except Exception:pass
result=dict(run=r.name,full_task_pass=p['full_task_pass'],wall_s=p['wall_verified_elapsed_s'],error=p['error'].splitlines()[-1],scope='PUBLIC directRGBD samecapture SDK and current recorded aperture gates only; no actor/force sensors',events=events,observations=rows,interpretation='Target shifted actual physical contact to tail as predicted (10.09mm close/10.06mm postlift aperture instead12.7mm collar). Original independent lift+preload evidence stable, but transport changes grasp127.56mrad/2.054mm and correctly aborts before any insertion. Geometric footprint alone did not establish dynamic stability; NOT qualified.')
Path('/tmp/fixed497-pin-tail-flat/actual497-failed.json').write_text(json.dumps(result,indent=2));print('rows')
for x in rows:print(x['work'],round(x['t'],3),np.round(np.asarray(x['G_center_m'])*1e3,3).tolist(),np.round(x['G_axis'],5).tolist(),round(x.get('G_center_step_m',0)*1e3,3),round(x.get('G_axis_step_rad',0)*1e3,3))
