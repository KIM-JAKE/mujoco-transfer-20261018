from pathlib import Path
import json,hashlib,time
from datetime import datetime,timezone
import numpy as np
from field_demo.random_perception import load_random_config,detect_packet
R=Path('/home/gpuadmin/workspace/jake/field-demo-random-fair-search-20261022');D=R.parent/'mujoco/.local/field-fair-diagnostic565'
p=D/'fresh-overhead-rgbd.npz';packet=dict(np.load(p));c=json.loads((R/'src/field_demo/stabilized/replay-camera.json').read_text());c.update(T_cell_camera=packet['T_cell_camera'].tolist(),calibration_id='v278_NEW_overhead_SIM_only',replay_source='Fresh virtual overhead exposure of actual physically parked v278 scene')
f=D/'fresh-camera.json';f.write_text(json.dumps(c,indent=2));cfg=load_random_config(f)
meta=dict(capture_id='v278-new-overhead-fair565',captured_at_utc=datetime.now(timezone.utc).isoformat(),timestamp_scope='Frozen offline clock for new render of archived parked scene; never live camera freshness',source='archived_Isaac_scene_new_virtual_RGBD',replay=True,units='m',camera_frame=cfg['camera_frame'],point_cloud_frame=cfg['camera_frame'],camera_serial=cfg['camera_serial'],calibration_id=cfg['calibration_id'],calibration_sha256=cfg['_calibration_sha256'],rectified_registered_depth=True,projection_kind='rectified_pinhole',depth_uncertainty_m=.001,source_packet_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),scene_epoch='v278-empty-park-frozen')
b=time.perf_counter();out=detect_packet(packet,cfg,meta,replay=True);out.update(source_packet=str(p),source_packet_sha256=meta['source_packet_sha256'],source_metadata=meta,component_elapsed_s=time.perf_counter()-b)
(D/'fresh-observation.json').write_text(json.dumps(out,indent=2));print([(v['part'],v['center_m'],v.get('geometrically_accessible'))for v in out['instances']]);print(out['component_elapsed_s'])
