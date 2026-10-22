from pathlib import Path
import shutil,json,hashlib,numpy as np,py_compile
b=Path('/home/gpuadmin/workspace/jake/mujoco');stage=Path('/tmp/pin-fixed-normal-20261022');out=stage/'fixed-camera-v4-overrides';out.mkdir(exist_ok=True)
for p in (stage/'fixed-camera-v3-overrides').glob('*.py'):shutil.copy2(p,out/p.name)
p=out/'direct_pin_skill.py';s=p.read_text();old="camera_name='fixture' if label.startswith('pin_component_postcarry') else 'supply'";assert s.count(old)==1;s=s.replace(old,"camera_name=('fixture' if label.startswith('pin_component_postcarry') else 'lift' if label.startswith('pin_lift') else 'supply')");p.write_text(s);py_compile.compile(str(p),doraise=True)
r=json.loads((stage/'fixed-public-reference-v3.json').read_text());source=next((b/'isaac_port/runs').glob('*v575-*'));rgb=source/'rgb-perception/observation-073-held/rgbd.npz';d=np.load(rgb);T=d['T_cell_camera'];eye=T[:3,3];target=eye+T[:3,2]*np.hypot(.20,.23)
r['fixed_camera_views']['lift']=dict(camera_id='lift',eye_world_m=eye.tolist(),target_world_m=target.tolist(),span_m=.1,fixed_extrinsics=True,reference_source=source.name,reference_observation='observation-073-held',reference_rgbd_SHA256=hashlib.sha256(rgb.read_bytes()).hexdigest(),simulation_camera_calibration_only=True,real_camera_intrinsics_or_mount_not_validated=True,placement_reason='Additional fixed low oblique supply camera for raised pin; selected offline, no runtime view search or micro correction')
(stage/'fixed-public-reference-v4.json').write_text(json.dumps(r,indent=2)+'\n')
for v,gpu in [(592,2),(593,3)]:
 j=json.loads(Path('/tmp/roas-column248-native/590-request-draft.json').read_text());j['run']=f'native-roas-table-pin-grip-transport-v{v}-fixed-lift-camera-repeat{v-591}';j['gpu']=gpu;j['config']['pin_normal_fixed_grasp']=r;j['config']['experiment_note']='Frozen pickup plus three fixed camera mounts: independent actual grip and transport only; no ring assembly, insertion, or micro recovery.';Path(f'/tmp/roas-column248-native/{v}-request-draft.json').write_text(json.dumps(j,indent=2)+'\n')
print(out)
