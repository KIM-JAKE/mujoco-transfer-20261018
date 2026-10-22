from pathlib import Path
import json,numpy as np,importlib.util,copy,hashlib
O=Path(__file__).parent;R=Path('/home/gpuadmin/workspace/jake/mujoco');cat=json.loads((R/'isaac_port/runs/native-roas-table-scatter-stable-v251-corrected-pin-catalog/pin-native-grasp-catalog.json').read_text())['candidates'][0]
def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
collar=load('collar','/tmp/fixed504-pin-collar-flat/overrides/pin_collar_contact_target.py');tilt=load('tilt',O/'overrides/pin_collar_tilt_target.py');c,_=collar.retarget(cat,-.001);before=copy.deepcopy(c);n,e=tilt.retarget(c);assert c==before
G,H,N,T=[np.array(x)for x in(c['grasp_transform'],c['pickup_H'],n['grasp_transform'],n['pickup_H'])];assert np.max(abs(H@G-T@N))<1e-12;point=np.r_[0.,0.,-.0155,1.];assert np.max(abs(G@point-N@point))<1e-12;assert e['physical_pickup_translation_m']<.0002 and abs(e['physical_pickup_rotation_rad']-np.deg2rad(10))<1e-12;assert not e['physical_grasp_tested'];assert'native_pad_support'not in n
count=0
for mode in('minus2','original0','wrongtilt','nan','nonrigid','wrongtailpoint'):
 d=copy.deepcopy(c)
 if mode=='minus2':d,_=collar.retarget(cat,-.002)
 elif mode=='original0':d=cat
 elif mode=='wrongtilt':d['grasp_transform'][0][0]+=.01
 elif mode=='nan':d['pickup_H'][0][3]=float('nan')
 elif mode=='nonrigid':d['pickup_H'][3][0]=1.
 elif mode=='wrongtailpoint':d['grip_local_pin_z_m']=-.014
 try:tilt.retarget(d);raise AssertionError(mode+'accepted')
 except ValueError:count+=1
(O/'target-tests.json').write_text(json.dumps({'passed':True,'invalid_cases_rejected':count,'world_part_target_exact':True,'same_physical_contact_point_exact':True,'input_unchanged':True,'stale_certificate_removed':True,'physical_target_evidence':e,'runtime_fullscene_or_physics_tested':False},indent=2)+'\n');print('targettestsPASS',e['physical_pickup_translation_m'],e['physical_pickup_rotation_rad'])
(O/'SHA256SUMS.json').write_text(json.dumps({'overrides/pin_collar_tilt_target.py':hashlib.sha256((O/'overrides/pin_collar_tilt_target.py').read_bytes()).hexdigest()},indent=2)+'\n')
