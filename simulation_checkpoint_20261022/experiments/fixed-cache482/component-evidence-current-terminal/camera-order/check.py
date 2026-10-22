from pathlib import Path
import json,numpy as np,ast
root=Path('/home/gpuadmin/workspace/jake/mujoco');run=next((root/'isaac_port/runs').glob('*v480*'));obs=json.load(open(run/'rgb-observations.json'));out=Path('/tmp/fixed482-large-camera-order');groups=[[22,23,24,25,26],[27,28,29,30,31],[32,33,34,35,36],[37,38,39,40,41]];qualified=[];skipped=[]
def supported(o):
 r=o['result'];p=r.get('pose',{});return bool(not r.get('error')and (p.get('front_back_from_RGBD')or p.get('fresh_partial_tracking')))
def consistent(a,b):
 x,y=a['result']['pose'],b['result']['pose'];return bool(np.linalg.norm(np.array(x['center_m'])-y['center_m'])<.003 and np.array(x['axis'])@np.array(y['axis'])>np.cos(.06))
for group in groups:
 entries=[obs[i-1]for i in group];support=[supported(o)for o in entries];assert support==[False,False,True,False,True];oldgood=[o for o in entries if supported(o)];newgood=[entries[2],entries[4]];assert consistent(*oldgood)and consistent(*newgood);assert oldgood[-1]['work']==newgood[-1]['work']
 center=np.array(newgood[-1]['result']['pose']['center_m']);identities=[]
 for index,o in zip(group,entries):
  H=np.array(o['hand_TCP_world']);short=H[:3,0].copy();short[2]=0;short/=np.linalg.norm(short);packet=np.load(run/'rgb-perception'/Path(o['work']).name/'rgbd.npz');eye=packet['T_cell_camera'][:3,3];candidates=[short*.38+[0,0,.40],short*.30+[0,0,.30],-short*.30+[0,0,.30],np.array([.28,.20,.32]),np.array([-.28,-.20,.32])];errors=[float(np.linalg.norm(eye-center-offset))for offset in candidates];best=int(np.argmin(errors));assert best==group.index(index) and min(errors)<.003;identities.append(dict(observation=index,old_view_index=best,view_offset_error_about_fresh_measured_center_m=min(errors),supported=supported(o)))
 skipped+= [entries[i]for i in [0,1,3]]
 qualified.append(dict(original_observations=group,old_two_qualified_frames=[group[2],group[4]],new_first_two_frames=[group[2],group[4]],original_consistency_gate_pass=True,selected_last_pose_unchanged=True,camera_view_identities_from_public_K_extrinsics_and_RGBD_center=identities))
# Far pre-approach observation remains oldordered (distance~0.6m); no reuse.
far=[]
for i in [19,20,21]:
 o=obs[i-1];dist=float(np.linalg.norm(np.array(o['hand_TCP_world'])[:3,3]-np.array(o['result']['pose']['center_m'])));assert dist>.1;far.append(dict(observation=i,public_Hand_to_observed_center_distance_m=dist,keeps_original_order=True))
profile=dict(source_run=run.name,observation_count=len(obs),capture_s=sum(o.get('capture_s',0)for o in obs),processing_s=sum(o.get('processing_s',0)for o in obs),queue_s=sum(o.get('queue_wait_s',0)for o in obs),ready_to_delivery_s=sum(o.get('delivered_wall_s',o['ready_wall_s'])-o['ready_wall_s']for o in obs),skipped_failed_original_frames=len(skipped),skipped_frames_capture_s=sum(o.get('capture_s',0)for o in skipped),skipped_frames_processing_s=sum(o.get('processing_s',0)for o in skipped),scope='Recorded camera readiness costs only, not a new end-to-end timing benchmark')
report=dict(status='PASS',scope='Public480 saved RGB-D packets/results/encoder Hand FK only; no actor poses or new captures',source_profile=profile,far_preapproach_original_order=far,groups=qualified,all_original_view_offsets_retained_as_fresh_fallback=True,original_two_fresh_frame_consistency_gates_unchanged=True,image_mask_or_pose_reuse=False,physical_validation_required=True)
(out/'qualification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(profile,indent=2))
