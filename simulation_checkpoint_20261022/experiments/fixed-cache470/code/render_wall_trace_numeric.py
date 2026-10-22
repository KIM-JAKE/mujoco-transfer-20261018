"""CPU replay of all recorded Isaac rigid bodies, at actual wall timestamps.

Only draws measured states. The fresh PhysX trial supplies assembly evidence;
this replay never declares success or drives a robot.
"""
from pathlib import Path
import os
os.sched_setaffinity(0,set(map(int,os.environ.get('M80_RENDER_CPUS','116,117').split(','))))
import sys,time,json,numpy as np
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE/'bin-picking/realtime-deps-python312'),str(HERE)]
from ray_camera_core import NumericRayCamera,rotation
RUNS=HERE.parents[1]/'runs';source=RUNS/sys.argv[1];out=RUNS/sys.argv[2];out.mkdir(exist_ok=True)
r=json.load(open(source/'pipeline-results.json'));passed=bool(r['full_task_pass']and r['vision_end_to_end_pass']);assert passed or '--allow-failed'in sys.argv,'Failed trials require explicit --allow-failed labeling'
z=np.load(source/'physx-sequence-trace.npz');paths=z['camera_body_paths'].tolist();goals=json.load(open(source/'arm-plan.json'))['goals'];names=z['part_names'].tolist();links=r['link_names'];roots=[links.index(s+'_NativeHand')for s in ['left','right']];camera=NumericRayCamera(source/'camera-local-models.npz');records=[];body_paths=json.load(open(source/'body-paths.json'))
def visibility(packet,path,tr):
    vertices=camera.geometry[path][0];sample=vertices[np.linspace(0,len(vertices)-1,min(256,len(vertices))).astype(int)];world=sample@rotation(tr[3:]).T+tr[:3];T=packet['T_cell_camera'];P=(world-T[:3,3])@T[:3,:3];K=packet['K'];uv=np.rint(P[:,:2]/np.maximum(P[:,2,None],1e-9)*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]).astype(int);mask=(P[:,2]>0)&(uv[:,0]>=0)&(uv[:,0]<packet['rgb'].shape[1])&(uv[:,1]>=0)&(uv[:,1]<packet['rgb'].shape[0]);valid=np.nonzero(mask)[0]
    return float(np.sum(abs(packet['depth_m'][uv[valid,1],uv[valid,0]]-P[valid,2])<.002)/max(1,len(sample)))
# Ten measured views per second, encoded at 30 fps without time compression.
selection=np.unique(np.r_[0,np.searchsorted(z['time_s'],np.arange(0,z['time_s'][-1],.1)),len(z['time_s'])-1]);selection=selection[selection<len(z['time_s'])]
for j in selection:
    t=float(z['time_s'][j]);parts=z['parts'][j];pose={path:v for path,v in zip(paths,z['camera_bodies'][j])};active=0
    for i,n in enumerate(names):
        tr=parts[i];axis=np.arccos(np.clip(1-2*(tr[3]**2+tr[4]**2),-1,1));clear=min(np.linalg.norm(z['links'][j,k,:3]-tr[:3])for k in roots)>.16
        if np.linalg.norm(tr[:3]-goals[n]['P'])<.0015 and axis<.05 and clear:active=min(i+1,2)
        else:active=i;break
    if not passed:active=0 if r.get('scattered_pick',{}).get('state')=='physical_handover_camera_confirmed' else 0 if z['phase_time_s'][j]<8.5 else 1 if z['phase_time_s'][j]<20.5 else 2
    if 'pin_phase'in z and str(z['pin_phase'][j]) in ['pin_tail_orient_free','pin_tail_carry_to_preinsert','pin_single_axial_insert','pin_full_depth_hold','pin_axial_depth_correction','pin_depth_check_hold','pin_clamp_hold','pin_release_after_full_depth','pin_axial_withdraw','pin_retention','all_parts_final_retention']:active=2
    file=out/f'frame_{len(records):04d}.ppm';record=dict(file=file.name,sim_time_s=t,phase_time_s=float(z['phase_time_s'][j]),active_part=names[active],event=names[active]+'_continuous_task');bin_view=active!=2 and parts[active,1]<-.50
    if file.exists() and not bin_view:records.append(record);continue
    if bin_view:
        if file.exists():left=np.frombuffer(file.read_bytes().split(b'\n',3)[3],np.uint8).reshape(360,960,3)[:,:480].copy()
        else:
            global_eye=np.array([-1.50,-1.85,1.95]);global_target=np.array([0.,-.18,1.18]);left=camera.capture(global_eye,global_target,float(np.linalg.norm(global_eye-global_target)*32/35),480,360,body_poses=pose,far_clip=5.)['rgb']
        right=camera.capture([.55,-.93,1.74],[.55,-.725,1.115],.54,480,360,body_poses=pose,far_clip=5.)['rgb'] if r.get('randomized_bin_config',{}).get('randomization_mode')=='full_bin_drop' else camera.capture([.75,-1.15,1.6],[.51,-.70,1.1],.28,480,360,body_poses=pose,far_clip=5.)['rgb'];rgb=np.concatenate([left,right],axis=1);file.write_bytes(b'P6\n960 360\n255\n'+rgb.tobytes());records.append(record);continue
    global_eye=np.array([-1.50,-1.85,1.95]);global_target=np.array([0.,-.18,1.18]);left=camera.capture(global_eye,global_target,float(np.linalg.norm(global_eye-global_target)*32/35),480,360,body_poses=pose,far_clip=5.)['rgb'];target=parts[active,:3].astype(float)
    if active==2:
        eye=target+(np.array([-.22,-.15,-.12]) if target[1]>.05 else np.array([-.22,0,.12]));right=camera.capture(eye,target,.10,480,360,body_poses=pose,far_clip=5.)['rgb'];file.write_bytes(b'P6\n960 360\n255\n'+np.concatenate([left,right],axis=1).tobytes());records.append(record);continue
    if target[1]>.05:eye=np.array([-.22,-.35,1.40]);target=np.array([-.22,.25,1.535]);focal=50
    elif target[1]<-.50:eye=target+[-.18,-.20,.65];focal=70 if active!=2 else 100
    else:eye=target+[.22,-.35,.30];focal=65
    packet=camera.capture(eye,target,float(np.linalg.norm(eye-target)*32/focal),480,360,body_poses=pose,far_clip=5.);path=body_paths['workpiece_'+names[active]];score=visibility(packet,path,parts[active]);best=packet
    if score<.12:
        center=parts[active,:3].astype(float)
        offsets=([[-.35,-.15,.5],[.35,-.15,.5],[0,.25,.65]]if center[1]<-.50 else [[-.25,-.35,.04],[.28,-.30,-.03],[0,-.30,-.15]])
        for offset in offsets:
            candidate_eye=center+offset;candidate=camera.capture(candidate_eye,center,float(np.linalg.norm(offset)*32/65),480,360,body_poses=pose,far_clip=5.);value=visibility(candidate,path,parts[active])
            if value>score:best=candidate;score=value
            if score>=.12:break
    right=best['rgb'];rgb=np.concatenate([left,right],axis=1);file=out/f'frame_{len(records):04d}.ppm';file.write_bytes(f'P6\n960 360\n255\n'.encode()+rgb.tobytes());records.append(dict(file=file.name,sim_time_s=t,phase_time_s=float(z['phase_time_s'][j]),active_part=names[active],event=names[active]+'_continuous_task'))
    if len(records)%20==0:print('WALL_REPLAY_FRAME',len(records),t,flush=True)
manifest=dict(source_run=source.name,source_full_task_pass=passed,source_physical_assembly_pass=passed,failed_attempt_visualization=not passed,renderer='CPU ray replay of Isaac CAD and every recorded PhysX rigid body',RTX=False,replay_only=True,kinematic_replay=True,synthetic_grasp_or_assembly_success=False,video_time_compressed=False,all_dynamic_bodies_measured=True,inspection_view_selection='Recorded CAD visibility; visualization only, never perception/control input',frames=records);(out/'video-frames.json').write_text(json.dumps(manifest,indent=2));print('WALL_REPLAY_DONE',len(records),flush=True)
