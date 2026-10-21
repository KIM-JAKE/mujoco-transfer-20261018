from pathlib import Path
import json,hashlib,numpy as np,fcl
from scipy.spatial.transform import Rotation
from field_demo import model_preset,collision
R=Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs/native-roas-table-scatter-stable-v348-gpu-carry-no-greedy-final-retreat-all3-446');S=Path('/tmp/field-link5-link7-348');z=np.load(R/'camera-local-models.npz');trace=np.load(R/'physx-sequence-trace.npz');cmd=np.load(R/'executed-actuator-commands.npz');obs=json.loads((R/'rgb-observations.json').read_text())[13];wall=obs['captured_wall_s'];ci=int(np.argmin(abs(cmd['time_s']-wall)));ti=int(np.argmin(abs(trace['time_s']-wall)));q=cmd['actual_dof_position'][ci,[1,3,5,7,9,11,13]].astype(float)
kin=json.loads((R/'robot-kinematics.json').read_text());poses=[np.eye(4)]
def proper(T):
 T=np.asarray(T,float).copy();u,_,v=np.linalg.svd(T[:3,:3]);T[:3,:3]=u@v;return T
for value,axis,(a,b)in zip(q,kin['axes'],kin['frames']):
 rr=np.eye(4);rr[:3,:3]=Rotation.from_rotvec(np.asarray(axis)*value).as_matrix();poses.append(poses[-1]@proper(a)@rr@proper(b))
base_idx=next(i for i,p in enumerate(trace['camera_body_paths'])if p.endswith('/right_arm_fr3v2_link0'));bp=trace['camera_bodies'][0,base_idx];base=np.eye(4);base[:3,:3]=Rotation.from_quat(bp[3:]).as_matrix();base[:3,3]=bp[:3];world={i:base@poses[i]for i in [5,7]}
def bvh(v,f,T):
 m=fcl.BVHModel();m.beginModel(len(v),len(f));m.addSubModel(np.asarray(v,float),np.asarray(f,int));m.endModel();return fcl.CollisionObject(m,fcl.Transform(T[:3,:3],T[:3,3]))
def measure(objects):
 req=fcl.DistanceRequest(enable_nearest_points=True);res=fcl.DistanceResult();d=fcl.distance(*objects,req,res);cr=fcl.CollisionResult();n=fcl.collide(*objects,fcl.CollisionRequest(num_max_contacts=100,enable_contact=True),cr);return dict(distance_m=float(d),colliding=bool(cr.is_collision),triangle_contacts=int(n),nearest_points_cell_m=[p.tolist()for p in res.nearest_points])
official=[];original=[];source=[]
for i in [5,7]:
 p=model_preset.ASSETS/f'link{i}.stl';raw=p.read_bytes();tri=np.frombuffer(raw[84:],dtype=[('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])['v'].astype(float);v=tri.reshape(-1,3);f=np.arange(len(v)).reshape(-1,3);official.append(bvh(v,f,world[i]))
 path=next(p for p in z['paths']if p.endswith('/right_arm_fr3v2_link'+str(i)));j=z['paths'].tolist().index(path);body=trace['camera_bodies'][ti,trace['camera_body_paths'].tolist().index(path)];T=np.eye(4);T[:3,:3]=Rotation.from_quat(body[3:]).as_matrix();T[:3,3]=body[:3];original.append(bvh(z['v'+str(j)],z['f'+str(j)],T));source.append(dict(link=i,official_collision_STL=str(p),official_sha256=hashlib.sha256(raw).hexdigest(),official_triangles=len(tri),original_Isaac_collision_vertices=len(z['v'+str(j)]),original_Isaac_collision_faces=len(z['f'+str(j)])))
rows=json.loads((model_preset.ASSETS/'arm-columns.json').read_text())['capsules'];caps=[]
for i,name in [(5,'link5_column_7_3'),(7,'link7_column_3_0')]:
 spec=next(c for c in rows if c['name']==name);T=world[i];a=np.asarray(spec['a'])@T[:3,:3].T+T[:3,3];b=np.asarray(spec['b'])@T[:3,:3].T+T[:3,3];caps.append(dict(name='right_'+name,link=i,a=a,b=b,original_CAD_column_radius_m=float(spec['radius_m']),offline_cable_allowance_m=.002))
dist=float(collision.segment_distance(caps[0]['a'],caps[0]['b'],caps[1]['a'],caps[1]['b']));rad=sum(c['original_CAD_column_radius_m']for c in caps);margin=.001;result=dict(scope='ONE exactactual348 held-state link5/link7 pair diagnostic; actualq and private renderer body transforms are EVALUATOR-ONLY. No command, exception, model activation, newview/grasp/grid or tracking-profile change.',hardware_tested=False,robot_command_sent=False,source_run=str(R),public_RGBD_capture_wall_s=wall,actual_encoder_wall_s=float(cmd['time_s'][ci]),actual_q_evaluator_only=q.tolist(),trace_wall_s=float(trace['time_s'][ti]),official_full_original_collision_STL_FCL=measure(official),original_recorded_Isaac_collision_mesh_FCL=measure(original),capsule_pair=dict(names=[c['name']for c in caps],centerline_distance_m=dist,CAD_column_radii_m=[c['original_CAD_column_radius_m']for c in caps],cable_allowance_each_m=.002,tracking_inflation_m=0.,temporal_sweep_m=0.,fixed_margin_m=margin,clearance_without_cable_or_margin_m=dist-rad,clearance_with_1mm_margin_no_cable_m=dist-rad-margin,clearance_with_2mm_cable_each_and_1mm_margin_m=dist-rad-.004-margin),models=source,default20mrad_UNKNOWN_unchanged=True)
(S/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
