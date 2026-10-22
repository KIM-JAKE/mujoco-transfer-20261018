"""Independent physical button stroke bench; no assembly success claim."""
from pathlib import Path
import json,math,sys,traceback
import numpy as np
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'multi_gpu':False,'anti_aliasing':0});code=0
try:
 import omni.usd,omni.timeline,omni.physx,carb
 from pxr import UsdPhysics,PhysxSchema,PhysicsSchemaTools
 from isaacsim.core.simulation_manager import SimulationManager as SM
 from isaacsim.core.experimental.prims import Articulation,RigidPrim
 from contact_scene import build,HERE
 out=HERE/'runs/button-contact-corrected';out.mkdir(exist_ok=True);source,paths,adapt=build(out)
 omni.usd.get_context().open_stage(str(out/'scene.usda'));stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
 from pose_feedback import PoseFeedback
 from fixture_clamps import apply
 apply(stage,paths)
 tr=np.load(HERE.parent/'keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/cap_button_parallel_alt_20261016/trajectory.npz');idx={j['name']:j['qpos_address']for j in source['joints']};times=tr['time_s'];start=7.7;end=float(times[-1])
 for pr in stage.Traverse():
  if 'workpiece_'in str(pr.GetPath()) and pr.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(pr).CreateCollisionEnabledAttr(False)
  if pr.IsA(UsdPhysics.RevoluteJoint)and pr.GetName()in idx:
   q0=float(np.interp(start,times,tr['qpos'][:,idx[pr.GetName()]]));PhysxSchema.JointStateAPI.Apply(pr,'angular').CreatePositionAttr(math.degrees(q0));UsdPhysics.DriveAPI.Apply(pr,'angular').CreateTargetPositionAttr(math.degrees(q0))
 SM.setup_simulation(dt=1/480,device='cpu');carb.settings.get_settings().set_bool('/physics/updateToUsd',False)
 tl=omni.timeline.get_timeline_interface();tl.play();app.update();tl.pause();robot=Articulation('/World/Cell/Geometry/official_FR3_Duo');button=RigidPrim('/World/Button');names=robot.dof_names
 raw=tr['qpos'][:,[idx[n]for n in names]];vel=np.gradient(raw,times,axis=0)/4
 feedback=PoseFeedback(HERE,HERE.parent/'keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/cap_button_parallel_alt_20261016',names,4)
 initial=np.array([np.interp(start,times,raw[:,j])for j in range(30)])[None,:];robot.set_dof_positions(initial);robot.set_dof_velocities(np.zeros_like(initial));robot.set_world_poses(positions=[[-.24903096,-.6,1]],orientations=[[math.sqrt(.5),0,0,math.sqrt(.5)]])
 k=[];d=[];maxf=[]
 for n in names:
  if '_arm_'in n:k.append(12000);d.append(220);maxf.append(87 if int(n[-1])<=4 else 12)
  elif n.endswith('left_driver_joint'):k.append(200);d.append(10);maxf.append(5)
  elif n.endswith('spring_link_joint'):k.append(.05);d.append(.00125);maxf.append(.5)
  else:k.append(0);d.append(.02);maxf.append(0)
 robot.set_dof_gains([k],[d]);robot.set_dof_max_efforts([maxf]);trace=[];forces=[];clock=[0.]
 def contact(headers,points):
  for h in headers:
   aa=str(PhysicsSchemaTools.intToSdfPath(h.actor0));bb=str(PhysicsSchemaTools.intToSdfPath(h.actor1))
   if '/World/Button'not in [aa,bb]:continue
   f=sum(float(np.linalg.norm(points[i].impulse))*480 for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
   if f>.001:forces.append(dict(t=clock[0],force_n=f,actor0=aa,actor1=bb))
 sub=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact)
 for step in range(round((.5+(end-start)*4+1)*480)):
  t=step/480;clock[0]=t;ref=float(np.clip(start+(t-.5)/4,start,end))
  if step%4==0:
   q=np.array([np.interp(ref,times,raw[:,j])for j in range(30)])[None,:];v=np.array([np.interp(ref,times,vel[:,j])for j in range(30)])[None,:]
   if t<.5 or ref>=end:v[:]=0
   for j,n in enumerate(names):
    if n.endswith('spring_link_joint'):q[0,j]=2.62;v[0,j]=0
   actual=robot.get_dof_positions().numpy()[0];offset,ov=feedback.update(ref,actual,np.zeros((3,3)),np.tile([1,0,0,0],(3,1)),{})
   robot.set_dof_position_targets(q+offset[None,:]);robot.set_dof_velocity_targets(v+ov[None,:])
  SM.step()
  if step%16==0:trace.append(dict(t=t,source_t=ref,travel_m=float(button.get_world_poses()[0].numpy()[0,1]+.24775)))
 travel=np.array([r['travel_m']for r in trace]);passed=bool((travel[-15:]>=.0018).all()and travel.max()<.0025 and len(forces)>0)
 result=dict(passed=passed,max_travel_m=float(travel.max()),final_travel_m=float(travel[-1]),peak_force_n=max((r['force_n']for r in forces),default=0),trace=trace,contacts=forces,scope='Isolated spring-button contact test only');(out/'result.json').write_text(json.dumps(result,indent=2));print('BUTTON_BENCH',passed,result['max_travel_m'],result['peak_force_n'],flush=True);code=0 if passed else 1
except Exception:traceback.print_exc();code=1
finally:app.close(wait_for_replicator=False,exit_code=code)
sys.exit(code)
