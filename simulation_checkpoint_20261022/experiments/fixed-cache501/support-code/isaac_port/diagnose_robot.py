"""Static-state collision diagnostic; deliberately excludes workpieces."""
from pathlib import Path
import math,json,sys,traceback,argparse
p=argparse.ArgumentParser();p.add_argument('--corrected',action='store_true');p.add_argument('--nominal',action='store_true');a=p.parse_args()
import numpy as np
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'multi_gpu':False,'anti_aliasing':0});code=0
try:
 import omni.usd,omni.timeline,omni.physx,carb
 from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,PhysicsSchemaTools
 from isaacsim.core.simulation_manager import SimulationManager as SM
 from isaacsim.core.experimental.prims import Articulation
 root=Path(__file__).resolve().parent;run=root/'runs/contact-probe-10-full';out=root/('runs/robot-collision-nominal'if a.nominal else'runs/robot-collision-corrected'if a.corrected else'runs/robot-collision-diagnostic');out.mkdir(exist_ok=True)
 # Root overrides inherit the actual tested task configuration.
 stage=Usd.Stage.CreateNew(str(out/'diagnostic.usda'));stage.GetRootLayer().subLayerPaths=['../contact-probe-10-full/configuration.usda','../contact-probe-10-full/scene.usda']
 stage.GetRootLayer().Save()
 omni.usd.get_context().open_stage(str(out/'diagnostic.usda'));stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
 src=json.loads((root/'generated/contact_task/contact-source.json').read_text());tr=np.load(root.parent/'keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/cap_button_parallel_alt_20261016/trajectory.npz');idx={j['name']:j['qpos_address']for j in src['joints']}
 trace=json.loads((run/'trace.json').read_text());tape=np.load(run/'physics-tape.npz');f=-1;ref=trace[f]['source_t'];qsrc=np.array([np.interp(ref,tr['time_s'],tr['qpos'][:,i])for i in range(30)])
 for pr in stage.Traverse():
  path=str(pr.GetPath())
  if 'workpiece_'in path and pr.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(pr).CreateCollisionEnabledAttr(False)
  if 'official_FR3_Duo'in path and pr.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(pr).CreateThresholdAttr(0)
  if pr.IsA(UsdPhysics.RevoluteJoint) and pr.GetName()in idx:
   PhysxSchema.JointStateAPI.Apply(pr,'angular').CreatePositionAttr(math.degrees(qsrc[idx[pr.GetName()]]))
 if a.corrected:
  from robot_collision_filters import apply as apply_filters
  apply_filters(stage,root,json.loads((run/'body-paths.json').read_text()))
  stage.GetPrimAtPath('/World/Environment/button_back').SetActive(False)
  c=UsdGeom.Cylinder.Define(stage,'/World/Environment/button_corrected');c.CreateRadiusAttr(.018);c.CreateHeightAttr(.004)
  xf=UsdGeom.Xformable(c);xf.AddTranslateOp().Set((.355969,-.2395,1.527));xf.AddRotateXOp().Set(90);UsdPhysics.CollisionAPI.Apply(c.GetPrim())
 SM.setup_simulation(dt=1/480,device='cpu');carb.settings.get_settings().set_bool('/physics/updateToUsd',False)
 tl=omni.timeline.get_timeline_interface();tl.play();app.update();tl.pause()
 robot=Articulation('/World/Cell/Geometry/official_FR3_Duo');names=robot.dof_names
 desired=np.array([qsrc[idx[n]]for n in names]);actual=desired if a.nominal else tape['joint_positions'][f];robot.set_dof_positions(actual[None,:]);robot.set_dof_velocities(np.zeros((1,30)));robot.set_dof_position_targets((desired+(np.zeros(30)if a.nominal else trace[f]['arm_offset_rad']))[None,:])
 contacts=[]
 def report(headers,points):
  for h in headers:
   if not h.num_contact_data:continue
   aa=str(PhysicsSchemaTools.intToSdfPath(h.actor0));bb=str(PhysicsSchemaTools.intToSdfPath(h.actor1));force=sum(float(np.linalg.norm(points[i].impulse))*480 for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
   if force>.1:contacts.append(dict(actor0=aa,actor1=bb,force_n=force))
 sub=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(report)
 for step in range(120):SM.step()
 result=dict(reference_time=ref,contacts=sorted(contacts,key=lambda x:-x['force_n']),joint_errors={n:float(v)for n,v in zip(names,robot.get_dof_positions().numpy()[0]-desired)})
 (out/'result.json').write_text(json.dumps(result,indent=2));print('DIAGNOSTIC',json.dumps(result['contacts'][:12]),flush=True)
except Exception:traceback.print_exc();code=1
finally:app.close(wait_for_replicator=False,exit_code=code)
sys.exit(code)
