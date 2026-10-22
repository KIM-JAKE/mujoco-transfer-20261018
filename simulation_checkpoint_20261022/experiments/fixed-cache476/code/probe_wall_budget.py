"""CPU timing diagnostic, never an assembly success trial."""
from pathlib import Path
import os,sys,time,json,numpy as np
from pxr import Usd,UsdUtils,UsdPhysics
import omni.kit.app,omni.physx,omni.physics.tensors as tensors,carb
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE));OUT=HERE.parents[1]/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
stage=Usd.Stage.Open(str(HERE.parents[1]/'runs/native-roas-table-rgbd-control05/scene-authored.usda'));ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);sim.simulate(1/480,0);sim.fetch_results();view=tensors.create_simulation_view('numpy',ident);art=view.create_articulation_view('/World/RobotMountJoint');idx=np.array([0],np.uint32);z=np.load(HERE.parents[1]/'runs/native-roas-table-rgbd-control05/executed-actuator-commands.npz');commands=z['command'];art.set_dof_positions(commands[:1],idx);art.set_dof_velocities(np.zeros_like(commands[:1]),idx)
report={};settings=carb.settings.get_settings()
for use_usd in [True,False]:
 settings.set_bool('/physics/updateToUsd',use_usd);settings.set_bool('/physics/updateVelocitiesToUsd',use_usd);start=time.perf_counter()
 for k in range(600):
  cmd=commands[k:k+1];art.set_dof_position_targets(cmd,idx);art.set_dof_velocity_targets((commands[k:k+1]-commands[max(0,k-1):max(0,k-1)+1])*480,idx);sim.simulate(1/480,k/480);sim.fetch_results()
 report[str(use_usd)]=dict(wall_s=time.perf_counter()-start,sim_s=600/480);print('STEP_PROFILE',use_usd,report[str(use_usd)],flush=True)
settings.set_bool('/physics/updateToUsd',True);settings.set_bool('/physics/updateVelocitiesToUsd',True)
from rgbd_material_sensor import capture
for width in [192]:
 start=time.perf_counter();capture(stage,[.75,-1.15,1.6],[.51,-.70,1.1],.28,width,width,out=OUT/'camera.npz');report['camera_'+str(width)]=time.perf_counter()-start;print('CAMERA_PROFILE',width,report['camera_'+str(width)],flush=True)
(OUT/'profile.json').write_text(json.dumps(report,indent=2));sim.detach_stage();omni.kit.app.get_app().post_quit(0)
