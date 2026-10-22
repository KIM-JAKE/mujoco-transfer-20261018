from pathlib import Path
import os,sys,json,numpy as np
from pxr import Usd,UsdPhysics,UsdUtils
import omni.physx,omni.physics.tensors as tensors,omni.kit.app
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[1])]
from isaac_kinematics import Arm
RUNS=HERE.parents[1]/'runs';OUT=RUNS/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
stage=Usd.Stage.Open(str(RUNS/'native-roas-table-bin-retimed14/scene-authored.usda'));ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);sim.simulate(1/480,0);sim.fetch_results();view=tensors.create_simulation_view('numpy',ident);art=view.create_articulation_view('/World/RobotMountJoint');names=list(art.shared_metatype.dof_names);idx=[names.index(f'right_arm_fr3v2_joint{j}')for j in range(1,8)];drives={p.GetName():UsdPhysics.DriveAPI(p,'angular')for p in stage.Traverse()if p.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in p.GetName()};arm=Arm(stage,'right',drives,art.get_dof_positions()[0,idx]);data={k:np.array(getattr(arm,k)).tolist()for k in ['base','tip','axes','frames','bounds']};data['validation']=arm.validation;(OUT/'arm-model.json').write_text(json.dumps(data,indent=2));print('ISAAC_MODEL_EXPORTED',arm.validation,flush=True);sim.detach_stage();omni.kit.app.get_app().post_quit(0)
