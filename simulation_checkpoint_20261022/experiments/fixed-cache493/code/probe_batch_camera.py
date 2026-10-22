"""Verify the batch camera inside Isaac and benchmark pixels + fresh detection."""
from pathlib import Path
import os,sys,time,json,numpy as np
from pxr import Usd,UsdUtils
import omni.kit.app,omni.physx,omni.physics.tensors as tensors
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE/'bin-picking/perception-deps-python311'),str(HERE.parents[2])];OUT=HERE.parents[1]/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
try:
 stage=Usd.Stage.Open(str(HERE.parents[1]/'runs/native-roas-table-rgbd-control05/scene-authored.usda'));ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);sim.simulate(1/480,0);sim.fetch_results();view=tensors.create_simulation_view('numpy',ident)
 from rgbd_batch_sensor import BatchRGBDCamera
 from rgbd_initial_fast import detect
 from isaac_port.fieldkit.perception import CadPoseEstimator
 cam=BatchRGBDCamera(stage,view);result={}
 for k in range(2):
  start=time.perf_counter();packet=cam.capture([.75,-1.15,1.6],[.51,-.70,1.1],.28,512,512,out=OUT/f'frame{k}.npz');elapsed=time.perf_counter()-start;answer,_=detect(packet,json.load(open(HERE/'bin-picking/asset-audit.json'))['baskets'][0],CadPoseEstimator());result[str(k)]=dict(capture_s=elapsed,detection=answer);print('BATCH_PROFILE',k,result[str(k)],flush=True)
 from PIL import Image
 Image.fromarray(packet['rgb']).save(OUT/'camera.png');(OUT/'result.json').write_text(json.dumps(result,indent=2));sim.detach_stage();omni.kit.app.get_app().post_quit(0)
except Exception:
 import traceback;(OUT/'error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
