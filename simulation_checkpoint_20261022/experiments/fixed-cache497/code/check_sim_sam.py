"""SAM and CAD registration on RGB-D produced by live Isaac scene queries."""
from pathlib import Path
import sys,os,json,time,traceback
os.environ['CUDA_VISIBLE_DEVICES']=''
import torch,numpy as np
from PIL import Image
torch.set_num_threads(8);torch.set_num_interop_threads(1)
HERE=Path(__file__).resolve().parent;PORT=HERE.parents[1];ROOT=PORT.parent/'robotics_agent/mujoco_duo_fr3';sys.path[:0]=[str(ROOT),str(ROOT/'runtime/sam31_source'),str(PORT.parent),str(HERE/'bin-picking/perception-deps-python311')]
from sam3.model.position_encoding import PositionEmbeddingSine
original_init=PositionEmbeddingSine.__init__
def cpu_init(self,*args,**kwargs):kwargs['precompute_resolution']=None;original_init(self,*args,**kwargs)
PositionEmbeddingSine.__init__=cpu_init
for name in ['arange','zeros','ones','empty','tensor','linspace']:
    f=getattr(torch,name)
    def factory(*args,_f=f,**kwargs):
        if str(kwargs.get('device','')).startswith('cuda'):kwargs['device']='cpu'
        return _f(*args,**kwargs)
    setattr(torch,name,factory)
torch.Tensor.pin_memory=lambda self,*args,**kwargs:self
from refill.sam31_adapter import SAM31MaskGenerator
from isaac_port.fieldkit.perception import CadPoseEstimator
run=PORT/'runs'/sys.argv[1];z=np.load(run/'basket-rgbd.npz');rgb=z['rgb'];depth=z['depth_m'];K=z['K'];T=z['T_cell_camera'];out=run/'sim-perception';out.mkdir(exist_ok=True)
report=dict(input='Live Isaac Sim PhysX RGB-D camera',ground_truth_used_for_inference=False,full_task_pass=False,poses=[])
try:
    detector=SAM31MaskGenerator(device='cpu',prompts=('metal ring','ring'),score_threshold=.25)
    with torch.autocast('cpu',dtype=torch.bfloat16):masks=detector.segment(rgb)
    report['segmentation']=detector.last_report;report['segmentation']['crop_source']='full_Isaac_camera_frame';report['segmentation']['class_and_pose_source']='SAM RGB masks plus Isaac depth observations and known CAD dimensions'
    np.savez_compressed(out/'sam-masks.npz',masks=np.stack([m['mask']for m in masks])if masks else np.empty((0,*depth.shape),bool),scores=[m['score']for m in masks])
    estimator=CadPoseEstimator();overlay=rgb.copy();accepted=[]
    for i,m in enumerate(sorted(masks,key=lambda x:-x['score'])):
        mask=m['mask']&np.isfinite(depth)
        if any((mask&a).sum()/max(1,(mask|a).sum())>.5 for a in accepted):continue
        accepted.append(mask);v,u=np.nonzero(mask);d=depth[v,u];pc=np.c_[(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d];pts=pc@T[:3,:3].T+T[:3,3]
        overlay[mask]=(overlay[mask]*.5+np.array([255,95,25])*.5).astype(np.uint8)
        if len(pts)<150:report['poses'].append(dict(state='insufficient_depth_points',points=len(pts)));continue
        center=np.median(pts,axis=0);_,_,V=np.linalg.svd(pts-center,full_matrices=False);normal=V[-1]
        if normal[2]<0:normal=-normal
        x=np.array([1.,0,0]);x-=normal*(x@normal);x/=np.linalg.norm(x);R=np.column_stack([x,np.cross(normal,x),normal]);local=(pts-center)@R;diameter=max(np.ptp(local[:,:2],axis=0));part='ring1'if diameter>.069 else'ring2';prior=np.eye(4);prior[:3,:3]=R;prior[:3,3]=center-normal*(.00775 if part=='ring1'else .007)
        try:
            pose=estimator.fit(part,pts,prior,max_shift_m=(.03,.03,.02),max_tilt_rad=.3,rms_limit_m=.002,min_inlier_fraction=.6);report['poses'].append(dict(state='registered',mask_score=m['score'],observed_extent_m=float(diameter),**pose.as_dict()))
        except Exception as e:report['poses'].append(dict(state='registration_rejected',part=part,error=str(e),depth_points=len(pts),observed_extent_m=float(diameter)))
    Image.fromarray(overlay).save(out/'isaac-sam-overlay.png');report['segmentation_masks']=len(masks);report['registered_poses']=sum(p['state']=='registered'for p in report['poses'])
except Exception:report.update(error=traceback.format_exc())
(out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
