"""Run real SAM3.1 CPU segmentation on RGB files; accepts no depth/labels."""
from pathlib import Path
import sys,os,json,time
os.environ['CUDA_VISIBLE_DEVICES']=''
import torch,numpy as np
from PIL import Image,ImageDraw
HERE=Path(__file__).resolve().parent;PORT=HERE.parents[1];ROOT=PORT.parent/'robotics_agent/mujoco_duo_fr3';sys.path[:0]=[str(ROOT),str(ROOT/'runtime/sam31_source'),str(HERE/'bin-picking/perception-deps-python311')]
torch.set_num_threads(6);torch.set_num_interop_threads(1)
from sam3.model.position_encoding import PositionEmbeddingSine
orig=PositionEmbeddingSine.__init__
def cpu_init(self,*args,**kwargs):kwargs['precompute_resolution']=None;orig(self,*args,**kwargs)
PositionEmbeddingSine.__init__=cpu_init
for name in ['arange','zeros','ones','empty','tensor','linspace']:
 f=getattr(torch,name)
 def factory(*args,_f=f,**kwargs):
  if str(kwargs.get('device','')).startswith('cuda'):kwargs['device']='cpu'
  return _f(*args,**kwargs)
 setattr(torch,name,factory)
torch.Tensor.pin_memory=lambda self,*args,**kwargs:self
from refill.sam31_adapter import SAM31MaskGenerator
detector=SAM31MaskGenerator(device='cpu',prompts=('ring','metal ring'),score_threshold=.05)
for arg in sys.argv[1:]:
 p=Path(arg);rgb=np.asarray(Image.open(p).convert('RGB'))
 with torch.autocast('cpu',dtype=torch.bfloat16):masks=detector.segment(rgb)
 np.savez_compressed(p.with_suffix('.sam.npz'),masks=np.stack([m['mask']for m in masks])if masks else np.zeros((0,*rgb.shape[:2]),bool),scores=[m['score']for m in masks])
 report=dict(detector.last_report);report.update(backend='SAM3.1_RGB_only',depth_input=False,class_and_pose_source='RGB masks only; metric pose not yet inferred',crop_source='full RGB frame');p.with_suffix('.sam.json').write_text(json.dumps(report,indent=2))
 overlay=rgb.copy()
 for i,m in enumerate(masks):
  color=np.array([(220,70,40),(30,180,230),(60,240,80),(230,60,230)][i%4]);overlay[m['mask']]=(overlay[m['mask']]*.5+color*.5).astype(np.uint8)
 im=Image.fromarray(overlay);draw=ImageDraw.Draw(im)
 for i,m in enumerate(masks):draw.text(tuple(m['bbox_xyxy'][:2]),str(i)+' '+str(round(m['score'],2)),fill='white')
 im.save(p.with_suffix('.sam.png'));print('SAM_RGB',p.name,len(masks),report['inference_seconds'],flush=True)
