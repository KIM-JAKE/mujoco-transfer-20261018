"""Run the existing SAM3.1 checkpoint on the user's actual supply photo on CPU.
This tests segmentation only; no metric pose or robot success is inferred.
"""
from pathlib import Path
import sys,os,json,time,traceback
os.environ['CUDA_VISIBLE_DEVICES']=''
import numpy as np
from PIL import Image,ImageDraw
import torch
torch.set_num_threads(8);torch.set_num_interop_threads(1)
HERE=Path(__file__).resolve().parent;OUT=HERE/'bin-picking';ROOT=HERE.parents[2]/'robotics_agent/mujoco_duo_fr3';sys.path.insert(0,str(ROOT))
from refill.sam31_adapter import SAM31MaskGenerator
# Official SAM3's positional-cache warmup hardcodes CUDA. Disable only that
# cache warmup in this process; learned weights and model inference are intact.
sys.path.insert(0,str(ROOT/'runtime/sam31_source'))
from sam3.model.position_encoding import PositionEmbeddingSine
original_position_init=PositionEmbeddingSine.__init__
def cpu_position_init(self,*args,**kwargs):
    kwargs['precompute_resolution']=None
    original_position_init(self,*args,**kwargs)
PositionEmbeddingSine.__init__=cpu_position_init
# The decoder also warms a coordinate cache with a CUDA default. Redirect
# tensor factories in this isolated CPU process, never in the shared runtime.
for factory_name in ['arange','zeros','ones','empty','tensor','linspace']:
    factory=getattr(torch,factory_name)
    def cpu_factory(*args,_factory=factory,**kwargs):
        if str(kwargs.get('device','')).startswith('cuda'):kwargs['device']='cpu'
        return _factory(*args,**kwargs)
    setattr(torch,factory_name,cpu_factory)
torch.Tensor.pin_memory=lambda self,*args,**kwargs:self
photo=Path('/home/gpuadmin/.codex/attachments/be41ba65-1179-4db0-ab8f-8d598bf4540b/codex-clipboard-f0f5393d-f9bb-4b13-8566-3f0bc9f358f4.png')
try:
    # Manual basket ROI; no per-object labels, synthetic masks or GT positions.
    im=Image.open(photo).convert('RGB');roi=(int(im.width*.20),int(im.height*.02),int(im.width*.82),int(im.height*.35));im=im.crop(roi);im.thumbnail((1008,1008));im.save(OUT/'photo-basket-roi.png')
    detector=SAM31MaskGenerator(device='cpu',prompts=('metal ring','bearing'),score_threshold=.25)
    start=time.monotonic()
    with torch.autocast('cpu',dtype=torch.bfloat16):masks=detector.segment(np.asarray(im))
    elapsed=time.monotonic()-start
    overlay=np.array(im).copy();palette=[np.array(c)for c in [(255,80,30),(30,210,255),(90,255,70),(240,70,240)]]
    for i,m in enumerate(masks):overlay[m['mask']]=(overlay[m['mask']]*.5+palette[i%len(palette)]*.5).astype(np.uint8)
    vis=Image.fromarray(overlay);d=ImageDraw.Draw(vis)
    for i,m in enumerate(masks):d.text(tuple(m['bbox_xyxy'][:2]),str(i)+': '+m['prompt']+' '+str(round(m['score'],2)),fill='white')
    vis.save(OUT/'photo-sam-overlay.png')
    if masks:np.savez_compressed(OUT/'photo-sam-masks.npz',masks=np.stack([m['mask']for m in masks]),scores=[m['score']for m in masks])
    report=dict(**detector.last_report,photo=str(photo),manual_basket_roi_pixels=roi,total_seconds=elapsed,counts=[dict(prompt=m['prompt'],score=m['score'],bbox=m['bbox_xyxy'])for m in masks],metric_pose_verified=False,grasp_verified=False,full_task_pass=False)
    report.update(backend='sam31_RGB_only_photo_check',crop_source='manually_selected_basket_region',class_and_pose_source='Text-prompt RGB segmentation only; no depth, metric pose or robot command')
    (OUT/'photo-sam-result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
except Exception:
    (OUT/'photo-sam-error.txt').write_text(traceback.format_exc());traceback.print_exc();raise
