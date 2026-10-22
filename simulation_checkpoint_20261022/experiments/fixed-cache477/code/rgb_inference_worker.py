"""Host CPU inference service for strict RGB-only camera files."""
from pathlib import Path
import sys,os,json,time,traceback
os.environ['CUDA_VISIBLE_DEVICES']=''
import torch,numpy as np
HERE=Path(__file__).resolve().parent;PORT=HERE.parents[1];ROOT=PORT.parent/'robotics_agent/mujoco_duo_fr3';sys.path[:0]=[str(HERE),str(ROOT),str(ROOT/'runtime/sam31_source'),str(PORT.parent),str(HERE/'bin-picking/perception-deps-python311')]
torch.set_num_threads(6);torch.set_num_interop_threads(1)
from sam3.model.position_encoding import PositionEmbeddingSine
orig=PositionEmbeddingSine.__init__
def cpu_init(self,*a,**kw):kw['precompute_resolution']=None;orig(self,*a,**kw)
PositionEmbeddingSine.__init__=cpu_init
for name in ['arange','zeros','ones','empty','tensor','linspace']:
 f=getattr(torch,name)
 def factory(*a,_f=f,**kw):
  if str(kw.get('device','')).startswith('cuda'):kw['device']='cpu'
  return _f(*a,**kw)
 setattr(torch,name,factory)
torch.Tensor.pin_memory=lambda self,*a,**kw:self
from refill.sam31_adapter import SAM31MaskGenerator
from rgb_stereo_inference import StereoInference
detector=SAM31MaskGenerator(device='cpu',score_threshold=.1,prompts=('ring','metal ring'));inference=StereoInference(detector)
root=Path(sys.argv[1]);print('RGB_WORKER_READY',root,flush=True)
while not (root/'STOP-RGB-WORKER').exists():
 pending=[p for p in sorted(root.glob('rgb-perception/*/request.json'))if not(p.parent/'response.json').exists()]
 if not pending:time.sleep(.1);continue
 for file in pending:
  work=file.parent;request=json.load(open(file));started=time.monotonic()
  try:
   packets=[]
   for name in request['cameras']:
    with np.load(work/name)as z:
     assert set(z.files)=={'rgb','K','T_cell_camera'}, 'Non-RGB sensor channel rejected'
     packets.append({k:z[k].copy()for k in z.files})
   result=inference.infer(packets,request,work)
  except Exception:result=dict(error=traceback.format_exc());print(result['error'],flush=True)
  result['inference_wall_s']=time.monotonic()-started;temp=work/'response.tmp';temp.write_text(json.dumps(result,indent=2));temp.rename(work/'response.json');print('RGB_REQUEST_FINISHED',work.name,result.get('error','ok'),result['inference_wall_s'],flush=True)
