from pathlib import Path
import sys,json,hashlib,time
import numpy as np
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parent.parent/'isaac_port/runs/native-roas-table-fixed-front-v488-small-binding-split-gpu-profile-nominal/code_snapshot'
sys.path.insert(0,str(SOURCE))
from gpu_pair_schedule import prove
from gpu_pair_binary_protocol import KEYS
fixture=ROOT.parent/'gpu-exact-row-dedup-model';z=np.load(fixture/'fixture.npz')
class Arm:
    def __init__(self,side,index):self.side=side;self.index=index;self.bank=z[side+'_bank']
    def bodies(self,q,width):
        return {str(i):self.bank[row] for i,row in enumerate(self.index[int(q[0])])}
class Checker:
    def __init__(self,side,index):
        self.arm=Arm(side,index);self.pair_boxes=[(str(i),str(i),tuple(z[side+'_'+str(i)+'_'+key]for key in KEYS))for i in range(int(z[side+'_count']))];self.payload_boxes=[];self.payload_path=None
unique=[];inverse=[]
for side in ['left','right']:
 u,inv=np.unique(z[side+'_index'],axis=0,return_inverse=True);unique.append(u);inverse.append(inv)
lc,rc=[Checker(side,u)for side,u in zip(['left','right'],unique)]
pairs=[(np.asarray([li]),.05,np.asarray([ri]),.05)for li,ri in zip(*inverse)]
diag={};socket='/workspace/project/mujoco/.local/collision-gpu2-exact-dedup/pair-binary.sock';t=time.perf_counter();mask=prove(lc,rc,pairs,dict(socket=socket,timeout_s=10,diagnostics=diag));wall=time.perf_counter()-t
assert len(mask)==55950 and mask.all(),diag
assert diag['transport']=='numpy_pickle5_exact_transform_bank'and'error'not in diag,diag
assert diag['source_sha256']==json.loads((ROOT/'ready.json').read_text())['source_sha256']
assert sum(x['rows']for x in diag['chunk_profiles'])==55950
out=dict(status='PASS',scope='Actual frozen488 gpu_pair_schedule.prove dispatcher, client packer and original protocol against independent dedup service via correctly suffixed alias; public483 native fixture only, not physical cycle',socket=socket,alias_target=str((ROOT/'pair-binary.sock').readlink()),rows=len(mask),clear_rows=int(mask.sum()),all_masks_equal_original_kernel=True,dispatcher_binary_verified=True,wall_s=wall,diagnostics=diag,source_hashes={n:hashlib.sha256((SOURCE/n).read_bytes()).hexdigest()for n in ['gpu_pair_schedule.py','gpu_pair_schedule_binary.py','gpu_pair_schedule_json.py']})
(ROOT/'dispatcher-alias-qualification.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items()if k!='diagnostics'},indent=2))
