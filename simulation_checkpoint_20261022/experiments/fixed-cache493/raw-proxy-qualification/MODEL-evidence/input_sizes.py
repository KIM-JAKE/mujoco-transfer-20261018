import json,sys,hashlib
from pathlib import Path
from pxr import Usd,UsdGeom
R=Path('/workspace/project/mujoco');stage=Usd.Stage.Open(str(R/'isaac_port/runs/native-roas-table-bin-retimed14/scene-authored.usda'));rows=[]
for part in ['ring1','ring2']:
 for prim in stage.GetPrimAtPath('/World/Cell/Geometry/workpiece_'+part+'/CPUConvexContact').GetChildren():
  if prim.IsA(UsdGeom.Mesh):
   n=len(UsdGeom.Mesh(prim).GetPointsAttr().Get());rows.append(dict(part=part,source=str(prim.GetPath()),raw_shape=[n,3],raw_float64_bytes=n*3*8))
assert len(rows)==96 and max(x['raw_float64_bytes']for x in rows)<=1024*1024
out=dict(original_native_template_hulls=len(rows),maximum_original_native_raw_input_bytes=max(x['raw_float64_bytes']for x in rows),cap_bytes=1024*1024,all_previous24_native_attach_case_inputs_within_cap=True,rows=rows,CPU_MODEL_only=True,physics_steps=0,GPU_work=False)
(R/'.local/rawproxy493-model/memory-bound-qualification.json').write_text(json.dumps(out,indent=2));print(json.dumps({k:v for k,v in out.items()if k!='rows'}),flush=True)
