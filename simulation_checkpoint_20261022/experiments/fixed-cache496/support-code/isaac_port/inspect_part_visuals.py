from pxr import Usd,UsdGeom,UsdShade
from pathlib import Path
import json
root=Path(__file__).resolve().parent;run=root/'runs/contact-probe-13-serialized';s=Usd.Stage.Open(str(run/'xray-preview/measured.usda'));paths=json.loads((run/'body-paths.json').read_text());time=23.5*30
for name in ['ring1','ring2','pin']:
 p=s.GetPrimAtPath(paths['workpiece_'+name]);print('BODY',name,str(p.GetPath()),UsdGeom.Imageable(p).ComputeVisibility(time),flush=True)
 for c in Usd.PrimRange(p):
  if not c.IsA(UsdGeom.Mesh):continue
  m=UsdGeom.Mesh(c);xf=UsdGeom.XformCache(time).GetLocalToWorldTransform(c);bbox=UsdGeom.BBoxCache(Usd.TimeCode(time),['default','render','proxy','guide'],False,True).ComputeWorldBound(c).ComputeAlignedRange();mat=UsdShade.MaterialBindingAPI(c).ComputeBoundMaterial()[0]
  print(str(c.GetPath()),'visible',UsdGeom.Imageable(c).ComputeVisibility(time),'purpose',UsdGeom.Imageable(c).ComputePurpose(),'points',len(m.GetPointsAttr().Get()),'worldbox',str(bbox),'mat',str(mat.GetPath())if mat else None,'transform',str(xf.ExtractTranslation()),flush=True)
