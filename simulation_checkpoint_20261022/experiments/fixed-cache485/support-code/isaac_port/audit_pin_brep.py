"""Check whether the pin/mate triangle contact represents a solid overlap.

Exact OpenCascade BRep Boolean, independent of PhysX/FCL collision meshes.
"""
import json,hashlib
from pathlib import Path
from OCP.BRep import BRep_Builder
from OCP.BRepTools import BRepTools
from OCP.TopoDS import TopoDS_Shape
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf,gp_Vec
HERE=Path(__file__).resolve().parent
root=HERE.parent/'keti_m80_mujoco_20261013/insertion_20261013/cad'
def read(i):
 s=TopoDS_Shape();assert BRepTools.Read_s(s,str(root/f'instance_{i:04d}.brep'),BRep_Builder());return s
mate=read(554);results={}
for i in [555,556,557,558]:
 tr=gp_Trsf();tr.SetTranslation(gp_Vec(0,0,-.05));part=BRepBuilderAPI_Transform(read(i),tr,True).Shape();op=BRepAlgoAPI_Common(part,mate);op.Build();g=GProp_GProps();BRepGProp.VolumeProperties_s(op.Shape(),g);results[str(i)]=dict(common_volume_mm3=g.Mass(),boolean_done=bool(op.IsDone()));print(i,results[str(i)],flush=True)

report=dict(method="OpenCascade exact BRep Boolean Common",target_translation_from_cad_mm=[0,0,-.05],mate=554,components=results,solid_interference_detected=any(abs(r["common_volume_mm3"])>1e-8 or not r["boolean_done"]for r in results.values()),source_sha256={str(i):hashlib.sha256((root/f"instance_{i:04d}.brep").read_bytes()).hexdigest()for i in [554,555,556,557,558]},scope="Nominal pin/mate solid intersection only; FCL triangle contact is preserved separately; no tolerance or hardware certification")
(HERE/"generated/contact_task/pin-brep-audit.json").write_text(json.dumps(report,indent=2)+"\n")
