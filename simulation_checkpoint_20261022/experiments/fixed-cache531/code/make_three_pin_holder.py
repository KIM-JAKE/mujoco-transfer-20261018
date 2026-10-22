"""Proposed small three-pin block, STEP/STL in millimetres.

Not a fixture verified on site. Receives the narrow CAD +Z tip pointing down;
the larger 10.5 mm opposite end must never be forced into this hole.
"""
from pathlib import Path
import json
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
from OCP.StlAPI import StlAPI_Writer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepCheck import BRepCheck_Analyzer
OUT=Path(__file__).resolve().parent/'bin-picking';OUT.mkdir(exist_ok=True)
shape=BRepPrimAPI_MakeBox(120,35,18).Shape()
for x in [25,60,95]:
    tool=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,17.5,14),gp_Dir(0,0,1)),3.0,5).Shape()
    cut=BRepAlgoAPI_Cut(shape,tool);cut.Build();assert cut.IsDone();shape=cut.Shape()
assert BRepCheck_Analyzer(shape).IsValid()
w=STEPControl_Writer();w.Transfer(shape,STEPControl_AsIs);assert int(w.Write(str(OUT/'three-pin-holder-proposal.step')))==1
BRepMesh_IncrementalMesh(shape,.08);w=StlAPI_Writer();w.ASCIIMode=False;assert w.Write(shape,str(OUT/'three-pin-holder-proposal-mm.stl'))
(OUT/'three-pin-holder-proposal.json').write_text(json.dumps(dict(units='mm',size=[120,35,18],hole_diameter=6.0,hole_pitch=35,hole_depth=4,hole_centers=[[x,17.5,18]for x in [25,60,95]],pin_CAD_positive_Z_tip_points_down=True,status='design_proposal_NOT_contact_or_field_validated',additional_table=False,installation='existing supply shelf',hardware_tested=False),indent=2))
print('THREE_PIN_HOLDER_CAD_WRITTEN',flush=True)
