"""Tessellate original closed CAD at 1um for contact-rich PhysX assembly."""
from pathlib import Path
import json,numpy as np,argparse
p=argparse.ArgumentParser();p.add_argument('--pin',action='store_true');p.add_argument('--pin-main',action='store_true');p.add_argument('--pin-shaft',action='store_true');args=p.parse_args()
MOVING=[556]if args.pin_shaft else[557]if args.pin_main else[555,556,557,558]if args.pin else[542,549]
IDENTIFIERS=[556]if args.pin_shaft else[554,557]if args.pin_main else[554,555,556,557,558]if args.pin else[538,539,540,541,542,545,546,547,548,549]
OUTPUT='precision-pin-shaft'if args.pin_shaft else'precision-pin-main'if args.pin_main else'precision-pin'if args.pin else'precision-cad'
from OCP.BRep import BRep_Builder,BRep_Tool
from OCP.BRepTools import BRepTools
from OCP.TopoDS import TopoDS_Shape,TopoDS
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_REVERSED
from OCP.TopLoc import TopLoc_Location
import trimesh
from prepare import HERE,SOURCES
DUO=SOURCES['cap_button'].parent.parent
CAD=DUO.parent/'insertion_20261013/cad'
config=json.loads((DUO/'cap_button_parallel_alt_20261016/config.json').read_text())
arrays={};meta={}
for identifier in IDENTIFIERS:
 shape=TopoDS_Shape();path=CAD/f'instance_{identifier:04d}.brep';assert BRepTools.Read_s(shape,str(path),BRep_Builder())
 BRepMesh_IncrementalMesh(shape,.001,False,.04,False).Perform()
 vertices=[];faces=[];ex=TopExp_Explorer(shape,TopAbs_FACE)
 while ex.More():
  face=TopoDS.Face(ex.Current());loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,loc)
  if tri is not None:
   offset=len(vertices)
   for j in range(1,tri.NbNodes()+1):
    point=tri.Node(j).Transformed(loc.Transformation());vertices.append([point.X()*.001,point.Y()*.001,point.Z()*.001])
   for j in range(1,tri.NbTriangles()+1):
    a,b,c=tri.Triangle(j).Get()
    if face.Orientation()==TopAbs_REVERSED:b,c=c,b
    faces.append([a+offset-1,b+offset-1,c+offset-1])
  ex.Next()
 vertices=np.array(vertices)
 if identifier in MOVING:vertices-=np.array(next(p['cad_origin_m']for p in config['parts']if identifier in p['source_ids']))
 mesh=trimesh.Trimesh(vertices,np.array(faces),process=False)
 mesh.merge_vertices(digits_vertex=7)
 mesh.update_faces(mesh.nondegenerate_faces(height=1e-10));mesh.remove_unreferenced_vertices()
 if identifier in MOVING:
  if not mesh.is_watertight:mesh.merge_vertices(digits_vertex=6)
  if args.pin_shaft and not mesh.is_watertight:
   before=float(abs(mesh.volume));trimesh.repair.fill_holes(mesh);print('SHAFT_REPAIR',bool(mesh.is_watertight),'volume_delta_m3',float(abs(mesh.volume))-before,flush=True)
  assert mesh.is_watertight,(identifier,'dynamic mesh is not watertight')
 key=f'instance_{identifier:04d}';arrays[key+'_vertices']=np.array(mesh.vertices);arrays[key+'_faces']=np.array(mesh.faces)
 meta[key]=dict(vertices=len(mesh.vertices),triangles=len(mesh.faces),watertight=bool(mesh.is_watertight),bounds=mesh.bounds.tolist(),deflection_m=.000001,volume_m3=float(abs(mesh.volume)))
 print(key,meta[key],flush=True)
np.savez_compressed(HERE/f'generated/contact_task/{OUTPUT}.npz',**arrays)
(HERE/f'generated/contact_task/{OUTPUT}.json').write_text(json.dumps(meta,indent=2)+'\n')
