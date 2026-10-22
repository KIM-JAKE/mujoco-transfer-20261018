"""Replace contact approximations with finely tessellated original CAD + SDF."""
from pathlib import Path
import numpy as np
from pxr import UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf

def apply(stage,root,body_paths,resolution=768):
 # Finite numerical contact compliance prevents stiff SDF contact chatter.
 # Bodies and rails remain rigid; these unmeasured solver parameters are audited.
 fixture_material=UsdShade.Material.Define(stage,'/World/Materials/fixture_contact');friction=UsdPhysics.MaterialAPI.Apply(fixture_material.GetPrim());friction.CreateStaticFrictionAttr(.6);friction.CreateDynamicFrictionAttr(.51);friction.CreateRestitutionAttr(0)
 arrays=np.load(root/'generated/contact_task/precision-cad.npz');count=0
 for identifier in [538,539,540,541,545,546,547,548]:
  name=f'instance_{identifier:04d}';path='/World/Environment/m80/'+name
  prim=stage.GetPrimAtPath(path)
  if not prim:continue
  mesh=UsdGeom.Mesh(prim);key=name
  mesh.GetPointsAttr().Set(arrays[key+'_vertices'].astype(np.float32));faces=arrays[key+'_faces'];mesh.GetFaceVertexCountsAttr().Set([3]*len(faces));mesh.GetFaceVertexIndicesAttr().Set(faces.reshape(-1))
  UsdGeom.Xformable(prim).ClearXformOpOrder();UsdShade.MaterialBindingAPI.Apply(prim).Bind(fixture_material,UsdShade.Tokens.strongerThanDescendants,'physics');count+=1
 for name,identifier in [('ring1',542),('ring2',549)]:
  parent=body_paths['workpiece_'+name]
  contact=stage.GetPrimAtPath(parent+'/Contact')
  if contact:contact.SetActive(False)
  mesh=UsdGeom.Mesh.Define(stage,parent+'/PrecisionContact');prim=mesh.GetPrim();key=f'instance_{identifier:04d}'
  mesh.CreatePointsAttr(arrays[key+'_vertices'].astype(np.float32));faces=arrays[key+'_faces'];mesh.CreateFaceVertexCountsAttr([3]*len(faces));mesh.CreateFaceVertexIndicesAttr(faces.reshape(-1));mesh.CreateSubdivisionSchemeAttr('none')
  UsdPhysics.CollisionAPI.Apply(prim);UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('sdf')
  sdf=PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim);sdf.CreateSdfResolutionAttr(resolution);sdf.CreateSdfSubgridResolutionAttr(6)
  collision=PhysxSchema.PhysxCollisionAPI.Apply(prim);collision.CreateContactOffsetAttr(.0001);collision.CreateRestOffsetAttr(0)
  material=UsdShade.Material(stage.GetPrimAtPath('/World/Materials/steel'))
  UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
  UsdGeom.Imageable(prim).CreateVisibilityAttr('invisible');count+=1
 # The pin's slotted shell is concave. A single convex hull fills its
 # insertion relief; use the original closed shell CAD as a separate SDF.
 pin=np.load(root/'generated/contact_task/precision-pin-main.npz')
 for identifier,moving in [(554,False),(557,True)]:
  key=f'instance_{identifier:04d}'
  if moving:
   stage.GetPrimAtPath(body_paths['workpiece_pin']+'/Contact/pin_mesh_206').SetActive(False)
   path=body_paths['workpiece_pin']+'/PrecisionPinShell'
  else:path='/World/Environment/m80/instance_0554'
  mesh=UsdGeom.Mesh.Define(stage,path);prim=mesh.GetPrim();mesh.CreatePointsAttr(pin[key+'_vertices'].astype(np.float32));f=pin[key+'_faces'];mesh.CreateFaceVertexCountsAttr([3]*len(f));mesh.CreateFaceVertexIndicesAttr(f.reshape(-1));mesh.CreateSubdivisionSchemeAttr('none');UsdGeom.Xformable(prim).ClearXformOpOrder()
  UsdPhysics.CollisionAPI.Apply(prim);UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('sdf'if moving else'none')
  if moving:
   sdf=PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim);sdf.CreateSdfResolutionAttr(512);sdf.CreateSdfSubgridResolutionAttr(6)
  ca=PhysxSchema.PhysxCollisionAPI.Apply(prim);ca.CreateContactOffsetAttr(.0001);ca.CreateRestOffsetAttr(0)
  material=UsdShade.Material(stage.GetPrimAtPath('/World/Materials/steel'))if moving else fixture_material;UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,UsdShade.Tokens.strongerThanDescendants,'physics');UsdGeom.Imageable(prim).CreateVisibilityAttr('invisible');count+=1
 # The closed tail CAD has a strongly concave profile. Its convex hull
 # fills 5.73 times the CAD volume and invents surfaces under the fingers.
 for mesh_name in ['pin_mesh_204','pin_mesh_207']:
  tail=stage.GetPrimAtPath(body_paths['workpiece_pin']+'/Contact/'+mesh_name);assert tail
  UsdPhysics.MeshCollisionAPI(tail).GetApproximationAttr().Set('sdf');tail_sdf=PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(tail);tail_sdf.CreateSdfResolutionAttr(512);tail_sdf.CreateSdfSubgridResolutionAttr(6);count+=1
 return dict(pin_closed_original_mesh_sdf=[204,207],pin_tail_sdf_resolution=512,concave_tail_convex_overfill_corrected=True,compliance_scope='clamp pads only; all metal insertion contacts remain rigid',contact_stiffness_n_m=0,contact_damping_n_s_m=0,precision_cad_colliders=count,sdf_resolution=resolution,pin_sdf_resolution=512,cad_deflection_m=.000001)
