"""Explicit quick-demo contact approximation; retains the CAD visual mesh."""
import numpy as np
from pxr import UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf

def apply(stage,body_path):
    root=body_path+'/CPUConvexContact';stage.RemovePrim(root);UsdGeom.Xform.Define(stage,root)
    specs=[('tail',.00525,-.01775,-.0065),('collar',.0065,-.0065,-.0035),('shaft',.0038,-.0035,.0135),('tip',.0025,.0135,.01775)]
    for name,radius,z0,z1 in specs:
        n=32;angle=np.arange(n)*2*np.pi/n;vertices=np.array([[radius*np.cos(a),radius*np.sin(a)-.00025,z]for z in [z0,z1]for a in angle]);faces=[]
        for i in range(n):
            j=(i+1)%n;faces.extend([[i,j,n+j],[i,n+j,n+i]])
        for i in range(1,n-1):faces.extend([[0,i+1,i],[n,n+i,n+i+1]])
        m=UsdGeom.Mesh.Define(stage,root+'/'+name);m.CreatePointsAttr(vertices.astype(np.float32));m.CreateFaceVertexCountsAttr([3]*len(faces));m.CreateFaceVertexIndicesAttr(np.asarray(faces).ravel());m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(1,.85,.12)]);UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull');c=PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim());c.CreateContactOffsetAttr(.00005);c.CreateRestOffsetAttr(0.);UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(UsdShade.Material.Get(stage,'/World/Materials/steel'),UsdShade.Tokens.strongerThanDescendants,'physics')
    return dict(model='Four coaxial convex cylinders, approximate CAD outer dimensions',length_m=.0355,tail_diameter_m=.0105,collar_diameter_m=.013,shaft_diameter_m=.0076,CAD_nominal_shaft_diameter_m=.008,radial_clearance_allowance_m=.0002,tip_diameter_m=.005,parts=4,original_slits_and_reliefs_not_simulated=True,hardware_validated=False)
