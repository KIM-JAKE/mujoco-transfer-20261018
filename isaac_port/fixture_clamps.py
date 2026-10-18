"""Finite-force fixture actuators. No workpiece is attached to these mechanisms.

Unmeasured jaw stroke/force are explicit simulation parameters, not CAD claims.
The optional legacy trial translates six sensor instances with their visuals.
This hypothetical test layout does not certify the original CAD or hardware.
"""
import math
from collections import deque
import numpy as np
from pxr import UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf

PARTS=['ring1','ring2','pin']

def apply(stage,body_paths,trial_layout=False):
    # Legacy experiment only. Explicit hypothetical layout corrections
    # affect complete visual and collision instances in this task layer only.
    shifted=[]
    for prim in list(stage.Traverse()) if trial_layout else []:
        if str(prim.GetPath()) not in [base+'/'+name for base in ['/World/Cell/Geometry/m80','/World/Environment/m80']for name in ['instance_0369','instance_0380','instance_0373','instance_0382','instance_0371','instance_0384']]:continue
        xf=UsdGeom.Xformable(prim)
        # Convert the local increment from world +Y through its parent frame.
        parent=UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent())
        dy=.012 if prim.GetName()in ['instance_0369','instance_0380']else (-.060 if prim.GetName()in ['instance_0371','instance_0384']else -.030)
        delta=parent.GetInverse().TransformDir(Gf.Vec3d(0,dy,0))
        op=xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble,'railClearance');op.Set(delta)
        xf.SetXformOpOrder([op]+[v for v in xf.GetOrderedXformOps()if v!=op])
        shifted.append(str(prim.GetPath()))
    assert len(shifted)==(12 if trial_layout else 0),shifted
    # Source inventory specifies an upright pin socket with a tip sleeve.
    # This passive hollow guide supports the bottom shaft during pickup;
    # a 12mm upward extraction clears its 6mm height. No part joint is added.
    from stock_fixture_geometry import pin_stock_sleeve
    vertices,faces,center=pin_stock_sleeve();sleeve=UsdGeom.Mesh.Define(stage,'/World/Environment/PinStockSleeve');sleeve.CreatePointsAttr(vertices.astype(np.float32));sleeve.CreateFaceVertexCountsAttr([3]*len(faces));sleeve.CreateFaceVertexIndicesAttr(faces.reshape(-1));sleeve.CreateSubdivisionSchemeAttr('none');sleeve.CreateDisplayColorAttr([Gf.Vec3f(.3,.35,.4)]);UsdGeom.Xformable(sleeve).AddTranslateOp().Set(Gf.Vec3d(*center));UsdPhysics.CollisionAPI.Apply(sleeve.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(sleeve.GetPrim()).CreateApproximationAttr('none');contact=PhysxSchema.PhysxCollisionAPI.Apply(sleeve.GetPrim());contact.CreateContactOffsetAttr(.00005);contact.CreateRestOffsetAttr(0);UsdShade.MaterialBindingAPI.Apply(sleeve.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/Materials/steel')),UsdShade.Tokens.strongerThanDescendants,'physics')
    specs={
      'ring1':dict(center=[-.123229342787646,.23984999618795,1.528],open_radius=.0301,stroke=.0024,half=[.00075,.001,.0004],count=3,sign=1,force=5),
      'ring2':dict(center=[-.239030957838364,.284249996187948,1.528],open_radius=.0240,stroke=.0020,half=[.00075,.001,.0004],count=3,sign=1,force=5),
      'pin':dict(center=[-.302030957838358,.227249996187946,1.537],open_radius=.0085,stroke=.0047,half=[.001,.0015,.001],count=2,sign=-1,force=3),
    }
    rows=[]
    material=UsdShade.Material.Define(stage,'/World/Materials/clamp_pad');friction=UsdPhysics.MaterialAPI.Apply(material.GetPrim());friction.CreateStaticFrictionAttr(1.0);friction.CreateDynamicFrictionAttr(.85);friction.CreateRestitutionAttr(0)
    compliant=PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim());compliant.CreateCompliantContactStiffnessAttr(1e6);compliant.CreateCompliantContactDampingAttr(100)
    for name,cfg in specs.items():
      for j in range(cfg['count']):
        angle=2*math.pi*j/cfg['count'];direction=np.array([math.cos(angle),math.sin(angle),0.])
        position=np.array(cfg['center'])+cfg['open_radius']*direction
        path=f'/World/Clamps/{name}/jaw_{j}';body=UsdGeom.Xform.Define(stage,path).GetPrim()
        xf=UsdGeom.Xformable(body);xf.AddTranslateOp().Set(Gf.Vec3d(*position));xf.AddOrientOp().Set(Gf.Quatf(math.cos(angle/2),Gf.Vec3f(0,0,math.sin(angle/2))))
        UsdPhysics.RigidBodyAPI.Apply(body);mass=UsdPhysics.MassAPI.Apply(body);mass.CreateMassAttr(.04);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-7))
        rb=PhysxSchema.PhysxRigidBodyAPI.Apply(body);rb.CreateDisableGravityAttr(True);rb.CreateSolverPositionIterationCountAttr(64);rb.CreateSolverVelocityIterationCountAttr(16)
        PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(0)
        box=UsdGeom.Cube.Define(stage,path+'/Jaw');box.CreateSizeAttr(2);box.CreateDisplayColorAttr([Gf.Vec3f(.95,.48,.08)])
        UsdGeom.Xformable(box).AddScaleOp().Set(Gf.Vec3f(*cfg['half']))
        UsdPhysics.CollisionAPI.Apply(box.GetPrim());coll=PhysxSchema.PhysxCollisionAPI.Apply(box.GetPrim());coll.CreateContactOffsetAttr(.00005);coll.CreateRestOffsetAttr(0)
        UsdShade.MaterialBindingAPI.Apply(box.GetPrim()).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
        joint=UsdPhysics.PrismaticJoint.Define(stage,path+'/Actuator');joint.CreateBody1Rel().SetTargets([body.GetPath()]);joint.CreateAxisAttr('X')
        joint.CreateLocalPos0Attr(Gf.Vec3f(*position));joint.CreateLocalRot0Attr(Gf.Quatf(math.cos(angle/2),Gf.Vec3f(0,0,math.sin(angle/2))))
        joint.CreateLowerLimitAttr(min(0,cfg['stroke']*cfg['sign']));joint.CreateUpperLimitAttr(max(0,cfg['stroke']*cfg['sign']))
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(40000);drive.CreateDampingAttr(80);drive.CreateMaxForceAttr(cfg['force']);drive.CreateTargetPositionAttr(0)
        # Jaws are contained in the fixture housing; mechanism-internal pairs
        # are excluded. All jaw-to-workpiece and jaw-to-robot pairs are active.
        filt=UsdPhysics.FilteredPairsAPI.Apply(body).CreateFilteredPairsRel()
        for identifier in ({'ring1':[538,539,540,541],'ring2':[545,546,547,548],'pin':[550,551,552,553,554]}[name]):
          filt.AddTarget(f'/World/Environment/m80/instance_{identifier:04d}')
        rows.append(dict(name=name,path=path,drive=drive,max_force=cfg['force'],closed=cfg['stroke']*cfg['sign'],initial=position,direction=direction))
    return rows,dict(layout='trial_rail_offsets' if trial_layout else 'original_cad',pin_stock_support=dict(model='passive hollow tip sleeve, assumed',inner_radius_m=.0026,outer_radius_m=.005,height_m=.006,center_m=center.tolist(),part_attachment=False),clamp_pad_material='rubber, assumed',clamp_contact_stiffness_n_m=1e6,clamp_contact_damping_n_s_m=100,fixture_model='finite-force internal expanding ring jaws + external pin jaws',clamp_parameters=specs,clamp_close_ramp_s={'ring1':.2,'ring2':.2,'pin':.1},clamp_drive_damping_n_s_m=80,rail_clearance_translation_m=({'0369,0380':[0,.012,0],'0373,0382':[0,-.030,0],'0371,0384':[0,-.060,0]} if trial_layout else {}),shifted_instances=shifted)

class Controller:
    def __init__(self,rows,soft_contact=False):
        self.rows=rows;self.requested={n:False for n in PARTS};self.verified={n:False for n in PARTS};self.events=[];self.since={n:None for n in PARTS};self.contact_history={n:deque(maxlen=24)for n in PARTS};self.close_started={}
        self.soft_contact=soft_contact;self.first_jaw_contact={}
        if soft_contact:
            for row in rows:
                if row['name']!='pin':row['drive'].GetMaxForceAttr().Set(2.)
    def update(self,t,ref,feedback,positions,quats,forces):
        reference=ref
        for name in PARTS:
            if name not in feedback.windows:continue
            ref=reference[feedback.windows[name][0]]if isinstance(reference,dict)else reference
            end=feedback.windows[name][2]
            pe,re=feedback.errors(name,end,positions,quats)
            eligible=ref>=feedback.insertion[name]if getattr(feedback,'fast_cycle',False)else ref>=end-.002
            if not self.requested[name] and eligible and np.linalg.norm(pe)<.00045 and np.linalg.norm(re)<.025:
                self.requested[name]=True;self.close_started[name]=t;self.events.append(dict(time_s=t,part=name,event='close_requested',position_error_m=float(np.linalg.norm(pe))))
            if self.requested[name]:
                ramp=.08 if getattr(feedback,'fast_cycle',False)else(.1 if name=='pin'else .2)
                fraction=min(1.,(t-self.close_started[name])/ramp)
                for row in self.rows:
                    if row['name']==name:
                        row['drive'].GetTargetPositionAttr().Set(row['closed']*fraction)
                        if self.soft_contact and name!='pin':
                            if forces.get(row['path'],0)>.1 and row['path']not in self.first_jaw_contact:
                                self.first_jaw_contact[row['path']]=t
                                self.events.append(dict(time_s=t,part=name,event='soft_jaw_contact',jaw=row['path']))
                            elapsed=t-self.first_jaw_contact.get(row['path'],t)
                            u=float(np.clip(elapsed/.08,0,1));weight=u**3*(10-15*u+6*u*u)
                            row['drive'].GetMaxForceAttr().Set(2.+(row['max_force']-2.)*weight)
            count=sum(v>.5 for k,v in forces.items()if k.startswith('/World/Clamps/'+name+'/'))
            if self.requested[name]:
                history=self.contact_history[name];history.append((t,count>=2))
                stable=(len(history)>=16 and history[-1][0]-history[0][0]>=.12 and sum(v for _,v in history)/len(history)>=.8)
                if stable and np.linalg.norm(pe)<.00045 and np.linalg.norm(re)<.025 and not self.verified[name]:
                    self.verified[name]=True;self.events.append(dict(time_s=t,part=name,event='bilateral_clamp_contact_verified',contact_coverage=sum(v for _,v in history)/len(history),position_error_m=float(np.linalg.norm(pe))))
