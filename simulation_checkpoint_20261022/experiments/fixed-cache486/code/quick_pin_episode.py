from pathlib import Path
from isaac_motion import Audit
report.setdefault('trial_resets',[]).append(dict(sim_time_s=c['now'](),part='empty_right_arm',reason='New independent pin attempt; reset empty arm to collision-free initial configuration after failed feedback',workpiece_moved=False));cells=dict(zip(c['step'].__code__.co_freevars,c['step'].__closure__));report.setdefault('aborted_trial_command_audits',[]).append(cells['motion_audit'].cell_contents.report());cmd[0,c['aidx']['right']]=c['initial_pose']['ready_q']['right'];art.set_dof_positions(cmd,c['idx']);art.set_dof_velocities(np.zeros_like(cmd),c['idx']);art.set_dof_position_targets(cmd,c['idx']);cells['motion_audit'].cell_contents=Audit(cmd,c['aidx'],dt);cells['previous_command'].cell_contents[:]=cmd;checkers['right'].detach_payload();wait(.5,'pin_independent_trial_robot_reset');p='pin';set_active(p)
from pxr import UsdGeom,Gf,UsdShade,PhysxSchema
for j in [0,1]:
 joint=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath('/World/Clamps/pin/jaw_'+str(j)+'/Actuator'));a=joint.GetLocalPos0Attr().Get();joint.GetLocalPos0Attr().Set(Gf.Vec3f(a[0],a[1],1.511));joint.GetUpperLimitAttr().Set(0.);UsdPhysics.DriveAPI(joint.GetPrim(),'linear').GetTargetPositionAttr().Set(0.)
 for op in UsdGeom.Xformable(stage.GetPrimAtPath('/World/Clamps/pin/jaw_'+str(j)+'/Jaw')).GetOrderedXformOps():
  if op.GetOpType()==UsdGeom.XformOp.TypeScale:op.Set(Gf.Vec3f(.001,.0015,.003))
for name,paths_for_material,k in [('PinElastic',[c['paths']['workpiece_pin']],100000.),('SpringDetents',['/World/Environment/m80/instance_'+v for v in ['0560','0562','0564','0566']],10000.)]:
 m=UsdShade.Material.Define(stage,'/World/Materials/'+name);ma=UsdPhysics.MaterialAPI.Apply(m.GetPrim());ma.CreateStaticFrictionAttr(.4);ma.CreateDynamicFrictionAttr(.3);ma.CreateRestitutionAttr(0.);pa=PhysxSchema.PhysxMaterialAPI.Apply(m.GetPrim());pa.CreateCompliantContactStiffnessAttr(k);pa.CreateCompliantContactDampingAttr(15.);pa.CreateCompliantContactAccelerationSpringAttr(False)
 for pr in stage.Traverse():
  if any(str(pr.GetPath()).startswith(path)for path in paths_for_material)and pr.HasAPI(UsdPhysics.CollisionAPI):UsdShade.MaterialBindingAPI.Apply(pr).Bind(m,UsdShade.Tokens.strongerThanDescendants,'physics')
import omni.physx
omni.physx.get_physx_simulation_interface().flush_changes();report.update(pin_collision_model='Four coaxial convex cylinders, CAD outer dimensions; quick prototype approximation',pin_clamp_center_z_m=1.511,pin_clamp_pad_height_m=.006,pin_clamp_open_extra_m=0.,pin_contact_stiffness_n_m=100000.,spring_detent_contact_stiffness_n_m=10000.,contact_compliance_hardware_calibrated=False,pin_clamp_hardware_calibrated=False)
report.setdefault('trial_resets',[]).append(dict(sim_time_s=c['now'](),part='pin',reason='Independent trial: collar grasp and clearance from fixture clamp; compliant pin and spring contacts',to_position=[.19,-.79,.91605]));parts['pin'].set_transforms(np.array([[.19,-.79,.91605,1.,0.,0.,0.]],np.float32),np.array([0],np.int32));parts['pin'].set_velocities(np.zeros((1,6),np.float32),np.array([0],np.int32));wait(.3,'pin_coherent_trial_restocked')
P,Rp=pose(p);lp=np.array([0.,0.,0.]);Rsource=Rp@exp(np.array([0,0,np.pi/2]));tcp=P+Rp@lp;fingers('right',.022,'pin_preshape');pre=IK('right',tcp+[0,0,.07],Rsource);route=checkers['right'].route(q('right'),pre,.022,c['paths']['workpiece_pin'],seconds=20)
if route is None:raise RuntimeError('Pickup route absent')
move('right',route,4.,'pin_pregrasp',True);hand_line('right',tcp,Rsource,3.,'pin_descend');before=pose(p)[0].copy();fingers('right',.0057,'pin_close');hand_line('right',tcp+[0,0,.07],Rsource,3.,'pin_lift');wait(.5,'pin_lift_stable')
if pose(p)[0][2]-before[2]<.05:raise RuntimeError('Pickup failed')
def pin_one(P,label):
 p='pin';lp,L=relative('right',p);actual,Rp=pose(p)
 if Rp[2,2]<-.95:Rp=exp(np.array([np.pi,0,0]))@Rp
 n=Rp[:,2];k=np.cross(n,[0,0,1]);angle=math.atan2(np.linalg.norm(k),n[2]);Rg=(exp(k/np.linalg.norm(k)*angle)if np.linalg.norm(k)>1e-7 else np.eye(3))@Rp;hR=Rg@L.T;hp=np.array(P)-hR@lp;checkers['right'].attach_payload(c['paths']['workpiece_pin'],frame(lp,L));hand_line('right',hp+hR@np.array([0,0,.1034]),hR,4.,label)
 if np.linalg.norm(pose(p)[0]-P)>.015:raise RuntimeError('Pin lost during '+label)
pin_one(np.array([-.10,-.25,1.35]),'pin_orient_free');G=np.array(plan['goals']['pin']['P']);pin_one(np.array([G[0],-.05,G[2]-.045]),'pin_raise_front');pin_one(G-[0,0,.045],'pin_preinsert');pin_one(G-[0,0,.031],'pin_partial_insert');checkers['right'].detach_payload()
clamp_pin(True);fingers('right',.022,'pin_partial_release');hp,hR=hand('right');hand_line('right',hp+hR@np.array([0,0,.1034])-[0,0,.07],hR,3.,'pin_partial_withdraw');wait(.5,'pin_partial_retention')
if np.linalg.norm(pose('pin')[0]-(G-[0,0,.031]))>.002:raise RuntimeError('Partial pin retention failed')
s=narrow_push['side'];P=pose('pin')[0].copy();fingers(s,.06,'pin_bevel_open');
import isaac_collision_queries as query_module
namespace=dict(query_module.__dict__);exec(compile(Path(query_module.__file__).read_text(),query_module.__file__,'exec'),namespace);type(checkers['right']).path=namespace['Checker'].path
from pxr import PhysicsSchemaTools,Sdf
ch=checkers[s];ch.resolution_rad=.003
if not hasattr(ch,'margin_rows'):
 ch.margin_rows=[]
 for i,(body,source_path,op,encoded)in enumerate(ch.rows):
  if 'NativeHand'not in source_path:ch.margin_rows.append((body,source_path,op,encoded));continue
  original=UsdGeom.Mesh(op.GetAttr().GetPrim());v=np.asarray(original.GetPointsAttr().Get());center=(v.max(0)+v.min(0))/2;half=np.maximum((v.max(0)-v.min(0))/2,.001);inflated=center+(v-center)*(1+.0015/half);path='/World/PlanningQueries/'+s+'_margin_'+str(i);m=UsdGeom.Mesh.Define(stage,path);m.CreatePointsAttr(inflated.astype(np.float32));m.CreateFaceVertexCountsAttr(original.GetFaceVertexCountsAttr().Get());m.CreateFaceVertexIndicesAttr(original.GetFaceVertexIndicesAttr().Get());m.CreateSubdivisionSchemeAttr('none');m.CreateVisibilityAttr('invisible');newop=UsdGeom.Xformable(m).AddTransformOp(UsdGeom.XformOp.PrecisionDouble);ch.margin_rows.append((body,source_path,newop,PhysicsSchemaTools.encodeSdfPath(Sdf.Path(path))))
ch.original_rows=ch.rows;ch.rows=ch.margin_rows

preP=narrow_push['tcp']+[0,0,P[2]-G[2]-.003];preq=arms[s].solve(preP,narrow_push['R'],narrow_push['q']);route=checkers[s].route(q(s),preq,.06,None,seconds=40)if preq is not None else None
if route is None:raise RuntimeError('Bevel pusher approach failed')
from isaac_motion import duration
path=np.array(route);path[0]=cmd[0,c['aidx'][s]];c['set_phase']('pin_bevel_approach');T=8.
for a,b in zip(path[:-1],path[1:]):
 segment_T=duration([a,b],max(.6,T/(len(path)-1)));n=max(1,math.ceil(segment_T/dt))
 for k in range(n):
  u=(k+1)/n;h=u**3*(10-15*u+6*u*u);cmd[0,c['aidx'][s]]=a+(b-a)*h;step()
wait(.3,'pin_bevel_approach_settled');ch.rows=ch.original_rows;ch.resolution_rad=.035;report['pusher_approach_clearance_margin_m']=.0015;report['pusher_approach_path_resolution_rad']=.003

if np.linalg.norm(pose('pin')[0]-P)>.002:raise RuntimeError('Pin disturbed during pusher approach')
hand_line(s,narrow_push['tcp']+[0,0,P[2]-G[2]+.0009],narrow_push['R'],2.,'pin_bevel_contact');clamp_pin(False);hand_line(s,narrow_push['tcp'],narrow_push['R'],5.,'pin_bevel_finish_insert');wait(.5,'pin_narrow_push_settle');answer=dict(pin=pose('pin'),error=np.linalg.norm(pose('pin')[0]-G),contacts=[dict(pair=pair,force=force)for pair,force in c['current_contacts'].items()])
