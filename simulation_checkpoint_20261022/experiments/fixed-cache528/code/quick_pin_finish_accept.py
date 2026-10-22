p='pin';set_active(p);G=np.array(plan['goals'][p]['P']);P,Rp=pose(p)
if np.linalg.norm(P-G)>.0015:raise RuntimeError('Pin has not reached final depth: '+str(P.tolist()))
clamp_pin(True);s=narrow_push['side'];hp,hR=hand(s);hand_line(s,hp+hR@np.array([0,0,.1034])-[0,0,.05],hR,3.,'pin_final_withdraw_contact');wait(1.,'pin_installed_hold');final_checks={}
for p in ['ring1','ring2','pin']:
 P,Rp=pose(p);Gp=np.array(plan['goals'][p]['P']);Rg=np.array(plan['goals'][p]['R']);e=float(np.linalg.norm(P-Gp));axis=float(math.acos(np.clip(Rp[:,2]@Rg[:,2],-1,1)));distances={side:float(np.linalg.norm(hand(side)[0]+hand(side)[1]@np.array([0,0,.1034])-P))for side in arms};final_checks[p]=dict(position_error_m=e,axis_error_rad=axis,hand_tcp_distances_m=distances)
 if e>.0015 or axis>.05 or min(distances.values())<.04:raise RuntimeError('Final released retention failed '+p+' '+str(final_checks[p]))
if 'pin'not in completed:completed.append('pin')
recovered=report.pop('error',None);report.update(state='staged_three_parts_assembled',full_task_pass=False,final_assembly_pass=True,continuous_cycle_pass=False,autonomous_policy_verified=False,cycle_time_s=None,simulated_elapsed_time_s=c['now'](),recovered_initial_error=recovered,checkpoint_source_run='native-roas-table-bin-quick07',starts_with_rings_already_installed=True,manual_intervention=True,trial_reset_count=len(report.get('trial_resets',[])),acceptance_tolerance_m=.0015,pin_axial_yaw_unconstrained=True,final_checks=final_checks,end_positions={p:pose(p)[0].tolist()for p in parts})
trace.append(dict(t=c['now'](),q=art.get_dof_positions()[0].copy(),parts=np.array([parts[p].get_transforms()[0]for p in parts]),links=art.get_link_transforms()[0].copy()))
from pxr import UsdGeom,Gf
from fast_physx_camera import render
for name,eye,target,focal in [('FinalGlobal',[-1.5,-1.85,1.95],[0,-.18,1.18],35),('FinalAssembly',[-.65,-.05,1.63],[-.22,.25,1.535],50)]:
 camera=UsdGeom.Camera.Define(stage,'/World/Cameras/'+name);camera.CreateFocalLengthAttr(focal);camera.CreateHorizontalApertureAttr(32);xf=UsdGeom.Xformable(camera);xf.ClearXformOpOrder();xf.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse());rgb=render(stage,str(camera.GetPath()),640,480)
 with(out/(name+'.ppm')).open('wb')as fp:fp.write(chr(10).join(['P6','640 480','255','']).encode()+rgb.tobytes())
answer=dict(completed=completed,checks=final_checks)
