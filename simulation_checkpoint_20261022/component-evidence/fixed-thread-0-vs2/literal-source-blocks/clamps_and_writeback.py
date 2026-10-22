tape=[];actuators=[];peak=0.;physical=0.;active_physics_substeps=1;corrections=[];floor_offset=np.zeros(3);live_pin_grasp=None
clamp_targets={p:[(UsdPhysics.DriveAPI(joint.GetPrim(),'linear').GetTargetPositionAttr(),float(joint.GetLowerLimitAttr().Get()if p=='pin'else joint.GetUpperLimitAttr().Get()))for joint in joints[p]]for p in truth}
report['usd_clamp_targets_batched']=True
for j,p in enumerate(truth):
    last_clamp[j]=controls[0,j]
    for joint in joints[p]:
        limit=joint.GetLowerLimitAttr().Get()if p=='pin'else joint.GetUpperLimitAttr().Get();drive=UsdPhysics.DriveAPI(joint.GetPrim(),'linear');value=float(limit*last_clamp[j])
        if drive.GetTargetPositionAttr().Get()!=value:drive.GetTargetPositionAttr().Set(value)
sim.flush_changes()
if clamp_driver is not None:clamp_driver.command(controls[0])
settings=carb.settings.get_settings();settings.set_bool('/physics/updateToUsd',False);settings.set_bool('/physics/updateVelocitiesToUsd',False)
