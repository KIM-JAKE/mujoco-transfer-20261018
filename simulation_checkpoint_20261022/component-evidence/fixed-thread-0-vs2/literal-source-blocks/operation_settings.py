if startup_config.get('native_contact_offset_m'):
    offset=float(startup_config['native_contact_offset_m']);assert .00005<=offset<=.001
    changed=0
    for prim in stage.Traverse():
        if '_finger/' in str(prim.GetPath()) and '_NativeHand/' in str(prim.GetPath()) and prim.HasAPI(UsdPhysics.CollisionAPI):
            api=PhysxSchema.PhysxCollisionAPI.Apply(prim);api.CreateContactOffsetAttr(offset);api.CreateRestOffsetAttr(0.);changed+=1
    sim.flush_changes();report['native_contact_generation']=dict(contact_offset_m=offset,rest_offset_m=0.,colliders=changed,applied_after_random_setup=True,collision_geometry_unchanged=True,friction_unchanged=True)
if startup_config.get('operation_solver_position_iterations'):
    operation_position=int(startup_config['operation_solver_position_iterations']);operation_velocity=int(startup_config.get('operation_solver_velocity_iterations',32))
    assert 16<=operation_position<=256 and 4<=operation_velocity<=128
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            api=PhysxSchema.PhysxRigidBodyAPI(prim);api.CreateSolverPositionIterationCountAttr(operation_position);api.CreateSolverVelocityIterationCountAttr(operation_velocity)
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            api=PhysxSchema.PhysxArticulationAPI(prim);api.CreateSolverPositionIterationCountAttr(operation_position);api.CreateSolverVelocityIterationCountAttr(operation_velocity)
    sim.flush_changes();report['operation_solver_iterations']=dict(position=operation_position,velocity=operation_velocity,applied_after_random_setup=True)
