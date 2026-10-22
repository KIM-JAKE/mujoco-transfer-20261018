from pathlib import Path
import ast,json,hashlib,difflib
base=Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs/native-roas-table-fixed-front-v522-entry-before-contact-pin-component/code_snapshot/direct_pin_skill.py');out=Path('/tmp/fixed526-postrelease-rgbd');s=base.read_text()
helper='''def _pin_postrelease_sdk_evidence(actual_H,command_H,withdraw_H,actual_fingers,command_fingers):
    """Public SDK completion only; never a PIN pose or force measurement."""
    A,C,W=[np.asarray(x,float)for x in [actual_H,command_H,withdraw_H]]
    a,c=[np.asarray(x,float)for x in [actual_fingers,command_fingers]]
    if any(x.shape!=(4,4)or not np.isfinite(x).all()for x in [A,C,W])or any(x.shape!=(2,)or not np.isfinite(x).all()for x in [a,c]):raise ValueError('Finite public SDK Hand/finger state required')
    actual_translation=float(np.linalg.norm(A[:3,3]-W[:3,3]));command_translation=float(np.linalg.norm(C[:3,3]-W[:3,3]))
    actual_rotation=float(np.linalg.norm(log(A[:3,:3]@W[:3,:3].T)));command_rotation=float(np.linalg.norm(log(C[:3,:3]@W[:3,:3].T)))
    released=bool(np.max(abs(c-.009))<=1e-6 and np.max(abs(a-.009))<=.0005)
    withdrawn=bool(max(actual_translation,command_translation)<.0015 and max(actual_rotation,command_rotation)<.05)
    return dict(confirmed=bool(released and withdrawn),release_confirmed=released,withdraw_confirmed=withdrawn,actual_fingers_m=a.tolist(),command_fingers_m=c.tolist(),release_target_per_finger_m=.009,release_per_finger_tolerance_m=.0005,actual_withdraw_translation_m=actual_translation,command_withdraw_translation_m=command_translation,actual_withdraw_rotation_rad=actual_rotation,command_withdraw_rotation_rad=command_rotation,withdraw_translation_limit_m=.0015,withdraw_rotation_limit_rad=.05,source='Current public gripper encoders, commanded positions and encoder FK only',uses_free_body_poses=False)


def _pin_postrelease_rgbd_evidence(pose,goal):
    """Same robust cap and full-depth tolerances, independent of robot Hand."""
    P=np.asarray(pose.get('center_m'),float);a=np.asarray(pose.get('axis'),float);goal=np.asarray(goal,float)
    if P.shape!=(3,)or a.shape!=(3,)or goal.shape!=(3,)or not np.isfinite(np.r_[P,a,goal]).all()or abs(np.linalg.norm(a)-1)>1e-6:raise ValueError('Finite directly measured PIN center/axis required')
    expected=float(pose.get('cap_expected_radius_m',float('nan')));radius=float(pose.get('visible_end_radius_m',float('nan')));count=int(pose.get('cap_circular_edge_inliers',0));total=int(pose.get('cap_total_hull_edge_points',0));coverage=float(pose.get('cap_edge_coverage_deg',0));rms=float(pose.get('cap_circular_edge_rms_m',float('inf')))
    direct=bool(pose.get('part')=='pin' and pose.get('uses_depth_sensor')is True and pose.get('uses_simulator_body_poses')is False and pose.get('robust_cap_circle_from_pixels')is True and pose.get('lateral_from_previous_RGBD_grasp_and_encoders')is False and min(abs(expected-.00525),abs(expected-.0025))<1e-8 and np.isfinite(radius)and abs(radius-expected)<=.00015 and count>=12 and total>=12 and count/total>=.75 and coverage>=240 and np.isfinite(rms)and rms<=.00006)
    position=float(np.linalg.norm(P-goal));angle=float(np.arccos(np.clip(a[2],-1,1)))
    return dict(confirmed=bool(direct and position<.0015 and angle<.05),direct_class_end_circle=direct,position_error_m=position,axis_error_rad=angle,position_limit_m=.0015,axis_limit_rad=.05,cap_edge_coverage_deg=coverage,cap_circular_edge_inliers=count,cap_total_hull_edge_points=total,cap_circular_edge_rms_m=rms,center_m=P.tolist(),axis=a.tolist(),source='New postrelease RGB-D pixels, original class-radius cap/plane gates and calibrated jig goal',uses_free_body_poses=False)


'''
marker='def execute(ctx):\n';assert s.count(marker)==1;s=s.replace(marker,helper+marker,1)
old="    qualified_full_depth=bool(pin_config.get('pin_full_depth_require_robust_circle'))\n";new=old+"    postrelease_rgbd=bool(pin_config.get('pin_postrelease_rgbd_retention',False))\n    if postrelease_rgbd and not(qualified_full_depth and visual_preinsert and pin_config.get('fixed_supply_mode') and pin_config.get('robust_pin_cap_circle') and pin_config.get('pin_full_depth_near_axis_views')):raise ValueError('Postrelease RGB-D requires existing fixed-mode qualified full-depth and near-axis robust-circle policies')\n    if postrelease_rgbd and ctx.get('defer_retention_to_recipe'):raise ValueError('Postrelease RGB-D requires the existing nondeferred0.5s retention window')\n";assert s.count(old)==1;s=s.replace(old,new,1)
old="    def observe(label,grasp=None,require_circle=False):\n        H=arm.fk(art.get_dof_positions()[0,ids]);prefetched=ctx.pop('prefetched_initial',None) if grasp is None else None\n";new="""    def observe(label,grasp=None,require_circle=False,world_reference=None):
        H=arm.fk(art.get_dof_positions()[0,ids]);prefetched=ctx.pop('prefetched_initial',None) if grasp is None and world_reference is None else None
        anchored_world=None
        if world_reference is not None:
            if not(postrelease_rgbd and label=='pin_postrelease_retention_RGBD' and require_circle):raise ValueError('World-anchored observation is restricted to qualified postrelease retention')
            anchored_world=np.asarray(world_reference,float).copy()
            if anchored_world.shape!=(4,4)or not np.isfinite(anchored_world).all():raise ValueError('Finite last qualified public world PIN pose required')
            grasp=np.linalg.inv(H)@anchored_world
""";assert s.count(old)==1;s=s.replace(old,new,1)
old="        views=_pin_ordered_camera_views(offsets,label,pin_config)\n        views=_pin_full_depth_near_axis_views(H,grasp,offsets,label,pin_config,require_circle)+views\n";new="""        view_label='pin_full_depth_RGBD'if anchored_world is not None else label
        views=_pin_ordered_camera_views(offsets,view_label,pin_config)
        views=_pin_full_depth_near_axis_views(H,grasp,offsets,view_label,pin_config,require_circle)+views
""";assert s.count(old)==1;s=s.replace(old,new,1)
old="            else:f=vision.request(e,arm.fk(art.get_dof_positions()[0,ids]))\n";new="""            else:
                capture_H=arm.fk(art.get_dof_positions()[0,ids])
                if anchored_world is not None:e['tracked_grasp']=(np.linalg.inv(capture_H)@anchored_world).tolist()
                f=vision.request(e,capture_H)
""";assert s.count(old)==1;s=s.replace(old,new,1)
s=s.replace("if qualified_full_depth and label=='pin_full_depth_RGBD'", "if qualified_full_depth and label in ['pin_full_depth_RGBD','pin_postrelease_retention_RGBD']")
old="                report.setdefault('pin_full_depth_qualified_observations',[]).append(";new="                report.setdefault('pin_postrelease_qualified_observations'if anchored_world is not None else'pin_full_depth_qualified_observations',[]).append(";assert s.count(old)==1;s=s.replace(old,new,1)
old="    for prim in stage.Traverse():\n        if ctx.get('close_pin_clamp',True)";new="    retained_world_pose=frame(pose['center_m'],axis_frame(pose['axis']))if postrelease_rgbd else None\n    for prim in stage.Traverse():\n        if ctx.get('close_pin_clamp',True)";assert s.count(old)==1;s=s.replace(old,new,1)
old="wait(0. if ctx.get('defer_retention_to_recipe') else .5,'pin_retention');report['retention_deferred_to_recipe']=bool(ctx.get('defer_retention_to_recipe'));tr=truth['pin'].get_transforms()[0];";new="""wait(0. if ctx.get('defer_retention_to_recipe') else .5,'pin_retention');report['retention_deferred_to_recipe']=bool(ctx.get('defer_retention_to_recipe'))
    if postrelease_rgbd:
        actual=art.get_dof_positions()[0]
        sdk=_pin_postrelease_sdk_evidence(arm.fk(actual[ids]),arm.fk(cmd[0,ids]),withdraw,actual[fi],cmd[0,fi]);report['pin_postrelease_SDK_completion']=sdk
        if not sdk['confirmed']:raise RuntimeError('Public SDK release/withdraw completion not confirmed; postrelease retention blocked')
        requested_after=time.perf_counter()-started
        retained=observe('pin_postrelease_retention_RGBD',require_circle=True,world_reference=retained_world_pose)
        captured=float(retained.get('captured_wall_s',float('nan')))
        if not np.isfinite(captured)or captured<requested_after:raise RuntimeError('Postrelease retention requires a new capture after release, withdrawal and original retention wait')
        measured=_pin_postrelease_rgbd_evidence(retained['result']['pose'],goal);measured.update(captured_wall_s=captured,ready_wall_s=retained.get('ready_wall_s'),after_release_withdraw_and_original_retention_wait=True,camera_world_anchor_from_last_qualified_RGBD=retained_world_pose.tolist(),robot_Hand_does_not_move_world_anchor=True,original_retention_wait_s=.5,one_observation_operation=True)
        report['pin_postrelease_RGBD_retention']=measured
        if not measured['confirmed']:raise RuntimeError('Fresh postrelease RGB-D retention not confirmed within original1.5mm/.05rad; completion blocked')
        # Simulator truth is explicitly EVALUATION ONLY under this flag.
        # It cannot set or change the controller's public-sensor verdict.
        try:
            evaluated=truth['pin'].get_transforms()[0];report['pin_retention_EVALUATION_ONLY']=dict(used_for_control=False,position_error_m=float(np.linalg.norm(evaluated[:3]-body_goal)),axis_error_rad=float(np.arccos(np.clip(1-2*(evaluated[3]**2+evaluated[4]**2),-1,1))))
        except Exception as evaluation_error:report['pin_retention_EVALUATION_ONLY']=dict(used_for_control=False,unavailable=str(evaluation_error))
        report.update(state='pin_single_stroke_contact_pass',pin_single_stroke_pass=True,final_position_error_m=measured['position_error_m'],final_axis_error_rad=measured['axis_error_rad'],pin_trial_wall_s=time.perf_counter()-started,retention_completion_source='Fresh postrelease RGB-D and public robot SDK only')
        return report
    tr=truth['pin'].get_transforms()[0];""";assert s.count(old)==1;s=s.replace(old,new,1)
ast.parse(s);target=out/'overrides/direct_pin_skill.py';target.write_text(s);(out/'direct_pin_skill.py.diff').write_text(''.join(difflib.unified_diff(base.read_text().splitlines(True),s.splitlines(True),fromfile=str(base),tofile=str(target))));(out/'config-delta.json').write_text(json.dumps(dict(pin_postrelease_rgbd_retention=True),indent=2)+'\n');(out/'source-manifest.json').write_text(json.dumps(dict(base=str(base),base_sha256=hashlib.sha256(base.read_bytes()).hexdigest(),source_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),new_flag='pin_postrelease_rgbd_retention',driver_whitelist_addition='pin_postrelease_rgbd_retention',default_off=True,new_imports=[],physics_jobs=0,whole_task_qualified=False),indent=2)+'\n');print(hashlib.sha256(target.read_bytes()).hexdigest())
