from pathlib import Path
import ast,json,difflib
out=Path('/tmp/fixed478-pin-qualified-preload');base=Path('/tmp/fixed476-full/overrides/direct_pin_skill.py');s=Path('/tmp/fixed477-pin-bounded-recovery/direct_pin_skill.py').read_text()
a="""    pin_hold_reference_G=G.copy()
    if ctx.get('before_transport')"""
b="""    if pin_config.get('pin_qualified_postlift_preload',False):
        if not pin_sensor_hold or not pin_axis_geometry or not ctx.get('set_grip_hold'):raise ValueError('Qualified PIN preload requires fresh physical-axis RGB-D/SDK hold confirmation and native hold callback')
        target_preload=float(pin_config.get('native_loaded_preload_after_contact_m_s',.015));ramp_s=float(pin_config.get('native_loaded_preload_ramp_s',.3))
        if not 0<target_preload<=.015 or not 0<=ramp_s<=2.:raise ValueError('Invalid qualified PIN preload ramp')
        preload_reference_G=G.copy()
        # Reuse the same validated native postcontact ramp as ring pickups.
        # This changes velocity PRELOAD, not the30N/pad effort cap or model.
        ctx['set_grip_hold'](True,preload_m_s=target_preload)
        wait(ramp_s+.05,'pin_confirmed_postlift_preload_ramp')
        # Same existing calibrated lift-confirm view order and fresh image;
        # no cached pose and no transport until the original hold gate passes.
        v=observe('pin_lift_confirm_RGBD',G,require_circle=True);confirmed_G=measured_grasp(v,G);gap=float(art.get_dof_positions()[0,fi].sum())
        force_confirmation=_pin_rgbd_sdk_hold_evidence(v['result']['pose'],preload_reference_G,confirmed_G,gap)
        report['pin_qualified_postlift_preload']=dict(target_velocity_preload_m_s=target_preload,ramp_s=ramp_s,effort_cap_per_pad_n=float(pin_config.get('native_grip_pad_force_n',30.)),effort_cap_unchanged=True,actual_contact_force_measured=False,confirmation=force_confirmation,uses_free_body_poses=False)
        print('PIN_QUALIFIED_POSTLIFT_PRELOAD',report['pin_qualified_postlift_preload'],flush=True)
        if not force_confirmation['confirmed']:raise RuntimeError('Fresh RGB-D/SDK PIN hold changed after qualified native preload ramp; transport blocked')
        G=confirmed_G
    pin_hold_reference_G=G.copy()
    if ctx.get('before_transport')"""
assert s.count(a)==1;s=s.replace(a,b);ast.parse(s)
(out/'direct_pin_skill.py').write_text(s);(out/'direct_pin_skill.py.diff').write_text(''.join(difflib.unified_diff(base.read_text().splitlines(True),s.splitlines(True),fromfile='a/direct_pin_skill.py',tofile='b/direct_pin_skill.py')))
# Driver diff only carries the native callback extension; root owns config whitelist.
p=Path('/tmp/fixed476-full/overrides/run_realtime_recipe.py');r=p.read_text();old="set_grip_hold=lambda enabled:set_native_grip_hold('right',enabled))";new="set_grip_hold=lambda enabled,preload_m_s=None:set_native_grip_hold('right',enabled,preload_m_s=preload_m_s))";assert r.count(old)==1;r=r.replace(old,new);ast.parse(r)
(out/'run_realtime_recipe.py.callback.diff').write_text(''.join(difflib.unified_diff(p.read_text().splitlines(True),r.splitlines(True),fromfile='a/run_realtime_recipe.py',tofile='b/run_realtime_recipe.py')))
(out/'config-delta.json').write_text(json.dumps({'pin_visual_independent_component_feedback':True,'pin_visual_bounded_component_recovery':True,'pin_visual_servo_gain':.65,'pin_qualified_postlift_preload':True},indent=2)+'\n')
