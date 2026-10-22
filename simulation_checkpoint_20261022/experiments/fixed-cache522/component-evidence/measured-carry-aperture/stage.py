from pathlib import Path
import ast,difflib,json,hashlib
O=Path(__file__).parent;B=Path('/tmp/fixed521-pin-entry-axis-view/overrides/direct_pin_skill.py');s=B.read_text()
helpers='''def _pin_carry_aperture_domain(ctx):
    """Existing actual hold band plus fresh spring command and one full tick."""
    actual=np.asarray(ctx['art'].get_dof_positions()[0,ctx['fi']],float)
    command=np.asarray(ctx['cmd'][0,ctx['fi']],float);dt=float(ctx['dt'])
    if actual.shape!=(2,) or command.shape!=(2,) or not np.isfinite(np.r_[actual,command,dt]).all() or dt<=0:
        raise RuntimeError('Invalid current PIN carry aperture state')
    gap=float(actual.sum());target=float(command.sum());reach=2*.0475*dt
    if not .0097<gap<.0135 or not 0<=target<=.08:
        raise RuntimeError('Actual PIN carry aperture outside original hold band')
    low=min(.0097,gap,target)-reach;high=max(.0135,gap,target)+reach
    if not 0<=low<=high<=.08:raise RuntimeError('Invalid PIN carry aperture proof domain')
    return dict(lower_m=low,upper_m=high,one_tick_total_reach_m=reach,
        actual_hold_band_m=[.0097,.0135],actual_at_admission_m=gap,command_at_admission_m=target,
        widths_m=np.unique(np.r_[np.linspace(low,high,5),gap,target]).tolist())


def _pin_check_carry_aperture(ctx,domain):
    actual=np.asarray(ctx['art'].get_dof_positions()[0,ctx['fi']],float)
    command=np.asarray(ctx['cmd'][0,ctx['fi']],float)
    if actual.shape!=(2,) or command.shape!=(2,) or not np.isfinite(np.r_[actual,command]).all():
        raise RuntimeError('Invalid current PIN carry aperture state')
    gap=float(actual.sum());target=float(command.sum());reach=domain['one_tick_total_reach_m']
    if not .0097<gap<.0135 or gap-reach<domain['lower_m'] or gap+reach>domain['upper_m'] or not domain['lower_m']<=target<=domain['upper_m']:
        raise RuntimeError('PIN carry aperture left original actual hold band or admitted command/lookahead domain')


def _pin_admit_measured_carry(ctx,issued,body_G,label):
    """Fresh full-payload/current-state admission, no actuation or sensors."""
    from pin_current_terminal import _other_contexts
    from dual_aperture_path_check import path_other_contexts
    from serial_gpu_pair_scope import serial_path_scope
    checker=ctx['checker'];domain=_pin_carry_aperture_domain(ctx)
    actual=ctx['art'].get_dof_positions()[0].copy()
    rows=np.vstack([actual[ctx['ids']],ctx['cmd'][0,ctx['ids']],issued])
    other,contexts=_other_contexts(ctx);widths=np.asarray(domain['widths_m'])
    saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
    stats={};began=time.perf_counter()
    scope=dict(ctx,checkers={'right':checker,'left':other},planning_diagnostics=ctx['report'].setdefault('pin_carry_pair_diagnostics',[]))
    try:
        checker.attach_payload(ctx['paths']['workpiece_pin'],frame([0,0,.1034],np.eye(3))@np.asarray(body_G,float))
        with serial_path_scope(scope,checker,other,contexts,rows,widths):
            clear,why=path_other_contexts(checker,other,contexts,rows,widths,ctx['paths']['workpiece_pin'],diagnostics=stats)
    finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
    if not clear:raise RuntimeError('Fresh full-width loaded PIN carry blocked before motion '+str(why))
    _pin_check_carry_aperture(ctx,domain)
    evidence=dict(label=label,issued_rows=len(issued)-1,actual_command_start_rows_added=2,
        other_contexts=len(contexts),full_payload_current_RGBD_G=True,uses_free_body_poses=False,
        wall_s=time.perf_counter()-began,**domain,guard=stats)
    ctx['report'].setdefault('pin_measured_carry_aperture_admissions',[]).append(evidence)
    print('PIN_MEASURED_CARRY_APERTURE_ADMITTED',evidence,flush=True)
    return domain


'''
assert s.count('def execute(ctx):\n')==1;s=s.replace('def execute(ctx):\n',helpers+'def execute(ctx):\n')
needle="    def step():ctx['step'](phase)\n";assert s.count(needle)==1;s=s.replace(needle,"    def carry_opening():\n        if not pin_config.get('pin_measured_carry_aperture_guard',False):return .0105\n        domain=_pin_carry_aperture_domain(ctx)\n        return max(domain['actual_at_admission_m'],domain['command_at_admission_m'])\n"+needle)
# Replace literal carry planner aperture ONLY; leave the archival default above.
start=s.index('    if ctx.get(\'optimized_motion_timing\') and not air_tangent_flip:');end=s.index('    adaptive_pin_gain=',start)
section=s[start:end];count=section.count('.0105');assert count==8,count;s=s[:start]+section.replace('.0105','carry_opening()')+s[end:]
needle="        phase=label\n        for row,hvalue in zip(q[1:],h[1:]):\n            cmd[0,ids]=row\n            if preshape:cmd[0,fi]=opening+(pickup_opening_m/2-opening)*hvalue\n            step()\n"
new="""        carry_domain=None
        if pin_config.get('pin_measured_carry_aperture_guard',False) and label in ['pin_tail_direct_carry_to_preinsert','pin_tail_carry_to_preinsert','pin_tail_orient_free']:
            carry_domain=_pin_admit_measured_carry(ctx,q,payload_grasp(),label)
        phase=label
        for row,hvalue in zip(q[1:],h[1:]):
            if carry_domain is not None:_pin_check_carry_aperture(ctx,carry_domain)
            cmd[0,ids]=row
            if preshape:cmd[0,fi]=opening+(pickup_opening_m/2-opening)*hvalue
            step()
            if carry_domain is not None:_pin_check_carry_aperture(ctx,carry_domain)
"""
assert s.count(needle)==1;s=s.replace(needle,new);ast.parse(s)
(O/'overrides/direct_pin_skill.py').write_text(s);(O/'direct_pin_skill.py.diff').write_text(''.join(difflib.unified_diff(B.read_text().splitlines(True),s.splitlines(True),fromfile='a/direct_pin_skill.py',tofile='b/direct_pin_skill.py')))
(O/'source-manifest.json').write_text(json.dumps({'base':str(B),'base_sha256':hashlib.sha256(B.read_bytes()).hexdigest(),'staged_sha256':hashlib.sha256(s.encode()).hexdigest(),'flag':'pin_measured_carry_aperture_guard','default':False,'planner_literal_replacements':count,'driver_whitelist_by_root':True,'protected_source_edits':False},indent=2)+'\n')
