from pathlib import Path
import ast,difflib,hashlib,json
O=Path(__file__).parent;B=Path('/tmp/fixed519-entry-feedback-full/overrides/direct_pin_skill.py');s=B.read_text()
helper='''def _pin_gpu_cubic_route_before_hint(checker,start,goal,opening,selected):
    """Existing GPU proposal plus unchanged live native continuous admission."""
    if not getattr(checker,'gpu_route_enabled',False):return None
    if not checker.clear(goal,opening,selected)[0]:return None
    from fr3_gpu_route import propose_route
    proposed=propose_route(checker,start,goal,opening,selected)
    if proposed is None:return None
    from gpu_route_curve_validation import cubic_geometry
    from grasp_transfer_feasibility import VerifiedCubicRoute
    continuous,curve_evidence=cubic_geometry(proposed,checker.arm.bounds,step_rad=.010)
    if continuous is None:return None
    old_resolution=getattr(checker,'resolution_rad',.035);checker.resolution_rad=.010
    try:continuous_clear,continuous_why=checker.path(continuous,opening,selected)
    finally:checker.resolution_rad=old_resolution
    if not continuous_clear:return None
    checker.gpu_route_stats['PhysX_verified_continuous_proposals']=checker.gpu_route_stats.get('PhysX_verified_continuous_proposals',0)+1
    print('PIN_GPU_ROUTE_BEFORE_HINT_NATIVE_VERIFIED',dict(curve_evidence,fixed_pair_payload_checked=True,issued_curve_and_FR3_retiming_still_required=True),flush=True)
    return VerifiedCubicRoute(proposed)


'''
needle='def execute(ctx):\n';assert s.count(needle)==1;s=s.replace(needle,helper+needle)
needle="                    from grasp_transfer_feasibility import robot_route_hint,hint_path_clear\n"
new="""                    if pin_config.get('pin_gpu_route_before_hint',False):
                        shortcut=_pin_gpu_cubic_route_before_hint(checker,cmd[0,ids].copy(),q,.0105,paths['workpiece_pin'])
                        if shortcut is not None:
                            report['pin_GPU_cubic_route_before_hint_used']=True;break
"""+needle
assert s.count(needle)==1;s=s.replace(needle,new)
ast.parse(s);(O/'overrides/direct_pin_skill.py').write_text(s)
(O/'direct_pin_skill.py.diff').write_text(''.join(difflib.unified_diff(B.read_text().splitlines(True),s.splitlines(True),fromfile='a/direct_pin_skill.py',tofile='b/direct_pin_skill.py')))
(O/'source-manifest.json').write_text(json.dumps({'base':str(B),'base_sha256':hashlib.sha256(B.read_bytes()).hexdigest(),'staged_sha256':hashlib.sha256(s.encode()).hexdigest(),'flag':'pin_gpu_route_before_hint','default':False,'driver_change_by_root':'whitelist only','protected_source_edits':False},indent=2)+'\n')
