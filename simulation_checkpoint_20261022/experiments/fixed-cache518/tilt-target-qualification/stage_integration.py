from pathlib import Path
import ast,json,hashlib,difflib,shutil
O=Path(__file__).parent;B=Path('/tmp/fixed517-preclose-gain-full/overrides/direct_pin_skill.py');old=B.read_text();s=old
needle="def execute(ctx):\n";patch="""def execute(ctx):
    tilt30_enabled=bool(ctx.get('config',{}).get('pin_collar_tilt30_target'))
    if tilt30_enabled and not(ctx.get('config',{}).get('pin_collar_flat_contact_target') and ctx.get('config',{}).get('pin_collar_flat_offset_m',-.002)==-.001):
        raise ValueError('PIN tilt30 target requires existing collar negative1mm mode')
    tilt30_proposal=None
""";assert s.count(needle)==1;s=s.replace(needle,patch,1)
needle="    art=ctx['art'];arm=ctx['arm'];";patch="""    if tilt30_enabled:
        from pin_collar_tilt_target import retarget as tilt30_retarget
        cand,tilt30_proposal=tilt30_retarget(cand)
    art=ctx['art'];arm=ctx['arm'];""";assert s.count(needle)==1;s=s.replace(needle,patch,1)
needle="    body_goal=goal.copy();pin_config=ctx.get('config',{})\n";patch="""    if tilt30_proposal is not None:report['pin_collar_tilt30_target']=tilt30_proposal;print('PIN_COLLAR_TILT30_TARGET',tilt30_proposal,flush=True)
    body_goal=goal.copy();pin_config=ctx.get('config',{})
""";assert s.count(needle)==1;s=s.replace(needle,patch,1)
ast.parse(s);P=O/'overrides/direct_pin_skill.py';P.write_text(s);(O/'direct_pin_skill.py.diff').write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(B),tofile=str(P))))
(O/'source-manifest.json').write_text(json.dumps(dict(base=str(B),base_sha256=hashlib.sha256(old.encode()).hexdigest(),runtime_files={str(p.relative_to(O)):hashlib.sha256(p.read_bytes()).hexdigest()for p in sorted((O/'overrides').glob('*.py'))},new_flag='pin_collar_tilt30_target',fresh_import='pin_collar_tilt_target',requires_existing_collar_offset_m=-.001,actual_fullscene_or_physics_tested=False),indent=2)+'\n')
D=O/'geometry-evidence';D.mkdir(exist_ok=True)
for name in('tilt-collar-native-subset-grid.json','tilt-native-proximal-grid.json','tilt_collar_probe.py','tilt_proximal_probe.py'):
 shutil.copy2(Path('/tmp/fixed515-pin-depth-diagnosis')/name,D/name)
print((O/'source-manifest.json').read_text())
