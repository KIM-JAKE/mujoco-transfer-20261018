from pathlib import Path
import sys,json,hashlib,types,ast
import numpy as np
ROOT=Path('/home/gpuadmin/workspace/jake/mujoco');STAGE=Path(__file__).parent;BASE=next((ROOT/'isaac_port/runs').glob('*v529-*'))/'code_snapshot';sys.path[:0]=[str(BASE),str(ROOT)]
old={};new={};exec((BASE/'rgbd_fast_tracker.py').read_text(),old);exec((STAGE/'overrides/rgbd_fast_tracker.py').read_text(),new);rows=[]
for version,index,prior in [(529,96,95),(527,75,74),(531,81,80)]:
 run=next((ROOT/'isaac_port/runs').glob(f'*v{version}-*'));obs=json.loads((run/'rgb-observations.json').read_text());o=obs[index-1];p=obs[prior-1]['result']['pose'];packet_path=run/'rgb-perception'/Path(o['work']).name/'rgbd.npz';packet=dict(np.load(packet_path));seed=np.array(p['center_m']);axis=np.array(p['axis']);r={'version':version,'observation':index,'packet_SHA256':hashlib.sha256(packet_path.read_bytes()).hexdigest()}
 a=old['FastRGBDTracker'](None);b=new['FastRGBDTracker'](None)
 try:original,mask=a.estimate(packet,'pin',seed,axis,robust_pin_cap_circle=True);r['original_accepted']=True
 except RuntimeError as e:original=None;r['original_accepted']=False;r['original_error']=str(e)
 try:default,dm=b.estimate(packet,'pin',seed,axis,robust_pin_cap_circle=True)
 except RuntimeError as e:assert original is None;r['default_error_unchanged']=str(e)==r['original_error']
 else:
  assert original is not None
  x=dict(original);y=dict(default);x.pop('processing_ms');y.pop('processing_ms');assert x==y and np.array_equal(mask,dm);r['default_pose_mask_exact']=True
 refined,rm=b.estimate(packet,'pin',seed,axis,robust_pin_cap_circle=True,postrelease_cap_plane_roi=True);assert refined['robust_cap_circle_from_pixels']and not refined['lateral_from_previous_RGBD_grasp_and_encoders'];assert refined['cap_edge_coverage_deg']>=240 and refined['cap_circular_edge_rms_m']<=.00006;r['proposal_pose']=refined
 if original:
  r['proposal_vs_original_center_m']=float(np.linalg.norm(np.array(refined['center_m'])-original['center_m']));r['proposal_vs_original_axis_rad']=float(np.arccos(np.clip(np.dot(refined['axis'],original['axis']),-1,1)))
 rows.append(r)
# Invalid class/phase/quality/private-input routes fail before the estimator.
b=new['FastRGBDTracker'](None)
for kw in [dict(part='ring1'),dict(initial_pin=True),dict(robust_pin_cap_circle=False),dict(postrelease_cap_plane_roi='yes'),dict(packet=dict(packet,render_snapshot={}))]:
 args=dict(packet=packet,part='pin',seed=seed,axis=axis,robust_pin_cap_circle=True,postrelease_cap_plane_roi=True);args.update(kw)
 try:b.estimate(**args)
 except ValueError:pass
 else:raise AssertionError('Invalid scope accepted')
# If the class ROI cannot qualify, the original10mm computation is delegated unchanged.
calls=[]
def fake(self,*args,**kw):
 calls.append(kw.get('pin_radial_roi_m',.010))
 if calls[-1]==.007:raise RuntimeError('invisible proposal')
 return {'token':'original'},np.ones((1,1),bool)
b._estimate_original=types.MethodType(fake,b);p,m=b.estimate(packet,'pin',seed,axis,robust_pin_cap_circle=True,postrelease_cap_plane_roi=True);assert calls==[.007,.010]and p['token']=='original'and p['postrelease_known_class_plane_roi']['original_fallback']
# The original detection body differs only in the existing radial ROI constant becoming an optional parameter.
a=ast.parse((BASE/'rgbd_fast_tracker.py').read_text());b=ast.parse((STAGE/'overrides/rgbd_fast_tracker.py').read_text());orig=next(n for n in ast.walk(a)if isinstance(n,ast.FunctionDef)and n.name=='estimate');body=next(n for n in ast.walk(b)if isinstance(n,ast.FunctionDef)and n.name=='_estimate_original');o=ast.unparse(ast.Module(body=orig.body,type_ignores=[]));n=ast.unparse(ast.Module(body=body.body,type_ignores=[]));assert o.replace('radial < 0.01','radial < pin_radial_roi_m')==n
# Existing readiness/freshness/final precision guards are retained literally.
s=(STAGE/'overrides/direct_pin_skill.py').read_text();before=(BASE/'direct_pin_skill.py').read_text();added="\n            if anchored_world is not None and pin_config.get('pin_postrelease_cap_plane_roi',False):e['postrelease_cap_plane_roi']=True";assert s.replace(added,'')==before
result=dict(component_only=True,new_physics_steps=0,no_GPU_service_changes=True,CPU_geometric_PIN_tracker=True,original_fallback_retained=True,original_quality_and_freshness_gates_unchanged=True,invalid_scope_rejections=5,rows=rows)
(STAGE/'component-results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps([dict(version=r['version'],old=r['original_accepted'],coverage=r['proposal_pose']['cap_edge_coverage_deg'],axis=r['proposal_pose']['axis'],d_center=r.get('proposal_vs_original_center_m'),d_axis=r.get('proposal_vs_original_axis_rad'))for r in rows],indent=2))
