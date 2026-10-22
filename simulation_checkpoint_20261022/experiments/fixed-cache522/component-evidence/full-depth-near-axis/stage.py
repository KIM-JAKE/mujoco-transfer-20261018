from pathlib import Path
import ast,hashlib,json,difflib
base=Path('/tmp/fixed520-pin-gpu-route-first/overrides/direct_pin_skill.py');out=Path('/tmp/fixed520-pin-depth-axis-view');s=base.read_text();helper='''def _pin_full_depth_near_axis_views(H,grasp,offsets,label,config,require_circle):
    """Two public-SDK/image-derived tail views; original quality stays mandatory."""
    if label!='pin_full_depth_RGBD' or not config.get('pin_full_depth_near_axis_views',False):return []
    if not(config.get('fixed_supply_mode') and config.get('pin_full_depth_require_robust_circle') and config.get('robust_pin_cap_circle') and require_circle):
        raise ValueError('Near-axis full-depth views require fixed mode and original qualified robust-circle gate')
    if grasp is None:raise ValueError('Near-axis depth views require a fresh image/SDK grasp')
    H=np.asarray(H,float);G=np.asarray(grasp,float)
    if H.shape!=(4,4)or G.shape!=(4,4)or not np.isfinite(H).all()or not np.isfinite(G).all():raise ValueError('Finite public Hand and image grasp required')
    axis=(H@G)[:3,2];length=float(np.linalg.norm(axis))
    if length<1e-8:raise ValueError('Invalid public signed PIN axis')
    axis=axis/length;tangent=H[:3,2]-axis*(H[:3,2]@axis)
    if np.linalg.norm(tangent)<1e-8:
        reference=np.eye(3)[int(np.argmin(abs(axis)))];tangent=reference-axis*(reference@axis)
    tangent=tangent/np.linalg.norm(tangent)
    # The lens is0.25m below the signed tail axis. The second public Hand-Z
    # projection provides a small alternative if that first view is occluded.
    return [(len(offsets)+i,(-.25*axis+lateral*tangent).tolist())for i,lateral in enumerate([0.,.04])]


'''
marker='def execute(ctx):\n';assert s.count(marker)==1;s=s.replace(marker,helper+marker,1);old="        views=_pin_ordered_camera_views(offsets,label,pin_config)\n";new=old+"        views=_pin_full_depth_near_axis_views(H,grasp,offsets,label,pin_config,require_circle)+views\n";assert s.count(old)==1;s=s.replace(old,new,1);ast.parse(s);target=out/'overrides/direct_pin_skill.py';target.write_text(s);(out/'direct_pin_skill.py.diff').write_text(''.join(difflib.unified_diff(base.read_text().splitlines(True),s.splitlines(True),fromfile=str(base),tofile=str(target))));(out/'config-delta.json').write_text(json.dumps(dict(pin_full_depth_near_axis_views=True,pin_full_depth_require_robust_circle=True),indent=2)+'\n');(out/'source-manifest.json').write_text(json.dumps(dict(base=str(base),base_sha256=hashlib.sha256(base.read_bytes()).hexdigest(),source=str(target),source_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),new_flag='pin_full_depth_near_axis_views',driver_whitelist_addition='pin_full_depth_near_axis_views',no_new_imports=True,whole_task_qualified=False),indent=2)+'\n');print(hashlib.sha256(target.read_bytes()).hexdigest())
