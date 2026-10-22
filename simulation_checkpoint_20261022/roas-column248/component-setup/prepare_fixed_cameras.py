from pathlib import Path
import shutil,py_compile
src=Path('/tmp/pin-fixed-normal-20261022/overrides');out=Path('/tmp/pin-fixed-normal-20261022/fixed-camera-overrides');out.mkdir(exist_ok=True)
for p in src.glob('*.py'):shutil.copy2(p,out/p.name)
p=out/'process_vision.py';s=p.read_text();anchor='        future=Future();number=len(self.jobs);'
addition="""        fixed_view=event.get('camera_fixed_view')
        if fixed_view is not None:
            if part!='pin' or mode not in ['pin_initial','held']:raise ValueError('Fixed camera override only supports declared PIN component observations')
            eye=np.asarray(fixed_view['eye_world_m'],float);target=np.asarray(fixed_view['target_world_m'],float);span=float(fixed_view['span_m'])
            if eye.shape!=(3,)or target.shape!=(3,)or not np.isfinite(np.r_[eye,target,span]).all()or not .05<=span<=1.2 or np.linalg.norm(eye-target)<.1:raise ValueError('Invalid fixed calibrated camera')
            event['fixed_camera_id']=fixed_view['camera_id'];event['fixed_camera_extrinsics']=True
""";assert s.count(anchor)==1;s=s.replace(anchor,addition+anchor);p.write_text(s)
p=out/'direct_pin_skill.py';s=p.read_text();anchor='        for view_index,offset in views:'
addition="""        fixed_view=None
        if fixed_normal is not None and fixed_normal.get('fixed_camera_views'):
            camera_name='fixture' if label.startswith('pin_component_postcarry') else 'supply'
            fixed_view=fixed_normal['fixed_camera_views'][camera_name]
            views=[(camera_name,None)]
""";assert s.count(anchor)==1;s=s.replace(anchor,addition+anchor)
anchor="            if grasp is not None:e.update(tracked_grasp=grasp.tolist(),camera_eye_offset=offset)";assert s.count(anchor)==1;s=s.replace(anchor,anchor+"\n            if fixed_view is not None:e['camera_fixed_view']=fixed_view")
p.write_text(s)
for p in out.glob('*.py'):py_compile.compile(str(p),doraise=True)
print(out)
