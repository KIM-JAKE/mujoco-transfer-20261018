exec(open('/tmp/pin-exact-field-handoff/profile_numeric.py').read().split("z=np.load('/tmp/pin481-terminal-public/")[0])
sys.path.insert(0,'/tmp/fixed493-small-balanced/overrides')
from balanced_small_receiver import propose
from isaac_kinematics import frame
x=json.loads(Path('/tmp/fixed493-small-reconfirmation/evidence.json').read_text());H=np.asarray(x['frames'][0]['H']);G=np.asarray(x['original_stationary_reference']['G']);part=H@G;oldH,oldPart=H.copy(),part.copy();target,report=propose(H,part);g=np.linalg.inv(target)@part
assert np.array_equal(H,oldH)and np.array_equal(part,oldPart)
assert abs(g[1,2])<1e-10 and abs(g[0,2]-.5)<3e-4 and abs(g[2,2]-np.cos(np.pi/6))<3e-4
assert np.linalg.norm(g[:3,3]-.0045*g[:3,2])<1e-10
assert report['translation_m']<=.006 and report['rotation_rad']<=.06
failures=[]
for name,h,p in [('badshape',H[:3],part),('nonfinite',H*np.nan,part),('nongraspfamily',H,H@frame([0,0,.004],np.eye(3))),('largecorrection',H,H@frame([.02,0,.004],G[:3,:3]))]:
 try:propose(h,p)
 except ValueError:failures.append(name)
 else:raise AssertionError(name)
Path('/tmp/fixed493-small-balanced/target-tests.json').write_text(json.dumps(dict(all_pass=True,live492_public_pose_only=True,caller_inputs_unchanged=True,exact_perpendicular_jaw_axis=True,preserved30degree_family=True,zero_lateral_axis_lever=True,malformed_or_unbounded_rejected=failures,physical_target_proposal=report),indent=2));print(json.dumps(dict(translation_m=report['translation_m'],rotation_rad=report['rotation_rad'],G_axis=g[:3,2].tolist(),tests='PASS'),indent=2))
