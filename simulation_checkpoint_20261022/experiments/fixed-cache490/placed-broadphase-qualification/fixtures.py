"""Original 4.5mm placed collision boundary and fresh-current-transform fixtures."""
import sys,types,json,importlib.util
from pathlib import Path
import numpy as np
R=Path('/home/gpuadmin/workspace/jake/mujoco');H=R/'isaac_port/native_hand_layout/roas-table';P=Path(__file__).parent
sys.path[:0]=[str(P/'overrides'),str(R/'.local/co-moving484-model'),str(H)]
from placed_part_clearance import PlacedPartClearance
spec=importlib.util.spec_from_file_location('baseline',R/'isaac_port/runs/native-roas-table-fixed-front-v488-small-binding-split-gpu-profile-nominal/code_snapshot/placed_part_clearance.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
from predictive_pair_clearance import proxy,transform_proxy
import types
mod=types.ModuleType('fcl_transport')
from scipy.spatial import ConvexHull
def convex(vertices):
 vertices=np.ascontiguousarray(vertices,dtype=np.float64);hull=ConvexHull(vertices);faces=hull.simplices.copy();normals=np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],vertices[faces[:,2]]-vertices[faces[:,0]]);flip=np.einsum('ij,ij->i',normals,hull.equations[:,:3])<0;faces[flip]=faces[flip][:,[0,2,1]];return fcl.Convex(vertices,len(faces),np.c_[np.full(len(faces),3,dtype=np.int64),faces].ravel())
mod.convex=convex;sys.modules['fcl_transport']=mod
import fcl
cube=np.array([[x,y,z]for x in [-.005,.005]for y in [-.005,.005]for z in [-.005,.005]]);g=proxy(cube);T=np.eye(4);robot=fcl.CollisionObject(convex(g[3]));guard=types.SimpleNamespace(rows=[('robot','nativeCAD',g)],objects=[robot],geometries=[robot],margin_m=.0005);ck=types.SimpleNamespace(same_arm_clearance=guard,payload_path=None,payload_relative=None,payload_boxes=[],arm=types.SimpleNamespace(hand_path='robot'));transforms=dict(robot=T)
old=m.PlacedPartClearance();new=PlacedPartClearance();results=[]
def part(gap):
 t=np.eye(4);t[0,3]=.01+gap
 return dict(placed=[transform_proxy(g,t)],objects=[fcl.CollisionObject(convex(g[3]),fcl.Transform(t[:3,:3],t[:3,3]))],uncertainty_m=.0035,bounds_cache={})
for gap in [.00449,.0045,.0045000005,.004501,.004502,.00451,.005,.02]:
 p=part(gap);old.parts=dict(ring=p);new.parts=dict(ring=p);a=old.clear(ck,transforms);b=new.clear(ck,transforms);assert a[0]==b[0];results.append(dict(gap_m=gap,original_clear=a[0],fast_clear=b[0],fallback=new.stats['early_broadphase_fallback']))
# Same local shape, new current robot transform must reject after a cached far CLEAR.
p=part(.02);old.parts=new.parts=dict(ring=p);assert new.clear(ck,transforms)[0]
near=np.eye(4);near[0,3]=.02;robot.setTransform(fcl.Transform(near[:3,:3],near[:3,3]));transforms['robot']=near;assert not old.clear(ck,transforms)[0]and not new.clear(ck,transforms)[0]
assert old.clear(ck,transforms,changed_bodies={'unrelated'})[0]and new.clear(ck,transforms,changed_bodies={'unrelated'})[0]
# Current G is recomputed even with identical row/class references.
robot.setTransform(fcl.Transform(np.eye(3),np.zeros(3)));transforms['robot']=np.eye(4);ck.payload_path='held';ck.payload_boxes=[('held','knownclassCAD',g)];ck.payload_relative=np.eye(4);ck.payload_relative[0,3]=-.1;assert new.clear(ck,transforms)[0]
ck.payload_relative=np.eye(4);ck.payload_relative[0,3]=.02;assert not old.clear(ck,transforms)[0]and not new.clear(ck,transforms)[0]
ck.payload_path=None
# Unsupported data cannot produce a new positive result.
bad=dict(robot=np.eye(4));bad['robot'][0,3]=np.nan;assert new._early_clear_current_obbs(ck,bad)is False
bad=dict(robot=np.eye(4));bad['robot'][0,3]=1e12;assert new._early_clear_current_obbs(ck,bad)is False
new.parts['ring']['bounds_cache'][.0005]=(np.array([[np.nan,0,0]]),np.ones((1,3)));assert new._early_clear_current_obbs(ck,transforms)is False
new.parts['ring']['bounds_cache'].clear();badg=list(g);badg[0]=np.array([np.nan,0,0]);guard.rows=[('robot','new-malformed-CAD',tuple(badg))];assert new._early_clear_current_obbs(ck,transforms)is False
# A new shape identity invalidates the cached local pack (no stale far CLEAR).
guard.rows=[('robot','nativeCAD',proxy(cube*10))];guard.objects=[fcl.CollisionObject(convex(guard.rows[0][2][3]))];guard.geometries=guard.objects;assert not old.clear(ck,transforms)[0]and not new.clear(ck,transforms)[0]
(P/'boundary-qualification.json').write_text(json.dumps(dict(boundary_fixtures=results,current_q_recomputed=True,current_G_recomputed=True,changed_bodies_original_semantics_retained=True,malformed_nonfinite_and_unsupported_fallback=True,new_geometry_invalidates=True,all_assertions_pass=True,physics_steps=0),indent=2));print('PASS 8 boundary + fresh q/G + changed-body +4 malformed/invalidation fixtures')
