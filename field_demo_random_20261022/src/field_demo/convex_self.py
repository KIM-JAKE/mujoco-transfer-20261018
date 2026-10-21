"""Opt-in whole-link/palm convex-hull refinement of original capsule rejections.

Closed hulls enclose ALL original official STL vertices. The runtime retains
both bodies' actual cable allowance, maximum original per-link tracking/time
inflation and the unchanged collision margin, plus0.5mm numerical padding per
body. No interarm-pair, finger, payload or UNKNOWN-label exception is provided.
"""
import argparse
from dataclasses import dataclass,field
from datetime import datetime,timezone
from functools import lru_cache
import hashlib,json
from pathlib import Path
import numpy as np
from .model_records import validate as validate_model

ASSETS=Path(__file__).resolve().parents[1]/'isaac_port/fieldkit/assets/convex-links'
MODEL_SHA256='a50c465a4f3c21863194375d748d962af81d9b2ef68ef0b61d32adb2e6fe96b1'
KEY='_validated_convex_self_context'
WORLD_KEY='_validated_convex_world_contexts'
FACTORY=ASSETS.parent/'fr3v2-factory'

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def model():
    path=ASSETS/'model.json'
    if digest(path)!=MODEL_SHA256:raise ValueError('Pinned convex-link model changed')
    data=json.loads(path.read_text())
    if digest(ASSETS/'hulls.npz')!=data['hulls_sha256']:raise ValueError('Pinned whole-link hull shape changed')
    for link in data['links']:
        if digest(ASSETS/link['source_file'])!=link['source_sha256']:raise ValueError('Original official STL changed')
    return data

@lru_cache(maxsize=1)
def _shapes():
    import fcl
    data=model();result={}
    with np.load(ASSETS/'hulls.npz',allow_pickle=False)as z:
        for info in data['links']:
            i=info['link'];vertices=z[f'link{i}_vertices'].copy();faces=z[f'link{i}_faces'].copy();eq=z[f'link{i}_equations']
            raw=(ASSETS/info['source_file']).read_bytes();tri=np.frombuffer(raw[84:],dtype=[('n','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])['v'].astype(float)
            if np.max(tri.reshape(-1,3)@eq[:,:3].T+eq[:,3])>1e-12:raise ValueError('Convex hull does not enclose original STL')
            packed=np.c_[np.full(len(faces),3),faces].astype(np.int32).reshape(-1)
            result[i]=fcl.Convex(vertices,len(faces),packed)
    return result

def validate_optin(config,root,dependencies,now):
    spec=config.get('convex_self_narrowphase');world=config.get('convex_world_narrowphase')
    if world is not None and(world!={'model_sha256':MODEL_SHA256}or spec!=world):raise ValueError('World hull refinement requires the SAME explicit validated model as self refinement')
    if spec is None:return False
    if not isinstance(spec,dict)or spec!={'model_sha256':MODEL_SHA256}:raise ValueError('Explicit pinned convex-self model binding required')
    data=model()
    # Every original factory model/installation/shape subject remains required.
    columns=json.loads((FACTORY/'arm-columns.json').read_text())['capsules']
    for side in ['left','right']:
        env=config['robots'][side]
        if env.get('geometry_backend')!='sdk_link_columns':raise ValueError('Convex refinement requires actual SDK per-link factory geometry')
        validate_model(env.get('measurement'),root,subject=env,dependencies=dependencies,name='convex_self_'+side,now=now)
        pad=float(env['native_hand_aperture_geometry']['cable_radial_allowance_m']);expected=[dict(r,radius_m=r['radius_m']+pad)for r in columns]
        if env['link_capsules_link_m']!=expected:raise ValueError('Original factory columns/cable allowance changed')
    try:_shapes()
    except ImportError as exc:raise RuntimeError('Opt-in convex self checker requires requirements-collision.txt')from exc
    dependencies['convex_self_model']=dict(path=str(ASSETS/'model.json'),sha256=MODEL_SHA256)
    dependencies['convex_self_hulls']=dict(path=str(ASSETS/'hulls.npz'),sha256=data['hulls_sha256'])
    for info in data['links']:dependencies['convex_self_STL_'+str(info['link'])]=dict(path=str((ASSETS/info['source_file']).resolve()),sha256=info['source_sha256'])
    return True

@dataclass
class ConvexSelfContext:
    frames: np.ndarray
    inflation_m: dict
    frame_sha256: str
    padding_m: float
    side: str
    cache: dict=field(default_factory=dict)
    queries: int=0
    accepted_link_pairs: list=field(default_factory=list)
    world_cache: dict=field(default_factory=dict)
    accepted_world_boxes: list=field(default_factory=list)
    def refine(self,a,b,margin):
        # Actual whole-link shapes only; original tool/payload rules remain.
        def body(row):
            i=row.get('link_index')
            if row.get('geometry_role')=='arm'and type(i)is int and 0<=i<8:return i
            if row.get('geometry_role')=='palm'and i==8 and row.get('hand_body')=='palm':return 8
            return None
        ia,ib=body(a),body(b)
        if ia is None or ib is None or abs(ia-ib)<=1 or a.get('robot_side')!=self.side or b.get('robot_side')!=self.side:return None
        key=tuple(sorted((ia,ib)))+(float(margin),)
        if key in self.cache:return self.cache[key]
        import fcl
        shapes=_shapes();first=fcl.CollisionObject(shapes[ia]);second=fcl.CollisionObject(shapes[ib]);threshold=self.inflation_m[ia]+self.inflation_m[ib]+2*self.padding_m+float(margin);minimum=np.inf
        for row in self.frames:
            for obj,i in [(first,ia),(second,ib)]:obj.setTransform(fcl.Transform(row[i,:3,:3],row[i,:3,3]))
            self.queries+=1
            contacts=fcl.CollisionResult();fcl.collide(first,second,fcl.CollisionRequest(num_max_contacts=1),contacts)
            if contacts.is_collision:self.cache[key]=None;return None
            answer=fcl.DistanceResult();distance=float(fcl.distance(first,second,fcl.DistanceRequest(),answer))
            if not np.isfinite(distance)or distance<=threshold:self.cache[key]=None;return None
            minimum=min(minimum,distance-threshold)
        proof=dict(clearance_m=float(minimum),whole_link_pair=[ia,ib],checked_FK_samples=len(self.frames),model_sha256=MODEL_SHA256,frame_sha256=self.frame_sha256,inflation_each_m=[self.inflation_m[ia],self.inflation_m[ib]],numeric_padding_each_m=self.padding_m,original_margin_m=float(margin))
        self.cache[key]=proof;self.accepted_link_pairs.append(proof);return proof
    def refine_box(self,capsule,obstacle,margin):
        i=capsule.get('link_index')
        eligible=(capsule.get('geometry_role')=='arm'and type(i)is int and 0<=i<8)or(capsule.get('geometry_role')=='palm'and i==8 and capsule.get('hand_body')=='palm')
        if not eligible or capsule.get('robot_side')!=self.side:return None
        from .collision import box
        import fcl
        T,half=box(obstacle);margin=float(margin)
        if not np.isfinite(margin)or margin<0:raise ValueError('Original nonnegative collision margin required')
        box_sha=hashlib.sha256(np.r_[T.ravel(),half].tobytes()).hexdigest();key=(i,box_sha,margin)
        if key in self.world_cache:return self.world_cache[key]
        fixed=fcl.CollisionObject(fcl.Box(*(2*half)),fcl.Transform(T[:3,:3],T[:3,3]));robot=fcl.CollisionObject(_shapes()[i]);threshold=self.inflation_m[i]+self.padding_m+margin;minimum=np.inf
        for row in self.frames:
            robot.setTransform(fcl.Transform(row[i,:3,:3],row[i,:3,3]));self.queries+=1
            contacts=fcl.CollisionResult();fcl.collide(robot,fixed,fcl.CollisionRequest(num_max_contacts=1),contacts)
            if contacts.is_collision:self.world_cache[key]=None;return None
            result=fcl.DistanceResult();distance=float(fcl.distance(robot,fixed,fcl.DistanceRequest(),result))
            if not np.isfinite(distance)or distance<=threshold:self.world_cache[key]=None;return None
            minimum=min(minimum,distance-threshold)
        proof=dict(clearance_m=float(minimum),whole_body_index=i,robot_side=self.side,checked_FK_samples=len(self.frames),model_sha256=MODEL_SHA256,frame_sha256=self.frame_sha256,original_box_geometry_sha256=box_sha,body_inflation_m=self.inflation_m[i],numeric_padding_m=self.padding_m,original_margin_m=margin,occupancy_unchanged=True)
        self.world_cache[key]=proof;self.accepted_world_boxes.append(proof);return proof
    def record(self):
        return dict(model_sha256=MODEL_SHA256,frame_sha256=self.frame_sha256,FK_samples=len(self.frames),FCL_queries=self.queries,accepted_link_pairs=self.accepted_link_pairs,accepted_world_boxes=self.accepted_world_boxes,scope='Original capsule rejections only; full closed link/palm hulls, all original uncertainty+margin; world boxes/occupancy unchanged',physical_execution_certified=False)

def context(envelope,rows,link_frames,side):
    fs=np.asarray(link_frames,float).copy()
    if fs.ndim!=4 or fs.shape[0]<1 or fs.shape[1:]!=(8,4,4)or not np.isfinite(fs).all()or not np.allclose(fs[:,:,3],[0,0,0,1],atol=1e-9)or not np.allclose(np.swapaxes(fs[:,:,:3,:3],-1,-2)@fs[:,:,:3,:3],np.eye(3),atol=1e-7)or np.any(abs(np.linalg.det(fs[:,:,:3,:3])-1)>1e-7):raise ValueError('Complete original checked SDK per-link FK samples required')
    if side not in ['left','right']:raise ValueError('Actual active robot side required')
    # The rigid palm uses the same actual SDK flange/EE and verified factory
    # mount as the existing column model. No finger or width exemption.
    F=np.asarray(envelope['F_T_EE_column_major'],float).reshape(4,4,order='F');M=np.asarray(envelope['native_hand_aperture_geometry']['T_tcp_hand'],float)
    Z=np.eye(4);Z[2,3]=.107
    palm=fs[:,7]@Z@F@M;fs=np.concatenate([fs,palm[:,None]],axis=1)
    pad=float(envelope['native_hand_aperture_geometry']['cable_radial_allowance_m']);inflation={}
    for i in range(9):
        body=[r for r in rows if r.get('geometry_role')==('arm'if i<8 else'palm')and r.get('link_index')==i]
        if not body:raise ValueError('Full original per-link uncertainty rows required')
        extra=max(float(np.max(np.asarray(r['r'])))-float(r['nominal_radius_m'])for r in body)
        if not np.isfinite(extra)or extra< -1e-12:raise ValueError('Original tracking/time inflation cannot be negative')
        # Maximum over EVERY original column of this link covers its complete
        # local reach, including newly filled hull material between surfaces.
        inflation[i]=pad+max(0.,extra)
    frame_sha=hashlib.sha256(fs.tobytes()).hexdigest();fs.flags.writeable=False
    return ConvexSelfContext(fs,inflation,frame_sha,float(model()['numeric_padding_each_body_m']),side)

def refine(envelope,a,b,margin):
    value=envelope.get(KEY)
    return value.refine(a,b,margin)if isinstance(value,ConvexSelfContext)else None

def refine_world(envelope,capsule,obstacle,margin):
    contexts=envelope.get(WORLD_KEY);value=contexts.get(capsule.get('robot_side'))if isinstance(contexts,dict)else None
    return value.refine_box(capsule,obstacle,margin)if isinstance(value,ConvexSelfContext)else None

def prepare(motion,output,*,world_boxes=False):
    motion,output=Path(motion).resolve(),Path(output).resolve()
    if output.exists()or output.parent!=motion.parent:raise ValueError('Use a NEW motion profile beside its source')
    config=json.loads(motion.read_text());config['convex_self_narrowphase']=dict(model_sha256=MODEL_SHA256)
    if world_boxes:config['convex_world_narrowphase']=dict(model_sha256=MODEL_SHA256)
    validate_optin(config,motion.parent,{},datetime.now(timezone.utc))
    output.write_text(json.dumps(config,indent=2)+'\n')
    return dict(state='optin_convex_world_profile_written'if config.get('convex_world_narrowphase')is not None else'optin_convex_self_profile_written',world_box_refinement_enabled=config.get('convex_world_narrowphase')is not None,profile=str(output),default20mrad_unchanged=config['joint_tracking_error_limit_rad']==.02,model_sha256=MODEL_SHA256,hardware_tested=False,full_task_verified=False)

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--motion',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--include-world-boxes',action='store_true');a=p.parse_args(argv);print(json.dumps(prepare(a.motion,a.output,world_boxes=a.include_world_boxes),indent=2));return 0
if __name__=='__main__':raise SystemExit(main())
