"""Isaac RGB camera bridge. Controller receives only image-derived estimates.

Camera aim uses calibrated workspace ROIs or measured robot encoders, never
workpiece transforms. Host inference reads a strict RGB/K/extrinsics packet.
"""
from pathlib import Path
import time,json,numpy as np
from rgbd_material_sensor import capture

def quaternion_from_axis(axis):
    n=np.asarray(axis,float);n/=np.linalg.norm(n);x=np.array([1.,0,0]);x-=n*(x@n)
    if np.linalg.norm(x)<.1:x=np.array([0.,1.,0]);x-=n*(x@n)
    x/=np.linalg.norm(x);R=np.column_stack([x,np.cross(n,x),n]);from pxr import Gf
    q=Gf.Matrix4d(Gf.Matrix3d(*R.T.ravel().tolist()),Gf.Vec3d(0)).ExtractRotationQuat();return np.r_[np.array(q.GetImaginary()),q.GetReal()]

class RGBDObserver:
    def __init__(self,stage,out,here,robot_tcp,sim_clock=lambda:0.):
        self.sim_clock=sim_clock;self.stage=stage;self.out=Path(out);self.here=Path(here);self.robot_tcp=robot_tcp;self.seq=0;self.history=[];self.total_wall_s=0.;self.root=self.out/'rgb-perception';self.root.mkdir(exist_ok=True)
        self.calibration=json.load(open(here/'bin-picking/asset-audit.json'))['baskets'][0]
    def request(self,mode,part=None,seed=None,axis=None):
        self.seq+=1;work=self.root/f'observation-{self.seq:03d}-{mode}';work.mkdir(exist_ok=True);start=time.monotonic();files=[]
        if mode in ['initial','floor']:
            target=np.array([.51,-.70,1.10]);eyes=[[.75,-1.15,1.6]];span=.28;width=512
        elif mode=='pin_initial':
            target=np.array([.19,-.79,.916]);eyes=[[.19,-.79,1.35],[.32,-.83,1.35]];span=.08;width=384
        else:
            H=self.robot_tcp();normal=-H[:3,2];normal*=1 if normal[2]>=0 else -1
            target=H[:3,3].copy();seed=target-normal*.003 if part!='pin'else target;axis=normal if part!='pin'else H[:3,2]
            deltas=([[-.25,.12,-.2],[-.25,-.12,-.2],[.25,-.15,-.15]]if part!='pin'else[[0,0,.30],[.15,-.05,.30],[-.15,-.05,.30]])
            eyes=[target+delta for delta in deltas];span=.16 if part!='pin'else .08;width=512 if part!='pin'else 384
        base_eye=eyes[0];capture(self.stage,base_eye,target,span,width,width,out=work/'rgbd.npz')
        files=['rgbd.npz']
        if mode=='pin_initial':seed=np.array([.19,-.79,.916]);axis=np.array([0.,0.,-1.])
        request=dict(mode=mode,part=part,cameras=files,seed=None if seed is None else np.asarray(seed).tolist(),axis=None if axis is None else np.asarray(axis).tolist(),calibration=self.calibration,depth_input=True,simulator_body_pose_input=False)
        pending=work/'request.tmp';pending.write_text(json.dumps(request,indent=2));pending.rename(work/'request.json')
        answer=work/'response.json'
        while not answer.exists():
            if time.monotonic()-start>600:raise RuntimeError('RGB perception worker timeout: '+str(work))
            time.sleep(.005)
        result=json.load(open(answer));elapsed=time.monotonic()-start;self.total_wall_s+=elapsed
        if result.get('error'):raise RuntimeError('RGB inference failed: '+result['error'])
        self.history.append(dict(scene_sim_time_s=float(self.sim_clock()),observation=self.seq,mode=mode,part=part,wall_s=elapsed,result=result,work=str(work)))
        (self.out/'rgb-observations.json').write_text(json.dumps(self.history,indent=2));print('RGB_OBSERVATION',self.seq,mode,part,elapsed,result.get('poses',result.get('pose')),flush=True)
        return result
    def initialize(self):
        result=self.request('initial');poses={r['part']:r for r in result['poses']}
        if set(poses)!={'ring1','ring2'}:raise RuntimeError('RGB ring size classification did not find one of each type')
        poses['pin']=self.request('pin_initial',part='pin')['pose'];self.initial=poses;return poses
    def observe(self,part,mode='held',seed=None):
        axis=np.array(self.calibration['basket_to_world_R'])[:,2] if mode=='floor'else None
        result=self.request(mode,part,seed,axis);pose=result['pose'];return np.r_[pose['center_m'],quaternion_from_axis(pose['axis'])]
