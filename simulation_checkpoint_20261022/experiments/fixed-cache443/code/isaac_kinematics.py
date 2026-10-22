"""Kinematics read directly from the live Isaac USD joint frames.

No other simulator, URDF approximation, or live robot connection. Validation
against the authored initial link poses is mandatory before solving a motion.
"""
import math
import numpy as np
from pxr import UsdGeom,UsdPhysics,Usd

def rotation(q):
    w=float(q.GetReal());x,y,z=map(float,q.GetImaginary())
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def frame(P,R):
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=P;return T

def world(stage,path):return np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T.copy()

def exp(v):
    t=float(np.linalg.norm(v))
    if t<1e-12:return np.eye(3)
    x,y,z=v/t;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(t)*K+(1-math.cos(t))*(K@K)

def log(R):
    angle=math.acos(np.clip((np.trace(R)-1)/2,-1,1));v=np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])
    if angle<1e-7:return v/2
    if angle>math.pi-1e-4:
        vals,vecs=np.linalg.eigh((R+np.eye(3))/2);a=vecs[:,np.argmax(vals)];return a*angle
    return v*(angle/(2*math.sin(angle)))

class Arm:
    def __init__(self,stage,side,drives,q0):
        self.frames=[];self.axes=[];self.bounds=[];self.link_paths=[]
        for j in range(1,8):
            prim=drives[f'{side}_arm_fr3v2_joint{j}'].GetPrim();joint=UsdPhysics.RevoluteJoint(prim)
            J=[]
            for i in [0,1]:
                P=getattr(joint,f'GetLocalPos{i}Attr')().Get();Q=getattr(joint,f'GetLocalRot{i}Attr')().Get();J.append(frame(P,rotation(Q)))
            self.frames.append((J[0],np.linalg.inv(J[1])));self.axes.append(np.eye(3)['XYZ'.index(joint.GetAxisAttr().Get())]);self.bounds.append([math.radians(joint.GetLowerLimitAttr().Get())+.003,math.radians(joint.GetUpperLimitAttr().Get())-.003])
            self.link_paths.append(str(joint.GetBody1Rel().GetTargets()[0]))
            if j==1:self.base=world(stage,str(joint.GetBody0Rel().GetTargets()[0]))
            if j==7:last=str(joint.GetBody1Rel().GetTargets()[0])
        hand=next(p for p in stage.Traverse()if p.GetName()==side+'_NativeHand');self.hand_path=str(hand.GetPath());Tlast=world(stage,last);Thand=world(stage,str(hand.GetPath()));self.tip=np.linalg.inv(Tlast)@Thand@frame([0,0,.1034],np.eye(3));self.bounds=np.array(self.bounds)
        # FR3 position-dependent velocity bounds contain a zero-speed band
        # before each hard limit. Avoid IK targets inside that band.
        from fr3_motion_control import LOWER,UPPER,OFFSET,DECEL
        margin=np.maximum(.02,OFFSET**2/(2*DECEL)+.01)
        self.bounds[:,0]=np.maximum(self.bounds[:,0],LOWER+margin);self.bounds[:,1]=np.minimum(self.bounds[:,1],UPPER-margin)
        ref=Thand@frame([0,0,.1034],np.eye(3));got=self.fk(np.array(q0));self.validation=dict(position_error_m=float(np.linalg.norm(got[:3,3]-ref[:3,3])),rotation_error_rad=float(np.linalg.norm(log(got[:3,:3]@ref[:3,:3].T))))
        if self.validation['position_error_m']>2e-5 or self.validation['rotation_error_rad']>2e-4:raise RuntimeError('USD joint-frame FK validation failed: '+str(self.validation))
    def fk(self,q):
        T=self.base.copy()
        for x,axis,(a,b)in zip(q,self.axes,self.frames):T=T@a@frame([0,0,0],exp(axis*x))@b
        return T@self.tip
    def bodies(self,q,opening):
        T=self.base.copy();result={}
        for path,x,axis,(a,b)in zip(self.link_paths,q,self.axes,self.frames):T=T@a@frame([0,0,0],exp(axis*x))@b;result[path]=T.copy()
        H=T@self.tip@frame([0,0,-.1034],np.eye(3));result[self.hand_path]=H
        side=self.hand_path.split('/')[-1].split('_')[0]
        for sign,label in [(1,'left'),(-1,'right')]:result[self.hand_path+'/'+side+'_'+label+'_finger']=H@frame([0,sign*opening/2,.0584],np.eye(3)if sign==1 else np.diag([-1.,-1.,1.]))
        return result
    def solve(self,P,R,seed,position_tolerance=1e-5,rotation_tolerance=1e-4):
        q=np.clip(np.array(seed,float),self.bounds[:,0],self.bounds[:,1]);P=np.array(P);R=np.array(R)
        for _ in range(220):
            T=self.base.copy();origins=[];world_axes=[]
            for x,axis,(a,b)in zip(q,self.axes,self.frames):
                joint=T@a;origins.append(joint[:3,3].copy());world_axes.append(joint[:3,:3]@axis);T=joint@frame([0,0,0],exp(axis*x))@b
            T=T@self.tip;err=np.r_[P-T[:3,3],log(R@T[:3,:3].T)]
            if np.linalg.norm(err[:3])<position_tolerance and np.linalg.norm(err[3:])<rotation_tolerance:return q
            axes=np.array(world_axes);J=np.vstack([np.cross(axes,T[:3,3]-np.array(origins)).T,axes.T])
            step=J.T@np.linalg.solve(J@J.T+np.eye(6)*1e-6,err);step*=min(1.,.12/max(np.max(abs(step)),1e-12));q=np.clip(q+step,self.bounds[:,0],self.bounds[:,1])
        return None
