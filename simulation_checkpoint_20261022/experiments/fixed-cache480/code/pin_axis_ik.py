"""Five-axis pin IK: part center and axis, with nominal arm posture reserve."""
import numpy as np
from isaac_kinematics import frame,exp

def solve(arm,P,axis,G,seed,nominal):
    q=np.clip(np.array(seed,float),arm.bounds[:,0],arm.bounds[:,1]);axis=np.asarray(axis,float);axis/=np.linalg.norm(axis);P=np.asarray(P);nominal=np.asarray(nominal)
    weights=np.array([3.,3.,3.,1.5,.7,1.,.15])
    for iteration in range(100):
        T=arm.base.copy();origins=[];axes=[]
        for x,a,(A,B)in zip(q,arm.axes,arm.frames):
            J=T@A;origins.append(J[:3,3]);axes.append(J[:3,:3]@a);T=J@frame([0,0,0],exp(a*x))@B
        H=T@arm.tip@G;position=H[:3,3];direction=H[:3,2];axes=np.array(axes)
        error=np.r_[P-position,axis-direction]
        if np.linalg.norm(error[:3])<2e-7 and np.linalg.norm(error[3:])<2e-6:return q
        jac=np.vstack([np.cross(axes,position-np.array(origins)).T,np.cross(axes,direction).T]);free=np.ones(7,dtype=bool)
        for active_pass in range(4):
            J=jac*free[None,:];pinv=np.linalg.pinv(J,rcond=1e-6);null=np.eye(7)-pinv@J;step=pinv@error+null@(.001*weights*(nominal-q)*free if iteration==0 else np.zeros(7));step[~free]=0
            blocked=((q<arm.bounds[:,0]+.0002)&(step<0))|((q>arm.bounds[:,1]-.0002)&(step>0));new=blocked&free
            if not new.any():break
            free[new]=False
        step*=min(1.,.08/max(abs(step).max(),1e-12));q=np.clip(q+step,arm.bounds[:,0],arm.bounds[:,1])
    return None

def path(arm,commands,clock,k,stop,G,reference,part_index):
    target_time=reference['time_s'];poses=reference['parts'][:,part_index];from native_pickup_probe import rotation
    axes=np.array([rotation(row[3:])[:,2]for row in poses]);result=[];seed=commands[k].copy();previous=commands[k].copy();release_offset=None
    for n in range(k,stop+1):
        base=commands[n];st=clock[n]
        if st>81.82:
            if release_offset is None:release_offset=seed-previous
            solution=base+release_offset;result.append(solution);seed=solution;previous=base;continue
        P=np.array([np.interp(st,target_time,poses[:,j])for j in range(3)]);axis=np.array([np.interp(st,target_time,axes[:,j])for j in range(3)])
        solution=solve(arm,P,axis,G,seed+base-previous,base)
        if solution is None and n==k:
            ref_i=int(np.argmin(abs(target_time-st)));R=rotation(poses[ref_i,3:])
            for yaw in [0.,-.3,.3,-.6,.6,-1.,1.,-1.5,1.5]:
                H=frame(P,R@exp(np.array([0.,0.,yaw])))@np.linalg.inv(G);alternative=arm.solve(H[:3,3],H[:3,:3],base)
                if alternative is None:continue
                solution=solve(arm,P,axis,G,alternative,base)
                if solution is not None:break
        if solution is None:raise RuntimeError('Pin center/axis IK failed at source '+str(st))
        result.append(solution);seed=solution;previous=base
    return np.array(result)
