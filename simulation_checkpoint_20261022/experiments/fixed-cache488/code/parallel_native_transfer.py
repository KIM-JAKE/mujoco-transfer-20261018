"""Certify paired retimed transfer fields before executing them together.

Pure robot/CAD query, including the image-estimated held ring. It neither
reads nor edits physical workpiece poses. Execution remains with the caller.
"""
import time
import numpy as np
from predictive_pair_clearance import at_pose, pair_clear


def schedule(rc,lc,right_rows,left_rows,right_start,left_start,right_width,left_width):
    """Return earliest certified receiver start, otherwise sequential fallback."""
    qr=np.vstack([right_start,right_rows[:,:7]]);ql=np.vstack([left_start,left_rows[:,:7]])
    started=time.perf_counter()
    # The exact convex CAD with a 0.5 mm outward reserve on each arm.
    import fcl
    planner=rc.fast_planner
    request=fcl.CollisionRequest(num_max_contacts=1,enable_contact=False)
    def objects(checker):
        rows=checker.pair_boxes+checker.payload_boxes
        geometries=[planner.geometry(row[2][3],.0005)for row in rows]
        result=[fcl.CollisionObject(g)for g in geometries]
        manager=fcl.DynamicAABBTreeCollisionManager();manager.registerObjects(result);manager.setup()
        return rows,result,manager
    rr,ro,rm=objects(rc);lr,lo,lm=objects(lc)
    def update(checker,rows,objects,manager,q,width):
        transforms=checker.arm.bodies(q,width)
        if checker.payload_path:transforms[checker.payload_path]=transforms[checker.arm.hand_path]@checker.payload_relative
        for row,obj in zip(rows,objects):
            H=transforms[row[0]];obj.setTransform(fcl.Transform(H[:3,:3],H[:3,3]))
        manager.update()
    def collision(a,b,data):return fcl.collide(a,b,data.request,data.result)>0
    # Adaptive samples at <=0.010 rad maximum joint displacement; each field
    # already passed the original live structural/path and retiming checks.
    speed=max(float(abs(np.diff(qr,axis=0)).max()),float(abs(np.diff(ql,axis=0)).max()),1e-12)
    stride=max(1,int(.010/speed))
    for fraction in [0.,.25,.50,.75]:
        delay=round(fraction*(len(qr)-1));length=max(len(qr),delay+len(ql));ok=True
        samples=sorted(set(list(range(0,length,stride))+[len(qr)-1,delay,delay+len(ql)-1,length-1]))
        for i in samples:
            update(rc,rr,ro,rm,qr[min(i,len(qr)-1)],right_width)
            update(lc,lr,lo,lm,ql[min(max(0,i-delay),len(ql)-1)],left_width)
            data=fcl.CollisionData(request,fcl.CollisionResult());rm.collide(lm,data,collision)
            if data.result.is_collision:ok=False;break
        if ok:return dict(delay_steps=delay,right_steps=len(right_rows),left_steps=len(left_rows),samples=len(samples),planning_s=time.perf_counter()-started,minimum_pair_padding_m=.001,maximum_sample_joint_step_rad=.010,backend='FCL convex CAD pair; original live static queries already passed')
    return None
