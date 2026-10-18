"""Reject a pin approach that strikes the stock pin before commanded closure."""
from pathlib import Path
import argparse,json
import numpy as np
import mujoco,fcl
from scipy.interpolate import CubicSpline
from plan_right_posture import Checker
from source_model import load_model


def audit(recording,plan_path,output,initial_scene=None):
    root=Path(__file__).resolve().parent
    source=json.loads((root/'generated/contact_task/contact-source.json').read_text())
    if initial_scene is not None:
        from stock_scene_variation import prepare_source
        source,_,_=prepare_source(source,initial_scene)
    z=np.load(recording/'trajectory.npz');plan=np.load(plan_path)
    model=load_model();data=mujoco.MjData(model)
    geometry=Checker.__new__(Checker);geometry.arr=np.load(root/'generated/contact_task/collision.npz');geometry.cache={};geometry.layout='physical'
    pin=source['parts']['pin'];R=geometry.rot(pin['quaternion']);P=np.array(pin['position']);pieces=[]
    for row in pin['colliders']:
        pieces.append(fcl.CollisionObject(geometry.shape(row),fcl.Transform(R@geometry.rot(row['quaternion']),P+R@row['position'])))
    robot=[]
    for row in source['geoms']:
        if row['body'].startswith('left_'):
            robot.append((row,fcl.CollisionObject(geometry.shape(row,True)),int(model.body(row['body']).id)))
    arms={side:np.array([int(model.joint(f'{side}_arm_fr3v2_joint{j}').qposadr[0])for j in range(1,8)])for side in ['left','right']}
    curves={side:CubicSpline(plan['time_s'],plan[side+'_qpos'],axis=0)for side in arms}
    schedule=json.loads((recording/'independent_schedule.json').read_text())
    end=next(r['start_s']for r in schedule['lanes']['left']if r['stage']['label']=='grasp_last_pin')
    start=next(r['start_s']for r in schedule['lanes']['left']if r['stage']['label']=='approach_last_pin')
    hits=[]
    for t in np.linspace(start,end,max(101,int((end-start)/.001))):
        data.qpos[:]=np.array([np.interp(t,z['time_s'],z['qpos'][:,j])for j in range(model.nq)])
        for side in arms:data.qpos[arms[side]]=curves[side](t)
        mujoco.mj_kinematics(model,data)
        for row,obj,body in robot:
            rot=data.xmat[body].reshape(3,3);pos=data.xpos[body]+rot@row['position']
            obj.setTransform(fcl.Transform(rot@geometry.rot(row['quaternion']),pos))
            for piece in pieces:
                result=fcl.CollisionResult()
                if fcl.collide(obj,piece,fcl.CollisionRequest(num_max_contacts=5,enable_contact=True),result):
                    hits.append(dict(reference=float(t),body=row['body'],contacts=[dict(position=np.asarray(c.pos).tolist(),depth=float(c.penetration_depth))for c in result.contacts]))
                    break
            if hits:break
        if hits:break
    report=dict(preclosure_triangle_clear=not hits,first_hits=hits,scope='Sampled robot convex hulls vs original stock pin compound triangles before grasp closure; not a dynamic grasp proof')
    output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--recording',type=Path,required=True);p.add_argument('--plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--initial-scene',type=Path)
    a=p.parse_args();audit(a.recording,a.plan,a.output,a.initial_scene)
