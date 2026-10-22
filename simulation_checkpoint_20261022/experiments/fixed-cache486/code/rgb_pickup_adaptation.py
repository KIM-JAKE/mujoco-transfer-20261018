"""Adapt taught pickup paths to RGB source observations; never move an object."""
import json,numpy as np
from isaac_kinematics import Arm

def adapt(arm,commands,clock,indices,observed,out,here):
    # These are nominal skill teaching datums, not live workpiece measurements.
    nominal=json.load(open(out/'arm-plan.json'))['scene_parts'];rows=[]
    for part,begin,stop in [('ring1',6.3,27.90),('pin',55.1,84.18)]:
        reference=np.array(nominal[part]['source_P'])
        if part=='pin':reference+=np.array(nominal[part]['source_R'])@np.array([0.,-.00025,0.])
        delta=np.array(observed[part]['center_m'])-reference
        if np.linalg.norm(delta)>.02:raise RuntimeError('RGB pickup exceeds this local taught-skill adaptation range: '+part)
        a=int(np.searchsorted(clock,begin));release=min(len(clock)-1,int(np.searchsorted(clock,stop)));b=min(len(clock)-1,release+360);first=a/480;lastq=commands[a,indices].copy();previous=lastq.copy()
        for k in range(a,b+1):
            base=commands[k,indices].copy();u=np.clip((k/480-first)/.4,0,1);w=u**3*(10-15*u+6*u*u);fade=np.clip((k-release)/360,0,1);w*=1-fade**3*(10-15*fade+6*fade*fade);H=arm.fk(base);H[:3,3]+=delta;sol=arm.solve(H[:3,3],H[:3,:3],lastq+base-previous,position_tolerance=1e-8,rotation_tolerance=1e-7)
            if sol is None:raise RuntimeError('RGB pickup target adaptation IK failed: '+part)
            commands[k,indices]=base+(sol-base)*w;previous=base;lastq=sol
        rows.append(dict(part=part,translation_m=delta.tolist(),robot_targets_only=True,source='RGB image inference'))
    np.savez_compressed(out/'rgb-initial-adapted-commands.npz',command=commands)
    (out/'rgb-pickup-adaptation.json').write_text(json.dumps(rows,indent=2));return commands
