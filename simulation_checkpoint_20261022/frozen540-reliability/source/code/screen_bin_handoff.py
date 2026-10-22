"""No-fixture two-arm regrasp candidates, screened against original cell CAD.

This is only motion preparation. PhysX contact transfer and full assembly are
mandatory separate gates. A mirrored ring is never substituted for the CAD.
"""
from pathlib import Path
import sys,json,itertools,copy
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parent)]
import screen_bin_reach as env
cc=env.cc;ct=env.ct;OUT=HERE/'bin-picking'
report=dict(scope=__doc__,original_part_orientation_preserved=True,additional_fixture=False,physics_pass=False,full_task_pass=False,parts={})
def save():(OUT/'handoff-screen.json').write_text(json.dumps(report,indent=2))
for part in ['ring1','ring2','pin']:
    base=ct.BANDS[part];P,Rp=env.goals[part];wins=[];fails=[]
    for yaw in [0,30,-30,60,-60,90,120,180,240,300]:
        Z=Rotation.from_euler('z',yaw,degrees=True).as_matrix();lp=Z@np.array(base['localP']);L=Z@np.array(base['localR']);tcp=P+Rp@lp;R=Rp@L
        for side in ['left','right']:
            qs,diag=env.branches(side,tcp,R,base['gap'],n=18)
            if not qs:fails.append(dict(side=side,yaw=yaw,**diag));continue
            wins.append(dict(side=side,localP=lp.tolist(),localR=L.tolist(),gap=base['gap'],q_insert=[q.tolist()for q in qs],tcp_insert=tcp.tolist(),R_insert=R.tolist()))
        if any(w['side']=='left'for w in wins):break
    record=dict(insertion_candidates=wins,insertion_failures=fails,handoff_candidates=[]);report['parts'][part]=record;save()
    print('HANDOFF_INSERT',part,len(wins),flush=True)
    if part=='pin':continue
    for w in [x for x in wins if x['side']=='left']:
        L=np.array(w['localR']);lp=np.array(w['localP']);Rgive=np.diag([1.,-1.,-1.]);gap=w['gap'];done=False
        for xyz in itertools.product([-.15,-.25,-.05],[-.25,-.35,-.15],[1.20,1.30,1.40]):
            C=np.array(xyz);give,gd=env.branches('right',C+[0,0,.003],Rgive,.075 if part=='ring1'else .062,n=10)
            if not give:continue
            receive,rd=env.branches('left',C+lp,L,.08,n=10)
            if not receive:continue
            for a,b in itertools.product(give,receive):
                if cc.arm_pair_clear((b,.08),(a,.075 if part=='ring1'else .062))and cc.arm_pair_clear((b,gap),(a,.08)):
                    record['handoff_candidates'].append(dict(part_center=C.tolist(),giver_q=a.tolist(),recipient_q=b.tolist(),recipient_recipe=w));done=True;break
            if done:break
        if done:break
    save();print('HANDOFF_CANDIDATES',part,len(record['handoff_candidates']),flush=True)
