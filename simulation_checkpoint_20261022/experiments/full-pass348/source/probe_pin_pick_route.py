"""Geometry-only pin approach test using saved RGB-D and robot encoders."""
from pathlib import Path
import json,time,numpy as np

def execute(c):
    h=Path(__file__).resolve().parent;r=h.parents[1]/'runs'/c['config']['pin_approach_probe_source_run'];report=json.loads((r/'pipeline-results.json').read_text());raw=np.load(r/'executed-actuator-commands.npz');cmd=raw['command'][-1];ri=[report['dof_names'].index('right_arm_fr3v2_joint'+str(i)) for i in range(1,8)];li=[report['dof_names'].index('left_arm_fr3v2_joint'+str(i)) for i in range(1,8)];obs=next(x for x in reversed(json.loads((r/'rgb-observations.json').read_text())) if x['mode']=='pin_initial' and 'pose'in x['result']);P=np.asarray(obs['result']['pose']['center_m']);pc=c['pin_context'];cand=pc['candidate'];H=np.asarray(cand['pickup_H']).copy();H[:3,3]+=P-np.asarray(json.loads((h.parents[1]/'runs/native-roas-table-pin-tail01/pin-direct-probe.json').read_text())['pickup_center_from_RGBD']);pre=H.copy();pre[2,3]+=.07;arm=c['arm'];rc=c['checkers']['right'];lc=c['checkers']['left'];start=cmd[ri];started=time.perf_counter();counts={};examples={};goals=[];solutions=[]
    def reject(key,why=None):
        counts[key]=counts.get(key,0)+1
        if why is not None and key not in examples:examples[key]=why
    seeds=[cand['pickup_q'],start]+list(np.random.default_rng(4121).uniform(arm.bounds[:,0],arm.bounds[:,1],(96,7)))
    with rc.other_arm_at(lc,cmd[li],float(cmd[[report['dof_names'].index('left_native_'+s) for s in ['left','right']]].sum())):
        start_clear=rc.clear(start,.014,pc['paths']['workpiece_pin'])
        print('PIN_APPROACH_START',start_clear,flush=True)
        for seed in seeds:
            q=arm.solve(pre[:3,3],pre[:3,:3],seed)
            if q is None:reject('pre_IK');continue
            if any(np.linalg.norm(q-old)<.05 for old in goals):continue
            clear,why=rc.clear(q,.014,pc['paths']['workpiece_pin'])
            if not clear:reject('pre_collision',why);continue
            goals.append(q)
        goals.sort(key=lambda q:np.linalg.norm(q-start))
        for q in goals:
            stroke=[q];valid=True
            for u in np.linspace(0,1,9)[1:]:
                nq=arm.solve(pre[:3,3]*(1-u)+H[:3,3]*u,H[:3,:3],stroke[-1])
                if nq is None:reject('descend_IK');valid=False;break
                clear,why=rc.path([stroke[-1],nq],.014,pc['paths']['workpiece_pin'])
                if not clear:reject('descend_collision',why);valid=False;break
                stroke.append(nq)
            if not valid:continue
            route=rc.route(start,q,.014,pc['paths']['workpiece_pin'],seconds=1.)
            if route is None:
                reject('route')
                for via in c['empty_return_waypoints']:
                    if rc.path([start,via,q],.014,pc['paths']['workpiece_pin'])[0]:route=[start,via,q];break
            if route is not None:
                solutions.append(dict(pre_q=q.tolist(),route=np.asarray(route).tolist(),descent=np.asarray(stroke).tolist()));break
    result=dict(state='pin_geometry_only',full_task_pass=False,inputs='Saved pin RGB-D and robot encoders; fixed CAD',uses_free_body_poses=False,start_clear=start_clear,counts=counts,examples=examples,pre_solutions=len(goals),solutions=solutions,planning_s=time.perf_counter()-started)
    print('PIN_APPROACH_DIAGNOSTIC',json.dumps(result),flush=True);return result
