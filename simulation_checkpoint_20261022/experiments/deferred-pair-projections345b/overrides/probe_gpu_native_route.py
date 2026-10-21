"""No-motion full loaded Checker profiling, recorded image+encoder inputs only."""
from pathlib import Path
import cProfile,pstats,json,time,numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame
from placed_part_clearance import PlacedPartClearance

def execute(c):
    out=c['vision'].out;root=out.parent;source=root/'native-roas-table-scatter-stable-v327-gpu-pickup-half-gain-all3-445'
    data=np.load(source/'executed-actuator-commands.npz');obs=json.loads((source/'rgb-observations.json').read_text());report=json.loads((source/'pipeline-results.json').read_text());names=report['dof_names'];ids={s:[names.index(s+'_arm_fr3v2_joint'+str(i))for i in range(1,8)]for s in ['left','right']};fi={s:[i for i,n in enumerate(names)if n.startswith(s+'_native_')]for s in ids};positions=data['actual_dof_position'];part='ring1';side='left';other='right';checker=c['checkers'][side];otherchecker=c['checkers'][other];offset=frame([0,0,.1034],np.eye(3))
    # Final eight LARGE prealignment captures use actual saved camera H, RGBD
    # centre/axis and original FR3 encoder configurations, never actor poses.
    captures=[obs[i]for i in [77,79,81,83,85,87,89,91]]
    start=int(np.searchsorted(data['time_s'],captures[0]['captured_wall_s'],side='right')-1);end=int(np.searchsorted(data['time_s'],captures[-1]['captured_wall_s'],side='right')-1)
    indices=np.unique(np.linspace(start,end,90,dtype=int));payload=c['part_collision_templates'][part]
    def grasp(o):return np.linalg.inv(np.asarray(o['hand_TCP_world']))@frame(o['result']['pose']['center_m'],axis_frame(o['result']['pose']['axis']))
    checker.attach_payload(payload,offset@grasp(captures[0]));placed=PlacedPartClearance()
    for cc in c['checkers'].values():cc.placed_part_clearance=placed
    smallgoal=np.asarray(c['goals']['ring2']['P']);small=[o for o in obs if o.get('part')=='ring2'and o['result'].get('pose')and o['captured_wall_s']<captures[0]['captured_wall_s']and np.linalg.norm(np.asarray(o['result']['pose']['center_m'])-smallgoal)<.0015]
    if not small:raise RuntimeError('Missing actually observed completed SMALL pose for full-checker case')
    small=max(small,key=lambda o:o['captured_wall_s']);sp=small['result']['pose'];placed.register(c['checkers']['right'],'ring2',c['part_collision_templates']['ring2'],frame(sp['center_m'],axis_frame(sp['axis'])),uncertainty_m=.0035)
    prepared=[]
    for k in indices:
        available=[o for o in captures if o['captured_wall_s']<=data['time_s'][k]+1e-9];o=max(available,key=lambda v:v['captured_wall_s'])if available else captures[0]
        prepared.append((int(k),positions[k,ids[side]].astype(float),float(positions[k,fi[side]].sum()),positions[k,ids[other]].astype(float),float(positions[k,fi[other]].sum()),offset@grasp(o),o['captured_wall_s']))

    import predictive_pair_clearance as pair_module
    equality=0;overlap_cases=0
    for row in prepared:
        _,q,w,oq,ow,G,_=row;checker.payload_relative=G
        for cc,pose,width in [(checker,q,w),(otherchecker,oq,ow)]:
            full=pair_module.at_pose(cc,pose,width)
            lazy=pair_module.at_pose(cc,pose,width,lazy=True)
            for (_,_,a),(_,_,b) in zip(full,lazy):
                if not all(np.array_equal(a[i],b[i]) for i in range(5)):
                    raise RuntimeError('Lazy full geometry changed')
                if b[4] is not b[4]:raise RuntimeError('Normal identity changed')
                # SAT contacts and translated separation use all original data.
                for shift in ([0.,.0009,.01,.15] if equality<12 else []):
                    T=np.eye(4);T[0,3]=shift
                    shifted=pair_module.transform_proxy(a,T)
                    x=pair_module.shape_overlap(a,shifted,margin=.0005)
                    y=pair_module.shape_overlap(b,shifted,margin=.0005)
                    if x!=y:raise RuntimeError('Lazy SAT verdict changed')
                    overlap_cases+=1
                equality+=1
    def query(row):
        k,q,w,oq,ow,G,t=row;checker.payload_relative=G
        with checker.other_arm_at(otherchecker,oq,ow):return checker.clear(q,w)
    profiles=[]
    try:
        for optimized in [False,True,False,True]:
            pair_module._LAZY_PAIR_PROJECTIONS=optimized
            query(prepared[0]);prof=cProfile.Profile();began=time.perf_counter();prof.enable()
            answers=[query(row)for row in prepared]
            prof.disable();elapsed=time.perf_counter()-began
            st=pstats.Stats(prof);top=[]
            for key,v in sorted(st.stats.items(),key=lambda x:x[1][3],reverse=True)[:25]:
                cc,nc,tt,ct,_=v;top.append(dict(function=key[2],calls=nc,self_s=tt,cumulative_s=ct))
            profiles.append(dict(lazy=optimized,wall_s=elapsed,answers=[dict(clear=bool(a),reason=b)for a,b in answers],top=top))
            print('LAZY345_BATCH',optimized,elapsed,flush=True)
    finally:pair_module._LAZY_PAIR_PROJECTIONS=True
    answers_equal=all(p['answers']==profiles[0]['answers']for p in profiles)
    # Same negative self-collision, fresh original full checker each time.
    ns=root/'native-roas-table-scatter-stable-v272-empty-park-preshape-446';nd=np.load(ns/'executed-actuator-commands.npz');nn=json.loads((ns/'pipeline-results.json').read_text())['dof_names']
    ni=[nn.index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)];nf=[i for i,n in enumerate(nn)if n.startswith('right_native_')]
    negatives=[]
    for optimized in [False,True]:
        pair_module._LAZY_PAIR_PROJECTIONS=optimized;a,b=otherchecker.clear(nd['actual_dof_position'][490,ni],float(nd['actual_dof_position'][490,nf].sum()));negatives.append(dict(clear=bool(a),reason=b))
    pair_module._LAZY_PAIR_PROJECTIONS=True
    baseline=sum(p['wall_s']for p in profiles if not p['lazy']);optimized=sum(p['wall_s']for p in profiles if p['lazy'])
    result=dict(state='deferred_exact_pair_vertex_projection_component',source=source.name,robot_motion_executed=False,hardware_tested=False,full_task_pass=False,uses_free_body_poses=False,full_geometry_equality_checks=equality,original_SAT_overlap_case_comparisons=overlap_cases,full_checker_queries_per_batch=len(prepared),all_full_checker_results_equal=answers_equal,negative_preserved=negatives[0]==negatives[1]and not negatives[0]['clear'],negative_results=negatives,speedup=baseline/optimized,profiles=profiles)
    result['approved']=result['all_full_checker_results_equal']and result['negative_preserved']and result['speedup']>1.1
    (out/'lazy345-full-checker-comparison.json').write_text(json.dumps(result,indent=2));print('LAZY345_RESULT',json.dumps({k:v for k,v in result.items()if k not in ['profiles','negative_results']}),flush=True)
    return result
