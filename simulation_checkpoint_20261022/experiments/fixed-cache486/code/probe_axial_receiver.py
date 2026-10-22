"""Fixed CAD/IK/PhysX insertion screen, without executing a robot motion."""
import time
import numpy as np
import axial_rim_receiver


def execute(c):
    started=time.perf_counter();results={}
    for part in ['ring1','ring2']:
        before=dict(c.get('_axial_receiver_fixture_counts',{}));approved=[];geometric=0
        incoming=np.asarray(c['handoff_reference'][part]['part_H'])
        deadline=time.perf_counter()+float(c.get('config',{}).get('axial_receiver_fixture_budget_s',45.))
        for target,G,width,metadata in axial_rim_receiver.geometric_candidates(c,part,incoming):
            geometric+=1
            if time.perf_counter()>deadline:break
            fixture=axial_rim_receiver.fixture_clear(c,part,G,width,deadline=deadline)
            if fixture is not None:
                approved.append(dict(target=target.tolist(),parameters=list(metadata),**fixture))
                if len(approved)>=4:break
        counts={k:v-before.get(k,0)for k,v in c.get('_axial_receiver_fixture_counts',{}).items()}
        results[part]=dict(geometric_candidates_tested=geometric,approved=approved,counts=counts)
        print('AXIAL_RECEIVER_FIXTURE_SCREEN',part,len(approved),counts,flush=True)
    return dict(state='axial_receiver_fixed_geometry_probe_only',full_task_pass=False,uses_free_body_poses=False,robot_motion_executed=False,physical_insertion_validated=False,elapsed_s=time.perf_counter()-started,parts=results,rejection_examples=c.get('_axial_receiver_fixture_examples',{}))
