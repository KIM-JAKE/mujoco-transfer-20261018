"""Check the optional OBB broad phase against original exact FCL on real poses."""
import compact_compare as cc
import os,time,json
import numpy as np

row=json.load(open(cc.HERE/'five_right_high_h1400_p35_yaw-45-scheduled.json'))
rng=np.random.default_rng(913);cases=[]
for _ in range(45):
    cycle=row['cycles'][int(rng.integers(0,5))];states={};held={}
    for side in ['left','right']:
        lane=cycle['lanes'][side];segment=lane[int(rng.integers(0,len(lane)))];states[side]=cc.state_at(segment,float(rng.random())*segment['duration_s']);held[side]=segment.get('payload')
    cases.append((states,held))
failure=cc.HERE/'coordination-last-failure.json'
if failure.exists():
    for node in json.load(open(failure))['frontier']:
        for event in node['blocked']:
            cases.append((dict(left=(np.asarray(event['left_q']),event['left_gap']),right=(np.asarray(event['right_q']),event['right_gap'])),dict(left=event['left_payload'],right=event['right_payload'])))
records=[]
for index,(states,held) in enumerate(cases):
    result=[];timings=[]
    for enabled in ['0','1']:
        os.environ['M80_PAIR_OBB_SCREEN']=enabled;start=time.perf_counter();result.append(cc._arm_pair_clear(states['left'],states['right'],left_payload=held['left'],right_payload=held['right']));timings.append(time.perf_counter()-start)
    records.append(dict(index=index,original=result[0],obb=result[1],same=result[0]==result[1],original_s=timings[0],obb_s=timings[1]))
    if index%10==0:print('PAIR_OBB_CASE',index,result,timings,flush=True)
report=dict(all_same=all(r['same']for r in records),cases=len(records),colliding_cases=sum(not r['original']for r in records),original_total_s=sum(r['original_s']for r in records),obb_total_s=sum(r['obb_s']for r in records),geometry='Original CAD FCL shapes unchanged; OBB separating-axis lower-bound screen only',results=records)
(cc.HERE/'pair-obb-equivalence.json').write_text(json.dumps(report,indent=2));print('PAIR_OBB_DONE',report['all_same'],report['cases'],report['colliding_cases'],report['original_total_s'],report['obb_total_s'],flush=True)
