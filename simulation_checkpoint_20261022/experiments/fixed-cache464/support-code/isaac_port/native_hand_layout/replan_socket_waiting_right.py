"""Replan pin entry with the other arm explicitly present at button standby.

Retains the existing carried-CAD proofs for unchanged segments. Changed empty
paths are checked against the full occupied cell plus the stationary right arm.
"""
from audit_five_geometry import *
from compact_compare import arm_pair_clear
from layout_search import ROBOT
import hashlib


def frozen_right(q):
    observed=objects('right',q,0.)[0];rows=[]
    for (meta,_),(name,obj) in zip(ROBOT['right'],observed[:len(ROBOT['right'])]):
        if name.endswith('link0'):continue
        shape=g.checker.shape(meta,True)
        rows.append(('waiting_'+name,fcl.CollisionObject(shape,fcl.Transform(obj.getRotation(),obj.getTranslation()))))
    for (_,shape,_,_),(name,obj) in zip(g.shapes,observed[len(ROBOT['right']):]):
        rows.append(('waiting_'+name,fcl.CollisionObject(shape,fcl.Transform(obj.getRotation(),obj.getTranslation()))))
    return rows


def replan(height):
    source=HERE/f'five-layout-right_high_h{height}_p35_yaw-45-socket-payload.json'
    row=json.load(open(source));button=json.load(open(HERE/'five-button-cache.json'));qright=np.asarray(button['button_approach'][-1]['b']);right=frozen_right(qright)
    parent_hash=hashlib.sha256(source.read_bytes()).hexdigest();base=obstacles(row)
    scene=FixedScene(list(base)+right,(base.cache_token,'waiting_right',tuple(qright)));set_env(scene)
    failures=[]
    for slot in row['pin_slots']:
        r=slot['recipe'];old=[s for s in r['segments']if s['label']=='empty_approach'];chain=[np.asarray(old[0]['a'])]+[np.asarray(s['b'])for s in old]
        print('WAIT_PIN_BEGIN',height,slot['slot'],flush=True)
        if not path_clear('left',chain,.02,41)[0]:
            replacement=route('left',chain[0],chain[-1],.02,limit=350)
            if replacement is None:
                failures.append(dict(slot=slot['slot'],stage='waiting_right_entry_route'));print('WAIT_PIN_ROUTE_FAIL',height,slot['slot'],flush=True);break
            entry=[seg(a,b,'empty_approach','pin',.02)for a,b in zip(replacement[:-1],replacement[1:])]
            r['segments']=entry+[s for s in r['segments']if s['label']!='empty_approach']
        # Every carried-CAD proof outside entry is unchanged. Independently
        # verify that the stationary right arm also clears the whole pin path.
        cross=[]
        for index,s in enumerate(r['segments']):
            held=r if s['label']in('grasp','clear_datum','extract','lift','transfer','insert','clamp','release')else None
            for u in np.linspace(0,1,max(21,int(np.ceil(np.max(abs(np.asarray(s['b'])-s['a']))/.015))+1)):
                q=(1-u)*np.asarray(s['a'])+u*np.asarray(s['b'])
                if not arm_pair_clear((q,s['gap_m']),(qright,0.),left_payload=held):
                    cross.append(dict(slot=slot['slot'],stage='waiting_right_cross',segment=index,label=s['label'],fraction=float(u)));break
        if cross:
            failures+=cross;print('WAIT_PIN_CROSS_FAIL',height,cross,flush=True);break
        r['duration_s']=sum(s['duration_s']for s in r['segments']);print('WAIT_PIN_DONE',height,slot['slot'],r['duration_s'],flush=True)
        row['recipes']['pin']=copy.deepcopy(row['pin_slots'][0]['recipe'])
        row['waiting_arm_replanning_progress']=dict(completed_slots=slot['slot'],right_q=qright.tolist())
        source.with_name(source.stem+'-waitright-progress.json').write_text(json.dumps(row,indent=2))
    row['label']+='_waitright';row['waiting_arm_replanning']=dict(parent_file=str(source),parent_sha256=parent_hash,right_q=qright.tolist(),entry_world_arm_reserve_m=.003,cross_arm_reserve_m=.001,failures=failures,passed=not failures,scope='Only empty entry paths changed; unchanged free transfers retain parent exact CAD proof')
    row['nominal_payload_paths_pass']=not failures
    row['recipes']['pin']=copy.deepcopy(row['pin_slots'][0]['recipe']);out=source.with_name(source.stem+'-waitright.json');out.write_text(json.dumps(row,indent=2));print('WAIT_LAYOUT_DONE',height,not failures,out,flush=True)


if __name__=='__main__':
    for argument in sys.argv[1:]:replan(int(argument))
