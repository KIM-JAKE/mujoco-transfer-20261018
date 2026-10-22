"""Saved SMALL donor carry; fresh joins and unchanged full current guards."""
from pathlib import Path
from contextlib import contextmanager
import hashlib,time,json
import numpy as np
from fixed_small_donor_plan import META,right_donor

class SavedDonorRoute:
    def __init__(self,issued,initial,evidence):
        self.issued=np.asarray(issued,float).copy();self.initial=np.asarray(initial,np.float32).copy();self.evidence=evidence
        if self.issued.ndim!=2 or self.issued.shape[1]!=9 or not len(self.issued)or not np.isfinite(self.issued).all():raise ValueError('Finite saved donor N×9 field required')
        self.issued.setflags(write=False);self.initial.setflags(write=False)
    def commands(self,c,side,width):
        if side!='right' or width is not None or abs(c['dt']-META['dt_s'])>1e-15:raise RuntimeError('Saved donor command policy changed')
        current=np.r_[c['cmd'][0,c['ids']],c['cmd'][0,c['fi']]].astype(np.float32)
        if not np.array_equal(current,self.initial):raise RuntimeError('Saved donor entry changed after preflight')
        return self.issued.copy()

def prepare(checkers,art,indices,fingers,paths,goals,config,cache_dir):
    """Compile/load the static-only SMALL donor envelope before first camera capture."""
    if not(config.get('fixed_supply_mode')and config.get('fixed_prevalidated_routes')):return None
    from fixed_static_route_certificate import StaticRows,static_world_seal,compile_static_rows,PrismaticFixtureSweeps,payload_geometry_sha256
    from exact_collision_proof_cache import own_model_fingerprint
    from isaac_kinematics import frame
    from fixed_saved_executor import audit_saved_field
    q=right_donor();start=time.perf_counter();audit=audit_saved_field(q)
    if hashlib.sha256(q.astype(np.float32).tobytes()).hexdigest()!=META['canonical_float32_field_sha256']:raise RuntimeError('Saved LEFT command bytes changed')
    lc,rc=checkers['right'],checkers['left'];seal=static_world_seal(lc);model=own_model_fingerprint(lc.same_arm_clearance)
    plan_identity=hashlib.sha256(json.dumps(META,sort_keys=True,separators=(',',':')).encode()).hexdigest();key=hashlib.sha256((seal['sha256']+model+plan_identity).encode()).hexdigest();target=Path(cache_dir)/('small-donor-'+key+'.json');target.parent.mkdir(parents=True,exist_ok=True)
    saved=(lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative);G=frame([0,0,.1034],np.eye(3))@np.asarray(META['nominal_TCP_observation_T'])
    try:
        lc.attach_payload(paths['workpiece_ring2'],G);cache_hit=False
        for candidate in[target,Path(__file__).resolve().parents[1]/'runs/native-fixed-more-static-fields-model441/SMALL_donor-static-certificate.json']:
            if not candidate.exists():continue
            try:
                cert=StaticRows.load(candidate);m=cert.metadata
                cache_hit=(m['world_sha256']==seal['sha256']and m['model_sha256']==model and m['payload_geometry_sha256']==payload_geometry_sha256(lc)and m['width_interval_m']==META['width_interval_m']and m['nominal_hand_object_T']==G.tolist()and m['translation_m']==META['payload_translation_m']and m['rotation_rad']==META['payload_rotation_rad'])
            except(ValueError,KeyError,OSError):cache_hit=False
            if cache_hit:break
        if not cache_hit:
            actual=art.get_dof_positions()[0]
            with lc.other_arm_at(rc,actual[indices['left']],float(actual[fingers['left']].sum())):
                cert,mask,why=compile_static_rows(lc,q,META['width_interval_m'],world_seal=seal,model_sha256=model,nominal_hand_object_T=G,translation_m=META['payload_translation_m'],rotation_rad=META['payload_rotation_rad'],fixture_sweep_clear=PrismaticFixtureSweeps(lc,seal))
            tmp=target.with_suffix('.'+str(__import__('os').getpid())+'.tmp');cert.save(tmp);tmp.replace(target)
        summary=dict(ready=True,cache_hit=cache_hit,preparation_s=time.perf_counter()-start,canonical_rows=len(q),static_certified_unique_rows=len(cert.row_keys),world_sha256=seal['sha256'],model_sha256=model,compile_excluded_from_cycle=True,saved_command_limit_audit=audit)
        from fixed_own_route_certificate import prepare_like_static
        try:
            own_proof=prepare_like_static(lc,q,cert,cache_dir)
        except Exception as error:
            own_proof=dict(certificate=None,summary=dict(ready=False,reason=type(error).__name__+': '+str(error),original_own_checks_retained=True))
        summary['own_envelope_preparation']=own_proof['summary']
        return dict(certificate=cert,own_certificate=own_proof['certificate'],world_sha256=seal['sha256'],model_sha256=model,summary=summary)
    finally:lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative=saved


@contextmanager
def static_scope(c,checker):
    state=c.get('fixed_prevalidated_small_donor_routes')
    if not state or not state.get('certificate'):yield {};return
    from fixed_static_route_certificate import bound_static_rows
    with bound_static_rows(checker,state['certificate'],current_world_sha256=state['world_sha256'],current_model_sha256=state['model_sha256'])as stats:
        from fixed_own_route_certificate import bound_own_rows
        own=state.get('own_certificate')
        if own is None:
            yield stats
        else:
            with bound_own_rows(checker,own,current_world_sha256=state['world_sha256'],current_model_sha256=state['model_sha256'])as own_stats:
                yield stats
            stats.update(own_stats)


def candidate(c,part,G,target_q):
    """Proposal only: caller MUST retain original actual issued full guards."""
    state=c.get('fixed_prevalidated_small_donor_routes')
    if part!='ring2' or not(c['config'].get('fixed_supply_mode')and c['config'].get('fixed_prevalidated_routes'))or not state or not state.get('certificate'):return None
    from fixed_static_route_certificate import grasp_in_envelope
    if not grasp_in_envelope(np.asarray(META['nominal_TCP_observation_T']),np.asarray(G),META['payload_translation_m'],META['payload_rotation_rad']):return None
    if abs(c['dt']-META['dt_s'])>1e-15 or max(1.,float(c['config'].get('stability_time_scale',1.)))!=1.:return None
    q=right_donor();ids,fi=c['ids'],c['fi'];initial=np.r_[c['cmd'][0,ids],c['cmd'][0,fi]].copy();actual=c['art'].get_dof_positions()[0];target_q=np.asarray(target_q,float)
    if target_q.shape!=(7,)or np.max(abs(initial[:7]-q[0]))>.15 or np.max(abs(target_q-q[-1]))>.15:return None
    lo,hi=META['width_interval_m']
    if not all(lo<=w<=hi for w in[float(actual[fi].sum()),float(initial[7:].sum())]):return None
    from fr3_path_timing import retime
    from fixed_saved_executor import audit_saved_field
    def bridge(a,b):
        values,h,timing=retime(np.asarray([a,b]),.05,c['dt'],initial[7:],initial[7:],optimize=True)
        return np.c_[values[1:],np.repeat(initial[None,7:],len(values)-1,axis=0)]
    try:
        entry=bridge(initial[:7],q[0]);middle=np.c_[q[1:],np.repeat(initial[None,7:],len(q)-1,axis=0)];exit=bridge(q[-1],target_q)
        issued=np.vstack([entry,middle,exit]).astype(np.float32).astype(float);audit=audit_saved_field(np.vstack([initial[:7],issued[:,:7]]).astype(np.float32).astype(float))
    except RuntimeError as error:
        c['planning_diagnostics'].append(dict(event='saved_SMALL_donor_rejected',stage='entry_exit_hardware_audit',reason=str(error)));return None
    evidence=dict(event='saved_SMALL_donor_proposed',canonical_rows=len(q)-1,fresh_entry_rows=len(entry),fresh_exit_rows=len(exit),fresh_RGBD_grasp=True,exact_original_middle=True,saved_command_limit_audit=audit,original_full_current_issued_guard_still_required=True,uses_free_body_poses=False)
    c['planning_diagnostics'].append(evidence)
    return SavedDonorRoute(issued,initial,evidence)
