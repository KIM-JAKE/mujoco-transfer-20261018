"""Saved native LEFT carry with a fresh entry and current full guards."""
from pathlib import Path
from contextlib import contextmanager
import hashlib,time,json
import numpy as np
from fixed_small_left_plan import META,left_carry

class SavedCommandRoute:
    """An immutable already-timed issued field, never polyline waypoints."""
    def __init__(self,issued,initial,diagnostics):
        self.issued=np.asarray(issued,float).copy();self.initial=np.asarray(initial,np.float32).copy();self.diagnostics=dict(diagnostics)
        if self.issued.ndim!=2 or self.issued.shape[1]!=9 or not len(self.issued)or not np.isfinite(self.issued).all():raise ValueError('Finite saved issued N×9 field required')
        self.issued.setflags(write=False);self.initial.setflags(write=False)
    def commands(self,c,side,width):
        if side!='left' or width is not None or abs(c['dt']-META['dt_s'])>1e-15:raise RuntimeError('Saved LEFT command field policy changed')
        current=np.r_[c['cmd'][0,c['left_ids']],c['cmd'][0,c['left_fi']]].astype(np.float32)
        if not np.array_equal(current,self.initial):raise RuntimeError('Saved LEFT entry command changed after preflight')
        return self.issued.copy()


def prepare(checkers,art,indices,fingers,paths,goals,config,cache_dir):
    """Compile/load the static-only LEFT envelope before first camera capture."""
    if not(config.get('fixed_supply_mode')and config.get('fixed_prevalidated_routes')):return None
    from fixed_static_route_certificate import StaticRows,static_world_seal,compile_static_rows,PrismaticFixtureSweeps,payload_geometry_sha256
    from exact_collision_proof_cache import own_model_fingerprint
    from isaac_kinematics import frame
    from fixed_saved_executor import audit_saved_field
    q=left_carry();start=time.perf_counter();audit=audit_saved_field(q)
    if hashlib.sha256(q.astype(np.float32).tobytes()).hexdigest()!=META['canonical_float32_field_sha256']:raise RuntimeError('Saved LEFT command bytes changed')
    if not np.array_equal(np.asarray(goals['ring2']['P']),np.asarray(META['goal'])):return dict(ready=False,reason='Stored SMALL fixture calibration changed')
    lc,rc=checkers['left'],checkers['right'];seal=static_world_seal(lc);model=own_model_fingerprint(lc.same_arm_clearance)
    plan_identity=hashlib.sha256(json.dumps(META,sort_keys=True,separators=(',',':')).encode()).hexdigest();key=hashlib.sha256((seal['sha256']+model+plan_identity).encode()).hexdigest();target=Path(cache_dir)/('small-left-'+key+'.json');target.parent.mkdir(parents=True,exist_ok=True)
    saved=(lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative);G=frame([0,0,.1034],np.eye(3))@np.asarray(META['nominal_TCP_observation_T'])
    try:
        lc.attach_payload(paths['workpiece_ring2'],G);cache_hit=False
        for candidate in[target,Path(__file__).resolve().parents[1]/'runs/native-fixed-more-static-fields-model441/SMALL_LEFT_carry-static-certificate.json']:
            if not candidate.exists():continue
            try:
                cert=StaticRows.load(candidate);m=cert.metadata
                cache_hit=(m['world_sha256']==seal['sha256']and m['model_sha256']==model and m['payload_geometry_sha256']==payload_geometry_sha256(lc)and m['width_interval_m']==META['width_interval_m']and m['nominal_hand_object_T']==G.tolist()and m['translation_m']==META['payload_translation_m']and m['rotation_rad']==META['payload_rotation_rad'])
            except(ValueError,KeyError,OSError):cache_hit=False
            if cache_hit:break
        if not cache_hit:
            actual=art.get_dof_positions()[0]
            with lc.other_arm_at(rc,actual[indices['right']],float(actual[fingers['right']].sum())):
                cert,mask,why=compile_static_rows(lc,q,META['width_interval_m'],world_seal=seal,model_sha256=model,nominal_hand_object_T=G,translation_m=META['payload_translation_m'],rotation_rad=META['payload_rotation_rad'],fixture_sweep_clear=PrismaticFixtureSweeps(lc,seal))
            tmp=target.with_suffix('.'+str(__import__('os').getpid())+'.tmp');cert.save(tmp);tmp.replace(target)
        summary=dict(ready=True,cache_hit=cache_hit,preparation_s=time.perf_counter()-start,canonical_rows=len(q),static_certified_unique_rows=len(cert.row_keys),world_sha256=seal['sha256'],model_sha256=model,compile_excluded_from_cycle=True,saved_command_limit_audit=audit)
        from fixed_own_route_certificate import prepare_like_static
        try:
            own_proof=prepare_like_static(lc,q,cert,cache_dir)
        except Exception as error:
            own_proof=dict(certificate=None,summary=dict(ready=False,reason=type(error).__name__+': '+str(error),original_own_checks_retained=True))
        summary['own_envelope_preparation']=own_proof['summary']
        from fixed_small_own_domains import prepare as prepare_additional_own
        try:additional=prepare_additional_own(lc,q,cert,cache_dir)
        except Exception as error:additional=dict(domains=[],summary=dict(ready=False,reason=type(error).__name__+': '+str(error),original_own_checks_retained=True))
        summary['additional_own_domain_preparation']=additional['summary']
        return dict(certificate=cert,own_certificate=own_proof['certificate'],additional_own_domains=additional['domains'],world_sha256=seal['sha256'],model_sha256=model,summary=summary)
    finally:lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative=saved


@contextmanager
def static_scope(c,checker):
    state=c.get('fixed_prevalidated_small_left_routes')
    if not state or not state.get('certificate'):yield {};return
    from fixed_static_route_certificate import bound_static_rows
    with bound_static_rows(checker,state['certificate'],current_world_sha256=state['world_sha256'],current_model_sha256=state['model_sha256'])as stats:
        from fixed_own_route_certificate import bound_own_rows
        from fixed_small_own_domains import select as select_own_domain
        own,domain_name=select_own_domain(checker,state)
        stats['own_domain_name']=domain_name
        if own is None:
            yield stats
        else:
            with bound_own_rows(checker,own,current_world_sha256=state['world_sha256'],current_model_sha256=state['model_sha256'])as own_stats:
                yield stats
            stats.update(own_stats)


def candidate(c,part,G,trajectory,verify):
    """No actuation: return a complete current-guarded saved field or None."""
    if part!='ring2' or not(c['config'].get('fixed_supply_mode')and c['config'].get('fixed_prevalidated_routes')):return None
    from fixed_static_route_certificate import grasp_in_envelope
    if not np.array_equal(np.asarray(c['goals'][part]['P']),np.asarray(META['goal'])):return None
    if not grasp_in_envelope(np.asarray(META['nominal_TCP_observation_T']),np.asarray(G),META['payload_translation_m'],META['payload_rotation_rad']):return None
    if abs(c['dt']-META['dt_s'])>1e-15:return None
    if max(1.,float(c['config'].get('stability_time_scale',1.)))!=1.:return None
    art,cmd,li,lf,lc=[c[k]for k in['art','cmd','left_ids','left_fi']]+[c['checkers']['left']]
    measured=art.get_dof_positions()[0];gaps=[float(measured[lf].sum()),float(cmd[0,lf].sum())];lo,hi=META['width_interval_m']
    if not all(lo<=gap<=hi for gap in gaps):return None
    q=left_carry();initial=np.r_[cmd[0,li],cmd[0,lf]].copy()
    if np.max(abs(cmd[0,li]-q[0]))>.15:return None
    # Canonical entry is a stopped boundary. The changed join is retimed
    # with the original hardware limits, and the WHOLE concatenated field is
    # independently audited before its first physical command.
    bridge=trajectory('left',[cmd[0,li].copy(),q[0]],.05,checked_edges=True)
    middle=np.c_[q[1:],np.repeat(cmd[0,lf][None,:],len(q)-1,axis=0)]
    issued=np.vstack([bridge,middle])
    from fixed_saved_executor import audit_saved_field
    try:audit=audit_saved_field(np.vstack([initial[:7],issued[:,:7]]).astype(np.float32).astype(float))
    except RuntimeError as error:
        c['planning_diagnostics'].append(dict(event='saved_SMALL_LEFT_carry_rejected',stage='command_limits',reason=str(error)));return None
    from isaac_kinematics import frame,exp
    goal=np.asarray(c['goals'][part]['P']);R=exp(np.array([0.,0.,META['ring_goal_azimuth_rad']]))
    pre=frame(goal-[0,0,META['preinsert_clearance_m']],R)@np.linalg.inv(G);final=frame(goal,R)@np.linalg.inv(G)
    saved=(lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative);stroke=[q[-1]];opening=max(gaps)
    try:
        lc.detach_payload()
        if not lc.hand_clear(pre,opening)[0]or not lc.hand_clear(final,opening)[0]:return None
        for u in np.linspace(0,1,9)[1:]:
            nq=lc.arm.solve(pre[:3,3]*(1-u)+final[:3,3]*u,final[:3,:3],stroke[-1])
            if nq is None:return None
            stroke.append(nq)
        # The same eight straight joint segments, with one frozen opposite
        # arm snapshot and one batch for their pair component. Original
        # native self/fixed/selected-contact path queries remain authority.
        started_stroke=time.perf_counter()
        from contextlib import ExitStack
        from gpu_pair_path_scope import certified_path_scope
        with ExitStack()as stack:
            if getattr(lc,'preview_other_boxes',None)is None:
                actual_other=getattr(lc,'other_configuration',None)
                if callable(actual_other):
                    other,other_q,other_opening=actual_other()
                    stack.enter_context(lc.other_arm_at(other,other_q,other_opening))
            stack.enter_context(certified_path_scope(c,lc,stroke,[opening]))
            stroke_clear,stroke_reason=lc.path(stroke,opening)
        c['planning_diagnostics'].append(dict(event='saved_SMALL_nominal_robot_stroke_batched',segments=len(stroke)-1,wall_s=time.perf_counter()-started_stroke,clear=bool(stroke_clear),reason=stroke_reason,original_fixed_self_pair_predicates=True,physical_steps=0))
        if not stroke_clear:return None
    finally:lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative=saved
    try:verify(c,part,G,issued)
    except RuntimeError as error:
        if not str(error).startswith('Loaded LEFT issued curve'):raise
        c['planning_diagnostics'].append(dict(event='saved_SMALL_LEFT_carry_rejected',stage='current_exact_issued_guard',reason=str(error)));return None
    evidence=dict(event='saved_SMALL_LEFT_carry_admitted',canonical_rows=len(q)-1,fresh_entry_rows=len(bridge),original_dt_s=META['dt_s'],fresh_RGBD_grasp=True,full_nominal_robot_stroke=True,original_full_payload_pair_guard=True,saved_command_limit_audit=audit,uses_free_body_poses=False)
    c['planning_diagnostics'].append(evidence)
    return dict(route=SavedCommandRoute(issued,initial,evidence),goal_R=R,azimuth=META['ring_goal_azimuth_rad'],clearance=META['preinsert_clearance_m'])
