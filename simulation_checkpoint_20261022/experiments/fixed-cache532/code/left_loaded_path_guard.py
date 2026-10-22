"""Current-encoder opposite-arm protection for an RGB-D-held LEFT payload."""
from contextlib import contextmanager
import numpy as np
from isaac_kinematics import frame
from aperture_path_check import path_openings,_samples
from predictive_pair_clearance import pair_clear

@contextmanager
def current_query(c,part,G=None):
    lc=c['checkers']['left'];rc=c['checkers']['right'];art=c['art']
    other_q=art.get_dof_positions()[0,c['ids']].copy();other_width=float(art.get_dof_positions()[0,c['fi']].sum())
    binding=getattr(lc,'loaded_contact_aperture',None)
    if binding is not None:binding.check(art.get_dof_positions()[0,c['left_fi']])
    saved=(lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative)
    lc.detach_payload()
    if G is not None:lc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
    try:
        with lc.other_arm_at(rc,other_q,other_width):yield
    finally:lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative=saved

def verify_issued(c,part,G,rows,allow_fixture_contact=False):
    if not c['config'].get('stability_serial'):
        raise RuntimeError('Loaded LEFT execution guard requires the configured serial policy; simultaneous arms need a paired time schedule')
    lc=c['checkers']['left'];art=c['art'];actual=art.get_dof_positions()[0,c['left_ids']].copy();start=c['cmd'][0,c['left_ids']].copy()
    issued=np.asarray(rows,dtype=np.float32).astype(float);qs=np.vstack([actual,start,issued[:,:7]])
    actual_width=float(art.get_dof_positions()[0,c['left_fi']].sum());command_width=float(c['cmd'][0,c['left_fi']].sum());widths=issued[:,7:].sum(1)
    openings=np.unique(np.linspace(min(actual_width,command_width,float(widths.min())),max(actual_width,command_width,float(widths.max())),5))
    right_command=np.r_[c['cmd'][0,c['ids']],c['cmd'][0,c['fi']]].copy();old=lc.resolution_rad if hasattr(lc,'resolution_rad')else .035;lc.resolution_rad=min(old,.005)
    try:
        with current_query(c,part,G):
            from exact_loaded_left_proof import key as full_proof_key,CACHE
            try:proof_key,cache_reason=full_proof_key(c,part,G,issued,allow_fixture_contact)
            except Exception as error:proof_key,cache_reason=None,'unsupported_exact_scope:'+type(error).__name__
            previous=c.get(CACHE)
            if proof_key is not None and previous is not None and previous['key']==proof_key:
                c['planning_diagnostics'].append(dict(event='loaded_LEFT_same_physics_epoch_full_proof_reuse',hit=True,physics_epoch=c['_collision_physics_epoch'],part=part,issued_samples=len(issued),exact_current_encoders_commands_payload_binding_model_world_and_placed_geometry=True,cache_survives_physics=False))
                return previous['right_command'].copy()
            c[CACHE]=None
            c['planning_diagnostics'].append(dict(event='loaded_LEFT_same_physics_epoch_full_proof_reuse',hit=False,physics_epoch=c.get('_collision_physics_epoch'),part=part,reason=cache_reason if proof_key is None else 'different_exact_scope',cache_survives_physics=False))
            from contextlib import ExitStack
            from gpu_pair_path_scope import certified_path_scope
            from fixed_small_left_executor import static_scope
            with ExitStack()as proof_stack:
                proof_stats=proof_stack.enter_context(static_scope(c,lc))
                proof_stack.enter_context(certified_path_scope(c,lc,qs,openings))
                if allow_fixture_contact:
                    # Intended payload/jig contact is retained physically. Its
                    # exemption never removes the loaded ring from arm-pair tests.
                    for opening in openings:
                        for _,_,q in _samples(lc,qs):
                            self_guard=getattr(lc,'same_arm_clearance',None)
                            if self_guard is not None:
                                clear,why=self_guard.clear(q,float(opening))
                                if not clear:raise RuntimeError('Loaded LEFT issued fixture curve intersects own nonadjacent robot '+str(why))
                            clear,why=pair_clear(lc,q,float(opening))
                            if not clear:raise RuntimeError('Loaded LEFT issued curve intersects current RIGHT '+str(why))
                else:
                    clear,why=path_openings(lc,qs,openings)
                    if not clear:raise RuntimeError('Loaded LEFT issued curve blocked with payload/current RIGHT '+str(why))
            if proof_key is not None:c[CACHE]=dict(key=proof_key,right_command=right_command.copy())
    finally:lc.resolution_rad=old
    if proof_stats:c['planning_diagnostics'].append(dict(event='saved_LEFT_static_route_query_reuse',**proof_stats))
    c['planning_diagnostics'].append(dict(event='left_loaded_checked_issued_curve',part=part,allow_intended_payload_fixture_contact=bool(allow_fixture_contact),actual_other_arm_encoders=True,payload_pair_checked=True,checked_openings_m=openings.tolist(),issued_samples=len(issued),uses_free_body_poses=False))
    return right_command
