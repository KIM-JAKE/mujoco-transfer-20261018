"""Read-only production helper: no physical steps, only original pair queries."""
import numpy as np
from aperture_path_check import _samples
from predictive_pair_clearance import pair_clear
from gpu_pair_width_scope import certified_width_path_scope

def compare(c,side,qs,openings,other_q,other_width):
    checker=c['checkers'][side];other=c['checkers']['right'if side=='left'else'left'];samples=list(_samples(checker,np.asarray(qs)))
    with checker.other_arm_at(other,other_q,float(other_width)):
        baseline=[pair_clear(checker,q,float(w))[0]for w in openings for _,_,q in samples]
        with certified_width_path_scope(c,checker,qs,openings):accelerated=[pair_clear(checker,q,float(w))[0]for w in openings for _,_,q in samples]
    if baseline!=accelerated:raise RuntimeError('Production original-vs-width-envelope pair predicate mismatch')
    return dict(side=side,q_samples=len(samples),original_width_queries=len(baseline),clear=sum(baseline),blocked=len(baseline)-sum(baseline),all_original_verdicts_equal=True,uses_free_body_poses=False,physical_steps=0,diagnostics=c['planning_diagnostics'][-1])


def qualify(checkers,config,paths,indices,fingers,art):
    """Full ORIGINAL native arm/link/Hand/class CAD, no physical commands."""
    import time
    from isaac_kinematics import frame
    from fixed_small_left_plan import META,left_carry
    start=time.perf_counter();c=dict(checkers=checkers,config=config,planning_diagnostics=[])
    lc,rc=checkers['left'],checkers['right'];saved=(lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative)
    actual=art.get_dof_positions()[0];q=left_carry();rows=[]
    try:
        lc.attach_payload(paths['workpiece_ring2'],frame([0,0,.1034],np.eye(3))@np.asarray(META['nominal_TCP_observation_T']))
        widths=np.linspace(.058,.064,5)
        for label,qs,otherq,otherwidth in[
            ('entry_vs_current_other',q[:3],actual[indices['right']],float(actual[fingers['right']].sum())),
            ('end_vs_current_other',q[-3:],actual[indices['right']],float(actual[fingers['right']].sum())),
            ('entry_vs_public_preopen_donor',q[:3],np.array([2.311258554458618,.5544891953468323,-1.3514734506607056,-1.2363888025283813,.17876854538917542,.5912362933158875,.633980929851532]),.0618140734732151)]:
            value=compare(c,'left',qs,widths,np.asarray(otherq,float),otherwidth);value['label']=label;rows.append(value)
        return dict(scope='Full original production native robot links+Hands and held48classCAD, original convex pair predicate versus swept original-inflated-OBB width certificate',all_original_verdicts_equal=True,rows=rows,wall_s=time.perf_counter()-start,physical_steps=0,uses_free_body_poses=False)
    finally:lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative=saved
