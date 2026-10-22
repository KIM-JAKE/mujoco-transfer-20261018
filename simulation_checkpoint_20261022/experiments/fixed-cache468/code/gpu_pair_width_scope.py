"""One GPU whole-pair proof per q for a conservative native width sweep.

Original full fixed/self/payload/placed and sampled aperture queries still
run. Only a whole-pair CLEAR of an OUTER swept finger enclosure may replace
its pair component. Unknown poses or service errors retain original CPU.
"""
from contextlib import contextmanager
import time,types
import numpy as np
from aperture_path_check import _samples
from predictive_pair_clearance import proxy
from gpu_pair_schedule_scope import _qkey,_geometry_version,certified_pair_scope
SIGNS=np.array([[x,y,z]for x in[-1.,1.]for y in[-1.,1.]for z in[-1.,1.]])


def swept(checker,reference_q,lo,hi):
    arm=checker.arm;low=arm.bodies(reference_q,float(lo));high=arm.bodies(reference_q,float(hi));middle=arm.bodies(reference_q,float((lo+hi)/2));rows=[]
    for body,source,box in checker.pair_boxes:
        if np.array_equal(low[body],high[body]):rows.append((body,source,box));continue
        if 'finger'not in body:raise ValueError('Unsupported width-dependent nonfinger geometry')
        a=np.linalg.inv(low[arm.hand_path])@low[body];b=np.linalg.inv(high[arm.hand_path])@high[body];m=np.linalg.inv(middle[arm.hand_path])@middle[body]
        if np.max(abs(a[:3,:3]-b[:3,:3]))>1e-12 or np.max(abs(m-(a+b)/2))>1e-12:raise ValueError('Native prismatic finger model changed')
        # Enclose the WHOLE original .5mm-inflated native OBB at every
        # permitted linear finger translation, including an outward reserve.
        corners=np.asarray(box[0])+SIGNS*(np.asarray(box[2])+.0005)@np.asarray(box[1]).T
        points=np.concatenate([corners@T[:3,:3].T+T[:3,3]for T in[a,b]])
        points=np.concatenate([points+offset for offset in SIGNS*1e-8])
        rows.append((arm.hand_path,source,proxy(points)))
    return types.SimpleNamespace(arm=arm,pair_boxes=rows,payload_path=checker.payload_path,payload_boxes=checker.payload_boxes,payload_relative=checker.payload_relative)


@contextmanager
def certified_width_path_scope(c,checker,qs,openings):
    start=time.perf_counter();openings=np.asarray(openings,float);samples=list(_samples(checker,np.asarray(qs)))
    left,right=c['checkers']['left'],c['checkers']['right'];other,other_q,other_width=checker.preview_other_configuration
    lo,hi=float(openings.min()),float(openings.max());stats=dict(event='GPU_native_width_sweep_pair_certificate',q_samples=len(samples),original_aperture_samples=len(samples)*len(openings),width_interval_m=[lo,hi],pair_only=True,own_fixed_self_placed_and_sampled_aperture_guards_preserved=True,exact_scope_hits=0,current_context_misses=0)
    installed=False;sentinel=object();previous=sentinel;approved=set();fallback=False
    try:
        try:
            if not np.isfinite(openings).all()or lo<0 or hi>.080001 or not samples:raise ValueError('Invalid native width interval')
            shadow=swept(checker,samples[0][2],lo,hi);mid=(lo+hi)/2
            pairs=[(q,mid,other_q,other_width)if checker is left else(other_q,other_width,q,mid)for _,_,q in samples]
            ls,rs=(shadow,right)if checker is left else(left,shadow)
            versions=(_geometry_version(left),_geometry_version(right));diag={}
            from gpu_pair_schedule import prove
            mask=np.asarray(prove(ls,rs,pairs,dict(socket=c['config'].get('gpu_pair_schedule_socket','/workspace/project/mujoco/.local/collision-gpu2/pair.sock'),timeout_s=float(c['config'].get('gpu_pair_schedule_timeout_s',10.)),diagnostics=diag)))
            if mask.dtype!=np.bool_ or mask.shape!=(len(samples),):raise RuntimeError('Malformed native width-sweep proof')
            geometry_sha=diag.get('geometry_sha256')
            if not isinstance(geometry_sha,str)or len(geometry_sha)!=64 or any(ch not in '0123456789abcdef'for ch in geometry_sha):raise RuntimeError('Width sweep proof lacks canonical registered geometry identity')
            if versions!=(_geometry_version(left),_geometry_version(right)):raise RuntimeError('Pair CAD changed during width sweep proof')
            approved={_qkey(q)for (_,_,q),clear in zip(samples,mask)if clear};oqkey=_qkey(other_q);owkey=np.float64(other_width).tobytes()
            stats.update(GPU_clear_q_samples=int(mask.sum()),uncertain_q_samples=int((~mask).sum()),service=diag)
            def accepts(current,q,width):
                try:
                    context=getattr(current,'preview_other_configuration',None)
                    if current is checker and context is not None and context[0]is other and _qkey(context[1])==oqkey and np.float64(context[2]).tobytes()==owkey and lo<=float(width)<=hi and versions==(_geometry_version(left),_geometry_version(right))and _qkey(q)in approved:
                        stats['exact_scope_hits']+=1;return True
                except Exception:pass
                stats['current_context_misses']+=1;return False
            if approved:
                previous=getattr(checker,'scoped_gpu_pair_clear_certificate',sentinel);checker.scoped_gpu_pair_clear_certificate=accepts;installed=True
        except Exception as error:
            stats['fallback_reason']=type(error).__name__+': '+str(error);fallback=True
        if fallback:
            original_pairs=[(q,float(w),other_q,other_width)if checker is left else(other_q,other_width,q,float(w))for w in openings for _,_,q in samples]
            with certified_pair_scope(c,left,right,pairs=original_pairs):yield
        else:yield
    finally:
        if installed:
            if previous is sentinel:delattr(checker,'scoped_gpu_pair_clear_certificate')
            else:checker.scoped_gpu_pair_clear_certificate=previous
        approved.clear();stats['wall_s']=time.perf_counter()-start;c['planning_diagnostics'].append(stats)
