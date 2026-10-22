"""Optional instance-only inclusive guard timers; no collision-policy changes.

Do not wrap SameArmClearance.clear or Arm.bodies: their active implementations
are part of persistent proof identity. Install after preparation; detach in the
caller's finally block, including when the task raises.
"""
import time
from functools import wraps

_MISSING=object()


def _identity(method):
    return getattr(method,'__self__',None),getattr(method,'__func__',method)


class IssuedGuardTimer:
    def __init__(self,checkers):
        self._bindings=[];self._bound=set();self._metrics={};self._detached=False
        self._checkers=list(checkers.items()) if hasattr(checkers,'items') else list(enumerate(checkers))
        self._protected=[];self._last_placed={};self._restoration_errors=[]
        for label,checker in self._checkers:
            guard=getattr(checker,'same_arm_clearance',None)
            arm=getattr(checker,'arm',None)
            for obj,name in [(guard,'clear'),(arm,'bodies')]:
                if obj is not None and callable(getattr(obj,name,None)):
                    self._protected.append((obj,name,_identity(getattr(obj,name))))
        try:
            for label,checker in self._checkers:
                label=str(label)
                self._wrap(checker,'clear',label+'.checker.clear',checker)
                self._wrap(checker,'_clear_transforms',label+'.checker._clear_transforms',checker)
                own=getattr(getattr(checker,'same_arm_clearance',None),'own_payload_clearance',None)
                if own is not None:self._wrap(own,'clear',label+'.own_payload_clearance.clear')
                self._discover_placed(checker)
            if not self._proof_identity_unchanged():raise RuntimeError('Profiler changed a protected proof method identity')
        except BaseException:
            self.detach_collect()
            raise

    def _proof_identity_unchanged(self):
        return all(all(current is saved for current,saved in zip(_identity(getattr(obj,name)),before))
                   for obj,name,before in self._protected)

    def _discover_placed(self,checker):
        placed=getattr(checker,'placed_part_clearance',None)
        if self._last_placed.get(id(checker),_MISSING) is placed:return
        self._last_placed[id(checker)]=placed
        if placed is not None:self._wrap(placed,'clear','placed_part_clearance.clear')

    def _wrap(self,obj,name,label,checker=None):
        key=(id(obj),name)
        if key in self._bound:return
        original=getattr(obj,name)
        if not callable(original):raise TypeError('Guard timer requires a callable '+label)
        old_attribute=vars(obj).get(name,_MISSING)
        metric=self._metrics.setdefault(label,dict(calls=0,exceptions=0,max_recursion_depth=0,
            inclusive_wall_s=0.,inclusive_thread_cpu_s=0.,inclusive_process_cpu_s=0.,
            outermost_calls=0,outermost_exceptions=0,outermost_wall_s=0.,
            outermost_thread_cpu_s=0.,outermost_process_cpu_s=0.))
        depth=0
        @wraps(original)
        def timed(*args,**kwargs):
            nonlocal depth
            # Discovery is bookkeeping only. It observes guard INSTANCE identity,
            # not geometry, poses, sensor values or a collision decision.
            if checker is not None:self._discover_placed(checker)
            outermost=depth==0;depth+=1
            metric['max_recursion_depth']=max(metric['max_recursion_depth'],depth)
            wall=time.perf_counter();thread=time.thread_time();process=time.process_time()
            try:
                return original(*args,**kwargs)
            except BaseException:
                metric['exceptions']+=1
                if outermost:metric['outermost_exceptions']+=1
                raise
            finally:
                elapsed_wall=time.perf_counter()-wall
                elapsed_thread=time.thread_time()-thread
                elapsed_process=time.process_time()-process
                metric['inclusive_wall_s']+=elapsed_wall
                metric['inclusive_thread_cpu_s']+=elapsed_thread
                metric['inclusive_process_cpu_s']+=elapsed_process
                metric['calls']+=1
                if outermost:
                    metric['outermost_calls']+=1
                    metric['outermost_wall_s']+=elapsed_wall
                    metric['outermost_thread_cpu_s']+=elapsed_thread
                    metric['outermost_process_cpu_s']+=elapsed_process
                depth-=1
        setattr(obj,name,timed)
        self._bound.add(key);self._bindings.append((obj,name,old_attribute,timed))

    def detach_collect(self):
        """Restore exact instance attributes/descriptors; do not mask task errors."""
        if not self._detached:
            for obj,name,old,timed in reversed(self._bindings):
                try:
                    if old is _MISSING:delattr(obj,name)
                    else:setattr(obj,name,old)
                except Exception as error:
                    self._restoration_errors.append(type(error).__name__+': '+str(error))
            self._detached=True
        return dict(scope='Inclusive instance guard timers only; no actuation, collision or sensor changes',
                    timing_basis='Wall, calling-thread CPU, and whole-process CPU. Nested methods overlap; DO NOT sum them. Outermost counters remove same-instance-method recursion only. Process CPU can include concurrent simulator threads.',
                    methods={key:dict(value)for key,value in self._metrics.items()},
                    wrapped_instances=len(self._bindings),detached=self._detached,
                    restoration_error_count=len(self._restoration_errors),restoration_errors=list(self._restoration_errors),
                    protected_proof_method_identity_unchanged=self._proof_identity_unchanged(),
                    same_arm_clearance_clear_wrapped=False,arm_bodies_wrapped=False)


def install(checkers):
    return IssuedGuardTimer(checkers)
