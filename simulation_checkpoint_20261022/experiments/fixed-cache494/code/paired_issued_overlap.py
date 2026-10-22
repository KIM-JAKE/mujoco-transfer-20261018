"""Bounded, exact-row two-arm overlap; no task/body-state shortcuts.

One already retimed LEFT chunk may advance with one RIGHT pickup motion.
The chunk cannot cross its boundary during that motion. A default-off second
phase freezes LEFT through RIGHT closure; only fresh opposed-pad RGB-D plus
stationary encoder binding permits a new two-payload lift/idle schedule.
"""
from contextlib import contextmanager
import time
import numpy as np
from isaac_kinematics import frame
from predictive_pair_clearance import pair_clear


@contextmanager
def _payload(checker,c,part,G):
    saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
    try:
        checker.detach_payload()
        if G is not None:
            checker.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
        yield
    finally:
        checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved


class PairedPickupReplanRequired(RuntimeError):
    """The exposed LEFT chunk moved safely; no RIGHT row was issued."""


class PairedIssuedOverlap:
    def __init__(self,c):
        self.c=c;self.chunk=None;self.active=False;self.token=None;self.contact_started=False
        self.prime=None;self.drain=None;self.lead_steps=None;self.draining=False;self.awaiting_prime=False;self.events=[]
        self.resume_loaded=bool(c['config'].get('fixed_supply_resume_after_right_grasp',False))
        self.right_binding=None;self.right_info=None;self.left_info=None;self.idle_stationary=False;self.supports_loaded_resume=False;self.right_field_active=False
        self.saved_right_geometry=None

    def begin_pick(self):
        if self.right_binding is not None:raise RuntimeError('Previous paired RIGHT binding is still active')
        self.active=True;self.contact_started=False;self.token=None;self.idle_stationary=False;self.supports_loaded_resume=False;self.right_field_active=False

    def end_pick(self):
        if self.resume_loaded and self.right_binding is not None:
            # The caller has completed all original lift/retention gates.
            # Finish LEFT in the current loaded RIGHT context before handing
            # control back to the original serial insertion/handover skills.
            self.token=None;self.idle_stationary=False;self.draining=True
            try:
                if self.drain is not None:self.drain()
            finally:self.draining=False;self.token=None
        self.release_right_binding()
        self.active=False;self.token=None;self.idle_stationary=False;self.right_field_active=False

    def abort_pick(self):
        # Error cleanup is deliberately motion-free.
        self.release_right_binding()
        self.active=False;self.token=None;self.idle_stationary=False;self.right_field_active=False

    def release_right_binding(self):
        if self.right_binding is None:return
        rc=self.c['checkers']['right'];binding=self.right_binding
        if getattr(rc,'loaded_contact_aperture',None) is not binding:
            raise RuntimeError('Paired RIGHT loaded-contact ownership changed unexpectedly')
        binding.release()
        saved=self.saved_right_geometry
        rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative,rc.loaded_contact_aperture=saved
        self.right_binding=None;self.right_info=None;self.saved_right_geometry=None

    def check_right_binding(self):
        if self.right_binding is not None:
            self.right_binding.check(self.c['art'].get_dof_positions()[0,self.c['fi']])

    def bind_right_payload(self,part,G,contact,samples,times,capture_age_s):
        if not self.resume_loaded or not self.supports_loaded_resume:return
        if not self.active or not self.contact_started:raise RuntimeError('RIGHT paired binding requires a frozen closure phase')
        if self.right_binding is not None:raise RuntimeError('RIGHT paired binding already exists')
        from loaded_contact_aperture import LoadedContactAperture
        rc=self.c['checkers']['right'];old=getattr(rc,'loaded_contact_aperture',None)
        if old is not None and getattr(old,'active',False):raise RuntimeError('Another skill owns the RIGHT loaded-contact binding')
        binding=LoadedContactAperture(side='right',part=part,payload_path=self.c['part_collision_templates'][part],encoder_samples=samples,sample_times_s=times,contact=contact,measured_grasp=G,control_dt_s=self.c['dt'],capture_age_physics_s=capture_age_s)
        self.saved_right_geometry=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative,old)
        self.right_binding=binding;self.right_info=dict(part=part,G=np.asarray(G,float).copy())
        rc.loaded_contact_aperture=binding
        rc.attach_payload(self.c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
        self.c['planning_diagnostics'].append(dict(event='paired_RIGHT_loaded_contact_bound_before_lift',**binding.record))

    def refresh_right_payload(self,G,contact):
        if self.right_binding is None:return
        if self.right_field_active:raise RuntimeError('RIGHT payload may refresh only between issued motion fields')
        if not contact.get('opposed_contact_compatible') or not contact.get('both_hulls_near_CAD'):
            raise RuntimeError('Paired RIGHT postlift refresh lacks fresh bilateral CAD contact')
        G=np.asarray(G,float);old=self.right_info['G'];translation=float(np.linalg.norm(G[:3,3]-old[:3,3]));angle=float(np.arccos(np.clip(G[:3,2]@old[:3,2],-1,1)))
        if translation>.002 or angle>.05:raise RuntimeError('Paired RIGHT postlift grasp changed outside2mm/.05rad retention bounds')
        self.check_right_binding();self.right_info['G']=G.copy()
        # A changed observed transform invalidates any unfinished stationary
        # proof immediately; the next LEFT step receives a fresh full scope.
        self.token=None;self.idle_stationary=False
        self.c['checkers']['right'].attach_payload(self.c['part_collision_templates'][self.right_info['part']],frame([0,0,.1034],np.eye(3))@G)
        self.c['planning_diagnostics'].append(dict(event='paired_RIGHT_postlift_fresh_payload_transform',part=self.right_info['part'],measured_grasp=G.tolist(),translation_change_m=translation,axis_change_rad=angle,contact=contact,uses_free_body_poses=False))

    def expose_left(self,rows,part,G,fixture=False):
        if self.chunk is not None:raise RuntimeError('Overlapping LEFT chunks are forbidden')
        rows=np.asarray(rows,dtype=np.float32).astype(float)
        self.awaiting_prime=False
        self.chunk=dict(rows=rows.copy(),cursor=0,part=part,G=None if G is None else np.asarray(G,float).copy(),fixture=bool(fixture))
        self.left_info={key:self.chunk[key]for key in ['part','G','fixture']}

    def finish_left(self):
        self.chunk=None

    def left_row_issued(self,row):
        if self.chunk is None:raise RuntimeError('Unexposed LEFT issued row')
        index=self.chunk['cursor']
        if not np.array_equal(np.asarray(row,dtype=np.float32).astype(float),self.chunk['rows'][index]):
            raise RuntimeError('LEFT row differs from immutable exposed chunk')
        self.chunk['cursor']+=1

    def authorize_left(self,row):
        token=self.token
        if token is None or token['index']>=len(token['right']):return False
        i=token['index'];expected=token['left'][i]
        right=np.r_[self.c['cmd'][0,self.c['ids']],self.c['cmd'][0,self.c['fi']]]
        return bool(np.array_equal(np.asarray(row,dtype=np.float32).astype(float),expected) and np.array_equal(np.asarray(right,dtype=np.float32).astype(float),token['right'][i]))

    def advance_left_this_step(self):
        if not self.active:return True
        if (self.draining or self.idle_stationary) and self.chunk is None:return True
        if (self.draining or self.idle_stationary) and self.chunk is not None and self.chunk['cursor']==len(self.chunk['rows']):return True
        token=self.token
        return bool(token is not None and token['advance'][token['index']])

    def step_completed(self):
        if self.token is not None:
            self.token['index']+=1
            if (self.draining or self.idle_stationary) and self.token['index']==len(self.token['right']):self.token=None

    def prepare_idle_left_step(self):
        if not self.active or self.right_binding is None or self.right_field_active:return
        if self.token is not None and not self.idle_stationary:return
        self.check_right_binding();self.idle_stationary=True
        self.before_stationary_left()

    def before_stationary_left(self):
        if self.chunk is None or self.token is not None:return
        remaining=self.chunk['rows'][self.chunk['cursor']:]
        if not len(remaining):return
        right=np.tile(np.r_[self.c['cmd'][0,self.c['ids']],self.c['cmd'][0,self.c['fi']]],(len(remaining),1)).astype(np.float32).astype(float)
        self._verify(remaining,right,self.chunk)
        self.token=dict(left=remaining.copy(),right=right,advance=np.ones(len(remaining),bool),index=0)

    def before_contact(self,*,grasp_type=None):
        """Run before fresh preclose sensing or enabling grip-hold effort."""
        if self.active and not self.contact_started:
            self.contact_started=True;self.token=None;self.idle_stationary=False
            self.supports_loaded_resume=bool(self.resume_loaded and grasp_type=='outer' and self.chunk is not None)
            if self.supports_loaded_resume:return
            self.draining=True
            try:
                if self.drain is not None:self.drain()
            finally:self.draining=False;self.token=None

    def before_right(self,rows,label):
        if not self.active:return
        self.right_field_active=True
        if self.idle_stationary:self.token=None;self.idle_stationary=False
        if any(word in label for word in ['close','release']):
            self.before_contact()
            if self.right_binding is not None:self.release_right_binding()
            # Draining can install the LEFT part and change the opposite
            # encoder context after the original RIGHT plan. Recheck this
            # very closure/aperture schedule in that new current world.
            right=np.asarray(rows,dtype=np.float32).astype(float)
            left=np.tile(np.r_[self.c['cmd'][0,self.c['left_ids']],self.c['cmd'][0,self.c['left_fi']]],(len(right),1)).astype(np.float32).astype(float)
            self._verify(left,right,self.left_info or dict(part=None,G=None,fixture=False))
        if self.contact_started and self.right_binding is None:return
        if self.chunk is None and self.awaiting_prime and self.prime is not None:self.prime()
        if self.chunk is None:return
        right=np.asarray(rows,dtype=np.float32).astype(float)
        remaining=self.chunk['rows'][self.chunk['cursor']:]
        if not len(remaining):
            # The generator has not crossed this chunk boundary: LEFT still
            # holds its observed payload at the last issued row. A later
            # RIGHT field needs a NEW pair proof even though LEFT is parked.
            left=np.tile(np.r_[self.c['cmd'][0,self.c['left_ids']],self.c['cmd'][0,self.c['left_fi']]],(len(right),1)).astype(np.float32).astype(float)
            self._verify(left,right,self.chunk)
            self.token=dict(left=left,right=right.copy(),advance=np.zeros(len(right),bool),index=0)
            return
        n=len(right);started=time.perf_counter();current_right=np.r_[self.c['cmd'][0,self.c['ids']],self.c['cmd'][0,self.c['fi']]].astype(np.float32).astype(float)
        candidates=list(dict.fromkeys([0,min(len(remaining),int(np.ceil(.25/self.c['dt']))),min(len(remaining),int(np.ceil(.50/self.c['dt']))),len(remaining)]))
        # All candidates below share one immutable scene, payload binding and
        # encoder snapshot. The local successful OWN proofs can be reused;
        # opposite CAD pair tests still run at every candidate's exact rows.
        candidate_own_proofs=dict(LEFT=set(),RIGHT=set())
        rejected=[];chosen=None
        for lead in candidates:
            combined_right=np.vstack([np.tile(current_right,(lead,1)),right])
            combined_left=remaining[np.minimum(np.arange(lead+n),len(remaining)-1)].copy()
            advance=np.arange(lead+n)<len(remaining)
            try:self._verify(combined_left,combined_right,self.chunk,own_proof_scope=candidate_own_proofs)
            except BaseException as error:
                if not isinstance(error,RuntimeError) or not str(error).startswith(('Paired LEFT issued curve blocked before motion','Paired RIGHT issued curve blocked before motion')):
                    candidate_own_proofs['LEFT'].clear();candidate_own_proofs['RIGHT'].clear()
                    raise
                rejected.append(dict(LEFT_lead_steps=lead,LEFT_lead_physics_s=lead*self.c['dt'],collision=str(error)))
                continue
            chosen=(lead,combined_left,combined_right,advance);break
        if chosen is None:
            # No RIGHT command has changed. Safely finish only this already
            # exposed LEFT chunk, then let the original RIGHT planner choose
            # a new route against that new actual opposite-arm context.
            stationary=np.tile(current_right,(len(remaining),1))
            try:self._verify(remaining,stationary,self.chunk,own_proof_scope=candidate_own_proofs)
            finally:
                # No reusable own proof may survive a physical callback or
                # an error unwind into caller-owned binding cleanup.
                candidate_own_proofs['LEFT'].clear();candidate_own_proofs['RIGHT'].clear()
            self.token=dict(left=remaining.copy(),right=stationary,advance=np.ones(len(remaining),bool),index=0)
            if self.lead_steps is None:raise RuntimeError('No macro step callback for checked LEFT lead')
            self.lead_steps(len(remaining));self.after_right()
            self.c['planning_diagnostics'].append(dict(event='paired_temporal_candidates_require_fresh_RIGHT_route',RIGHT_label=label,rejected=rejected,checked_LEFT_chunk_executed_rows=len(remaining),RIGHT_motion_rows_issued=0,uses_free_body_poses=False))
            raise PairedPickupReplanRequired('All checked LEFT lead phases blocked this RIGHT route; current LEFT chunk safely finished before replanning')
        # Even a zero-lead choice next issues physical RIGHT rows. Clear the
        # scope now, before any actual step, binding change or callback.
        candidate_own_proofs['LEFT'].clear();candidate_own_proofs['RIGHT'].clear()
        lead,left,combined_right,advance=chosen
        self.token=dict(left=left,right=combined_right,advance=advance,index=0)
        if lead:
            if self.lead_steps is None:raise RuntimeError('No macro step callback for checked LEFT lead')
            self.lead_steps(lead)
        event=dict(event='paired_exact_issued_overlap_preflight',right_label=label,RIGHT_rows=n,LEFT_rows_advanced=int(advance.sum()),LEFT_chunk_rows_remaining=len(remaining),LEFT_part=self.chunk['part'],LEFT_payload_RGBD=self.chunk['G'] is not None,LEFT_lead_steps=lead,LEFT_lead_physics_s=lead*self.c['dt'],temporal_candidates_rejected=rejected,drains_current_LEFT_chunk_before_RIGHT=lead==len(remaining),checks='Both original fixed/self/full payload and exact simultaneous opposite CAD at every issued row plus <=.005rad/1mm subdivisions and actual/command starts',crosses_LEFT_chunk_boundary=False,gripper_contact_phases_frozen=True,planning_s=time.perf_counter()-started,uses_free_body_poses=False)
        self.events.append(event);self.c['planning_diagnostics'].append(event)
        print('PAIRED_ISSUED_OVERLAP',event,flush=True)

    def after_right(self):
        if self.token is not None and self.token['index']!=len(self.token['right']):
            raise RuntimeError('Paired issued chunk did not execute exactly its checked row count')
        self.token=None;self.right_field_active=False

    def _verify(self,left,right,chunk,*,own_proof_scope=None):
        c=self.c;lc=c['checkers']['left'];rc=c['checkers']['right'];measured=c['art'].get_dof_positions()[0]
        actual_l=np.r_[measured[c['left_ids']],measured[c['left_fi']]]
        actual_r=np.r_[measured[c['ids']],measured[c['fi']]]
        command_l=np.r_[c['cmd'][0,c['left_ids']],c['cmd'][0,c['left_fi']]]
        command_r=np.r_[c['cmd'][0,c['ids']],c['cmd'][0,c['fi']]]
        # These sets exist only inside this immutable preflight or its enclosing
        # temporal-candidate search. No physics,
        # new observation, payload change or installed-world update occurs
        # until that scope is cleared. An exact repeated own pose retains its completed
        # original fixed/self/payload proof; EVERY new opposite pose still
        # receives the original pair narrowphase including carried CAD.
        if own_proof_scope is None:proved_left=set();proved_right=set()
        else:proved_left=own_proof_scope['LEFT'];proved_right=own_proof_scope['RIGHT']
        stats=dict(LEFT_full_own_proofs=0,RIGHT_full_own_proofs=0,LEFT_exact_own_proofs_reused=0,RIGHT_exact_own_proofs_reused=0,scope='one immutable temporal-candidate search' if own_proof_scope is not None else 'one immutable paired preflight',rounded_keys=False,cross_temporal_candidate_reuse=own_proof_scope is not None,cross_physics_or_search_cache=False)
        def own_key(q,width):return(np.asarray(q).tobytes(),float(width))
        def clear(l,r):
            lw=float(l[7:].sum());rw=float(r[7:].sum())
            left_key=own_key(l[:7],lw);right_key=own_key(r[:7],rw)
            with lc.other_arm_at(rc,r[:7],rw):
                if left_key in proved_left:
                    stats['LEFT_exact_own_proofs_reused']+=1
                    ok,why=pair_clear(lc,l[:7],lw)
                elif chunk['fixture']:
                    # Original verify_loaded_left_issued already certified
                    # this exact fixed/self issued chunk with its existing
                    # intended payload/jig contact policy. Only the moving
                    # opposite CAD context changes here; this adds no new
                    # fixed-world/contact exclusion.
                    guard=getattr(lc,'same_arm_clearance',None)
                    ok,why=(True,None) if guard is None else guard.clear(l[:7],lw)
                    if ok:ok,why=pair_clear(lc,l[:7],lw)
                else:ok,why=lc.clear(l[:7],lw)
                if not ok:raise RuntimeError('Paired LEFT issued curve blocked before motion '+str(why))
                if left_key not in proved_left:proved_left.add(left_key);stats['LEFT_full_own_proofs']+=1
            with rc.other_arm_at(lc,l[:7],lw):
                if right_key in proved_right:
                    stats['RIGHT_exact_own_proofs_reused']+=1
                    ok,why=pair_clear(rc,r[:7],rw)
                else:
                    ok,why=rc.clear(r[:7],rw)
                    if ok:proved_right.add(right_key);stats['RIGHT_full_own_proofs']+=1
            if not ok:raise RuntimeError('Paired RIGHT issued curve blocked before motion '+str(why))
        # Materialize the exact ORIGINAL sample sequence once. GPU and CPU
        # consume these same values; no sparse path/row replacement is used.
        states=[]
        for l in [actual_l,command_l]:
            for r in [actual_r,command_r]:states.append((l.copy(),r.copy()))
        previous_l=actual_l;previous_r=actual_r
        for l,r in zip(np.vstack([command_l,left]),np.vstack([command_r,right])):
            count=max(1,int(np.ceil(max(np.max(abs(l[:7]-previous_l[:7])),np.max(abs(r[:7]-previous_r[:7])))/.005)),int(np.ceil(max(abs(l[7:].sum()-previous_l[7:].sum()),abs(r[7:].sum()-previous_r[7:].sum()))/.001)))
            for u in np.arange(1,count+1)/count:states.append((previous_l*(1-u)+l*u,previous_r*(1-u)+r*u))
            previous_l=l;previous_r=r
        with _payload(lc,c,chunk['part'],chunk['G']):
            from contextlib import ExitStack
            from gpu_pair_schedule_scope import certified_pair_scope
            from fixed_small_left_executor import static_scope as saved_LEFT_scope
            from fixed_saved_executor import static_scope as saved_RIGHT_scope
            with ExitStack()as proofs:
                # Existing sealed route callbacks authorize ONLY their exact
                # stored q, bounded current width/G and unchanged model/world.
                # Changed rows and all current opposite pairs remain fresh.
                left_saved=proofs.enter_context(saved_LEFT_scope(c,lc))
                right_saved=proofs.enter_context(saved_RIGHT_scope(c,rc))
                proofs.enter_context(certified_pair_scope(c,lc,rc,states))
                for l,r in states:clear(l,r)
            self.c['planning_diagnostics'].append(dict(event='paired_saved_route_component_reuse',LEFT=left_saved,RIGHT=right_saved,scope='one immutable paired preflight',all_current_pairs_checked=True,changed_rows_use_original_full_guards=True))
        self.c['planning_diagnostics'].append(dict(event='paired_exact_own_proof_scope_reuse',**stats))
