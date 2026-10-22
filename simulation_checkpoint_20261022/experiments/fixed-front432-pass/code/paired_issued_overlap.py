"""Bounded, exact-row two-arm overlap; no task/body-state shortcuts.

One already retimed LEFT chunk may advance with one RIGHT pickup motion.
The chunk cannot cross its boundary during that motion. Gripper closure and
all subsequent uncertain-contact phases freeze LEFT until pickup finishes.
"""
from contextlib import contextmanager
import time
import numpy as np
from isaac_kinematics import frame
from predictive_pair_clearance import pair_clear

class PairedPickupReplanRequired(RuntimeError):
    """The exposed LEFT chunk moved safely; no RIGHT row was issued."""


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


class PairedIssuedOverlap:
    def __init__(self,c):
        self.c=c;self.chunk=None;self.active=False;self.token=None;self.contact_started=False
        self.prime=None;self.drain=None;self.lead_steps=None;self.draining=False;self.awaiting_prime=False;self.events=[]

    def begin_pick(self):
        self.active=True;self.contact_started=False;self.token=None

    def end_pick(self):
        self.active=False;self.token=None

    def expose_left(self,rows,part,G,fixture=False):
        if self.chunk is not None:raise RuntimeError('Overlapping LEFT chunks are forbidden')
        rows=np.asarray(rows,dtype=np.float32).astype(float)
        self.awaiting_prime=False
        self.chunk=dict(rows=rows.copy(),cursor=0,part=part,G=None if G is None else np.asarray(G,float).copy(),fixture=bool(fixture))

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
        if self.draining and self.chunk is None:return True
        if self.draining and self.chunk is not None and self.chunk['cursor']==len(self.chunk['rows']):return True
        token=self.token
        return bool(token is not None and token['advance'][token['index']])

    def step_completed(self):
        if self.token is not None:
            self.token['index']+=1
            if self.draining and self.token['index']==len(self.token['right']):self.token=None

    def before_stationary_left(self):
        if self.chunk is None or self.token is not None:return
        remaining=self.chunk['rows'][self.chunk['cursor']:]
        if not len(remaining):return
        right=np.tile(np.r_[self.c['cmd'][0,self.c['ids']],self.c['cmd'][0,self.c['fi']]],(len(remaining),1)).astype(np.float32).astype(float)
        self._verify(remaining,right,self.chunk)
        self.token=dict(left=remaining.copy(),right=right,advance=np.ones(len(remaining),bool),index=0)

    def before_contact(self):
        """Run before fresh preclose sensing or enabling grip-hold effort."""
        if self.active and not self.contact_started:
            self.contact_started=True
            self.draining=True
            try:
                if self.drain is not None:self.drain()
            finally:self.draining=False;self.token=None

    def before_right(self,rows,label):
        if not self.active:return
        if any(word in label for word in ['close','release']):
            self.before_contact()
            # Draining can install the LEFT part and change the opposite
            # encoder context after the original RIGHT plan. Recheck this
            # very closure/aperture schedule in that new current world.
            right=np.asarray(rows,dtype=np.float32).astype(float)
            left=np.tile(np.r_[self.c['cmd'][0,self.c['left_ids']],self.c['cmd'][0,self.c['left_fi']]],(len(right),1)).astype(np.float32).astype(float)
            self._verify(left,right,dict(part=None,G=None,fixture=False))
        if self.contact_started:return
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
        rejected=[];chosen=None
        for lead in candidates:
            combined_right=np.vstack([np.tile(current_right,(lead,1)),right])
            combined_left=remaining[np.minimum(np.arange(lead+n),len(remaining)-1)].copy()
            advance=np.arange(lead+n)<len(remaining)
            try:self._verify(combined_left,combined_right,self.chunk)
            except RuntimeError as error:
                if not str(error).startswith(('Paired LEFT issued curve blocked before motion','Paired RIGHT issued curve blocked before motion')):raise
                rejected.append(dict(LEFT_lead_steps=lead,LEFT_lead_physics_s=lead*self.c['dt'],collision=str(error)))
                continue
            chosen=(lead,combined_left,combined_right,advance);break
        if chosen is None:
            # No RIGHT command has changed. Safely finish only this already
            # exposed LEFT chunk, then let the original RIGHT planner choose
            # a new route against that new actual opposite-arm context.
            stationary=np.tile(current_right,(len(remaining),1))
            self._verify(remaining,stationary,self.chunk)
            self.token=dict(left=remaining.copy(),right=stationary,advance=np.ones(len(remaining),bool),index=0)
            if self.lead_steps is None:raise RuntimeError('No macro step callback for checked LEFT lead')
            self.lead_steps(len(remaining));self.after_right()
            self.c['planning_diagnostics'].append(dict(event='paired_temporal_candidates_require_fresh_RIGHT_route',RIGHT_label=label,rejected=rejected,checked_LEFT_chunk_executed_rows=len(remaining),RIGHT_motion_rows_issued=0,uses_free_body_poses=False))
            raise PairedPickupReplanRequired('All checked LEFT lead phases blocked this RIGHT route; current LEFT chunk safely finished before replanning')
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
        self.token=None

    def _verify(self,left,right,chunk):
        c=self.c;lc=c['checkers']['left'];rc=c['checkers']['right'];measured=c['art'].get_dof_positions()[0]
        actual_l=np.r_[measured[c['left_ids']],measured[c['left_fi']]]
        actual_r=np.r_[measured[c['ids']],measured[c['fi']]]
        command_l=np.r_[c['cmd'][0,c['left_ids']],c['cmd'][0,c['left_fi']]]
        command_r=np.r_[c['cmd'][0,c['ids']],c['cmd'][0,c['fi']]]
        # These sets exist only inside this immutable preflight. No physics,
        # new observation, payload change or installed-world update occurs
        # until it returns. An exact repeated own pose retains its completed
        # original fixed/self/payload proof; EVERY new opposite pose still
        # receives the original pair narrowphase including carried CAD.
        proved_left=set();proved_right=set()
        stats=dict(LEFT_full_own_proofs=0,RIGHT_full_own_proofs=0,LEFT_exact_own_proofs_reused=0,RIGHT_exact_own_proofs_reused=0,scope='one immutable paired preflight',rounded_keys=False,cross_call_cache=False)
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
        with _payload(lc,c,chunk['part'],chunk['G']):
            for l in [actual_l,command_l]:
                for r in [actual_r,command_r]:clear(l,r)
            previous_l=actual_l;previous_r=actual_r
            for l,r in zip(np.vstack([command_l,left]),np.vstack([command_r,right])):
                count=max(1,int(np.ceil(max(np.max(abs(l[:7]-previous_l[:7])),np.max(abs(r[:7]-previous_r[:7])))/.005)),int(np.ceil(max(abs(l[7:].sum()-previous_l[7:].sum()),abs(r[7:].sum()-previous_r[7:].sum()))/.001)))
                for u in np.arange(1,count+1)/count:clear(previous_l*(1-u)+l*u,previous_r*(1-u)+r*u)
                previous_l=l;previous_r=r
        self.c['planning_diagnostics'].append(dict(event='paired_exact_own_proof_scope_reuse',**stats))
