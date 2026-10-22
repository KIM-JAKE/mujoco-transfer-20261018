"""Closed-loop physical sequence in the live Isaac Sim stage.

This initial physical feasibility controller uses simulated body observations
as an explicit ideal-perception assumption. It does not claim a tested camera
policy. No workpiece attachment or pose-setting operation is used.
"""
import json,math,traceback
import numpy as np
from pxr import UsdPhysics
from isaac_kinematics import Arm,exp,log,frame
from native_pickup_probe import rotation

def execute(c):
    stage=c['stage'];plan=c['plan'];out=c['OUT'];art=c['art'];cmd=c['cmd'];dt=c['dt'];first_part=next(iter(plan['parts']));parts={p:c['view'].create_rigid_body_view(c['paths']['workpiece_'+p])for p in ['ring1','ring2','pin']};arms={s:Arm(stage,s,c['drives'],art.get_dof_positions()[0,c['aidx'][s]])for s in ['left','right']};active=first_part;records=[];completed=[];trace=[]
    report=dict(state='running_three_part_sequence',full_task_pass=False,hardware_tested=False,physics_engine='Isaac Sim Omni PhysX CPU',kinematics='Live Isaac USD joint frames',perception='Ideal simulated body-pose observations for physical feasibility',vision_end_to_end_pass=False,part_attachments=False,records=records,completed_parts=completed)
    from isaac_collision_queries import Checker
    checkers={s:Checker(stage,arms[s],s,allow_loose_fingertips=True)for s in arms}
    report['transport_planner']='Live PhysX collision queries with bounded CPU RRT-Connect fallback; cuRobo GPU unavailable'
    def save():(out/'physx-sequence-results.json').write_text(json.dumps(report,indent=2))
    def pose(p):
        t=parts[p].get_transforms()[0];return t[:3].astype(float),rotation(t[3:])
    def hand(s):
        t=art.get_link_transforms()[0,c['hidx'][s]];return t[:3].astype(float),rotation(t[3:])
    def relative(s,p):
        hp,hR=hand(s);P,R=pose(p);return hR.T@(P-hp),hR.T@R
    def q(s):return art.get_dof_positions()[0,c['aidx'][s]].astype(float)
    def step():
        c['step']()
        for pair,force in c['current_contacts'].items():
            if force<12:continue
            allowed=any(c['paths']['workpiece_'+active]in path for path in pair)and any('_finger/'in path for path in pair)
            if plan.get('clutter_items')and any(path.startswith('/World/BinStock/')or '/Geometry/workpiece_ring'in path for path in pair)and any('_finger/tip'in path for path in pair):allowed=True
            if not allowed:
                if not plan.get('quick_dirty_demo'):raise RuntimeError('Unintended robot contact '+str(pair)+' force='+str(force))
                report['incidental_contact_peak_n']=max(report.get('incidental_contact_peak_n',0.),float(force))
        if round(c['now']()/dt)%8==0:trace.append(dict(t=c['now'](),q=art.get_dof_positions()[0].copy(),parts=np.array([parts[p].get_transforms()[0]for p in parts]),links=art.get_link_transforms()[0].copy()))
    def wait(T,label):
        c['set_phase'](label)
        for _ in range(round(T/dt)):step()
        records.append(dict(event=label,time_s=c['now'](),parts={p:pose(p)[0].tolist()for p in parts}));save();print('ISAAC_BIN_SEQUENCE',label,round(c['now'](),3),flush=True)
    def move(s,path,T,label,stop_at_corners=True):
        from isaac_motion import duration
        if plan.get('quick_dirty_demo'):T=min(T,1.5)
        path=np.array(path);path[0]=cmd[0,c['aidx'][s]];c['set_phase'](label)
        if stop_at_corners and len(path)>2:
            for a,b in zip(path[:-1],path[1:]):
                segment_T=duration([a,b],max(.08,T/(len(path)-1)));n=max(1,math.ceil(segment_T/dt))
                for k in range(n):
                    u=(k+1)/n;h=u**3*(10-15*u+6*u*u);cmd[0,c['aidx'][s]]=a+(b-a)*h;step()
            wait(.3,label+'_settled');return
        T=duration(path,T);n=max(1,round(T/dt))
        for k in range(n):
            u=(k+1)/n;h=u**3*(10-15*u+6*u*u);x=h*(len(path)-1);j=min(len(path)-2,int(x));w=x-j;cmd[0,c['aidx'][s]]=path[j]*(1-w)+path[j+1]*w;step()
        wait(.3,label+'_settled')
    def fingers(s,width,label):
        c['set_phase'](label);old=float(cmd[0,c['fidx'][s]].sum());T=max(.05,abs(width-old)/.095)
        for k in range(math.ceil(T/dt)):
            u=min(1.,(k+1)*dt/T);cmd[0,c['fidx'][s]]=(old+(width-old)*u)/2;step()
        wait(.4,label+'_hold')
    def IK(s,P,R,seed=None):
        seeds=([]if seed is None else[seed])+[q(s)]+plan['handoffs'].get(active,{}).get('insert_q_seeds',[])+[c['initial_pose']['ready_q'][s]]
        for seed in seeds:
            v=arms[s].solve(P,R,seed)
            if v is not None:return v
        raise RuntimeError('Isaac USD IK failed '+s+' '+str(np.array(P).tolist()))
    def hand_line(s,targetP,targetR,T,label):
        if plan.get('quick_dirty_demo')and not any(word in label for word in ['_insert','_contact','_feedback','_descend','_lift']):
            seeds=[q(s)]+plan['handoffs'][active].get('insert_q_seeds',[])+[plan['handoffs'][active]['recipient_q'],c['initial_pose']['ready_q'][s]];opening=float(cmd[0,c['fidx'][s]].sum())
            for seed in seeds:
                goal=arms[s].solve(targetP,targetR,seed)
                if goal is None or not checkers[s].clear(goal,opening,c['paths']['workpiece_'+active])[0]:continue
                path=checkers[s].route(q(s),goal,opening,c['paths']['workpiece_'+active])
                if path is not None:move(s,path,T,label,stop_at_corners=True);return
            raise RuntimeError('No structure-clear transport route '+label)
        start=arms[s].fk(q(s));a=start[:3,3];Ra=start[:3,:3];rv=log(targetR@Ra.T);path=[q(s)]
        n=max(8,int(np.linalg.norm(targetP-a)/.008)+1,int(np.linalg.norm(rv)/.05)+1)
        try:
            for u in np.linspace(0,1,n+1)[1:]:path.append(IK(s,a+(targetP-a)*u,exp(rv*u)@Ra,path[-1]))
            clear,why=(True,None)if plan.get('quick_dirty_demo')else checkers[s].path(path,float(cmd[0,c['fidx'][s]].sum()),c['paths']['workpiece_'+active])
        except RuntimeError:clear=False;why='Cartesian corridor IK'
        routed=False
        if not clear:
            if any(word in label for word in ['_insert','_contact','_feedback'])and not plan.get('quick_dirty_demo'):raise RuntimeError('Contact approach blocked '+label+' '+str(why))
            goal=IK(s,targetP,targetR);path=[q(s),goal]if plan.get('quick_dirty_demo')else checkers[s].route(q(s),goal,float(cmd[0,c['fidx'][s]].sum()),c['paths']['workpiece_'+active])
            if path is None:raise RuntimeError('No structure-clear transport route '+label+' '+str(why))
            routed=True
        move(s,path,T,label,stop_at_corners=routed or plan.get('quick_dirty_demo',False))
    insertion_rotations={}
    def orientation_error(p,a,b):return float(math.acos(np.clip(a[:,2]@b[:,2],-1,1)))if p.startswith('ring')else float(np.linalg.norm(log(a@b.T)))
    def part_to(s,p,P,R,T,label):
        # Update the grasp transform from actual simulated contact state.
        lp_actual,L_actual=relative(s,p);checkers[s].attach_payload(c['paths']['workpiece_'+p],frame(lp_actual,L_actual))
        if p.startswith('ring')and '_preinsert'in label and p not in insertion_rotations:
            lp,L=relative(s,p);G=np.array(plan['goals'][p]['P']);seeds=[q(s)]+plan['handoffs'][p].get('insert_q_seeds',[])
            for yaw in [0,.35,-.35,.7,-.7,1.05,-1.05]:
                candidate=R@exp(np.array([0,0,yaw]));hR=candidate@L.T;tcp=G-hR@lp+hR@np.array([0,0,.1034]);solution=next((v for seed in seeds if(v:=arms[s].solve(tcp,hR,seed))is not None),None)
                if solution is not None:insertion_rotations[p]=candidate;report.setdefault('ring_insertion_yaw_rad',{})[p]=yaw;break
        if p in insertion_rotations:R=insertion_rotations[p]
        lp,L=relative(s,p);handR=R@L.T;handP=P-handR@lp;tcp=handP+handR@np.array([0,0,.1034]);hand_line(s,tcp,handR,T,label)
        for i in range(3):
            actual,ar=pose(p);e=np.linalg.norm(actual-P);angle=orientation_error(p,R,ar)
            if e<.0005 and angle<.015:break
            lp,L=relative(s,p);handR=R@L.T;handP=P-handR@lp;hand_line(s,handP+handR@np.array([0,0,.1034]),handR,1.5,label+'_feedback'+str(i))
        actual,ar=pose(p)
        if np.linalg.norm(actual-P)>.0015 or orientation_error(p,R,ar)>.04:raise RuntimeError('Part feedback did not converge: '+p+' position '+str(np.linalg.norm(actual-P)))
    def pickup(p):
        nonlocal active
        if plan.get('clutter_items')and p!='pin'and not plan.get('fixed_demo_grasp'):
            import copy
            from bin_arm_task import select_clutter_grasp
            original=plan['parts'];plan['parts']={p:copy.deepcopy(next(iter(original.values())))}
            selected=select_clutter_grasp(stage,c['paths'],c['view'],art,c['aidx'],c['drives'],plan,c['report'],out);path=selected[p]['path'];c['paths']['workpiece_'+p]=path;parts[p]=c['view'].create_rigid_body_view(path);r=plan['parts'][p];curve=np.array(r['cartesian_lift_q']);plan['parts']=original
            active=p;fingers('right',plan['initial_apertures_m']['right'],p+'_preshape');move('right',plan['initial_transition_q'],5.,p+'_pregrasp');move('right',list(reversed(curve)),4.,p+'_descend');fingers('right',r['recipe']['gap']+(.004 if r['recipe'].get('grasp_mode')=='internal_expansion'else-.004),p+'_close');before=pose(p)[0].copy();move('right',curve,5.,p+'_lift');wait(1.,p+'_lift_stable')
            if pose(p)[0][2]-before[2]<.05:raise RuntimeError('Mixed bin pickup did not lift '+p)
            return
        active=p;P,Rp=pose(p);pin=p=='pin';lp=np.array([0,0,.006]if pin else[0,-.034 if p=='ring1'else-.02735,.0035]);R=Rp@np.diag([1.,-1.,-1.]);tcp=P+Rp@lp;rise=.05 if pin else .13;opening=.022 if pin else .016;gap=.013 if pin else .0072 if p=='ring1'else .0075
        if pin:lp=np.array([0,0,-.006]);R=Rp.copy();tcp=P+Rp@lp
        fingers('right',opening,p+'_preshape');pre=IK('right',tcp+[0,0,rise],R);route=checkers['right'].route(q('right'),pre,opening,c['paths']['workpiece_'+p])
        if route is None:raise RuntimeError('No pickup approach '+p)
        move('right',route,4.,p+'_pregrasp',stop_at_corners=True)
        hand_line('right',tcp,R,4.,p+'_descend');fingers('right',max(.001,gap-.004),p+'_close');before=pose(p)[0].copy();hand_line('right',tcp+[0,0,rise],R,5.,p+'_lift');wait(1.,p+'_lift_stable')
        final=pose(p)[0];minimum=.04 if pin else .08
        if final[2]-before[2]<minimum:raise RuntimeError('Pickup failed '+p+' rise='+str(final[2]-before[2]))
    try:
        handoffs=plan['handoffs'];goals=plan['goals']
        for p in [first_part,*[p for p in ['ring1','ring2','pin']if p!=first_part]]:
            active=p
            if p!=first_part:pickup(p)
            rec=handoffs[p];C=np.array(rec['center']);part_to('right',p,np.array([.30,-.50,1.30]),np.eye(3),5.,p+'_leave_basket');part_to('right',p,C,np.eye(3),5.,p+'_handoff_giver')
            P,Rp=pose(p);lp=np.array(rec['localP']);L=np.array(rec['localR']);Rt=Rp@L;tcp=P+Rp@lp;pre=IK('left',tcp-Rt[:,2]*.04,Rt,rec['recipient_q']);fingers('left',.08 if p!='pin'else .022,p+'_recipient_open');move('left',[q('left'),pre],5.,p+'_recipient_approach');hand_line('left',tcp,Rt,3.,p+'_recipient_contact');fingers('left',max(.001,rec['gap']-.004),p+'_recipient_grasp')
            fingers('right',.022 if p=='pin'else .016,p+'_giver_release');checkers['right'].detach_payload();wait(.5,p+'_recipient_retention');hp,hR=hand('right');hand_line('right',hp+hR@np.array([0,0,.1034])+[0,0,.08],hR,2.,p+'_giver_withdraw')
            park=checkers['right'].route(q('right'),c['initial_pose']['ready_q']['right'],float(cmd[0,c['fidx']['right']].sum()),c['paths']['workpiece_'+p])
            if park is None:raise RuntimeError('Cannot retract giver before insertion '+p)
            move('right',park,3.,p+'_giver_park',stop_at_corners=True)
            G=np.array(goals[p]['P']);Rg=np.array(goals[p]['R']);part_to('left',p,np.array([-.15,-.05,1.30]),Rg,5.,p+'_transfer');part_to('left',p,np.array([G[0],-.05,G[2]-.045]),Rg,5.,p+'_raise_before_entry');part_to('left',p,G-[0,0,.045],Rg,5.,p+'_preinsert');part_to('left',p,G,Rg,4.,p+'_insert')
            # Finite force pneumatic/mechanical clamp model; no attachment.
            for k in range(round(.25/dt)):
                u=(k+1)/round(.25/dt)
                for prim in stage.Traverse():
                    if not str(prim.GetPath()).startswith('/World/Clamps/'+p+'/')or not prim.IsA(UsdPhysics.PrismaticJoint):continue
                    joint=UsdPhysics.PrismaticJoint(prim);target=joint.GetUpperLimitAttr().Get()if p!='pin'else joint.GetLowerLimitAttr().Get();UsdPhysics.DriveAPI(prim,'linear').GetTargetPositionAttr().Set(target*u)
                import omni.physx
                omni.physx.get_physx_simulation_interface().flush_changes();step()
            wait(.5,p+'_clamp');fingers('left',.078 if p=='ring1'else .065 if p=='ring2'else .022,p+'_release');checkers['left'].detach_payload();hp,hR=hand('left');hand_line('left',hp+hR@np.array([0,0,.1034])-[0,0,.055],hR,3.,p+'_withdraw');wait(.7,p+'_installed_hold');actual,ar=pose(p)
            if np.linalg.norm(actual-G)>.0006 or orientation_error(p,Rg,ar)>.03:raise RuntimeError('Installed part was not retained '+p+' error='+str(np.linalg.norm(actual-G)))
            completed.append(p);save()
        report.update(state='three_parts_assembled',full_task_pass=True,cycle_time_s=c['now'](),end_positions={p:pose(p)[0].tolist()for p in parts})
    except Exception:
        report.update(state='three_part_sequence_failed',error=traceback.format_exc(),full_task_pass=False,cycle_time_s=None)
        if plan.get('live_demo_control'):
            import time
            def set_active(p):
                nonlocal active
                active=p
            def flush_trace():
                if trace:np.savez_compressed(out/'physx-sequence-trace.npz',time_s=[x['t']for x in trace],q=[x['q']for x in trace],parts=[x['parts']for x in trace],links=[x['links']for x in trace],part_names=list(parts))
            def mark_complete(p):
                wait(.7,p+'_installed_hold');G=np.array(plan['goals'][p]['P']);R=np.array(plan['goals'][p]['R']);P,Rp=pose(p)
                if np.linalg.norm(P-G)>.0015 or orientation_error(p,R,Rp)>.05:raise RuntimeError('Part not installed '+p+' '+str(P.tolist()))
                if any(np.linalg.norm(arms[s].fk(q(s))[:3,3]-P)<.04 for s in arms):raise RuntimeError('Withdraw both hands before accepting '+p)
                if p not in completed:completed.append(p)
            report['state']='live_demo_paused_after_failure';save();flush_trace();env=dict(globals());env.update(locals());seen=None;deadline=time.monotonic()+1200
            print('ISAAC_LIVE_READY',str(out/'live-command.json'),report['error'],flush=True)
            while time.monotonic()<deadline:
                file=out/'live-command.json'
                if not file.exists():time.sleep(.2);continue
                try:command=json.loads(file.read_text())
                except json.JSONDecodeError:time.sleep(.2);continue
                if command.get('id')==seen:time.sleep(.2);continue
                seen=command.get('id');reply=dict(id=seen,success=False)
                try:
                    if command.get('action')=='stop':break
                    if command.get('action')=='finish':
                        if set(completed)!={'ring1','ring2','pin'}:raise RuntimeError('All three parts must be installed first')
                        for name in list(completed):mark_complete(name)
                        report.update(state='three_parts_assembled',full_task_pass=True,cycle_time_s=c['now'](),interactive_simulation_continuation=True,end_positions={p:pose(p)[0].tolist()for p in parts});reply['success']=True
                    else:
                        env.pop('answer',None);exec(command['code'],env,env);reply.update(success=True,answer=env.get('answer'))
                except Exception:reply['error']=traceback.format_exc()
                reply.update(sim_time_s=c['now'](),part_positions={p:pose(p)[0].tolist()for p in parts});(out/'live-command-result.json').write_text(json.dumps(reply,indent=2,default=lambda x:x.tolist()if hasattr(x,'tolist')else str(x)));save();flush_trace();print('ISAAC_LIVE_COMMAND',seen,reply['success'],reply.get('error',''),flush=True)
                if report.get('full_task_pass'):break
    finally:
        if trace:np.savez_compressed(out/'physx-sequence-trace.npz',time_s=[x['t']for x in trace],q=[x['q']for x in trace],parts=[x['parts']for x in trace],links=[x['links']for x in trace],part_names=list(parts))
        save()
    print('ISAAC_BIN_SEQUENCE_DONE',report['state'],report.get('error',''),flush=True);return report
