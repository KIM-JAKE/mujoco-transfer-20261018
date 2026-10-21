"""Mocked full-cycle feedback and withdrawal tests. No hardware connections."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from isaac_port.fieldkit.hand import FrankaHand
from field_demo import cycle,handover


def identity():return [[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]


class PLC:
    def __init__(self):
        self.signals={part+'_'+suffix:dict(kind='coil' if suffix=='clamp_command' else 'discrete',address=i,active=True)
            for i,(part,suffix) in enumerate((p,s) for p in cycle.PARTS for s in ('grasped','seated','clamped','retained','clamp_command'))}
        self.bits=dict.fromkeys(self.signals,False);self.missing_clamp=False
    def read(self,name):return self.bits[name]
    def write(self,name,value):
        self.bits[name]=value
        if name.endswith('_clamp_command') and not self.missing_clamp:self.bits[name.replace('_clamp_command','_clamped')]=True
    def wait(self,name,*,cancelled,**kwargs):
        cancelled()
        if not self.bits[name]:raise TimeoutError('Missing actual feedback '+name)


class Hand(FrankaHand):
    def __init__(self,site,side):self.site=site;self.side=side;self.part=None;self.width=.07
    def status(self):return dict(is_grasped=self.part is not None,width_m=self.width)
    def move(self,*,width_m,grasp=False,cancelled,**kwargs):
        cancelled();self.site.events.append(('hand',self.side,'grasp' if grasp else 'open',self.site.current_part))
        if grasp:
            self.part=self.site.current_part;self.site.plc.bits[self.part+'_grasped']=True
        elif self.part:
            part=self.part;self.part=None;self.site.plc.bits[part+'_grasped']=False
            self.site.plc.bits[part+'_retained']=self.site.plc.bits[part+'_clamped'] and self.site.plc.bits[part+'_seated']
        self.width=width_m;cancelled();return self.status()


class Site:
    def __init__(self,root):
        self.root=root;self.filename=root/'site.json';self.current_part=None;self.plc=PLC();self.expected_held={};self.stopped=False
        self.events=[];self.tcp=dict(left=.1,right=.1);self.withdraw_failure=False
        self.drop_on_pin=False;self.final_near_previous=False;self.drop_during_final=False;self.final_began=None
        self.grippers={s:Hand(self,s) for s in ('left','right')}
        self.config=dict(stages={},motion_binary='/validator',state_binary='/state',
            robots={s:dict(ip='192.0.2.1' if s=='left' else '192.0.2.2',T_cell_base=identity()) for s in ('left','right')},
            gripper_actions={p:dict(open=dict(width_m=.07,speed_m_s=.005),
                grasp=dict(width_m=.04,speed_m_s=.005,force_n=5,epsilon_inner_m=.0005,epsilon_outer_m=.0005),
                release=dict(width_m=.07,speed_m_s=.005)) for p in cycle.PARTS})
    def path(self,name):return self.root/name
    def validate_motion_config(self,side):pass
    def require_ready(self):
        if self.stopped:raise RuntimeError('stopped')
        if self.drop_during_final and self.final_began is not None and time.monotonic()-self.final_began>.04:
            self.plc.bits['ring1_retained']=False
        for part in self.expected_held.values():
            if not self.plc.bits[part+'_grasped']:raise RuntimeError('held feedback lost')
    def guarded(self,work):
        def cancel():self.require_ready();return False
        self.require_ready();return work(cancel)
    def motion(self,side,mode,target):
        self.require_ready();name=Path(target[0]).stem;_,action,label=name.split('.')
        part=label.removesuffix('_withdraw');self.current_part=part
        self.events.append(('motion',side,action,part))
        if action=='insert':self.plc.bits[part+'_seated']=True
        if action=='grasp' and part=='pin' and self.drop_on_pin:self.plc.bits['ring1_retained']=False
        if action=='approach':self.tcp[side]=.01 if self.withdraw_failure else .1
        self.require_ready()
    def event(self,event,**kwargs):
        self.events.append((event,kwargs))
        if event=='part_withdrawn_and_retained' and kwargs['part']=='pin':self.final_began=time.monotonic()
    def stop(self):self.stopped=True;self.events.append(('stop',))


class CycleContracts(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.site=Site(self.root)
        self.path=self.root/'cycle.json';self.review=self.root/'review.json'
        self.recipe=dict(schema_version=1,feedback_timeout_s=1.,feedback_dwell_s=.1,
            release_width_tolerance_m=.001,parts=[],stage_sha256={},review_file='review.json')
        for part,side in [('ring1','right'),('ring2','left'),('pin','left')]:
            entry=dict(part=part,pickup_side=side,insert_side=side,T_cell_part=identity(),min_tcp_clearance_m=.04)
            for key,action,label in [('pickup_stage','grasp',part),('insert_stage','insert',part),
                                      ('release_stage','release',part),('withdraw_stage','approach',part+'_withdraw')]:
                name=side+'.'+action+'.'+label;self.add_stage(name);entry[key]=name
            self.recipe['parts'].append(entry)
        self.sign()

    def add_stage(self,name):
        path=self.root/(name+'.csv')
        path.write_text('time_s,'+','.join('q'+str(i) for i in range(1,8))+'\n0,0,0,0,0,0,0,0\n3,0,0,0,0,0,0,0\n')
        digest=hashlib.sha256(path.read_bytes()).hexdigest();side=name.split('.')[0]
        self.site.config['stages'][name]=dict(csv=path.name,sha256=digest,tool_type='franka_hand',
            robot_ip=self.site.config['robots'][side]['ip'],field_path_reviewed=True,F_T_EE_column_major=[0]*16)
        self.recipe['stage_sha256'][name]=digest

    def sign(self):
        self.site.filename.write_text(json.dumps(self.site.config))
        self.recipe['site_config_sha256']=handover.digest(self.site.filename)
        self.review.write_text(json.dumps(dict(recipe_content_sha256=handover.recipe_content_sha256(self.recipe),
            reviewer='synthetic test fixture',at_utc='2020-01-01T00:00:00Z',full_path_reviewed=True,checked_actual_cell=True)))
        self.recipe['review_sha256']=handover.digest(self.review)
        self.path.write_text(json.dumps(self.recipe))

    def native(self,args,**kwargs):
        if args[0]=='/validator':return SimpleNamespace(stdout='')
        self.assertEqual(args[0],'/state')
        side='left' if args[1]=='192.0.2.1' else 'right'
        x=.01 if self.site.final_near_previous and self.site.final_began is not None and side=='right' else self.site.tcp[side]
        t=[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,x,0.,0.,1.]
        return SimpleNamespace(stdout=json.dumps(dict(read_only=True,server_protocol_version=10,O_T_EE_column_major=t)))

    def test_all_three_complete_only_after_release_withdraw_and_final_retention(self):
        with patch('field_demo.cycle.subprocess.run',side_effect=self.native):result=cycle.execute(self.site,self.path)
        self.assertEqual(result['completed_parts'],list(cycle.PARTS));self.assertTrue(result['all_insert_hands_withdrawn'])
        self.assertFalse(self.site.stopped)
        for part in cycle.PARTS:
            self.assertTrue(self.site.plc.bits[part+'_retained'])
            release=next(i for i,e in enumerate(self.site.events) if e[:4]==('motion','right' if part=='ring1' else 'left','release',part))
            withdrawal=next(i for i,e in enumerate(self.site.events) if len(e)>3 and e[0]=='motion' and e[2:] == ('approach',part))
            self.assertLess(release,withdrawal)
        self.assertEqual(self.site.events[-1][0],'three_part_cycle_completed')

    def test_handover_preserves_receiver_grasp_through_insertion(self):
        entry=self.recipe['parts'][1]
        old=entry['pickup_stage'];self.recipe['stage_sha256'].pop(old)
        entry['pickup_side']='right';entry['pickup_stage']='right.grasp.ring2';self.add_stage(entry['pickup_stage'])
        hpath=self.root/'handover.json';hpath.write_text('{"synthetic":true}')
        entry['handover_recipe']=hpath.name;entry['handover_sha256']=handover.digest(hpath);self.sign()
        transfer_recipe=dict(part='ring2',donor_side='right',receiver_side='left',stage_sha256={})
        def transfer(site,path):
            self.assertEqual(site.grippers['right'].part,'ring2')
            site.grippers['right'].part=None;site.grippers['right'].width=.07
            site.grippers['left'].part='ring2';site.grippers['left'].width=.04
            site.events.append(('handover','ring2'))
        with patch('field_demo.cycle.subprocess.run',side_effect=self.native), \
             patch('field_demo.cycle.handover.validate',return_value=transfer_recipe), \
             patch('field_demo.cycle.handover.transfer',side_effect=transfer):
            result=cycle.execute(self.site,self.path)
        self.assertEqual(result['state'],'three_part_cycle_completed')
        h=self.site.events.index(('handover','ring2'))
        insertion=self.site.events.index(('motion','left','insert','ring2'))
        self.assertFalse(any(e[:3]==('hand','left','open') for e in self.site.events[h+1:insertion]))

    def test_missing_clamp_never_opens_held_part(self):
        self.site.plc.missing_clamp=True
        with patch('field_demo.cycle.subprocess.run',side_effect=self.native):
            with self.assertRaises(TimeoutError):cycle.execute(self.site,self.path)
        self.assertTrue(self.site.stopped);self.assertEqual(self.site.grippers['right'].part,'ring1')
        self.assertFalse(any(e[0]=='part_withdrawn_and_retained' for e in self.site.events))

    def test_withdrawal_clearance_failure_does_not_mark_part_or_cycle_complete(self):
        self.site.withdraw_failure=True
        with patch('field_demo.cycle.subprocess.run',side_effect=self.native):
            with self.assertRaisesRegex(RuntimeError,'clearance'):cycle.execute(self.site,self.path)
        self.assertTrue(self.site.stopped)
        self.assertFalse(any(e[0] in ('part_withdrawn_and_retained','three_part_cycle_completed') for e in self.site.events))

    def test_previous_part_drop_during_next_pickup_stops_cycle(self):
        self.site.drop_on_pin=True
        with patch('field_demo.cycle.subprocess.run',side_effect=self.native):
            with self.assertRaisesRegex(RuntimeError,'feedback lost'):cycle.execute(self.site,self.path)
        self.assertTrue(self.site.stopped)
        self.assertFalse(any(e[0]=='three_part_cycle_completed' for e in self.site.events))

    def test_final_current_other_arm_near_prior_part_cannot_claim_clear(self):
        self.site.final_near_previous=True
        with patch('field_demo.cycle.subprocess.run',side_effect=self.native):
            with self.assertRaisesRegex(RuntimeError,'clearance'):cycle.execute(self.site,self.path)
        self.assertTrue(self.site.stopped)
        self.assertEqual(sum(e[0]=='part_withdrawn_and_retained' for e in self.site.events),3)
        self.assertFalse(any(e[0]=='three_part_cycle_completed' for e in self.site.events))

    def test_final_dwell_rechecks_all_three_retained(self):
        self.site.drop_during_final=True
        with patch('field_demo.cycle.subprocess.run',side_effect=self.native):
            with self.assertRaisesRegex(RuntimeError,'feedback lost'):cycle.execute(self.site,self.path)
        self.assertFalse(any(e[0]=='three_part_cycle_completed' for e in self.site.events))
        self.assertEqual(sum(e[0]=='part_withdrawn_and_retained' for e in self.site.events),3)

    def test_initial_stale_feedback_blocks_actuation(self):
        self.site.plc.bits['pin_seated']=True
        with patch('field_demo.cycle.subprocess.run',side_effect=self.native):
            with self.assertRaisesRegex(RuntimeError,'cleared'):cycle.execute(self.site,self.path)
        self.assertEqual(self.site.events,[('stop',)])

    def test_read_only_validation_and_no_actuation_default(self):
        with patch('field_demo.cycle.subprocess.run',side_effect=self.native), \
             patch.object(self.site.plc,'read',side_effect=AssertionError('no device read')):
            cycle.validate(self.site,self.path)
        with patch('field_demo.cycle.Site',side_effect=AssertionError('no device setup')), \
             contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):cycle.main(['--config','/missing','--recipe','/missing'])
        with patch('field_demo.cycle.Site',side_effect=AssertionError('no device setup')),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(cycle.main(['--dry-run']),0)

    def test_site_setting_changes_invalidate_cycle_review(self):
        self.site.config['gripper_actions']['ring1']['release']['width_m']=.075
        self.site.filename.write_text(json.dumps(self.site.config))
        with patch('field_demo.cycle.subprocess.run',side_effect=self.native):
            with self.assertRaisesRegex(ValueError,'site configuration changed'):cycle.validate(self.site,self.path)

    def test_missing_handover_or_changed_recipe_rejected(self):
        self.recipe['parts'][1]['pickup_side']='right';self.sign()
        with patch('field_demo.cycle.subprocess.run',side_effect=self.native):
            with self.assertRaises(ValueError):cycle.validate(self.site,self.path)
        self.recipe['parts'][1]['pickup_side']='left';self.sign()
        self.recipe['parts'][0]['min_tcp_clearance_m']=.05;self.path.write_text(json.dumps(self.recipe))
        with patch('field_demo.cycle.subprocess.run',side_effect=self.native):
            with self.assertRaisesRegex(ValueError,'review'):cycle.validate(self.site,self.path)


if __name__=='__main__':unittest.main()
