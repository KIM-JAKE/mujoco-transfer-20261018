"""Synthetic taught handover integration contracts; no hardware connections."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from isaac_port.fieldkit.hand import FrankaHand
from field_demo.handover import main,recipe_content_sha256,transfer,validate


class PLC:
    def __init__(self):
        self.signals={name:dict(kind='discrete',address=index,active=True) for index,name in enumerate(('donor','receiver'))}
        self.bits=dict(donor=True,receiver=False)
    def read(self,name):return self.bits[name]
    def wait(self,name,*,cancelled,**kwargs):
        cancelled()
        if not self.bits[name]:raise TimeoutError('No fresh receiver feedback')


class Hand(FrankaHand):
    def __init__(self,site,side,held):self.site=site;self.side=side;self.held=held;self.width=.04 if held else .07
    def status(self):return dict(is_grasped=self.held,width_m=self.width)
    def move(self,*,width_m,grasp=False,cancelled,**kwargs):
        cancelled();self.site.events.append((self.side,'grasp' if grasp else 'open'))
        self.width=width_m
        if self.side=='left' and grasp:
            self.held=True
            if not self.site.no_receiver_feedback:self.site.plc.bits['receiver']=True
        elif self.side=='right':
            self.held=False;self.site.plc.bits['donor']=False
        cancelled()
        return self.status()


class Site:
    def __init__(self,root):
        self.root=root;self.filename=root/'site.json';self.plc=PLC();self.events=[];self.stopped=False
        self.no_receiver_feedback=False;self.drop_during_retreat=False
        self.grippers=dict(left=Hand(self,'left',False),right=Hand(self,'right',True))
        self.config=dict(robots=dict(left=dict(ip='192.0.2.1'),right=dict(ip='192.0.2.2')),
                         stages={},motion_binary='/offline-validator')
    def path(self,name):return self.root/name
    def validate_motion_config(self,side):pass
    def require_ready(self):
        if self.stopped:raise RuntimeError('stopped')
    def guarded(self,work):
        def cancel():self.require_ready();return False
        self.require_ready();return work(cancel)
    def motion(self,side,mode,target):
        self.require_ready();self.events.append(('motion',Path(target[0]).stem))
        if self.drop_during_retreat and 'retreat' in str(target[0]):self.plc.bits['receiver']=False
        self.require_ready()
    def stop(self):self.stopped=True;self.events.append(('stop',))
    def event(self,*args,**kwargs):pass


class HandoverContracts(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.site=Site(self.root)
        self.path=self.root/'handover.json';self.review=self.root/'review.json'
        stages=['right.approach.ring2_handover','left.approach.ring2_receive','right.approach.ring2_retreat']
        for name in stages:
            f=self.root/(name+'.csv')
            f.write_text('time_s,'+','.join('q'+str(i) for i in range(1,8))+'\n0,0,0,0,0,0,0,0\n3,0,0,0,0,0,0,0\n')
            self.site.config['stages'][name]=dict(csv=f.name,sha256=hashlib.sha256(f.read_bytes()).hexdigest(),
                tool_type='franka_hand',robot_ip=self.site.config['robots'][name.split('.')[0]]['ip'],
                field_path_reviewed=True,F_T_EE_column_major=[0]*16)
        self.recipe=dict(schema_version=1,part='ring2',donor_side='right',receiver_side='left',
            donor_transport_stage=stages[0],receiver_approach_stage=stages[1],donor_retreat_stage=stages[2],
            donor_grasped_signal='donor',receiver_grasped_signal='receiver',
            stage_sha256={name:self.site.config['stages'][name]['sha256'] for name in stages},
            receiver_open=dict(width_m=.07,speed_m_s=.005),
            receiver_grasp=dict(width_m=.04,speed_m_s=.005,force_n=5,epsilon_inner_m=.0005,epsilon_outer_m=.0005),
            donor_release=dict(width_m=.07,speed_m_s=.005),feedback_timeout_s=1.,feedback_dwell_s=.1,
            release_width_tolerance_m=.001,review_file='review.json')
        self.sign()

    def sign(self):
        self.site.filename.write_text(json.dumps(self.site.config))
        self.recipe['site_config_sha256']=hashlib.sha256(self.site.filename.read_bytes()).hexdigest()
        self.review.write_text(json.dumps(dict(recipe_content_sha256=recipe_content_sha256(self.recipe),
            reviewer='synthetic fixture only',at_utc='2020-01-01T00:00:00Z',full_path_reviewed=True,checked_actual_cell=True)))
        self.recipe['review_sha256']=hashlib.sha256(self.review.read_bytes()).hexdigest()
        self.path.write_text(json.dumps(self.recipe))

    def test_success_releases_donor_only_after_receiver_grasp(self):
        with patch('field_demo.handover.subprocess.run'):
            result=transfer(self.site,self.path)
        self.assertEqual(result['state'],'handover_unit_completed')
        self.assertFalse(result['pickup_and_insertion_executed'])
        self.assertLess(self.site.events.index(('left','grasp')),self.site.events.index(('right','open')))
        self.assertTrue(self.site.grippers['left'].held);self.assertFalse(self.site.stopped)
        self.assertEqual(self.site.events[-1],('motion','right.approach.ring2_retreat'))

    def test_no_fresh_receiver_feedback_stops_without_donor_release(self):
        self.site.no_receiver_feedback=True
        with patch('field_demo.handover.subprocess.run'):
            with self.assertRaises(TimeoutError):transfer(self.site,self.path)
        self.assertTrue(self.site.stopped);self.assertTrue(self.site.grippers['right'].held)
        self.assertNotIn(('right','open'),self.site.events)

    def test_receiver_drop_during_retreat_stops_without_auto_open(self):
        self.site.drop_during_retreat=True
        with patch('field_demo.handover.subprocess.run'):
            with self.assertRaisesRegex(RuntimeError,'feedback lost'):transfer(self.site,self.path)
        self.assertTrue(self.site.stopped)
        self.assertEqual(self.site.events.count(('left','open')),1)
        self.assertTrue(self.site.grippers['left'].held)

    def test_stale_receiver_high_rejected_before_finger_or_arm_motion(self):
        self.site.plc.bits['receiver']=True
        with patch('field_demo.handover.subprocess.run'):
            with self.assertRaisesRegex(RuntimeError,'start empty'):transfer(self.site,self.path)
        self.assertEqual(self.site.events,[('stop',)])

    def test_receiver_feedback_high_before_grasp_is_not_reused(self):
        original=self.site.motion
        def early_high(side,mode,target):
            original(side,mode,target)
            if side=='left':self.site.plc.bits['receiver']=True
        self.site.motion=early_high
        with patch('field_demo.handover.subprocess.run'):
            with self.assertRaisesRegex(RuntimeError,'before the commanded grasp'):transfer(self.site,self.path)
        self.assertTrue(self.site.stopped)
        self.assertNotIn(('left','grasp'),self.site.events)
        self.assertNotIn(('right','open'),self.site.events)

    def test_alias_feedback_and_changed_recipe_review_are_rejected(self):
        self.site.plc.signals['receiver']['address']=self.site.plc.signals['donor']['address']
        with patch('field_demo.handover.subprocess.run'):
            with self.assertRaisesRegex(ValueError,'alias'):validate(self.site,self.path)
        self.site.plc.signals['receiver']['address']=1
        self.recipe['donor_release']['width_m']=.06;self.path.write_text(json.dumps(self.recipe))
        with patch('field_demo.handover.subprocess.run'):
            with self.assertRaisesRegex(ValueError,'review'):validate(self.site,self.path)

    def test_changed_stage_or_unbound_stage_hash_is_rejected(self):
        self.recipe['stage_sha256']={};self.sign()
        with patch('field_demo.handover.subprocess.run'):
            with self.assertRaisesRegex(ValueError,'stage hashes'):validate(self.site,self.path)

    def test_no_execute_or_validate_flag_fails_before_site_creation(self):
        with patch('field_demo.handover.Site',side_effect=AssertionError('no setup')):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                main(['--config','/missing','--recipe','/missing'])

    def test_read_only_validation_does_not_read_sensors_or_move(self):
        with patch('field_demo.handover.subprocess.run'), \
             patch.object(self.site.plc,'read',side_effect=AssertionError('no device read')):
            validate(self.site,self.path)
        self.assertEqual(self.site.events,[])


if __name__=='__main__':unittest.main()
