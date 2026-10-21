"""Full phase state machine tests with explicit fake image/native/PLC devices."""
import copy
from datetime import datetime,timezone,timedelta
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from field_demo import random_cycle


def pose(x=0,y=0,z=0):
    t=np.eye(4);t[:3,3]=[x,y,z];return t.tolist()


def recipe():
    action=dict(open=dict(width_m=.07,speed_m_s=.005),grasp=dict(width_m=.04,speed_m_s=.005,
        force_n=5,epsilon_inner_m=.0005,epsilon_outer_m=.0005),release=dict(width_m=.07,speed_m_s=.005))
    parts=[]
    for part in random_cycle.PARTS:
        h=dict(donor_T_cell_tcp=pose(.4,0,.4),donor_waypoints_cell_tcp=[],receiver_T_part_tcp=pose(.01,0,.02),
            receiver_approach_offset_part_m=[0,0,.07],donor_retreat_offset_cell_m=[.1,0,.06],
            donor_grasped_signal=part+'_donor',receiver_grasped_signal=part+'_receiver',
            receiver_open=action['open'],receiver_grasp=action['grasp'],donor_release=action['release'],receiver_release=action['release'])
        parts.append(dict(part=part,pickup_side='right',insert_side='left' if part!='pin' else 'right',
            grasp_families=[dict(name='measured',T_part_tcp=pose(0,0,.02),actions=copy.deepcopy(action))],
            pregrasp_offset_part_m=[0,0,.07],lift_offsets_cell_m=[[0,0,.1]],
            insert_entry_offset_part_m=[0,0,.08],withdraw_offset_cell_m=[0,0,.08],min_tcp_clearance_m=.04,
            payload_dimensions=dict(radius_m=.02,length_m=.02),handover=h))
    return dict(parts=parts,insertion_targets={p:pose(.65+i*.1,0,.1) for i,p in enumerate(random_cycle.PARTS)},
        max_task_s=60,max_observation_age_s=1.,feedback_timeout_s=.5,feedback_dwell_s=.1,
        release_width_tolerance_m=.001,insert_pose_confirmation_tolerance_m=.001,
        target_position_tolerance_m=.001,target_angle_tolerance_rad=.01)


class FakeRuntime:
    replay=True
    def __init__(self,root,c):
        self.recipe=c;self.root=root;self.monitor=lambda:None;self.stopped=False;self.events=[];self.counter=0
        self.signals={p+'_'+suffix:False for p in random_cycle.PARTS for suffix in ('grasped','seated','clamped','retained','donor','receiver')}
        self.hands={s:dict(part=None,width_m=.07,G=pose(.01,-.006,-.02)) for s in ('left','right')}
        self.tcps={s:pose(.1,0,.5) for s in ('left','right')}
        self.site=SimpleNamespace(expected_held={},config=dict(gripper_actions={e['part']:e['grasp_families'][0]['actions'] for e in c['parts']}))
        self.plans=[];self.installed_parts=[];self.reject_first=False;self.no_seat=False;self.no_receiver=False
        self.camera_fail_scope=None;self.drop_on_pin=False;self.bad_final_clearance=False;self.stale=False
    def ready(self):
        if self.stopped:raise RuntimeError('stopped')
        self.monitor()
    def read(self,name):return self.signals[name]
    def wait(self,name,timeout,dwell):
        self.ready()
        if not self.read(name):raise TimeoutError('Missing actual '+name)
    def clamp(self,part):self.signals[part+'_clamped']=True;self.events.append(('clamp',part));self.ready()
    def tcp(self,side):return self.tcps[side]
    def hand_status(self,side):return dict(is_grasped=self.hands[side]['part'] is not None,width_m=self.hands[side]['width_m'])
    def object_pose(self,part):
        for side,h in self.hands.items():
            if h['part']==part:return (np.array(self.tcps[side])@np.array(h['G'])).tolist()
        if self.signals[part+'_retained']:return self.recipe['insertion_targets'][part]
        return pose(.2+random_cycle.PARTS.index(part)*.06,.2,.1)
    def hand_move(self,side,part,mode,parameters):
        self.ready();self.events.append(('hand',side,part,mode));hand=self.hands[side]
        if mode=='grasp':
            current=np.array(self.object_pose(part));hand['part']=part
            hand['G']=(np.linalg.inv(np.array(self.tcps[side]))@current).tolist()
            # A real grasp offset error: source image, never the nominal family,
            # supplies G used for subsequent insertion correction.
            hand['G'][0][3]+=.0003
            self.signals[part+'_grasped']=True
            e=next(e for e in self.recipe['parts'] if e['part']==part)
            self.signals[part+('_donor' if side==e['pickup_side'] else '_receiver')]=not(self.no_receiver and side==e['insert_side'] and side!=e['pickup_side'])
        elif hand['part'] is not None:
            released=hand['part'];hand['part']=None
            entry=next(e for e in self.recipe['parts'] if e['part']==released)
            self.signals[released+('_donor' if side==entry['pickup_side'] else '_receiver')]=False
            self.signals[released+'_grasped']=any(h['part']==released for h in self.hands.values())
            if self.signals[released+'_clamped']:self.signals[released+'_retained']=True
        hand['width_m']=parameters['width_m'];self.ready();return self.hand_status(side)
    def observe(self,scope,part,*,side=None,previous=None):
        self.ready();self.counter+=1;self.events.append(('observe',scope,part,self.counter))
        if scope==self.camera_fail_scope:raise RuntimeError('No actual visible image')
        stamp=(datetime.now(timezone.utc)-timedelta(seconds=10 if self.stale else .001)).isoformat()
        metadata=dict(capture_id='image-'+str(self.counter),captured_at_utc=stamp,calibration_sha256='actual-camera-test',source='realsense_live',replay=self.replay)
        instance=dict(instance_id=metadata['capture_id']+':pixel-0',part=part,T_cell_part=self.object_pose(part),geometrically_accessible=True)
        values=[instance]
        if scope=='basket' and self.reject_first:
            first=copy.deepcopy(instance);first['instance_id']+=':blocked';values=[first,instance]
        result=dict(state='random_ring_observation',schema_version=2,replay=self.replay,source_metadata=metadata,
            **{k:metadata[k] for k in ('capture_id','captured_at_utc','calibration_sha256')},
            instances=values,ring_pose_seed_used=scope=='held',_scope=scope,_part=part,_side=side)
        path=self.root/('image-'+str(self.counter)+'.json');path.write_text(json.dumps(result));result['_path']=str(path)
        return result
    def plan(self,phase,part,side,targets,observation,held_parts,*,contact=None):
        self.events.append(('plan',phase,part,observation['capture_id']))
        if phase=='pickup' and observation['selected_instance']['instance_id'].endswith(':blocked'):
            raise RuntimeError('Actual scene blocks this candidate')
        plan=dict(phase=phase,part=part,side=side,targets=copy.deepcopy(targets),observation=copy.deepcopy(observation),held=copy.deepcopy(held_parts))
        self.plans.append(plan);return plan
    def execute(self,plan,observation):
        self.ready();self.events.append(('move',plan['phase'],plan['part'],plan['side']))
        self.tcps[plan['side']]=plan['targets'][-1]
        if plan['phase']=='insert':
            if not self.no_seat:self.signals[plan['part']+'_seated']=True
            actual=self.object_pose(plan['part']);target=self.recipe['insertion_targets'][plan['part']]
            np.testing.assert_allclose(actual,target,atol=1e-12)
        if plan['part']=='pin' and self.drop_on_pin:self.signals['ring1_retained']=False
        self.ready()
    def mark_installed(self,part,pose,dimensions,observation,*,capsules_part_m=None):self.installed_parts.append(part)
    def verify_clearance(self,c):
        self.ready();self.events.append(('clearance',))
        if self.bad_final_clearance and len(self.installed_parts)==3:raise RuntimeError('Actual second Hand too close')
    def event(self,name,**values):self.events.append((name,values))
    def stop(self):self.stopped=True;self.events.append(('stop',))


class RandomCycleContracts(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.recipe=recipe();self.runtime=FakeRuntime(Path(self.tmp.name),self.recipe)
    def test_full_three_parts_transfer_measured_G_and_reobserve_removed_basket(self):
        result=random_cycle.run(self.runtime,self.recipe)
        self.assertEqual(result['state'],'random_cycle_replay_contract_completed');self.assertFalse(result['motion_executed'])
        self.assertEqual(result['completed_parts'],list(random_cycle.PARTS));self.assertFalse(self.runtime.stopped)
        self.assertEqual(self.runtime.installed_parts,list(random_cycle.PARTS))
        phases=[p['phase'] for p in self.runtime.plans]
        self.assertEqual(phases.count('donor_carry'),2);self.assertEqual(phases.count('insert'),3)
        basket=[event for event in self.runtime.events if event[:2]==('observe','basket')]
        self.assertEqual([e[2] for e in basket],['ring1','ring2']);self.assertNotEqual(basket[0][3],basket[1][3])
        for part in ('ring1','ring2'):
            grasp=next(i for i,e in enumerate(self.runtime.events) if e==('hand','left',part,'grasp'))
            insertion=next(i for i,e in enumerate(self.runtime.events) if e==('move','insert',part,'left'))
            self.assertNotIn(('hand','left',part,'open'),self.runtime.events[grasp:insertion])
    def test_blocked_image_candidate_rejected_before_motion_then_next_image_candidate(self):
        self.runtime.reject_first=True
        random_cycle.run(self.runtime,self.recipe)
        events=self.runtime.events
        first_move=next(i for i,e in enumerate(events) if e[0]=='move')
        self.assertGreaterEqual(sum(e[0]=='plan' for e in events[:first_move]),2)
    def test_ambiguous_ring_face_is_obstacle_not_a_pickup_candidate(self):
        original=self.runtime.observe
        def ambiguous(scope,part,**kwargs):
            observation=original(scope,part,**kwargs)
            for instance in observation['instances']:instance['geometrically_accessible']=False
            return observation
        self.runtime.observe=ambiguous
        with self.assertRaisesRegex(RuntimeError,'No screened pickup'):random_cycle.run(self.runtime,self.recipe)
        self.assertFalse(any(e[0] in ('move','hand') for e in self.runtime.events))

    def test_missing_camera_has_no_fallback_motion(self):
        self.runtime.camera_fail_scope='held'
        with self.assertRaisesRegex(RuntimeError,'visible'):random_cycle.run(self.runtime,self.recipe)
        self.assertTrue(self.runtime.stopped)
        self.assertFalse(any(e[:2]==('move','lift') for e in self.runtime.events))
        self.assertTrue(self.runtime.hand_status('right')['is_grasped'])
    def test_receiver_missing_independent_grasp_never_opens_donor(self):
        self.runtime.no_receiver=True
        with self.assertRaises(TimeoutError):random_cycle.run(self.runtime,self.recipe)
        self.assertTrue(self.runtime.stopped)
        self.assertNotIn(('hand','right','ring1','release'),self.runtime.events)
    def test_missing_actual_seat_never_releases(self):
        self.runtime.no_seat=True
        with self.assertRaises(TimeoutError):random_cycle.run(self.runtime,self.recipe)
        self.assertNotIn(('hand','left','ring1','release'),self.runtime.events)
        self.assertTrue(self.runtime.stopped)
    def test_installed_drop_stops_following_part(self):
        self.runtime.drop_on_pin=True
        with self.assertRaisesRegex(RuntimeError,'feedback lost'):random_cycle.run(self.runtime,self.recipe)
        self.assertTrue(self.runtime.stopped)
    def test_final_other_hand_clearance_required(self):
        self.runtime.bad_final_clearance=True
        with self.assertRaisesRegex(RuntimeError,'second Hand'):random_cycle.run(self.runtime,self.recipe)
        self.assertTrue(self.runtime.stopped)
    def test_live_adapter_refreshes_after_planning_and_requires_dynamic_proof(self):
        from unittest.mock import MagicMock
        runtime=random_cycle.LiveRuntime.__new__(random_cycle.LiveRuntime)
        runtime.recipe=self.recipe;runtime.output=Path(self.tmp.name);runtime.installed_parts=[]
        runtime.site=MagicMock();runtime.planner=MagicMock();runtime.ready=MagicMock()
        self.runtime.replay=False
        observed=self.runtime.observe('basket','ring1',side='right')
        selected=random_cycle.select_observation(observed,observed['instances'][0])
        fresh=copy.deepcopy(observed);fresh['capture_id']='another-current-image'
        fresh['source_metadata']['capture_id']=fresh['capture_id']
        fresh['instances'][0]['instance_id']='new-image-object-id'
        runtime.observe=MagicMock(return_value=fresh)
        csv=Path(self.tmp.name)/'phase.csv';csv.write_text('exact screened data')
        plan=dict(csv=str(csv),csv_sha256=random_cycle.sha(csv),stage_row=dict(csv=str(csv),sha256=random_cycle.sha(csv),
            planning_source='dynamic_field_envelope',field_path_reviewed=False),_runtime_observation=selected,_runtime_side='right')
        runtime.execute(plan,selected)
        runtime.observe.assert_called_once()
        runtime.planner.revalidate.assert_called_once()
        self.assertEqual(runtime.planner.revalidate.call_args.kwargs['observation']['capture_id'],'another-current-image')
        runtime.site.motion.assert_called_once()
        runtime.site.motion.reset_mock();plan['stage_row']['field_path_reviewed']=True
        with self.assertRaisesRegex(ValueError,'fabricated'):runtime.execute(plan,selected)
        runtime.site.motion.assert_not_called()

    def test_cli_without_action_mode_never_constructs_hardware(self):
        import contextlib,io
        with patch('field_demo.random_cycle.Site',side_effect=AssertionError('no hardware')),contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):random_cycle.main(['--recipe','/missing'])
            with self.assertRaises(SystemExit):random_cycle.main(['--recipe','/missing','--dry-run'])

    def test_live_report_cannot_use_stale_replay_or_seeded_basket(self):
        self.runtime.replay=False;self.runtime.stale=True
        with self.assertRaisesRegex(RuntimeError,'stale'):random_cycle.check_observation(self.runtime,self.runtime.observe('basket','ring1',side='right'))
        self.assertFalse(any(e[0]=='move' for e in self.runtime.events))


if __name__=='__main__':unittest.main()
