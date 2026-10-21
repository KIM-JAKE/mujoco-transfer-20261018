"""Bounded candidate generalization in explicit fake devices; no robot calls."""
import copy
from datetime import datetime,timezone,timedelta
from pathlib import Path
import subprocess,tempfile,types,unittest
from unittest.mock import patch
import numpy as np
import test_random_cycle as fixture
from field_demo import random_cycle as policy,random_planner

class SearchTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.recipe=fixture.recipe();self.runtime=fixture.FakeRuntime(Path(self.temp.name),self.recipe);self.entry=self.recipe['parts'][0]
    def test_other_visible_ring_before_all_yaws_of_blocked_wall_ring(self):
        self.runtime.reject_first=True
        family=copy.deepcopy(self.entry['grasp_families'][0]);family['name']='other_yaw';family['T_part_tcp'][0][0]=-1.;family['T_part_tcp'][1][1]=-1.;self.entry['grasp_families'].append(family)
        seen=[];plan=self.runtime.plan
        def recorded(*args,**kwargs):seen.append(args[4]['selected_instance']['instance_id']);return plan(*args,**kwargs)
        self.runtime.plan=recorded
        selected=policy.screen_pickup(self.runtime,self.entry,self.runtime.observe('basket','ring1',side='right'),[])
        self.assertEqual(len(seen),2);self.assertTrue(seen[0].endswith(':blocked'));self.assertFalse(seen[1].endswith(':blocked'))
        self.assertEqual(selected[1]['name'],'measured');self.assertFalse(any(e[0]in('hand','move')for e in self.runtime.events))
    def test_fresh_capture_ids_do_not_repeat_identical_failed_physical_targets(self):
        self.runtime.reject_first=True;seen=[]
        def blocked(*args,**kwargs):seen.append(args[4]['capture_id']);raise RuntimeError('measured geometry blocked')
        self.runtime.plan=blocked
        with self.assertRaisesRegex(RuntimeError,'No screened pickup'):policy.screen_pickup(self.runtime,self.entry,self.runtime.observe('basket','ring1',side='right'),[])
        self.assertEqual(len(seen),2);self.assertEqual(self.runtime.counter,2)
        self.assertFalse(any(e[0]in('hand','move')for e in self.runtime.events))
    def test_expired_frame_recaptured_before_any_planning(self):
        self.runtime.replay=False;old=self.runtime.observe('basket','ring1',side='right');stamp=(datetime.now(timezone.utc)-timedelta(seconds=3)).isoformat();old['captured_at_utc']=stamp;old['source_metadata']['captured_at_utc']=stamp
        result=policy.screen_pickup(self.runtime,self.entry,old,[]);self.assertNotEqual(result[0]['capture_id'],old['capture_id']);self.assertEqual(self.runtime.counter,2)
    def test_target_changes_before_first_arm_motion_reselects_one_new_image_pose(self):
        old_execute=self.runtime.execute;old_pose=self.runtime.object_pose;shifted=[False]
        def object_pose(part):
            pose=copy.deepcopy(old_pose(part))
            if part=='ring1'and shifted[0]and not any(h['part']==part for h in self.runtime.hands.values())and not self.runtime.signals[part+'_retained']:pose[0][3]+=.04
            return pose
        def execute(plan,observation):
            if plan['phase']=='pickup'and plan['part']=='ring1'and not shifted[0]:shifted[0]=True;raise policy.PreMotionPickupChanged('synthetic fresh image moved; no arm process launched')
            return old_execute(plan,observation)
        self.runtime.object_pose=object_pose;self.runtime.execute=execute
        result=policy.run(self.runtime,self.recipe);self.assertEqual(result['completed_parts'],list(policy.PARTS))
        retries=[e for e in self.runtime.events if e[0]=='pickup_pre_motion_reselection'];self.assertEqual(len(retries),1)
    def test_arbitrary_execution_failure_never_retries_unknown_motion_state(self):
        def fail(*args):raise RuntimeError('synthetic native arm failure; execution state unknown')
        self.runtime.execute=fail
        with self.assertRaisesRegex(RuntimeError,'state unknown'):policy.run(self.runtime,self.recipe)
        self.assertTrue(self.runtime.stopped);self.assertFalse(any(e[0]=='pickup_pre_motion_reselection'for e in self.runtime.events))
    def test_opening_happens_before_width_sensitive_pickup_planning(self):
        self.runtime.width_aware=True;self.runtime.prepare_pickup_hand=types.MethodType(policy.LiveRuntime.prepare_pickup_hand,self.runtime)
        self.entry['grasp_families'][0]['actions']['open']['width_m']=.08
        old=self.runtime.plan;seen=[]
        def plan(*args,**kwargs):
            if args[0]=='pickup'and args[1]=='ring1':seen.append(self.runtime.hand_status('right')['width_m'])
            return old(*args,**kwargs)
        self.runtime.plan=plan;policy.run(self.runtime,self.recipe,single_pick='ring1');self.assertEqual(seen,[.08])

class NativeFailureTests(unittest.TestCase):
    def test_only_pinned_no_ik_target_failure_is_candidate_retryable(self):
        error=subprocess.CalledProcessError(1,['synthetic'],stderr='Read-only dynamic model failed: No measured-model bounded IK for this phase\n')
        with patch.object(random_planner.subprocess,'run',side_effect=error):
            with self.assertRaisesRegex(RuntimeError,'candidate unavailable'):random_planner.run_native('/not-run','--read-model','192.0.2.1',[np.eye(4)])
        error=subprocess.CalledProcessError(1,['synthetic'],stderr='Read-only dynamic model failed: SDK network connection failed\n')
        with patch.object(random_planner.subprocess,'run',side_effect=error):
            with self.assertRaises(subprocess.CalledProcessError):random_planner.run_native('/not-run','--read-model','192.0.2.1',[np.eye(4)])

if __name__=='__main__':unittest.main()
