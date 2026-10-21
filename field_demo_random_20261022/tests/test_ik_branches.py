"""Offline read-only IK seed contract; no SDK/device connections."""
import unittest
from unittest.mock import patch,Mock
import numpy as np
from field_demo.ik_branches import candidates,LOWER,UPPER
from field_demo.random_planner import run_native

class BranchTests(unittest.TestCase):
    def test_alternatives_are_bounded_and_do_not_mutate_measured_start(self):
        measured=(LOWER+UPPER)/2;original=measured.copy()
        seeds=candidates(measured,[[0]*7,[0,0,.8,0,0,0,0],[0,0,-.8,0,0,0,0]])
        self.assertEqual(len(seeds),3);np.testing.assert_array_equal(measured,original)
        np.testing.assert_allclose(seeds[1][2]-seeds[0][2],.8)
        bounded=candidates(UPPER-.001,[[1]*7,[0]*7])
        self.assertEqual(len(bounded),1);self.assertTrue(np.all(np.asarray(bounded)<=UPPER-.001))
    def test_seed_reaches_only_read_only_native_argv_with_targets_preserved(self):
        target=np.eye(4);target[0,3]=.42;seed=((LOWER+UPPER)/2).tolist()
        def native(argv,**kwargs):
            self.assertEqual(argv[1:3],['--read-model','192.0.2.1'])
            self.assertEqual(argv[4],'--ik-seed');np.testing.assert_allclose(list(map(float,argv[5:])),seed)
            np.testing.assert_allclose(np.loadtxt(argv[3]).reshape(4,4),target)
            return Mock(stdout='{"read_only":true,"motion_executed":false}')
        with patch('field_demo.random_planner.subprocess.run',side_effect=native)as call:
            result=run_native('/nonexistent/FAKE','--read-model','192.0.2.1',[target],ik_seed=seed)
        self.assertEqual(call.call_count,1);self.assertFalse(result['motion_executed'])
    def test_seed_rejected_for_state_query_before_any_process(self):
        with patch('field_demo.random_planner.subprocess.run')as call:
            with self.assertRaises(ValueError):run_native('/FAKE','--read-state','192.0.2.1',ik_seed=[0]*7)
        call.assert_not_called()

if __name__=='__main__':unittest.main()
