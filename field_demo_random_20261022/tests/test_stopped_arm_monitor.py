"""Synthetic actual-SDK protocol boundary only; never hardware evidence."""
import copy,threading,time,unittest
import numpy as np
from field_demo.stopped_arm_monitor import validate,StoppedArmMonitor


def state():
    return dict(read_only=True,server_protocol_version=10,robot_mode_name='idle',robot_time_s=10.,q=[0.]*7,q_d=[0.]*7,dq=[0.]*7,dq_d=[0.]*7,O_F_ext_hat_K=[0.]*6)

class StoppedArmTests(unittest.TestCase):
    def test_same_existing_aggregate_limits_and_stationary_pose_not_pad_force(self):
        row=state();row['O_F_ext_hat_K']=[3.,4.,0.,0.,0.,1.];validate(row,[0.]*7,.02)
        for key,newvalue in [('O_F_ext_hat_K',[5.01,0,0,0,0,0]),('O_F_ext_hat_K',[0,0,0,0,0,1.01]),('q_d',[.00001]+[0.]*6),('dq',[.011]+[0.]*6),('q',[.006]+[0.]*6),('robot_mode_name','move')]:
            changed=state();changed[key]=newvalue
            with self.assertRaises(RuntimeError):validate(changed,[0.]*7,.02)
    def test_stalled_sdk_clock_and_read_error_revoke_actual_sample(self):
        now=[0.];row=state();monitor=StoppedArmMonitor(lambda:copy.deepcopy(row),[0.]*7,.02,clock=lambda:now[0]);self.addCleanup(monitor.close)
        monitor.status();now[0]=.21
        with self.assertRaisesRegex(RuntimeError,'stale'):monitor.status()
        advanced=state();advanced['robot_time_s']=11.;monitor.publish(advanced);monitor.status()
        with self.assertRaisesRegex(RuntimeError,'wrench'):monitor.publish(dict(advanced,O_F_ext_hat_K=[9,0,0,0,0,0]))
    def test_reader_failure_never_preserves_last_good_proof(self):
        trigger=threading.Event();calls=[0]
        def read():
            calls[0]+=1
            if calls[0]>1:
                trigger.set();raise RuntimeError('synthetic SDK read failed')
            return state()
        monitor=StoppedArmMonitor(read,[0.]*7,.02);self.addCleanup(monitor.close);trigger.wait(.2)
        deadline=time.monotonic()+.3
        while not monitor.error and time.monotonic()<deadline:time.sleep(.005)
        with self.assertRaisesRegex(RuntimeError,'read failed'):monitor.status()

if __name__=='__main__':unittest.main()
