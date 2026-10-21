"""Synthetic RGB-D/SDK-shaped math and command admission; no hardware."""
import copy,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
from datetime import datetime,timezone,timedelta
import numpy as np
from scipy.spatial.transform import Rotation
from field_demo import pin_precision as p
from field_demo.random_cycle import LiveRuntime

def T(x=0,y=0,z=0):
    t=np.eye(4);t[:3,3]=[x,y,z];return t

def state(actual,command=None,t=1.):
    command=actual if command is None else command
    return dict(read_only=True,server_protocol_version=10,robot_mode_name='idle',robot_time_s=t,q=[0]*7,q_d=[0]*7,dq=[0]*7,dq_d=[0]*7,O_F_ext_hat_K=[0]*6,F_T_EE_column_major=np.eye(4).ravel(order='F').tolist(),O_T_EE_column_major=actual.ravel(order='F').tolist(),O_T_EE_desired_model_column_major=command.ravel(order='F').tolist())

class PinPrecisionTests(unittest.TestCase):
    def test_axis_origin_and_arbitrary_fixture_orientation(self):
        goal=T(.1,.2,.8);goal[:3,:3]=Rotation.from_euler('xyz',[.3,.4,.8]).as_matrix();observed=goal.copy();observed[:3,3]-=goal[:3,2]*.045
        e=p.errors(observed,goal);self.assertTrue(e['aligned']);self.assertAlmostEqual(e['remaining_depth_m'],.045)
        np.testing.assert_allclose((observed@p.AXIS_OFFSET)[:3,3],observed[:3,3]+observed[:3,:3]@p.BODY_TO_AXIS_M)
        end,_=p.axial_target(T(.1,.2,.5),observed,goal);np.testing.assert_allclose(end[:3,3],np.array([.1,.2,.5])+goal[:3,2]*.045)
    def test_geometric_feedback_half_step_and_command_actual_bias(self):
        goal=T(z=1.5);observed=T(.002,0,1.455);G=T(z=.02);A=observed@np.linalg.inv(G);C=A.copy();C[0,3]+=.0002
        target,e=p.correction(C,A,G,observed,goal,gain=.5)
        self.assertAlmostEqual(target[0,3],C[0,3]-.001);self.assertAlmostEqual(e['command_vs_actual_translation_m'],.0002)
        np.testing.assert_allclose(target[:3,:3],C[:3,:3])
    def test_precision_not_generic_millimeter_gate_and_signed_axis(self):
        goal=T(z=1.5);bad=T(.00008,0,1.455);self.assertFalse(p.errors(bad,goal)['aligned'])
        with self.assertRaises(RuntimeError):p.axial_target(np.eye(4),bad,goal)
        flipped=T(z=1.455);flipped[:3,:3]=Rotation.from_rotvec([np.pi,0,0]).as_matrix()
        with self.assertRaises(RuntimeError):p.correction(np.eye(4),np.eye(4),np.eye(4),flipped,goal)
    def test_large_bias_or_correction_rejected(self):
        goal=T(z=1.5);A=T(z=1.435);G=T(z=.02);C=A.copy();C[0,3]=.0021
        with self.assertRaises(RuntimeError):p.correction(C,A,G,A@G,goal)
        with self.assertRaises(RuntimeError):p.correction(T(.03,0,1.435),T(.03,0,1.435),G,T(.03,0,1.455),goal)
    def test_actual_encoder_window_uses_robot_clock_and_pose(self):
        rows=[state(T(),t=1+i*.03)for i in range(5)];self.assertIsNotNone(p.encoder_window(rows,.005))
        quick=[state(T(),t=1+i*.01)for i in range(5)];self.assertIsNone(p.encoder_window(quick,.005))
        drifting=copy.deepcopy(rows);drifting[-1]['O_T_EE_column_major']=T(.000026).ravel(order='F').tolist();self.assertIsNone(p.encoder_window(drifting,.005))
        stale=copy.deepcopy(rows);stale[-1]['robot_time_s']=stale[-2]['robot_time_s']
        with self.assertRaises(RuntimeError):p.encoder_window(stale,.005)
        force=copy.deepcopy(rows);force[-1]['O_F_ext_hat_K'][0]=5.01
        with self.assertRaises(RuntimeError):p.encoder_window(force,.005)
    def test_current_payload_G_used_before_every_feedback_move(self):
        with tempfile.TemporaryDirectory()as td:
            r=LiveRuntime.__new__(LiveRuntime);r.root=Path(td);(r.root/'motion.json').write_text(json.dumps(dict(joint_tracking_error_limit_rad=.005)));r.recipe=dict(planner_config='motion.json');r.site=SimpleNamespace(config=dict(robots=dict(right=dict(T_cell_base=np.eye(4).tolist()))));r.event=Mock()
            goal=T(z=1.5);G=T(z=.02);latest={};samples=[];now=datetime.now(timezone.utc)
            for i,x in enumerate([.002,.001,.00005]):
                obj=T(x,0,1.455);A=obj@np.linalg.inv(G);C=A.copy();C[0,3]+=.0002;s=state(A,C,t=1+i)
                s['_read_returned_utc']=(now+timedelta(seconds=i)).isoformat();s['_read_started_utc']=(now+timedelta(seconds=i,milliseconds=50)).isoformat()
                obs=dict(capture_id='FAKE'+str(i),captured_at_utc=(now+timedelta(seconds=i,milliseconds=25)).isoformat(),measured_tcp_pose=A.tolist(),T_cell_part=obj.tolist())
                samples.append((s,obs,dict(T_tcp_part=G.tolist(),version=i)))
            r._pin_precision_window=Mock(side_effect=[(v[0],dict(mocked_SDK=True))for v in samples]);r._pin_precision_sdk=Mock(side_effect=[v[0]for v in samples]);images=iter(samples)
            def image(*args):_,o,c=next(images);return o,c
            def update(c):latest.update(c)
            calls=[]
            def move(*args):self.assertEqual(latest['version'],len(calls));calls.append(args)
            entry=dict(part='pin',insert_side='right',pin_precision_preinsert=dict(enabled=True,gain=.5,max_iterations=8))
            with patch('field_demo.random_cycle.held_measurement',side_effect=image):obs,carry,target=r.precision_pin_preinsert(entry,goal,None,{},move,update)
            self.assertEqual(len(calls),2);self.assertEqual(carry['version'],2);self.assertEqual(latest['version'],2)
            np.testing.assert_allclose(target[:3,:3],np.eye(3));self.assertAlmostEqual(target[2,3],1.48)
    def test_fresh_dispatch_rejects_later_drift_before_sdk_or_motion(self):
        r=LiveRuntime.__new__(LiveRuntime);r._pin_precision_insert_guard=dict(goal=T(z=1.5).tolist(),side='right');r._pin_precision_sdk=Mock()
        with self.assertRaises(RuntimeError):r._pin_precision_dispatch(dict(phase='insert',part='pin',_runtime_side='right'),dict(T_cell_part=T(.0002,0,1.455).tolist()))
        r._pin_precision_sdk.assert_not_called()
    def test_fresh_dispatch_checks_actual_TCP_state_and_age(self):
        with tempfile.TemporaryDirectory()as td:
            r=LiveRuntime.__new__(LiveRuntime);r.root=Path(td);(r.root/'motion.json').write_text(json.dumps(dict(joint_tracking_error_limit_rad=.005)));r.recipe=dict(planner_config='motion.json');r.site=SimpleNamespace(config=dict(robots=dict(right=dict(T_cell_base=np.eye(4).tolist()))));r.event=Mock();r.facts=SimpleNamespace(sdk_grasp=Mock());r.hand_status=Mock(return_value=dict(is_grasped=True,width_m=.011))
            A=T(z=1.435);v=state(A);v['_read_returned_utc']=datetime.now(timezone.utc).isoformat();r._pin_precision_sdk=Mock(return_value=v)
            r._pin_precision_insert_guard=dict(goal=T(z=1.5).tolist(),side='right',q_d=[0]*7,F_T_EE_column_major=np.eye(4).ravel(order='F').tolist())
            plan=dict(phase='insert',part='pin',_runtime_side='right');fresh=dict(capture_id='FAKE-final',T_cell_part=T(.00005,0,1.455).tolist(),measured_tcp_pose=A.tolist())
            with patch('field_demo.random_cycle.check_observation',return_value=fresh)as check:r._pin_precision_dispatch(plan,fresh)
            check.assert_called_once();r.facts.sdk_grasp.assert_called_once();r.event.assert_called_once()
            v['_read_returned_utc']=(datetime.now(timezone.utc)-timedelta(seconds=.21)).isoformat()
            with patch('field_demo.random_cycle.check_observation',return_value=fresh):
                with self.assertRaises(RuntimeError):r._pin_precision_dispatch(plan,fresh)
    def test_command_change_after_alignment_rejects_before_dispatch(self):
        r=LiveRuntime.__new__(LiveRuntime);A=T(z=1.435);v=state(A);v['q_d'][0]=.000002
        r._pin_precision_sdk=Mock(return_value=v);r._pin_precision_insert_guard=dict(goal=T(z=1.5).tolist(),side='right',q_d=[0]*7,F_T_EE_column_major=np.eye(4).ravel(order='F').tolist())
        with self.assertRaises(RuntimeError):r._pin_precision_dispatch(dict(phase='insert',part='pin',_runtime_side='right'),dict(T_cell_part=T(z=1.455).tolist()))

    def test_optin_options_do_not_change_default(self):
        self.assertIsNone(p.options(None));self.assertIsNone(p.options(dict(enabled=False)))
        self.assertEqual(p.options(dict(enabled=True))['gain'],.5)
        with self.assertRaises(ValueError):p.options(dict(enabled=True,max_iterations=9))

if __name__=='__main__':unittest.main()
