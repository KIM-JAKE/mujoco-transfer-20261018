"""Focused mocked RGB-D/SDK integration. No camera, robot or hardware motion."""
import copy,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from field_demo import pin_pickup as p
from field_demo.random_cycle import select_observation


def T(x=0,y=0,z=0):
    v=np.eye(4);v[:3,3]=[x,y,z];return v


def sdk(A):
    return dict(read_only=True,server_protocol_version=10,robot_mode_name='idle',robot_time_s=1.,q=[0.]*7,q_d=[0.]*7,dq=[0.]*7,dq_d=[0.]*7,O_F_ext_hat_K=[0.]*6,F_T_EE_column_major=np.eye(4).ravel(order='F').tolist(),O_T_EE_column_major=A.ravel(order='F').tolist(),O_T_EE_desired_model_column_major=A.ravel(order='F').tolist())


def image(x=.001):
    item=dict(part='pin',instance_id='mock-pixel-pin',T_cell_part=T(x).tolist(),robust_cap_circle_from_pixels=True,lateral_from_previous_RGBD_grasp_and_encoders=False,fallback_used=False)
    return select_observation(dict(state='random_ring_observation',replay=True,instances=[item],capture_id='mock-only',captured_at_utc='2026-01-01T00:00:00+00:00',calibration_sha256='MOCK',source_metadata=dict(capture_id='mock-only',captured_at_utc='2026-01-01T00:00:00+00:00',calibration_sha256='MOCK'),_scope='pin_holder'),item)


class FakeRuntime:
    """SDK-shaped state and command recorder; explicitly not real execution."""
    def __init__(self,root):
        self.root=root;(root/'motion.json').write_text(json.dumps(dict(joint_tracking_error_limit_rad=.02,phases=dict(pin_empty_park={}))))
        self.recipe=dict(planner_config='motion.json',parts=[],insertion_targets=dict(pin=T(z=1.).tolist()))
        self.replay=True;self.A={'right':T(),'left':T()};self.site=SimpleNamespace(config=dict(robots={s:dict(T_cell_base=T().tolist())for s in self.A}),expected_held={})
        self.payload=SimpleNamespace(owner={'right':'pin','left':None});self.plans=[];self.commands=[];self.events=[];self.hand={'right':dict(is_grasped=False,width_m=.024),'left':dict(is_grasped=False,width_m=.08)};self.current=image();self.block=None
    def ready(self):pass
    def event(self,*args,**kwargs):self.events.append((args,kwargs))
    def hand_status(self,side):return self.hand[side]
    def observe(self,*args,**kwargs):return copy.deepcopy(self.current)
    def _pin_precision_sdk(self,side):return sdk(self.A[side])
    def _selected_record(self,v):return v
    def plan(self,phase,part,side,targets,observation,held,**kwargs):
        if phase==self.block:raise RuntimeError('MOCK actual original geometry rejects')
        v=dict(phase=phase,side=side,targets=targets,held=copy.deepcopy(held),capture=observation['capture_id']);self.plans.append(v);return v
    def execute(self,plan,observation):self.commands.append(plan);self.A[plan['side']]=np.asarray(plan['targets'][-1])


class PinPickupTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.r=FakeRuntime(self.root)
        self.e=dict(part='pin',pickup_side='right',insert_side='right',pin_pickup_feedback=dict(enabled=True,max_corrections=3,gain=.5),insert_entry_offset_part_m=[0,0,-.045]);self.r.recipe['parts']=[self.e]
        self.f=dict(T_part_tcp=T().tolist())
    def tearDown(self):self.tmp.cleanup()
    def test_default_off_has_no_sdk_or_motion(self):
        self.assertIsNone(p.options(None));self.assertIsNone(p.options(dict(enabled=False)))
        e=dict(self.e,pin_pickup_feedback=dict(enabled=False));v=image();self.assertIs(p.preclose(self.r,e,self.f,v),v);self.assertEqual(self.r.plans,[])
    def test_actual_fresh_feedback_replans_three_times_then_admits(self):
        result=p.preclose(self.r,self.e,self.f,image());self.assertEqual(len(self.r.commands),2);self.assertEqual(len(self.r.plans),2)
        self.assertLess(np.linalg.norm(self.r.A['right'][:3,3]-[.001,0,0]),.0003);self.assertEqual(result['capture_id'],'mock-only');self.assertEqual(result['selected_instance']['T_cell_part'],result['instances'][0]['T_cell_part']);self.assertEqual(self.r._pin_pickup_guard['side'],'right')
        for row in self.r.plans:self.assertEqual(row['phase'],'pickup');self.assertEqual(row['held'],[])
    def test_original_collision_failure_never_issues_correction(self):
        self.r.block='pickup'
        with self.assertRaises(RuntimeError):p.preclose(self.r,self.e,self.f,image())
        self.assertEqual(self.r.commands,[])
    def test_no_circle_or_tight_aperture_never_moves(self):
        self.r.current['instances'][0]['robust_cap_circle_from_pixels']=False
        with self.assertRaises(RuntimeError):p.preclose(self.r,self.e,self.f,image())
        self.r.current=image();self.r.hand['right']['width_m']=.014
        with self.assertRaises(RuntimeError):p.preclose(self.r,self.e,self.f,image())
        self.assertEqual(self.r.commands,[])
    def test_changed_target_and_large_sdk_bias_fail_closed(self):
        self.r.current=image(.006)
        with self.assertRaises(RuntimeError):p.preclose(self.r,self.e,self.f,image())
        with self.assertRaises(RuntimeError):p.preclose_target(T(.0021),T(),T(),T())
        with self.assertRaises(ValueError):p.options(dict(enabled=True,max_corrections=4))
    def test_fresh_predispatch_drift_rejects_after_admission(self):
        p.preclose(self.r,self.e,self.f,image());self.r.current=image(.0015)
        with self.assertRaises(RuntimeError):p.dispatch_guard(self.r,'right','pin','grasp',self.r.current)
        self.assertFalse(any('hand' in row for row in self.r.commands))
    def test_dispatch_requires_admission_and_current_direct_pixels(self):
        with self.assertRaises(RuntimeError):p.dispatch_guard(self.r,'right','pin','grasp',image())
        p.preclose(self.r,self.e,self.f,image());p.dispatch_guard(self.r,'right','pin','grasp',image())
        bad=image();bad['selected_instance']['lateral_from_previous_RGBD_grasp_and_encoders']=True
        with self.assertRaises(RuntimeError):p.dispatch_guard(self.r,'right','pin','grasp',bad)
    def setup_park(self):
        evidence=self.root/'field.txt';evidence.write_text('MOCK field configuration only; no hardware test')
        record=dict(verified=True,by='MOCK',at_utc='2026-01-01T00:00:00+00:00',evidence_file='field.txt')
        self.e['pin_pickup_feedback']['other_empty_park']=dict(enabled=True,phase='pin_empty_park',target_offset_base_m=[.2,-.12,.45],measurement=record)
        self.r.site.expected_held={'right':'pin'};self.r.hand['right']['is_grasped']=True
        return dict(part='pin',robot='right',T_tcp_part=T().tolist())
    def test_park_protects_pin_then_uses_attained_actual_state(self):
        carry=self.setup_park();fresh=image()
        with patch('field_demo.random_cycle.held_measurement',return_value=(fresh,carry)):
            p.park_other(self.r,self.e,fresh,carry)
        self.assertEqual([v['phase']for v in self.r.plans],['pin_empty_park','preinsert']);self.assertEqual(len(self.r.commands),1);self.assertEqual(self.r.commands[0]['side'],'left');self.assertEqual(self.r.commands[0]['held'],[carry])
        np.testing.assert_allclose(self.r.A['left'][:3,3],[.2,-.12,.45]);self.assertEqual(self.r.payload.owner['right'],'pin');self.assertFalse(self.r.events[-1][1]['terminal_stroke_proof'])
    def test_occupied_or_not_open_other_arm_cannot_park(self):
        carry=self.setup_park();self.r.payload.owner['left']='ring1'
        with self.assertRaises(RuntimeError):p.park_other(self.r,self.e,image(),carry)
        self.r.payload.owner['left']=None;self.r.hand['left']['width_m']=.01
        with self.assertRaises(RuntimeError):p.park_other(self.r,self.e,image(),carry)
        self.assertEqual(self.r.commands,[])
    def test_blocked_park_never_moves_and_terminal_rejection_retains(self):
        carry=self.setup_park();self.r.block='pin_empty_park'
        with self.assertRaises(RuntimeError):p.park_other(self.r,self.e,image(),carry)
        self.assertEqual(self.r.commands,[]);self.r.block='preinsert'
        with patch('field_demo.random_cycle.held_measurement',return_value=(image(),carry)):
            with self.assertRaises(RuntimeError):p.park_other(self.r,self.e,image(),carry)
        self.assertEqual(len(self.r.commands),1);self.assertEqual(self.r.payload.owner['right'],'pin')
    def test_missing_real_corridor_or_evidence_never_moves(self):
        carry=self.setup_park();(self.root/'field.txt').unlink()
        with self.assertRaises(ValueError):p.park_other(self.r,self.e,image(),carry)
        self.assertEqual(self.r.commands,[])

if __name__=='__main__':unittest.main()
