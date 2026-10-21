"""Synthetic contract tests only: no camera, PLC or robot is connected."""
import copy
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from field_demo.calibration import load_config
from field_demo.motion import candidate, digest, register, target, validate_binding


def transform(x=0., y=0., z=0.):
    return [[1.,0.,0.,x],[0.,1.,0.,y],[0.,0.,1.,z],[0.,0.,0.,1.]]


class MotionContracts(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.camera=self.root/'camera.json';self.grasp=self.root/'grasp.json'
        self.observation=self.root/'latest.json';self.site=self.root/'site.json'
        self.plan=self.root/'pickup-candidate.json';self.review=self.root/'review.json'
        (self.root/'evidence.md').write_text('Synthetic test evidence only. Not hardware calibration or path review.')
        self.camera_data=dict(schema_version=1,units='m',calibration_id='synthetic-test',
            camera_frame='color_optical',cell_frame='cell',camera_serial='TEST-NO-HARDWARE',
            measured=dict(verified=True,by='test fixture',method='synthetic fixture',
                evidence_file='evidence.md',correspondences=4,residual_m=.0005,at_utc='2020-01-01T00:00:00Z'),
            T_cell_camera=transform(.1,0,1),
            robots=dict(left=dict(base_frame='left_base',T_cell_base=[[0.,-1.,0.,1.],[1.,0.,0.,2.],[0.,0.,1.,0.],[0.,0.,0.,1.]]),
                        right=dict(base_frame='right_base',T_cell_base=transform(-1,0,0))),
            nominal_stock_poses={part:transform(.8,2.3,.4) for part in ('ring1','ring2','pin')},
            part_robot=dict(ring1='right',ring2='left',pin='left'),pin_holder_tracks_pin=False,
            max_observation_age_s=2.)
        self.write(self.camera,self.camera_data)
        self.loaded=load_config(self.camera)
        self.grasp_data=dict(schema_version=1,units='m',part='pin',robot='left',
            T_part_tcp=transform(.02,0,.03),measured=dict(verified=True,by='test fixture',
                at_utc='2020-01-01T00:00:00Z',evidence_file='evidence.md'))
        self.write(self.grasp,self.grasp_data)
        self.obs=dict(schema_version=1,units='m',replay=False,calibration_sha256=self.loaded['_calibration_sha256'],
            proposals=dict(pin=dict(robot='left',base_frame='left_base',units='m',replay=False,
                T_cell_part=transform(.8,2.3,.4),
                T_base_part=[[0.,1.,0.,.3],[-1.,0.,0.,.2],[0.,0.,1.,.4],[0.,0.,0.,1.]])))
        self.recapture()
        self.site_data=dict(schema_version=1,motion_binary='/synthetic/m80_site_motion',
            planner_binary='/synthetic/m80_camera_plan',stages={},assembly_commissioned=True,
            robots={side:dict(ip='192.0.2.1' if side=='left' else '192.0.2.2',
                T_cell_base=self.camera_data['robots'][side]['T_cell_base'],tool_mass_kg=.7,load_mass_kg=0.)
                for side in ('left','right')})
        self.write(self.site,self.site_data)
        self.model=dict(read_only=True,motion_executed=False,collision_checked=False,server_protocol_version=10,
            duration_s=5.,q_start=[0.,-.4,0.,-1.8,0.,1.4,.7],q_goal=[0.,-.4,0.,-1.8,0.,1.4,.71],
            state=dict(m_ee_kg=.7,m_load_kg=0.,F_T_EE_column_major=[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,.1,1.]))
        self.native_calls=[]
        self.expected_target=(.3,.18,.43)

    @staticmethod
    def write(path,data):path.write_text(json.dumps(data))

    def recapture(self, *, age=.02, capture_id='capture-a'):
        stamp=(datetime.now(timezone.utc)-timedelta(seconds=age)).isoformat()
        self.obs.update(capture_id=capture_id,captured_at_utc=stamp)
        self.obs['source_metadata']=dict(units='m',camera_frame='color_optical',point_cloud_frame='color_optical',
            calibration_id='synthetic-test',calibration_sha256=self.loaded['_calibration_sha256'],
            source='realsense_live',replay=False,camera_serial='TEST-NO-HARDWARE',frame_number=20,
            capture_id=capture_id,captured_at_utc=stamp,exposure_s=dict(rgb=.005,depth=.005))
        self.obs['proposals']['pin']['source_capture_id']=capture_id
        self.write(self.observation,self.obs)

    def fake_native(self,args,**kwargs):
        self.native_calls.append(args)
        self.assertNotIn('--execute',args)
        if args[1]=='--read-model':
            values=[float(value) for value in Path(args[3]).read_text().split()]
            self.assertEqual(len(values),16)
            for index,expected in zip((3,7,11),self.expected_target):self.assertAlmostEqual(values[index],expected)
            return SimpleNamespace(stdout=json.dumps(self.model))
        self.assertEqual(args[1],'--validate-csv')
        return SimpleNamespace(stdout='')

    def make_candidate(self):
        with patch('field_demo.motion.subprocess.run',side_effect=self.fake_native):
            return candidate(self.observation,self.camera,self.grasp,'pin',self.plan,
                             site=self.site,read_hardware=True)

    def review_candidate(self):
        self.write(self.review,dict(candidate_sha256=digest(self.plan),reviewer='synthetic test reviewer',
            full_path_reviewed=True,checked_actual_cell=True,at_utc='2020-01-01T00:00:00Z'))

    def register_candidate(self):
        self.make_candidate();self.review_candidate()
        register(self.plan,self.site,'left.grasp.pin',self.review,self.observation)
        config=json.loads(self.site.read_text());row=config['stages']['left.grasp.pin']
        return config,row,row['observation_binding']

    def check(self,config,row,binding):
        return validate_binding(binding,self.root,row=row,side='left',site_config=config)

    def test_target_applies_inverse_robot_base_and_measured_tcp_offset(self):
        result=target(self.observation,self.camera,self.grasp,'pin',live=True)
        for actual,expected in zip([row[3] for row in result['T_base_tcp'][:3]],[.3,.18,.43]):
            self.assertAlmostEqual(actual,expected,places=12)
        self.assertEqual(result['robot'],'left')
        self.assertEqual(result['T_base_tcp'][0][:3],[0.,1.,0.])

    def test_live_replay_is_rejected_before_native_subprocess(self):
        self.obs['replay']=True;self.write(self.observation,self.obs)
        with patch('field_demo.motion.subprocess.run',side_effect=AssertionError('must not connect')):
            with self.assertRaises(ValueError):
                candidate(self.observation,self.camera,self.grasp,'pin',self.plan,site=self.site,read_hardware=True)

    def test_target_rejects_wrong_units_robot_base_and_capture_identity(self):
        original=copy.deepcopy(self.obs)
        mutations=[lambda d:d.update(units='mm'),
                   lambda d:d['proposals']['pin'].update(units='mm'),
                   lambda d:d['proposals']['pin'].update(robot='right'),
                   lambda d:d['proposals']['pin'].update(base_frame='cell'),
                   lambda d:d['proposals']['pin'].update(source_capture_id='another'),
                   lambda d:d['source_metadata'].update(capture_id='another'),
                   lambda d:d['source_metadata'].update(captured_at_utc='2020-01-01T00:00:00Z'),
                   lambda d:d['proposals']['pin']['T_base_part'][0].__setitem__(3,.5)]
        for index,mutate in enumerate(mutations):
            with self.subTest(index=index):
                self.obs=copy.deepcopy(original);mutate(self.obs);self.write(self.observation,self.obs)
                with self.assertRaises((ValueError,RuntimeError)):
                    target(self.observation,self.camera,self.grasp,'pin',live=True)

    def test_preview_cannot_register_even_with_review(self):
        result=candidate(self.observation,self.camera,self.grasp,'pin',self.plan)
        self.assertFalse(result['robot_model_read']);self.review_candidate()
        with self.assertRaisesRegex(ValueError,'candidate'):
            register(self.plan,self.site,'left.grasp.pin',self.review,self.observation)

    def test_native_plan_review_register_and_fresh_recapture(self):
        config,row,binding=self.register_candidate()
        self.assertEqual(len(self.native_calls),2)
        self.assertFalse(config['assembly_commissioned'])
        self.assertEqual(self.check(config,row,binding)['part'],'pin')
        self.recapture(capture_id='capture-b')
        result=self.check(config,row,binding)
        self.assertEqual(result['source_capture_id'],'capture-b')
        self.assertTrue((self.root/row['csv']).is_file())

    def test_changed_binding_target_or_identity_is_rejected(self):
        config,row,binding=self.register_candidate()
        for key,value in [('part','ring2'),('robot','right'),('calibration_sha256','changed'),('grasp_sha256','changed')]:
            with self.subTest(key=key):
                modified=copy.deepcopy(binding);modified[key]=value
                with self.assertRaises(ValueError):self.check(config,row,modified)
        modified=copy.deepcopy(binding);modified['T_base_tcp'][0][3]+=.01
        with self.assertRaises(ValueError):self.check(config,row,modified)

    def test_changed_candidate_review_and_csv_are_rejected(self):
        config,row,binding=self.register_candidate()
        for path in (self.plan,self.review,self.root/row['csv']):
            with self.subTest(path=path.name):
                old=path.read_bytes();path.write_bytes(old+b'\n')
                try:
                    with self.assertRaises(ValueError):self.check(config,row,binding)
                finally:path.write_bytes(old)

    def test_changed_live_stage_context_is_rejected(self):
        config,row,binding=self.register_candidate()
        for key,value in [('robot_ip','192.0.2.99'),('sha256','changed'),('review_sha256','changed'),
                          ('field_path_reviewed',False),('F_T_EE_column_major',[0.]*16)]:
            with self.subTest(key=key):
                edited=copy.deepcopy(row);edited[key]=value
                with self.assertRaises(ValueError):self.check(config,edited,binding)
        with self.assertRaises(ValueError):validate_binding(binding,self.root)
        wrong_site=copy.deepcopy(config);wrong_site['robots']['left']['T_cell_base'][0][3]+=.1
        with self.assertRaises(ValueError):self.check(wrong_site,row,binding)

    def test_stale_or_moved_observation_blocks_unchanged_reviewed_path(self):
        config,row,binding=self.register_candidate()
        self.recapture(age=10.,capture_id='old')
        with self.assertRaisesRegex(RuntimeError,'stale'):self.check(config,row,binding)
        self.recapture(capture_id='fresh-moved')
        self.obs['proposals']['pin']['T_cell_part'][1][3]+=.005
        self.obs['proposals']['pin']['T_base_part'][0][3]+=.005
        self.write(self.observation,self.obs)
        with self.assertRaisesRegex(RuntimeError,'moved'):self.check(config,row,binding)

    def test_register_rejects_wrong_policy_arm_mapping(self):
        report=self.make_candidate()
        report['target']['robot']='right';report['robot_ip']='192.0.2.2'
        self.write(self.plan,report);self.review_candidate()
        with self.assertRaisesRegex(ValueError,'mapping|policy|arm'):
            register(self.plan,self.site,'right.grasp.pin',self.review,self.observation)

    def alternate_pickup_arm(self):
        self.camera_data['part_robot']['pin']='right'
        self.camera_data['robots']['right']['T_cell_base']=transform(.5,2.,0.)
        self.write(self.camera,self.camera_data);self.loaded=load_config(self.camera)
        self.obs['calibration_sha256']=self.loaded['_calibration_sha256']
        self.obs['proposals']['pin'].update(robot='right',base_frame='right_base',T_base_part=transform(.3,.3,.4))
        self.grasp_data['robot']='right';self.write(self.grasp,self.grasp_data)
        self.site_data['robots']['right']['T_cell_base']=transform(.5,2.,0.)
        self.write(self.site,self.site_data);self.recapture();self.expected_target=(.32,.3,.43)
        self.make_candidate();self.review_candidate()
        recipe=self.root/'cycle.json'
        self.write(recipe,dict(schema_version=1,parts=[dict(part='ring1',pickup_side='right',pickup_stage='right.grasp.ring1'),
            dict(part='ring2',pickup_side='left',pickup_stage='left.grasp.ring2'),
            dict(part='pin',pickup_side='right',pickup_stage='right.grasp.pin')]))
        return recipe

    def test_explicit_cycle_pickup_mapping_registers_alternate_arm(self):
        recipe=self.alternate_pickup_arm()
        result=register(self.plan,self.site,'right.grasp.pin',self.review,self.observation,cycle_recipe=recipe)
        self.assertEqual(result['state'],'reviewed_pickup_registered')
        config=json.loads(self.site.read_text());row=config['stages']['right.grasp.pin']
        fresh=validate_binding(row['observation_binding'],self.root,row=row,side='right',site_config=config)
        self.assertEqual(fresh['robot'],'right')
        self.assertAlmostEqual(fresh['T_base_tcp'][0][3],.32)

    def test_explicit_cycle_arm_or_stage_mismatch_rejects_even_reviewed_candidate(self):
        recipe=self.alternate_pickup_arm();original=json.loads(recipe.read_text())
        before=self.site.read_bytes()
        for key,value in [('pickup_side','left'),('pickup_stage','left.grasp.pin')]:
            with self.subTest(key=key):
                modified=copy.deepcopy(original);modified['parts'][2][key]=value
                self.write(recipe,modified)
                with self.assertRaisesRegex(ValueError,'explicit cycle'):
                    register(self.plan,self.site,'right.grasp.pin',self.review,self.observation,cycle_recipe=recipe)
                self.assertEqual(self.site.read_bytes(),before)

    def test_missing_review_or_changed_site_blocks_registration(self):
        self.make_candidate();self.review_candidate()
        review=json.loads(self.review.read_text());review['full_path_reviewed']=False
        self.write(self.review,review)
        with self.assertRaises(ValueError):register(self.plan,self.site,'left.grasp.pin',self.review,self.observation)
        self.review_candidate()
        altered=json.loads(self.site.read_text());altered['assembly_commissioned']=False;self.write(self.site,altered)
        with self.assertRaisesRegex(ValueError,'configuration changed'):
            register(self.plan,self.site,'left.grasp.pin',self.review,self.observation)


if __name__=='__main__':unittest.main()
