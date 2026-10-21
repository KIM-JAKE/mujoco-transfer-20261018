"""Offline model-authoring/collision contracts; synthetic SDK data, no hardware."""
import copy
from datetime import datetime,timezone
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from field_demo import model_preset as m,model_records as records,model_geometry as g
from field_demo.__main__ import init_random
from field_demo.random_planner import measured

class StarterTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.inst=m.template();self.inst.update(verified=True,by='SYNTHETIC OFFLINE TEST',at_utc=datetime.now(timezone.utc).isoformat(),evidence_file='synthetic-evidence.txt')
        self.inst['facts']=dict(self.inst['expected'],no_unmodeled_adapters=True,cable_radial_allowance_m=.002)
        (self.root/'synthetic-evidence.txt').write_text('Synthetic fixture only; not actual installation proof.')
        self.ip=self.root/'installation.json';self.ip.write_text(json.dumps(self.inst))
        self.F=np.eye(4);self.F[2,3]=.1034
        self.state=dict(read_only=True,server_protocol_version=10,q=[0.]*7,q_d=[0.]*7,dq=[0.]*7,dq_d=[0.]*7,F_T_EE_column_major=self.F.ravel(order='F').tolist(),m_ee_kg=.73,F_x_Cee_m=[0.,0.,.04],I_ee_column_major_kg_m2=(np.eye(3)*.001).ravel().tolist(),m_load_kg=0.,F_x_Cload_m=[0.]*3,I_load_column_major_kg_m2=[0.]*9)
        self.sp=self.root/'SYNTHETIC-sdk.json';self.sp.write_text(json.dumps(self.state))
        self.site=self.root/'site';init_random(self.site)
    def author(self):
        return m.author(self.site,self.ip,self.sp,self.sp)
    def test_generator_bindings_and_tamper_rejection(self):
        result=self.author();self.assertFalse(result['hardware_ready']);self.assertEqual(result['derived_ring_sectors'],128)
        motion=json.loads((self.site/'random-motion.json').read_text());env=motion['robots']['right'];self.assertEqual(env['geometry_backend'],'sdk_link_columns');self.assertGreater(env['tool_capsule_offset'],32)
        record=env['measurement'];self.assertFalse(record['verified']);self.assertEqual(record['kind'],'model_derived')
        records.validate(record,self.site,subject=env)
        tampered=copy.deepcopy(env);tampered['link_capsules_link_m'][0]['radius_m']*=.1
        with self.assertRaisesRegex(ValueError,'geometry differs'):records.validate(record,self.site,subject=tampered)
        self.state['F_T_EE_column_major'][14]+=.01;self.sp.write_text(json.dumps(self.state))
        with self.assertRaisesRegex(ValueError,'source changed'):records.validate(record,self.site,subject=env)
    def test_factory_and_payload_vertices_enclosed_without_filled_ring_bore(self):
        self.author();motion=json.loads((self.site/'random-motion.json').read_text());env=motion['robots']['right']
        with np.load(m.ASSETS/'collision-models.npz')as z:
            from field_demo.collision import segment_distance
            for link in range(8):
                v=z[f'link{link}_vertices'];rows=[r for r in env['link_capsules_link_m']if r['link_index']==link];dist=[]
                for row in rows:dist.append(segment_distance(v,v,row['a'],row['b'])-row['radius_m'])
                self.assertLessEqual(float(np.min(dist,axis=0).max()),0.)
                # Original binary STL triangles, not only their endpoints:
                # deterministic face interiors and edge midpoints must be covered.
                raw=(m.ASSETS/f'link{link}.stl').read_bytes()
                triangles=np.frombuffer(raw[84:],dtype=[('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])['v'].astype(float)
                probes=np.concatenate([triangles.mean(1),(triangles[:,0]+triangles[:,1])/2,(triangles[:,1]+triangles[:,2])/2,(triangles[:,2]+triangles[:,0])/2])
                clear=[segment_distance(probes,probes,row['a'],row['b'])-row['radius_m']for row in rows]
                self.assertLessEqual(float(np.min(clear,axis=0).max()),0.)

            for part in ['ring1','ring2','pin']:
                rows=motion['loose_part_envelopes'][part]['capsules_part_m'];self.assertEqual(len(rows),64 if part.startswith('ring') else 4)
                for i,row in enumerate(rows):
                    v=z[f'{part}_sector{i}'];self.assertLessEqual(float((segment_distance(v,v,row['a'],row['b'])-row['radius_m']).max()),0.)
                if part.startswith('ring'):
                    self.assertTrue(all(np.linalg.norm((np.asarray(r['a'])+r['b'])[:2]/2)>r['radius_m']+.015 for r in rows))
    def test_model_record_never_authorizes_missing_actual_match_or_future_date(self):
        self.inst['verified']=False;self.ip.write_text(json.dumps(self.inst))
        with self.assertRaisesRegex(ValueError,'ONE actual'):self.author()
        self.assertFalse((self.site/'model-preset.json').exists())
    def test_every_generated_family_passes_actual_native_adapter_action_parser(self):
        self.author();cycle=json.loads((self.site/'random-cycle.json').read_text())
        from isaac_port.fieldkit.hand import FrankaHand
        for part in cycle['parts']:
            for family in part['grasp_families']:
                for mode in ['open','release']:FrankaHand.validate_action(family['actions'][mode])
                FrankaHand.validate_action(family['actions']['grasp'],grasp=True)
                self.assertGreater(family['actions']['open']['width_m'],family['actions']['grasp']['width_m']+family['actions']['grasp']['epsilon_outer_m'])

    def test_actual_sdk_tcp_and_native_mount_transform(self):
        self.F[:3,3]=[.01,-.02,.14];self.state['F_T_EE_column_major']=self.F.ravel(order='F').tolist();self.sp.write_text(json.dumps(self.state));self.author()
        motion=json.loads((self.site/'random-motion.json').read_text());cycle=json.loads((self.site/'random-cycle.json').read_text())
        T=np.linalg.inv(self.F)@np.asarray(cycle['inventory_inspection']['native_hand_mount']['right']['T_flange_hand']);self.assertAlmostEqual(float(np.arctan2(T[1,0],T[0,0])),-np.pi/4)
        first=motion['robots']['right']['tool_capsules_tcp_m'][0]
        self.assertTrue(np.isfinite(first['a']).all())
        families=cycle['parts'][0]['grasp_families'];self.assertEqual(len(families),8)
        for family in families:
            P=np.asarray(family['T_part_tcp'])@T
            actual_contact=P[:3,:3]@np.array([0.,0.,.1034])+P[:3,3]
            np.testing.assert_allclose(actual_contact,[0,0,0],atol=1e-10)
            np.testing.assert_allclose(family['pregrasp_offset_part_m'],-P[:3,2]*.06,atol=1e-10)

        payload=json.loads((self.site/'payload-profile.json').read_text())
        from field_demo.payload_boundary import measured as payload_record
        payload_record(payload['parts']['ring2'],self.site)
        self.assertEqual(payload['robots']['right']['tool']['mass_kg'],.73)
        self.assertAlmostEqual(payload['parts']['ring2']['mass_kg'],.0717855177388847)
        self.assertEqual(payload['robots']['right']['empty_payload']['mass_kg'],0)
    def test_actual_per_link_sdk_frames_required_and_checked(self):
        fs=np.tile(np.eye(4),(2,8,1,1));points=fs[:,:,:3,3].copy();tcp=np.tile(np.eye(4),(2,1,1));tcp[:,2,3]=.107+.1034
        self.assertEqual(g.frames(fs,points,tcp,self.F).shape,(2,8,4,4))
        bad=fs.copy();bad[:,7,0,3]=.001
        with self.assertRaisesRegex(ValueError,'link-point'):g.frames(bad,points,tcp,self.F)
        with self.assertRaisesRegex(ValueError,'frames required'):g.frames(None,points,tcp,self.F)
    def test_same_body_or_adjacent_only_no_remote_self_exception(self):
        self.assertTrue(g.self_connected(dict(link_index=2),dict(link_index=2)))
        self.assertTrue(g.self_connected(dict(link_index=7),dict(link_index=8)))
        self.assertFalse(g.self_connected(dict(link_index=2),dict(link_index=4)))
        self.assertFalse(g.self_connected(dict(link_index=5),dict(link_index=7)))

if __name__=='__main__':unittest.main()

class SupportTests(unittest.TestCase):
    def test_only_exact_fixed_base_above_measured_named_top_can_use_support(self):
        from field_demo.mount_support import certify
        plane=dict(obstacle_name='actual_table',normal_cell=[0,0,1],origin_cell_m=[0,0,0],no_base_cables_below_support=True)
        T=np.eye(4);T[2,3]=-.05;obstacle=dict(name='actual_table',T_cell_box=T.tolist(),half_extents_m=[1,1,.05])
        vertices=np.array([[0,0,0],[.1,.1,.2]])
        proof=certify(plane,obstacle,np.eye(4),vertices);self.assertEqual(proof['all_original_base_vertices_checked'],2)
        bad=vertices.copy();bad[0,2]=-.001
        with self.assertRaisesRegex(ValueError,'penetrates'):certify(plane,obstacle,np.eye(4),bad)
        shifted=copy.deepcopy(obstacle);shifted['T_cell_box'][2][3]+=.001
        with self.assertRaisesRegex(ValueError,'top does not match'):certify(plane,shifted,np.eye(4),vertices)

class RuntimeSelfTests(unittest.TestCase):
    def test_nonadjacent_link5_link7_overlap_is_rejected_by_actual_guard(self):
        from field_demo import collision as c
        rows=[]
        for i in range(8):
            p=np.array([[i*2.,0.,0.]])
            rows.append(dict(name='link'+str(i),link_index=i,geometry_role='arm',a=p,b=p.copy(),r=.02))
        rows[7]['a']=rows[5]['a'].copy();rows[7]['b']=rows[5]['b'].copy();rows[0]['actual_tcp']=np.zeros((1,3))
        T=np.eye(4);T[2,3]=10
        workspace=dict(T_cell_box=np.eye(4).tolist(),half_extents_m=[30]*3)
        scene=dict(workspace=workspace,obstacles=[dict(name='far',T_cell_box=T.tolist(),half_extents_m=[.1]*3)])
        phase=dict(corridors=[workspace],tcp_corridor_radius_m=.01)
        env=dict(geometry_backend='sdk_link_columns',self_ignore_pairs=[],payload_robot_ignore_capsules=[])
        with self.assertRaisesRegex(c.CollisionError,'link5/link7'):c.check_envelopes(rows,[],scene,phase,env,margin=.001)
