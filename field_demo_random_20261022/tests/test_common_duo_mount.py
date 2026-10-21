"""Synthetic offline contracts only. No actual mount/SDK/hardware evidence."""
import copy,json
from pathlib import Path
from datetime import datetime,timezone
import unittest
import numpy as np
from field_demo import common_duo_mount as m,collision as c
import test_model_starter as fixture

class CommonDuoTests(unittest.TestCase):
    def setUp(self):
        f=fixture.StarterTests(methodName='runTest');f.setUp();self.addCleanup(f.doCleanups);f.author();self.root=f.site
        self.config=json.loads((self.root/'random-motion.json').read_text());T=m.model()['T_left_base_right_base']
        self.site=dict(robots=dict(left=dict(ip='192.0.2.1',T_cell_base=np.eye(4).tolist()),right=dict(ip='192.0.2.2',T_cell_base=T)))
        self.sitepath=self.root/'SYNTHETIC-site.json';self.sitepath.write_text(json.dumps(self.site));self.proof=self.root/'SYNTHETIC-common-proof.json'
        m.prepare(self.root/'random-motion.json',self.sitepath,self.proof)
    def verified(self):
        p=json.loads(self.proof.read_text());p.update(verified=True,actual_common_rigid_mount_installed=True,by='SYNTHETIC OFFLINE TEST ONLY',at_utc=datetime.now(timezone.utc).isoformat(),evidence_file='SYNTHETIC-common-evidence.txt')
        (self.root/'SYNTHETIC-common-evidence.txt').write_text('Synthetic protocol fixture. NOT actual common mount evidence.');self.proof.write_text(json.dumps(p))
        self.config['common_duo_mount']=dict(proof_file=self.proof.name,proof_sha256=m.digest(self.proof),evidence_sha256=m.digest(self.root/'SYNTHETIC-common-evidence.txt'))
        return m.binding(self.config,self.site,self.root,{},datetime.now(timezone.utc))
    def rows(self,first=0,second=0):
        def row(side,i):return dict(name=side+'_link'+str(i),link_index=i,geometry_role='arm',robot_side=side,speed_bound_m_s=0.,a=np.array([[0.,0,0]]),b=np.array([[0.,0,0]]),r=.02)
        a=row('right',first);a['actual_tcp']=np.zeros((1,3));rows=[a]
        for i in range(1,8):
            r=row('right',i);r['a']=np.array([[2.*i,0.,0.]]);r['b']=r['a'].copy();rows.append(r)
        return rows,[row('left',second)]
    def check(self,binding,first=0,second=0):
        a,b=self.rows(first,second);W=dict(T_cell_box=np.eye(4).tolist(),half_extents_m=[30]*3);T=np.eye(4);T[2,3]=10
        scene=dict(workspace=W,obstacles=[dict(name='far',T_cell_box=T.tolist(),half_extents_m=[.1]*3)])
        env=dict(geometry_backend='sdk_link_columns',self_ignore_pairs=[],payload_robot_ignore_capsules=[])
        if binding is not None:env[m.KEY]=binding
        return c.check_envelopes(a,b,scene,dict(corridors=[W],tcp_corridor_radius_m=.01),env,margin=.001)
    def test_default_independent_arms_strict_and_unverified_not_enabled(self):
        self.assertIsNone(m.binding(self.config,self.site,self.root,{},datetime.now(timezone.utc)))
        with self.assertRaisesRegex(c.CollisionError,'Other-arm collision'):self.check(None)
        self.config['common_duo_mount']=dict(proof_file=self.proof.name,proof_sha256=m.digest(self.proof),evidence_sha256='0'*64)
        with self.assertRaisesRegex(ValueError,'Actual verified'):m.binding(self.config,self.site,self.root,{},datetime.now(timezone.utc))
    def test_only_verified_static_link0_pair_may_share_fixed_assembly(self):
        token=self.verified();checked=self.check(token);self.assertGreater(checked['common_duo_fixed_base_pairs_excluded'],0)
        for a,b in [(0,1),(1,0),(1,1),(5,7)]:
            with self.subTest(pair=(a,b)),self.assertRaisesRegex(c.CollisionError,'Other-arm collision'):self.check(token,a,b)
        a,b=self.rows();a[0]['speed_bound_m_s']=.001
        self.assertFalse(m.fixed_pair_allowed({m.KEY:token},a[0],b[0]))
        b[0]['geometry_role']='payload';self.assertFalse(m.fixed_pair_allowed({m.KEY:token},a[0],b[0]))
    def test_json_flag_or_wrong_side_is_not_a_validated_binding(self):
        a,b=self.rows();self.assertFalse(m.fixed_pair_allowed({m.KEY:dict(verified=True)},a[0],b[0]));token=self.verified();b[0]['robot_side']='right';self.assertFalse(m.fixed_pair_allowed({m.KEY:token},a[0],b[0]))
    def test_actual_site_base_change_rejects_even_inside_factory_match_tolerance(self):
        self.verified();site=copy.deepcopy(self.site);site['robots']['right']['T_cell_base'][0][3]+=.00001
        with self.assertRaisesRegex(ValueError,'base placement differs'):m.binding(self.config,site,self.root,{},datetime.now(timezone.utc))
        site=copy.deepcopy(self.site);site['robots']['right']['T_cell_base'][0][3]+=.01
        with self.assertRaisesRegex(ValueError,'does not match'):m.binding(self.config,site,self.root,{},datetime.now(timezone.utc))
    def test_model_preset_or_installation_change_invalidates_proof(self):
        self.verified();altered=copy.deepcopy(self.config);altered['robots']['right']['link_capsules_link_m'][0]['radius_m']+=.000001
        with self.assertRaisesRegex(ValueError,'Numeric model geometry'):m.binding(altered,self.site,self.root,{},datetime.now(timezone.utc))
        p=self.root/'model-preset.json';p.write_text(p.read_text()+'\n')
        with self.assertRaisesRegex(ValueError,'preset changed'):m.binding(self.config,self.site,self.root,{},datetime.now(timezone.utc))
    def test_source_proof_evidence_edit_is_rejected_and_activation_is_new_file(self):
        self.verified();out=self.root/'motion-common-duo.json';m.activate(self.root/'random-motion.json',self.sitepath,self.proof,out)
        settings=json.loads(out.read_text());self.assertEqual(settings['joint_tracking_error_limit_rad'],.02);self.assertIsNotNone(m.binding(settings,self.site,self.root,{},datetime.now(timezone.utc)))
        with self.assertRaises(ValueError):m.activate(self.root/'random-motion.json',self.sitepath,self.proof,out)
        evidence=self.root/'SYNTHETIC-common-evidence.txt';saved=evidence.read_text();evidence.write_text('changed actual source bytes')
        with self.assertRaisesRegex(ValueError,'evidence is missing or changed'):m.binding(settings,self.site,self.root,{},datetime.now(timezone.utc))
        evidence.write_text(saved)
        self.proof.write_text(self.proof.read_text()+'\n')
        with self.assertRaisesRegex(ValueError,'proof changed'):m.binding(settings,self.site,self.root,{},datetime.now(timezone.utc))
if __name__=='__main__':unittest.main()
