"""Conservative full hull/world box regressions; archived SIM fixture, no I/O."""
import copy,json,hashlib,unittest
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
from field_demo import convex_self as m,collision as c
import test_convex_self as fixture

class WorldTests(unittest.TestCase):
    def setUp(self):
        self.data=json.loads((Path(__file__).parent/'fixtures/convex-world-postfloor587.json').read_text())
    def fixture(self,name='swap-0.json'):
        x=copy.deepcopy(self.data[name]);f=np.asarray(x['whole_body_frames']);f.flags.writeable=False
        ctx=m.ConvexSelfContext(f,{int(k):v for k,v in x['inflation_m'].items()},x['frame_sha256'],x['numeric_padding_m'],'right');cap=x['capsule']
        for k in ['a','b']:cap[k]=np.asarray(cap[k])
        return ctx,cap,x['obstacle']
    def test_original_merged_box_preserved_nominal_swapped_hull_clears(self):
        ctx,cap,box=self.fixture();original=copy.deepcopy(box);self.assertIsNone(m.refine_world({},cap,box,.001))
        proof=m.refine_world({m.WORLD_KEY:{'right':ctx}},cap,box,.001);self.assertGreater(proof['clearance_m'],.0073);self.assertTrue(proof['occupancy_unchanged']);self.assertEqual(original,box)
        self.assertAlmostEqual(proof['body_inflation_m'],.002);self.assertEqual(proof['original_margin_m'],.001);self.assertEqual(proof['numeric_padding_m'],.0005)
    def test_full_merged_overlap_and_true_unknown_inside_hull_always_reject(self):
        ctx,cap,box=self.fixture('original-0.json');self.assertIsNone(ctx.refine_box(cap,box,.001))
        ctx,cap,box=self.fixture();shape=m._shapes()[5];vertices=np.load(m.ASSETS/'hulls.npz')['link5_vertices'];center=vertices.mean(0)@ctx.frames[0,5,:3,:3].T+ctx.frames[0,5,:3,3];T=np.eye(4);T[:3,3]=center;inside=dict(name='true UNKNOWN inside full hull',source='conservative_registered_RGBD_rays',occupancy_kind='unknown',T_cell_box=T.tolist(),half_extents_m=[.00001]*3)
        self.assertIsNone(ctx.refine_box(cap,inside,.001));self.assertEqual(inside['occupancy_kind'],'unknown')
    def test_full_link_tracking_time_inflation_and_later_FK_collision_cannot_clear(self):
        ctx,cap,box=self.fixture('swap-0.005.json');self.assertGreater(ctx.inflation_m[5],.02);self.assertIsNone(ctx.refine_box(cap,box,.001))
        ctx,cap,box=self.fixture();fs=np.repeat(ctx.frames,2,axis=0);fs[1,5,:3,3]=np.asarray(box['T_cell_box'])[:3,3];later=m.ConvexSelfContext(fs,ctx.inflation_m,'different_FK',ctx.padding_m,'right');self.assertIsNone(later.refine_box(cap,box,.001));self.assertGreaterEqual(later.queries,2)
    def test_world_optin_requires_same_actual_model_binding(self):
        f=fixture.ConvexTests(methodName='runTest');f.setUp();self.addCleanup(f.doCleanups);config=f.config
        config['convex_world_narrowphase']=dict(model_sha256=m.MODEL_SHA256)
        with self.assertRaisesRegex(ValueError,'SAME explicit'):m.validate_optin(config,f.root,{},datetime.now(timezone.utc))
        config['convex_self_narrowphase']=dict(model_sha256=m.MODEL_SHA256);self.assertTrue(m.validate_optin(config,f.root,{},datetime.now(timezone.utc)))
        config['convex_world_narrowphase']['model_sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'SAME explicit'):m.validate_optin(config,f.root,{},datetime.now(timezone.utc))
    def test_changed_box_no_cache_reuse_fingers_payload_and_forged_tokens_reject(self):
        ctx,cap,box=self.fixture();env={m.WORLD_KEY:{'right':ctx}};self.assertIsNotNone(m.refine_world(env,cap,box,.001));changed=copy.deepcopy(box);changed['half_extents_m']=[3]*3;self.assertIsNone(m.refine_world(env,cap,changed,.001))
        for row in [dict(cap,geometry_role='finger',link_index=8,hand_body='left_finger'),dict(cap,geometry_role='payload'),dict(cap,robot_side='left')]:self.assertIsNone(m.refine_world(env,row,box,.001))
        self.assertIsNone(m.refine_world({m.WORLD_KEY:{'right':dict(verified=True)}},cap,box,.001))
    def test_actual_collision_hook_enabled_only_by_validated_context(self):
        ctx,cap,box=self.fixture();active=[cap]
        for i in range(1,8):
            p=np.array([[3.*i,0,0.]])
            active.append(dict(name='far_'+str(i),link_index=0,geometry_role='arm',robot_side='right',a=p,b=p.copy(),r=.001))
        active[0]['actual_tcp']=np.zeros((1,3));W=dict(T_cell_box=np.eye(4).tolist(),half_extents_m=[100]*3);scene=dict(workspace=W,obstacles=[box]);phase=dict(corridors=[W],tcp_corridor_radius_m=.01);env=dict(geometry_backend='sdk_link_columns',self_ignore_pairs=[],payload_robot_ignore_capsules=[])
        with self.assertRaisesRegex(c.CollisionError,'intersects measured obstacle'):c.check_envelopes(active,[],scene,phase,env,margin=.001)
        env[m.WORLD_KEY]={'right':ctx};answer=c.check_envelopes(active,[],scene,phase,env,margin=.001);self.assertGreater(answer['minimum_clearance_m'],0);self.assertTrue(ctx.accepted_world_boxes)
        active[0]=dict(cap,geometry_role='finger',link_index=8,hand_body='left_finger')
        with self.assertRaisesRegex(c.CollisionError,'intersects measured obstacle'):c.check_envelopes(active,[],scene,phase,env,margin=.001)
if __name__=='__main__':unittest.main()
