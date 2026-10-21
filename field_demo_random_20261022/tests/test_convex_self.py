"""Exact recorded-pose and negative closed-hull tests; no device calls."""
import copy,json,tempfile,unittest,shutil
from pathlib import Path
from datetime import datetime,timezone
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from field_demo import convex_self as m,model_geometry as g,collision as c
import test_model_starter as fixture

class ConvexTests(unittest.TestCase):
    def setUp(self):
        f=fixture.StarterTests(methodName='runTest');f.setUp();self.addCleanup(f.doCleanups);f.author();self.root=f.site;self.config=json.loads((self.root/'random-motion.json').read_text());self.env=self.config['robots']['right']
        self.record=json.loads((Path(__file__).parent/'fixtures/convex-self-actual348.json').read_text());T=np.eye(4);frames=[T.copy()]
        def proper(a):
            a=np.asarray(a).copy();u,_,v=np.linalg.svd(a[:3,:3]);a[:3,:3]=u@v;return a
        for q,axis,(a,b)in zip(self.record['q'],self.record['axes'],self.record['frames']):
            E=np.eye(4);E[:3,:3]=Rotation.from_rotvec(np.asarray(axis)*q).as_matrix();T=T@proper(a)@E@proper(b);frames.append(T.copy())
        self.frames=np.asarray(self.record['base'])[None,None]@np.asarray(frames)[None]
    def rows(self,frames=None,extra=0.):
        frames=self.frames if frames is None else frames;tcp=frames[:,7].copy();tcp[:,2,3]+=.2
        rows=g.robot_capsules(frames,tcp,self.env,np.zeros(9),np.zeros(7),0.,np.zeros(9),np.zeros(7),'right_')
        for row in rows:row['robot_side']='right';row['r']+=extra
        return rows
    def pair(self,rows):return next(r for r in rows if r['name']=='right_link5_column_7_3'),next(r for r in rows if r['name']=='right_link7_column_3_0')
    def test_actual348_proxy_rejects_but_closed_hulls_clear_same_cable_and_margin(self):
        rows=self.rows();a,b=self.pair(rows);clear=float(c.segment_distance(a['a'],a['b'],b['a'],b['b'])[0]-a['r']-b['r']-.001);self.assertLess(clear,0)
        ctx=m.context(self.env,rows,self.frames,'right');proof=ctx.refine(a,b,.001)
        self.assertIsNotNone(proof);self.assertAlmostEqual(proof['inflation_each_m'][0],.002);self.assertAlmostEqual(proof['inflation_each_m'][1],.002);self.assertGreater(proof['clearance_m'],.014)
        self.assertEqual(proof['original_margin_m'],.001);self.assertEqual(proof['numeric_padding_each_m'],.0005)
    def test_true_hull_contact_containment_and_later_curve_collision_reject(self):
        fs=np.repeat(self.frames,2,axis=0);fs[1,7]=fs[1,5];rows=self.rows(fs);a,b=self.pair(rows);ctx=m.context(self.env,rows,fs,'right');self.assertIsNone(ctx.refine(a,b,.001));self.assertGreaterEqual(ctx.queries,2)
        contained=fs[1:];rows=self.rows(contained);a,b=self.pair(rows);self.assertIsNone(m.context(self.env,rows,contained,'right').refine(a,b,.001))
    def test_tracking_and_time_extra_is_maximum_over_every_original_link_column(self):
        rows=self.rows();a,b=self.pair(rows)
        # A different original column carries the largest uncertainty; it must
        # still inflate this link's WHOLE hull rather than just the hit column.
        r=next(r for r in rows if r['link_index']==5 and r is not a);r['r']+=.03
        ctx=m.context(self.env,rows,self.frames,'right');self.assertAlmostEqual(ctx.inflation_m[5],.032);self.assertIsNone(ctx.refine(a,b,.001))
    def test_optional_model_shape_and_original_geometry_bindings_fail_closed(self):
        self.assertFalse(m.validate_optin(self.config,self.root,{},datetime.now(timezone.utc)))
        self.config['convex_self_narrowphase']=dict(model_sha256='0'*64)
        with self.assertRaisesRegex(ValueError,'pinned'):m.validate_optin(self.config,self.root,{},datetime.now(timezone.utc))
        self.config['convex_self_narrowphase']=dict(model_sha256=m.MODEL_SHA256);self.assertTrue(m.validate_optin(self.config,self.root,{},datetime.now(timezone.utc)))
        bad=copy.deepcopy(self.config);bad['robots']['right']['link_capsules_link_m'][0]['radius_m']+=1e-6
        with self.assertRaisesRegex(ValueError,'model geometry'):m.validate_optin(bad,self.root,{},datetime.now(timezone.utc))
        with tempfile.TemporaryDirectory()as td:
            out=Path(td);shutil.copy2(m.ASSETS/'model.json',out/'model.json');(out/'hulls.npz').write_bytes(b'changed shape')
            with patch.object(m,'ASSETS',out),self.assertRaisesRegex(ValueError,'hull shape changed'):m.model()
    def test_FK_dependency_immutable_context_no_cross_pose_clear_cache(self):
        rows=self.rows();a,b=self.pair(rows);input_frames=self.frames.copy();ctx=m.context(self.env,rows,input_frames,'right');self.assertIsNotNone(ctx.refine(a,b,.001));input_frames[:,7]=input_frames[:,5]
        self.assertFalse(np.array_equal(ctx.frames,input_frames));new_rows=self.rows(input_frames);aa,bb=self.pair(new_rows);changed=m.context(self.env,new_rows,input_frames,'right');self.assertNotEqual(ctx.frame_sha256,changed.frame_sha256);self.assertIsNone(changed.refine(aa,bb,.001))
        invalid=self.frames.copy();invalid[0,7,0,0]=2.
        with self.assertRaisesRegex(ValueError,'SDK per-link FK'):m.context(self.env,rows,invalid,'right')
    def test_actual_collision_hook_refines_only_original_self_proxy_rejection(self):
        rows=self.rows();a,b=self.pair(rows);a=copy.deepcopy(a);b=copy.deepcopy(b);active=[a]
        for i in range(1,7):
            p=np.array([[3.*i,0,0.]])
            active.append(dict(name='synthetic_far_'+str(i),link_index=0,geometry_role='arm',robot_side='right',a=p,b=p.copy(),r=.001))
        active.append(b);active[0]['actual_tcp']=np.zeros((1,3))
        W=dict(T_cell_box=np.eye(4).tolist(),half_extents_m=[100]*3)
        T=np.eye(4);T[2,3]=50
        scene=dict(workspace=W,obstacles=[dict(name='far',T_cell_box=T.tolist(),half_extents_m=[.1]*3)])
        phase=dict(corridors=[W],tcp_corridor_radius_m=.01)
        env=dict(geometry_backend='sdk_link_columns',self_ignore_pairs=[],payload_robot_ignore_capsules=[])
        with self.assertRaisesRegex(c.CollisionError,'right_link5_column_7_3/right_link7_column_3_0'):
            c.check_envelopes(active,[],scene,phase,env,margin=.001)
        ctx=m.context(self.env,rows,self.frames,'right');env[m.KEY]=ctx
        answer=c.check_envelopes(active,[],scene,phase,env,margin=.001)
        self.assertGreater(ctx.queries,0);self.assertGreater(answer['minimum_clearance_m'],0);self.assertEqual(len(ctx.accepted_link_pairs),1)
    def test_rigid_palm_original_STL_frame_and_uncertainty_retained(self):
        Z=np.eye(4);Z[2,3]=.107
        F=np.asarray(self.env['F_T_EE_column_major']).reshape(4,4,order='F');tcp=self.frames[:,7]@Z@F
        rows=g.robot_capsules(self.frames,tcp,self.env,np.zeros(9),np.zeros(7),0.,np.zeros(9),np.zeros(7),'right_')
        for row in rows:row['robot_side']='right'
        ctx=m.context(self.env,rows,self.frames,'right')
        np.testing.assert_allclose(ctx.frames[:,8],tcp@np.asarray(self.env['native_hand_aperture_geometry']['T_tcp_hand']))
        rejected=[]
        for a in rows:
            if a.get('link_index')!=5:continue
            for b in rows:
                if b.get('geometry_role')!='palm':continue
                if float(c.segment_distance(a['a'],a['b'],b['a'],b['b'])[0])-a['r']-b['r']-.001<=0:rejected.append((a,b))
        self.assertTrue(rejected)
        for a,b in rejected:self.assertIsNotNone(ctx.refine(a,b,.001))
        palm=next(row for row in rows if row.get('geometry_role')=='palm');palm['r']+=.1
        self.assertIsNone(m.context(self.env,rows,self.frames,'right').refine(*rejected[0],.001))
    def test_other_arm_tools_payload_or_forged_json_cannot_use_refinement(self):
        rows=self.rows();a,b=self.pair(rows);ctx=m.context(self.env,rows,self.frames,'right');env={m.KEY:ctx}
        self.assertIsNone(m.refine({m.KEY:dict(verified=True)},a,b,.001))
        for change in [dict(robot_side='left'),dict(geometry_role='payload'),dict(link_index=8)]:
            changed=dict(b,**change);self.assertIsNone(m.refine(env,a,changed,.001))
if __name__=='__main__':unittest.main()
