"""Actual-state contract tests use explicitly synthetic samples; no devices."""
import copy
from datetime import datetime,timezone,timedelta
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from field_demo import hand_aperture as h,model_preset as m,collision as c
import test_model_starter as factory

def samples(width=.064):
    return {side:dict(completed=True,is_grasped=side=='right',robot_time_s=10.,captured_at_utc=datetime.now(timezone.utc).isoformat(),width_m=width,max_width_m=.08)for side in ['left','right']}

class ApertureTests(unittest.TestCase):
    def test_real_width_timestamp_bounds_and_drift_stop(self):
        initial=samples();bound=h.binding(initial)
        h.current_within(bound,samples(.0642))
        with self.assertRaisesRegex(RuntimeError,'left the checked'):h.current_within(bound,samples(.065))
        stale=samples();stale['right']['captured_at_utc']=(datetime.now(timezone.utc)-timedelta(seconds=.3)).isoformat()
        with self.assertRaisesRegex(RuntimeError,'stale'):h.binding(stale)
        fake=samples();fake['right']['robot_time_s']=9.
        with self.assertRaisesRegex(RuntimeError,'clock regressed'):h.current_within(bound,fake)
        with self.assertRaises(ValueError):h.binding({'right':samples()['right']})
    def test_open_close_sweep_covers_entire_actual_motion_and_grasp_epsilon(self):
        bound=h.binding(samples(.08),sweep_side='right',target_width_m=.062,epsilon_inner_m=.002,epsilon_outer_m=.002)
        self.assertLessEqual(bound['intervals_m']['right'][0],.0595+1e-12)
        self.assertGreaterEqual(bound['intervals_m']['right'][1],.08)
        for width in np.linspace(.06,.08,21):
            current=samples(.08);current['right']['width_m']=float(width);h.current_within(bound,current)
        changed=copy.deepcopy(bound);changed['intervals_m']['left']=[0,.08]
        with self.assertRaisesRegex(ValueError,'Only actual commanded'):h.validate(changed)
    def test_official_columns_follow_measured_width_and_full_sweep_not_commanded_width_only(self):
        case=factory.StarterTests();case.setUp();self.addCleanup(case.temp.cleanup);case.author()
        env=json.loads((case.site/'random-motion.json').read_text())['robots']['right']
        narrow=h.tools(env,[.0635,.0645]);sweep=h.tools(env,[.0635,.0805]);wide=h.tools(env,[.0795,.0805])
        self.assertEqual([r['name']for r in narrow],[r['name']for r in wide])
        first=next(i for i,r in enumerate(narrow)if r['geometry_role']=='finger')
        self.assertNotEqual(narrow[first]['a'],wide[first]['a'])
        # Every full official finger vertex at all intermediate SDK apertures
        # must be enclosed by the swept named columns.
        with np.load(m.ASSETS/'collision-models.npz')as z:v0=z['finger_vertices']
        T=np.asarray(env['native_hand_aperture_geometry']['T_tcp_hand'])
        for width in np.linspace(.0635,.0805,17):
            for sign,label in [(1,'left_finger'),(-1,'right_finger')]:
                v=v0.copy();v[:,:2]*=sign;v[:,1]+=sign*width/2;v[:,2]+=.0584;v=v@T[:3,:3].T+T[:3,3]
                relevant=[r for r in sweep if r['name'].startswith(label)]
                distances=[c.segment_distance(v,v,r['a'],r['b'])-r['radius_m']for r in relevant]
                self.assertLessEqual(float(np.min(distances,axis=0).max()),0.)
    def test_closed_aperture_passes_real_fixture_clearance_that_full_open_width_blocks(self):
        # Same official geometry and real rigid obstacle; only measured aperture changes.
        case=factory.StarterTests();case.setUp();self.addCleanup(case.temp.cleanup);case.author()
        env=json.loads((case.site/'random-motion.json').read_text())['robots']['right']
        narrow=h.tools(env,[.0635,.0645]);wide=h.tools(env,[.0795,.0805])
        # Place thin plates just outside64mm Hand but inside80mm fingers.
        # Search independent deterministic CAD-aligned boxes, not scaling shapes.
        T=np.asarray(env['native_hand_aperture_geometry']['T_tcp_hand']);found=None
        for y in np.linspace(.059,.068,20):
            plate=np.eye(4);plate[:3,3]=[0,y,.097];plate=T@plate
            obstacle=dict(name='fixed fixture',T_cell_box=plate.tolist(),half_extents_m=[.025,.0005,.005])
            def clearance(rows):return min(float(c.segment_box_distance(r['a'],r['b'],obstacle)-r['radius_m'])for r in rows)
            a,b=clearance(narrow),clearance(wide)
            if a>.0005 and b<0:found=(a,b);break
        self.assertIsNotNone(found,'Need actual official finger width-sensitive fixture example')

if __name__=='__main__':unittest.main()

class StoppedProofTests(unittest.TestCase):
    def test_stopped_sdk_geometry_branch_never_fabricates_a_c2_arm_trace(self):
        import test_random_planner as fixture
        from field_demo import random_planner as planner
        case=fixture.PlannerTests();case.setUp();self.addCleanup(case.doCleanups)
        static=copy.deepcopy(case.other)
        result,trace=planner.geometry_check(static,case.other,case.config,case.site,'pickup','left','ring1',case.observation,[],case.root,{},case.now,contact='grasp',stopped_aperture=True)
        self.assertEqual(trace['sample_count'],1);self.assertFalse(result['independent_c2_curve_bound']);self.assertFalse(result['unsampled_motion_inflation']);self.assertTrue(result['stopped_SDK_aperture_sweep'])
    def test_already_installed_observed_part_blocks_next_pick_before_contact(self):
        import test_random_planner as fixture
        from field_demo import random_planner as planner
        case=fixture.PlannerTests();case.setUp();self.addCleanup(case.doCleanups)
        observed=copy.deepcopy(case.observation);point=case.native['samples'][0]['link_points_base_m'][4]
        observed['installed_parts']=[dict(part='ring1',T_cell_part=fixture.pose(*point),measurement=case.measurement,payload_dimensions=dict(radius_m=.01,length_m=.01,measurement=case.measurement))]
        with self.assertRaisesRegex(c.CollisionError,'Observed loose-part collision'):
            planner.geometry_check(case.native,case.other,case.config,case.site,'pickup','left','ring1',observed,[],case.root,{},case.now,contact='grasp')
        observed['installed_parts'][0]['T_cell_part']=fixture.pose(1.,1.,1.)
        planner.geometry_check(case.native,case.other,case.config,case.site,'pickup','left','ring1',observed,[],case.root,{},case.now,contact='grasp')
