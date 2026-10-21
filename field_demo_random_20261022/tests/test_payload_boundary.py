"""Only synthetic measured fixtures/fake devices; never connects to hardware."""
from contextlib import contextmanager
from datetime import datetime, timezone
import copy
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
import numpy as np
from field_demo.payload_boundary import PayloadBoundaries, combine, object_in_flange, parameters, request_text, execute_boundary


def row(mass, *, frame='flange', center=(0, 0, 0), diagonal=(.001, .001, .001)):
    return dict(units='kg,m,kg*m^2', inertia_about='center_of_mass', frame=frame, mass_kg=mass,
                center_m=list(center), inertia_column_major_kg_m2=np.diag(diagonal).ravel(order='F').tolist(),
                measurement=dict(verified=True, by='synthetic offline fixture',
                                 at_utc=datetime.now(timezone.utc).isoformat(), evidence_file='measured.txt'))


class FakeSite:
    def __init__(self, root):
        self.log_dir=root; self.process=None; self.cancelled=threading.Event(); self.depth=0; self.events=[]
        self.hands={'left':False,'right':False};self.signals={}
        self.grippers={side:SimpleNamespace(status=lambda s=side:dict(is_grasped=self.hands[s])) for side in self.hands}
        self.plc=SimpleNamespace(read=lambda name:self.signals.get(name,False))
        self.config={'robots':{s:{'ip':'192.0.2.'+str(i), 'load_mass_kg':0.} for i,s in enumerate(self.hands,1)}}
    @contextmanager
    def control_lock(self):
        self.depth+=1
        try:yield
        finally:self.depth-=1
    def require_ready(self):
        if self.cancelled.is_set():raise RuntimeError('cancelled')
    def event(self, name, **kw):self.events.append((name,kw))


class PayloadBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        (self.root/'measured.txt').write_text('Synthetic measured-properties fixture; never hardware evidence')
        self.site=FakeSite(self.root);self.calls=[]
        self.profile=dict(schema_version=1,driver_binary='UNUSED_OFFLINE',
            robots={s:dict(tool=row(.73),empty_payload=row(0.,diagonal=(0,0,0))) for s in ('left','right')},
            parts={p:row(.1,frame='part',center=(.01,0,0),diagonal=(1e-5,2e-5,2e-5)) for p in ('ring1','ring2','pin')})
        def apply(site,binary,side,tool,before,after,frame):
            self.assertGreater(site.depth,0);self.calls.append((side,copy.deepcopy(before),copy.deepcopy(after)))
        self.manager=PayloadBoundaries(self.site,self.profile,self.root,apply=apply)
        self.eye=np.eye(4)

    def grasp(self):
        self.site.hands['right']=True;self.site.signals['ring2_grasped']=True
        self.manager.after_grasp('right','ring2',self.eye,self.eye)

    def test_transforms_COM_and_inertia_into_flange_not_TCP_axes(self):
        F=self.eye.copy();F[:3,3]=[.1,0,0]
        G=self.eye.copy();G[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];G[:3,3]=[0,.2,0]
        result=object_in_flange(self.profile['parts']['ring2'],F,G)
        _,center,I=parameters(result)
        np.testing.assert_allclose(center,[.1,.21,0],atol=1e-12)
        np.testing.assert_allclose(np.diag(I),[2e-5,1e-5,2e-5],atol=1e-12)

    def test_hand_is_not_added_to_object_payload(self):
        self.grasp()
        self.assertEqual(self.calls[-1][2]['mass_kg'],.1)
        self.assertEqual(self.profile['robots']['right']['tool']['mass_kg'],.73)

    def test_fresh_held_observation_updates_COM_without_changing_mass_or_tool(self):
        self.grasp();G=self.eye.copy();G[0,3]=.02
        self.manager.refresh_held('right','ring2',self.eye,G)
        self.assertEqual(self.calls[-1][2]['mass_kg'],.1)
        self.assertAlmostEqual(self.calls[-1][2]['center_m'][0],.03)
        self.assertEqual(self.manager.owner['right'],'ring2')

    def test_nonzero_empty_payload_combines_with_parallel_axis_terms(self):
        baseline=row(.2,center=(-.1,0,0));part=row(.1,center=(.1,0,0))
        m,c,I=parameters(combine(baseline,part))
        self.assertAlmostEqual(m,.3);self.assertAlmostEqual(c[0],-1/30)
        self.assertGreater(I[1,1],.002)

    def test_missing_inertia_and_fabricated_zero_inertia_rejected(self):
        for value in (None,[0.]*9):
            profile=copy.deepcopy(self.profile);profile['parts']['pin']['inertia_column_major_kg_m2']=value
            with self.assertRaises((ValueError,TypeError)):
                PayloadBoundaries(self.site,profile,self.root,apply=lambda *args:self.fail('No writes'))

    def test_missing_measurement_rejected_before_driver(self):
        profile=copy.deepcopy(self.profile);profile['parts']['pin']['measurement']['evidence_file']='missing'
        with self.assertRaisesRegex(ValueError,'Nonempty'):
            PayloadBoundaries(self.site,profile,self.root,apply=lambda *args:self.fail('No writes'))

    def test_grasp_ACK_without_actualHand_and_PLC_cannot_add_load(self):
        with self.assertRaisesRegex(RuntimeError,'Fresh actual'):
            self.manager.after_grasp('right','ring2',self.eye,self.eye)
        self.assertEqual(self.calls,[])

    def test_retained_release_requires_all_three_fixture_signals(self):
        self.grasp();self.site.hands['right']=False
        self.site.signals.update(ring2_seated=True,ring2_clamped=True,ring2_retained=False)
        with self.assertRaisesRegex(RuntimeError,'Confirmed seat'):
            self.manager.after_retained_release('right','ring2',self.eye)
        self.site.signals['ring2_retained']=True
        self.manager.after_retained_release('right','ring2',self.eye)
        self.assertIsNone(self.manager.owner['right']);self.assertEqual(self.calls[-1][2]['mass_kg'],0.)

    def test_transfer_cannot_switch_load_before_donor_release(self):
        self.grasp();self.site.hands['left']=True;self.site.signals.update(donor=True,receiver=True)
        with self.assertRaisesRegex(RuntimeError,'Confirmed donor release'):
            self.manager.after_transfer_release('right','left','ring2',{'left':self.eye,'right':self.eye},self.eye,'donor','receiver')
        self.assertEqual(len(self.calls),1)

    def test_transfer_updates_receiver_then_empty_donor_under_one_cell_lock(self):
        self.grasp();self.site.hands.update(right=False,left=True);self.site.signals.update(donor=False,receiver=True)
        self.manager.after_transfer_release('right','left','ring2',{'left':self.eye,'right':self.eye},self.eye,'donor','receiver')
        self.assertEqual([c[0] for c in self.calls],['right','left','right'])
        self.assertEqual(self.manager.owner,{'left':'ring2','right':None})
        self.assertEqual(self.calls[-1][2]['mass_kg'],0.)

    def test_partial_transfer_write_failure_inhibits_both_arms_without_rollback(self):
        self.grasp();self.site.hands.update(right=False,left=True);self.site.signals.update(donor=False,receiver=True)
        def apply(site,binary,side,*args):
            if side=='right':raise RuntimeError('readback unavailable')
        self.manager.apply=apply
        with self.assertRaisesRegex(RuntimeError,'readback unavailable'):
            self.manager.after_transfer_release('right','left','ring2',{'left':self.eye,'right':self.eye},self.eye,'donor','receiver')
        self.assertFalse(self.manager.valid);self.assertTrue(self.site.cancelled.is_set())

    def test_profile_or_evidence_change_inhibits_further_use(self):
        self.manager.profile['parts']['ring2']['mass_kg']=.2
        self.site.hands['right']=True;self.site.signals['ring2_grasped']=True
        with self.assertRaisesRegex(RuntimeError,'profile changed'):
            self.manager.after_grasp('right','ring2',self.eye,self.eye)
        self.assertFalse(self.manager.valid);self.assertEqual(self.calls,[])

    def test_bin_return_empty_load_requires_current_actual_wide_open_hand(self):
        self.grasp();self.site.hands['right']=False
        proof=dict(verified=True,source_capture_ids=['actual-view-a','actual-view-b'])
        self.site.grippers['right'].status=lambda:dict(is_grasped=False,width_m=.002)
        with self.assertRaisesRegex(RuntimeError,'empty-Hand proof'):self.manager.after_bin_return_release('right','ring2',self.eye,proof)
        self.assertEqual(self.manager.owner['right'],'ring2')
        self.site.grippers['right'].status=lambda:dict(is_grasped=False,width_m=.08)
        self.manager.after_bin_return_release('right','ring2',self.eye,proof)
        self.assertIsNone(self.manager.owner['right'])

    def test_native_protocol_handshake_and_readback_with_fake_executable(self):
        binary=self.root/'fake-driver'
        binary.write_text('#!/usr/bin/python3\nimport sys,json\nr=open(sys.argv[-1]).read().splitlines()\na=[float(x) for x in r[3].split()]\nprint("PAYLOAD_READY_REQUIRED",flush=True)\nassert sys.stdin.readline()=="READY\\n"\nprint(json.dumps(dict(payload_set_and_read_back=True,tool_unchanged=True,mass_kg=a[0],F_x_Cload_m=a[1:4],I_load_column_major_kg_m2=a[4:])),flush=True)\n')
        binary.chmod(0o755)
        tool=self.profile['robots']['right']['tool'];empty=self.profile['robots']['right']['empty_payload'];after=object_in_flange(self.profile['parts']['ring2'],self.eye,self.eye)
        result=execute_boundary(self.site,binary,'right',tool,empty,after,self.eye)
        self.assertTrue(result['payload_set_and_read_back']);self.assertEqual(self.site.config['robots']['right']['load_mass_kg'],.1)

    def test_protocol_rejects_total_load_and_nonphysical_inertia(self):
        tool=self.profile['robots']['right']['tool'];empty=self.profile['robots']['right']['empty_payload']
        with self.assertRaisesRegex(ValueError,'three-kg'):
            request_text(tool,empty,row(2.5),self.eye)
        with self.assertRaisesRegex(ValueError,'triangle'):
            parameters(row(.1,diagonal=(1e-5,1e-5,1e-3)))


if __name__=='__main__':unittest.main()
