"""An edited or reused skill artifact must never reach the simulator backend."""
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
import hashlib,json,unittest,io,subprocess
from contextlib import redirect_stdout
from .adaptive import main
from .adaptive import M80Skill


class ArtifactTests(unittest.TestCase):
    def write_plan(self,root,files):
        root.mkdir(parents=True,exist_ok=True)
        for name,data in files.items():(root/name).write_bytes(data)
        (root/'skill-plan.json').write_text(json.dumps(dict(state='offline_screened_requires_PhysX',
            hardware_ready=False,files={n:hashlib.sha256(b).hexdigest()for n,b in files.items()})))

    def controller_plan(self, root, requested, profile):
        self.write_plan(root, {'fr3-concurrent-profile.json': json.dumps(profile).encode()})
        path = root/'skill-plan.json'
        report = json.loads(path.read_text())
        report['experimental_controller'] = requested
        path.write_text(json.dumps(report))

    def test_unknown_or_inconsistent_controller_contract_never_launches_native(self):
        for requested, profile in [({'unimplemented_feature':True}, {'unimplemented_feature':True}),
                                   ({'pin_entry_raw_pose':True}, {'pin_entry_raw_pose':False}),
                                   ({'pin_entry_raw_pose':1}, {'pin_entry_raw_pose':1})]:
            with self.subTest(requested=requested), TemporaryDirectory() as directory:
                root=Path(directory);plan=root/'plan'
                self.controller_plan(plan,requested,profile)
                with patch('subprocess.run') as backend:
                    with self.assertRaises(ValueError):
                        M80Skill(root/'baseline',plan).run_physx(root/'run',initial_scene=root/'truth.json')
                    backend.assert_not_called()

    def test_older_native_ignoring_a_requested_controller_option_cannot_be_credited(self):
        for reported in [{}, {'pin_entry_raw_pose':False}, {'pin_entry_raw_pose':1}, None]:
            with self.subTest(reported=reported), TemporaryDirectory() as directory:
                root=Path(directory);plan=root/'plan';run=root/'run'
                self.controller_plan(plan,{'pin_entry_raw_pose':True},{'pin_entry_raw_pose':True})
                def native(*args,**kwargs):
                    run.mkdir();(run/'fr3-concurrent-profile.json').write_bytes((plan/'fr3-concurrent-profile.json').read_bytes())
                    if reported is not None:(run/'feedback-parameters.json').write_text(json.dumps(reported))
                    return subprocess.CompletedProcess(args[0],0)
                with patch('isaac_port.fieldkit.adaptive.REPO',root),patch('isaac_port.fieldkit.adaptive.PORT',root/'isaac_port'),patch('subprocess.run',side_effect=native) as backend:
                    with self.assertRaisesRegex(RuntimeError,'Native (controller|did not acknowledge)'):
                        M80Skill(root/'baseline',plan).run_physx(run,initial_scene=root/'truth.json')
                    self.assertEqual(backend.call_count,1)
                self.assertTrue((run/'observed-plan/skill-plan.json').exists())

    def test_matching_native_controller_options_reach_independent_audits(self):
        with TemporaryDirectory() as directory:
            root=Path(directory);plan=root/'plan';run=root/'run'
            requested={'pin_entry_raw_pose':True,'pin_feedback_mode':'grasp_compensated'}
            self.controller_plan(plan,requested,requested)
            def native(*args,**kwargs):
                run.mkdir();(run/'fr3-concurrent-profile.json').write_bytes((plan/'fr3-concurrent-profile.json').read_bytes())
                (run/'feedback-parameters.json').write_text(json.dumps(requested))
                return subprocess.CompletedProcess(args[0],0)
            def audit(script,*args,**kwargs):
                if script=='evaluate_contact.py':(run/'acceptance.json').write_text('{"state":"test_audit_result","hardware_ready":false}')
            with patch('isaac_port.fieldkit.adaptive.REPO',root),patch('isaac_port.fieldkit.adaptive.PORT',root/'isaac_port'),patch('subprocess.run',side_effect=native),patch.object(M80Skill,'_run',side_effect=audit) as audits:
                result=M80Skill(root/'baseline',plan).run_physx(run,initial_scene=root/'truth.json')
                self.assertEqual(audits.call_count,4)
                self.assertEqual(result['state'],'test_audit_result')
                self.assertFalse(result['hardware_ready'])

    def test_plan_cannot_hash_or_archive_a_member_outside_its_directory(self):
        with TemporaryDirectory()as directory:
            root=Path(directory);external=root/'external.json';external.write_text('{}')
            plan=root/'plan';plan.mkdir()
            (plan/'skill-plan.json').write_text(json.dumps(dict(state='offline_screened_requires_PhysX',
                files={'../external.json':hashlib.sha256(external.read_bytes()).hexdigest()})))
            with self.assertRaisesRegex(RuntimeError,'Unsafe plan member'):
                M80Skill(root/'baseline',plan).verify_plan()

    def test_native_input_mutation_cannot_be_credited_as_a_screened_run(self):
        with TemporaryDirectory()as directory:
            root=Path(directory);plan=root/'plan';run=root/'run'
            self.write_plan(plan,{'arm-posture-fr3.npz':b'original screened arm path'})
            def native(*args,**kwargs):
                run.mkdir();(run/'arm-posture-fr3.npz').write_bytes(b'different native input')
                return subprocess.CompletedProcess(args[0],0)
            with patch('isaac_port.fieldkit.adaptive.REPO',root),patch('isaac_port.fieldkit.adaptive.PORT',root/'isaac_port'),patch('subprocess.run',side_effect=native)as backend:
                with self.assertRaisesRegex(RuntimeError,'Native input differs'):
                    M80Skill(root/'baseline',plan).run_physx(run,initial_scene=root/'truth.json')
                self.assertEqual(backend.call_count,1)
            self.assertEqual((run/'observed-plan/arm-posture-fr3.npz').read_bytes(),b'original screened arm path')

    def test_failed_native_run_retains_frozen_observation_inputs(self):
        with TemporaryDirectory()as directory:
            root=Path(directory);plan=root/'plan';run=root/'run'
            self.write_plan(plan,{'observations.json':b'{"capture":"test"}'})
            def native(*args,**kwargs):
                run.mkdir();raise subprocess.CalledProcessError(1,args[0])
            with patch('isaac_port.fieldkit.adaptive.REPO',root),patch('isaac_port.fieldkit.adaptive.PORT',root/'isaac_port'),patch('subprocess.run',side_effect=native):
                with self.assertRaises(subprocess.CalledProcessError):
                    M80Skill(root/'baseline',plan).run_physx(run,initial_scene=root/'truth.json')
            self.assertEqual((run/'observed-plan/observations.json').read_bytes(),b'{"capture":"test"}')

    def test_modified_observation_blocks_execution_before_any_subprocess(self):
        with TemporaryDirectory()as directory:
            root=Path(directory);observations=root/'observations.json';observations.write_text('{}\n')
            report=dict(state='offline_screened_requires_PhysX',files={observations.name:hashlib.sha256(observations.read_bytes()).hexdigest()},hardware_ready=False)
            (root/'skill-plan.json').write_text(json.dumps(report))
            skill=M80Skill(root/'baseline',root)
            self.assertFalse(skill.verify_plan()['hardware_ready'])
            observations.write_text('{"changed":true}\n')
            with patch('subprocess.run')as backend:
                with self.assertRaisesRegex(RuntimeError,'Plan changed'):
                    skill.run_physx(root/'new-run',initial_scene=root/'truth.json')
                backend.assert_not_called()

    def test_cli_propagates_conservative_handoff_to_callable_skill(self):
        with patch('isaac_port.fieldkit.adaptive.M80Skill')as cls,redirect_stdout(io.StringIO()):
            cls.return_value.prepare.return_value={'hardware_ready':False}
            main(['--baseline','baseline','--output','new-plan','--snapshot','capture.npz',
                  '--camera-config','camera.json','--replay','--conservative-ring-handoff'])
            self.assertTrue(cls.return_value.prepare.call_args.kwargs['conservative_ring_handoff'])
            self.assertTrue(cls.return_value.prepare.call_args.kwargs['replay'])

    def test_prepare_cannot_overwrite_an_existing_plan(self):
        with TemporaryDirectory()as directory:
            root=Path(directory);(root/'record.txt').write_text('previous measured result')
            with patch('subprocess.run')as backend:
                with self.assertRaises(FileExistsError):
                    M80Skill(root/'baseline',root).prepare(root/'capture.npz',root/'camera.json')
                backend.assert_not_called()
            self.assertEqual((root/'record.txt').read_text(),'previous measured result')


if __name__=='__main__':unittest.main()
