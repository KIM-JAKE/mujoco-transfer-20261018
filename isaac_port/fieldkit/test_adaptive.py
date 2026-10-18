"""An edited or reused skill artifact must never reach the simulator backend."""
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
import hashlib,json,unittest
from .adaptive import M80Skill


class ArtifactTests(unittest.TestCase):
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

    def test_prepare_cannot_overwrite_an_existing_plan(self):
        with TemporaryDirectory()as directory:
            root=Path(directory);(root/'record.txt').write_text('previous measured result')
            with patch('subprocess.run')as backend:
                with self.assertRaises(FileExistsError):
                    M80Skill(root/'baseline',root).prepare(root/'capture.npz',root/'camera.json')
                backend.assert_not_called()
            self.assertEqual((root/'record.txt').read_text(),'previous measured result')


if __name__=='__main__':unittest.main()
