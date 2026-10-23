"""Read-only raw-run structure checks and adversarial scope/evidence accounting."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import zipfile


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/audit_native_artifacts.py"
SPEC = importlib.util.spec_from_file_location("native_artifact_audit", SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)
# Archive-dependent legacy simulation checks stay in the original source tree.
# This portable suite retains only fixture-based evidence-accounting checks.


class ArtifactAuditTests(unittest.TestCase):


    def test_missing_or_mock_evidence_never_promotes_pass_flags(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "pipeline-results.json"
            path.write_text(json.dumps({"full_task_pass": True, "part_attachments": "False",
                                       "capx_native_bridge": {"unit_tests_pass": True, "actual_physics_executions": 1, "events": [], "simulation_only": True}}))
            report = AUDIT.audit_native_artifacts(path, expected_scope="full_task")
            self.assertIsNone(report["raw_claims"]["part_attachments"])
            self.assertEqual(report["same_report_three_part_flow_records"], "unverified")
            self.assertEqual(report["issued_commands"]["status"], "unknown")
            self.assertEqual(report["fr3_command_audit"]["status"], "unknown")
            self.assertEqual(report["bridge"]["status"], "unknown_or_inconsistent")
            self.assertFalse(report["qualification"]["motion_pass_inferred"])
            path.write_text(json.dumps({"full_task_pass": True, "pin_only_component": True}))
            with self.assertRaises(AUDIT.ScopeContradiction):
                AUDIT.audit_native_artifacts(path, expected_scope="component")

    def test_header_only_placeholder_command_archive_is_not_issued_motion(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "issued.npz"
            text = repr({"descr": "<f8", "fortran_order": False, "shape": (10, 18)}).encode()
            header_without_array = b"\x93NUMPY\x01\x00" + len(text).to_bytes(2, "little") + text
            with zipfile.ZipFile(path, "w") as archive:
                archive.writestr("command.npy", header_without_array)
            result = AUDIT._issued(path)
            self.assertEqual(result["status"], "unknown")
            self.assertIn("Missing/truncated", result["reason"])


if __name__ == "__main__":
    unittest.main()
