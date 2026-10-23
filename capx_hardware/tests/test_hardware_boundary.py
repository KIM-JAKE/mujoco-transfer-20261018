import copy
import hashlib
import importlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
from capx_deploy.cli import initialize
_fixture = tempfile.TemporaryDirectory(prefix="capx-backend-release-test-")
A = Path(_fixture.name)
initialize(A/"hardware_configs")
(A/"hardware_configs/hardware.template.json").write_bytes((A/"hardware_configs/hardware.json").read_bytes())
sys.path.insert(0, str(A))
from hardware_v1.backend import HardwareBackend, MotionDisabled
from hardware_v1.config import canonical, sha
from hardware_v1.drivers import DriverLoader


class HardwareBoundary(unittest.TestCase):
    def test_offline_template_never_connects_or_starts_a_process(self):
        with patch.object(socket.socket, "connect", side_effect=AssertionError("network")), patch.object(subprocess, "run", side_effect=AssertionError("process")), patch.object(subprocess, "Popen", side_effect=AssertionError("process")):
            b = HardwareBackend(A/"hardware_configs/hardware.template.json")
            r = b.preflight()
            self.assertFalse(r["local_config_complete"])
            self.assertFalse(r["device_connected"])
            self.assertIn("admission.hardware_qualified", {x["field"] for x in r["blockers"]})
            self.assertIsNone(b._site)
            for call in (lambda: b.execute_stage("right", "none"), lambda: b.hand_move("right", {}), lambda: b.clamp("ring1", True)):
                with self.assertRaises(MotionDisabled):
                    call()

    def test_real_original_hand_caps_are_not_changed(self):
        c = json.loads((A/"hardware_configs/hardware.template.json").read_text())
        m = c["driver_manifest"]
        loader = DriverLoader(A/"hardware_configs"/m["path"], m["sha256"])
        original = loader.module("fieldkit", "hand").FrankaHand
        # Actual existing validate_action, not a reimplementation.
        with self.assertRaises(ValueError):
            original.validate_action(dict(width_m=.012, speed_m_s=.01, force_n=30, epsilon_inner_m=.0005, epsilon_outer_m=.0005), grasp=True)
        original.validate_action(dict(width_m=.012, speed_m_s=.01, force_n=10, epsilon_inner_m=.0005, epsilon_outer_m=.0005), grasp=True)

    def test_relative_and_absolute_driver_manifests_and_tamper(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            (p/"vendor").mkdir()
            (p/"vendor/__init__.py").write_text("")
            (p/"vendor/device.py").write_text("VALUE=42\n")
            for absolute in (False, True):
                files = {str(q) if absolute else str(q.relative_to(p)): sha(q) for q in (p/"vendor").iterdir()}
                manifest = {"packages": {"example": str(p/"vendor") if absolute else "vendor"}, "files": files}
                path = p/("absolute.json" if absolute else "relative.json")
                path.write_text(json.dumps(manifest))
                loader = DriverLoader(path, sha(path))
                self.assertEqual(loader.module("example", "device").VALUE, 42)
                self.assertFalse((p/"vendor/__pycache__").exists())
            (p/"vendor/device.py").write_text("VALUE=43\n")
            with self.assertRaises(ValueError):
                loader.verify()

    def test_empty_or_malformed_manifest_is_not_integrity_success(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/"bad.json"
            for value in ({"files": {}, "packages": {}}, {"files": [], "packages": {}}, []):
                p.write_text(json.dumps(value))
                with self.assertRaises(ValueError):
                    DriverLoader(p, sha(p))

    def make_site(self, root):
        csv = root/"field.csv"
        h = ["time_s"]+[s+str(j) for s in ("q", "dq", "ddq") for j in range(1, 8)]
        csv.write_text(",".join(h)+"\n"+",".join(map(str, [0]+[0]*21))+"\n"+",".join(map(str, [2]+[0]*21))+"\n")
        site = {"schema_version": 1, "stages": {"right.carry.ring1": {"side": "right", "csv": "field.csv", "sha256": sha(csv), "field_path_reviewed": True, "tool_type": "franka_hand"}}}
        p = root/"site.json"
        p.write_text(json.dumps(site))
        cfg = {"schema": "capx-fr3-hardware/v1", "site_config": "site.json", "site_config_sha256": sha(p), "motion_enabled": False, "task_source_sha256": "c"*64}
        c = root/"hardware.json"
        c.write_text(json.dumps(cfg))
        return HardwareBackend(c), csv, p

    def make_task_request(self, csv, site):
        # Import the ACTUAL separate task package for cross-package integration.
        task_src = os.environ.get("CAPX_RING_TASK_SRC", str(A.parent/"capx-fr3-skills-20261023/hardware_ring_task_v1/src"))
        sys.path.insert(0, task_src)
        from capx_ring_tasks.bridge import StageRequest, RingTaskBridge
        now = time.monotonic()
        I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        request = StageRequest.create(part="ring1", side="right", phase="carry", stage_key="right.carry.ring1",
            csv_path=csv, csv_sha256=sha(csv), current_states={s: {"q": [0]*7, "q_command": [0]*7, "width_m": .012, "captured_monotonic_s": now} for s in ("left", "right")},
            observation={"part": "ring1", "signed_class_valid": True, "domain": "hardware", "replay": False, "uses_oracle": False, "calibration_measured": True, "capture_id": "offline-integration-only", "raw_sha256": "a"*64, "calibration_sha256": "b"*64, "captured_monotonic_s": now},
            world_part_goal=I, observed_tcp_part=I, canonical_end_q=[0]*7,
            source_sha256="c"*64, site_config_sha256=sha(site))
        return request, RingTaskBridge

    def test_real_task_package_registers_full_context_then_motion_is_blocked(self):
        with tempfile.TemporaryDirectory() as d:
            b, csv, site = self.make_site(Path(d))
            request, Bridge = self.make_task_request(csv, site)
            bridge = Bridge(b, source_sha256="c"*64, site_config_sha256=sha(site), max_sensor_age_s=3, max_state_age_s=1)
            original_site_bytes = site.read_bytes()
            with patch.object(socket.socket, "connect", side_effect=AssertionError("network")), patch.object(subprocess, "Popen", side_effect=AssertionError("process")):
                with self.assertRaises(MotionDisabled):
                    bridge.execute(request)
            stored = json.loads(b._stages[("right", "right.carry.ring1")])
            self.assertEqual(stored["task_context"], request.payload)
            digest = hashlib.sha256(canonical(stored).encode()).hexdigest()
            self.assertEqual(b._runtime_rows[("right", "right.carry.ring1")]["capx_request_sha256"], digest)
            self.assertEqual(site.read_bytes(), original_site_bytes)
            with self.assertRaises(ValueError):
                bridge.execute(request)

    def test_registered_task_context_side_or_calibration_mutation_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            b, csv, site = self.make_site(Path(d))
            request, _ = self.make_task_request(csv, site)
            p = request.payload
            mapped = {k: p[k] for k in ("request_id", "side", "part", "phase", "stage_key", "csv_path", "csv_sha256")}
            mapped.update(capture_id=p["observation"]["capture_id"], calibration_id=p["observation"]["calibration_sha256"], T_tcp_part=p["observed_tcp_part"], task_context=p)
            for path, value in ((["task_context", "side"], "left"), (["task_context", "observation", "calibration_sha256"], "d"*64), (["task_context", "simulation_bias_enabled"], True)):
                changed = copy.deepcopy(mapped)
                cur = changed
                for key in path[:-1]:
                    cur = cur[key]
                cur[path[-1]] = value
                with self.assertRaises(ValueError):
                    b.register_stage(changed)
            registered = b.register_stage(mapped)
            self.assertEqual(registered["request_sha256"], hashlib.sha256(canonical(mapped).encode()).hexdigest())
            self.assertFalse(registered["collision_approved"])
            with self.assertRaises(ValueError):
                b.register_stage(mapped)


if __name__ == "__main__":
    unittest.main()
