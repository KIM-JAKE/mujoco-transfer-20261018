import contextlib
import io
import json
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from capx_deploy.cli import initialize, main, pin_files, verify_vendor, PACKAGE_ROOT


class PortableInstall(unittest.TestCase):
    def test_fresh_site_has_no_device_side_effects_and_motion_is_blocked(self):
        from hardware_v1.backend import HardwareBackend, MotionDisabled
        with tempfile.TemporaryDirectory() as directory:
            site = Path(directory)/"actual-site"
            with patch.object(socket.socket, "connect", side_effect=AssertionError("network")), \
                 patch.object(subprocess, "run", side_effect=AssertionError("process")), \
                 patch.object(subprocess, "Popen", side_effect=AssertionError("process")):
                initialize(site)
                backend = HardwareBackend(site/"hardware.json")
                report = backend.preflight()
                self.assertFalse(report["local_config_complete"])
                self.assertFalse(report["device_connected"])
                self.assertFalse(report["motion_issued"])
                self.assertIn("admission.hardware_qualified", {r["field"] for r in report["blockers"]})
                for fn in (lambda: backend.execute_stage("right", "any"),
                           lambda: backend.hand_move("right", {}),
                           lambda: backend.clamp("ring1", True)):
                    with self.assertRaises(MotionDisabled):
                        fn()
                self.assertIsNone(backend._loader)

    def test_manifest_loads_actual_bundled_drivers_from_a_new_site(self):
        from hardware_v1.drivers import DriverLoader
        with tempfile.TemporaryDirectory() as directory:
            site = Path(directory)/"relocated site with spaces"
            initialize(site)
            config = json.loads((site/"hardware.json").read_text())
            loader = DriverLoader(site/config["driver_manifest"]["path"], config["driver_manifest"]["sha256"])
            hand = loader.module("fieldkit", "hand").FrankaHand
            with self.assertRaises(ValueError):
                hand.validate_action(dict(width_m=.012, speed_m_s=.01, force_n=30,
                                          epsilon_inner_m=.0005, epsilon_outer_m=.0005), grasp=True)
            loader.module("fieldkit", "site")
            loader.module("field_demo", "camera")
            self.assertIsNone(loader.module("field_demo", "camera").RealSenseCapture(
                {"camera_serial": "offline-unused"}).pipe)

    def test_existing_site_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            site = Path(directory)/"site"
            initialize(site)
            before = (site/"hardware.json").read_bytes()
            with self.assertRaises(FileExistsError):
                initialize(site)
            self.assertEqual(before, (site/"hardware.json").read_bytes())

    def test_pin_records_file_changes_without_enabling_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            site = Path(directory)/"site"
            initialize(site)
            config = json.loads((site/"hardware.json").read_text())
            config["motion_enabled"] = True
            (site/"hardware.json").write_text(json.dumps(config))
            before = config["site_config_sha256"]
            with (site/"site.json").open("a") as stream:
                stream.write("\n")
            pin_files(site/"hardware.json")
            changed = json.loads((site/"hardware.json").read_text())
            self.assertFalse(changed["motion_enabled"])
            self.assertNotEqual(before, changed["site_config_sha256"])

    def test_cpp_build_inputs_ship_with_package(self):
        source = PACKAGE_ROOT/"capx_vendor/fieldkit"
        for name in ("CMakeLists.txt", "site_motion.cpp", "franka_hand.cpp", "read_state.cpp",
                     "libfranka_stage.cpp", "libfranka_stage.hpp", "servo_core.hpp",
                     "check_site_cli.cmake", "test_telemetry.cpp"):
            self.assertTrue((source/name).is_file(), name)
        self.assertGreater(verify_vendor()["verified_vendor_files"], 20)

    def test_template_cli_reports_incomplete_site_with_exit_two(self):
        with tempfile.TemporaryDirectory() as directory:
            site = Path(directory)/"site"
            initialize(site)
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                code = main(["preflight", "--config", str(site/"hardware.json")])
            self.assertEqual(code, 2)
            self.assertFalse(json.loads(output.getvalue())["local_config_complete"])


if __name__ == "__main__":
    unittest.main()
