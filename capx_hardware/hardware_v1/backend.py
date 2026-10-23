"""Concrete original-driver bridge. Construction/preflight never connects devices.

Motion requires a commissioned site AND an independently supplied current-world
admission executable. Neither simulation success nor a saved certificate arms it.
The preparation CLI does not expose arm/Hand/clamp execution.
"""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import threading
import time
from .config import canonical, finite, load, preflight, resolve, sha
from .drivers import DriverLoader


class MotionDisabled(RuntimeError):
    pass


class HardwareBackend:
    def __init__(self, config_path, *, log_dir=None):
        self.config, self.filename = load(config_path)
        self.base = self.filename.parent
        self.config_digest = sha(self.filename)
        self.log_dir = Path(log_dir).resolve() if log_dir else self.base/"hardware-logs"
        self._site = None
        self._camera = None
        self._loader = None
        self._stages = {}
        self._lock = threading.RLock()
        self._consumed_requests = set()
        self._registered_requests = set()
        self._runtime_rows = {}

    def preflight(self):
        return preflight(self.config, self.filename)

    def _unchanged(self):
        if sha(self.filename) != self.config_digest:
            raise RuntimeError("Hardware configuration changed after construction")

    def _drivers(self):
        self._unchanged()
        if self._loader is None:
            record = self.config["driver_manifest"]
            self._loader = DriverLoader(resolve(self.base, record["path"]), record["sha256"])
        self._loader.verify()
        return self._loader

    def _binary(self, name, path):
        expected = self.config["binaries"][name]
        actual = resolve(self.base, expected["path"])
        if Path(path).resolve() != actual or sha(actual) != expected["sha256"]:
            raise RuntimeError("Original binary missing or hash changed: "+name)
        return actual

    def _site_config(self):
        p = resolve(self.base, self.config["site_config"])
        raw = p.read_bytes()
        expected = self.config.get("site_config_sha256")
        if not expected or hashlib.sha256(raw).hexdigest() != expected:
            raise RuntimeError("Site configuration must have a pinned SHA-256")
        return json.loads(raw), p

    def _get_site(self):
        # Site constructor creates a log and lazy Modbus objects; no device I/O.
        self._site_config()
        if self._site is None:
            self._site = self._drivers().module("fieldkit", "site").Site(
                resolve(self.base, self.config["site_config"]), self.log_dir)
        return self._site

    def _motion_gate(self):
        self._unchanged()
        if self.config.get("motion_enabled") is not True:
            raise MotionDisabled("Motion disabled; this package prepares connections only")
        result = self.preflight()
        if not result["local_config_complete"]:
            raise MotionDisabled("Site commissioning/configuration incomplete: "+canonical(result["blockers"]))

    def read_state(self, side):
        if side not in ("left", "right"):
            raise ValueError("Explicit arm side required")
        self._unchanged()
        cfg, path = self._site_config()
        binary = self._binary("state", resolve(path.parent, cfg["state_binary"]))
        import ipaddress
        ip = str(ipaddress.ip_address(cfg["robots"][side]["ip"]))
        began = time.monotonic()
        result = subprocess.run([str(binary), ip], check=True, capture_output=True, text=True, timeout=10)
        data = json.loads(result.stdout)
        for key, n in (("q", 7), ("q_d", 7), ("dq", 7), ("F_T_EE_column_major", 16), ("O_T_EE_column_major", 16)):
            a = data.get(key)
            if not isinstance(a, list) or len(a) != n or not all(finite(x) for x in a):
                raise RuntimeError("Invalid actual FR3 state field: "+key)
        return {"side": side, "state": data, "request_monotonic_s": began,
                "received_monotonic_s": time.monotonic(), "domain": "hardware",
                "source": "libfranka_readOnce", "robot_command_sent": False}

    def hand_status(self, side):
        site = self._get_site()
        self._binary("hand", site.grippers[side].binary)
        return site.grippers[side].status()

    def plc_read(self, name):
        return self._get_site().plc.read(name)

    def hand_move(self, side, action):
        self._motion_gate()
        if not isinstance(action, dict) or set(action)-{"width_m", "speed_m_s", "grasp", "force_n", "epsilon_inner_m", "epsilon_outer_m", "timeout_s"}:
            raise ValueError("Unsupported real Hand action schema")
        site = self._get_site()
        self._binary("hand", site.grippers[side].binary)
        # Original validate_action rejects simulation's 30N, no clipping.
        original = dict(action)
        grasp = original.pop("grasp", False)
        if type(grasp) is not bool:
            raise ValueError("grasp must be bool")
        original.pop("timeout_s", None)
        site.grippers[side].validate_action(original, grasp=grasp)
        with site.control_lock():
            registered = site.config.get("gripper_actions", {})
            if not any(original == values for part in ("ring1", "ring2") for values in registered.get(part, {}).values()):
                raise RuntimeError("Hand action is not in the measured site profile")
            self._admit_io({"kind": "hand", "side": side, "action": action})
            return site.guarded(lambda cancelled: site.grippers[side].move(**action, cancelled=cancelled))

    def clamp(self, part, asserted):
        self._motion_gate()
        if part not in ("ring1", "ring2") or type(asserted) is not bool:
            raise ValueError("Explicit ring clamp boolean required")
        site = self._get_site()
        with site.control_lock():
            site.require_ready()
            self._admit_io({"kind": "clamp", "part": part, "asserted": asserted})
            site.plc.write(part+"_clamp_command", asserted)
            site.plc.wait(part+"_clamped", expected=asserted,
                          cancelled=lambda: site.cancelled.is_set())
            return {"part": part, "clamped": site.plc.read(part+"_clamped"), "domain": "hardware"}

    def _admit_io(self, action):
        """Continuous aperture/world and supported clamp transitions need proof too."""
        site = self._get_site()
        site.require_ready()
        began = time.monotonic()
        states = {s: self.read_state(s) for s in ("left", "right")}
        hands = {s: self.hand_status(s) for s in states}
        evidence = {"schema": "capx-hardware-io-admission/v1", "domain": "hardware",
                    "action": action, "hardware_config_sha256": self.config_digest,
                    "site_config_sha256": self.config["site_config_sha256"],
                    "states": states, "hands": hands, "geometry": self.config["geometry_record"],
                    "tools": self.config["tools"]}
        checker = self.config["admission"]["executable"]
        executable = resolve(self.base, checker["path"])
        if sha(executable) != checker["sha256"]:
            raise RuntimeError("Hardware admission executable changed")
        proof = json.loads(subprocess.run([str(executable)], input=canonical(evidence), text=True,
                                         capture_output=True, check=True, timeout=30).stdout)
        required = ("full_world", "both_arms", "continuous_aperture", "payload", "installed_and_free_parts", "supported_release", "measured_frames")
        if proof.get("domain") != "hardware" or proof.get("request_sha256") != hashlib.sha256(canonical(evidence).encode()).hexdigest() or any(proof.get(k) is not True for k in required):
            raise RuntimeError("Missing complete current hardware I/O transition proof")
        if time.monotonic()-began > 2:
            raise RuntimeError("Hardware I/O approval expired")
        for side in states:
            fresh = self.read_state(side)
            if any(abs(a-b) > 1e-6 for k in ("q", "q_d") for a, b in zip(states[side]["state"][k], fresh["state"][k])) or abs(hands[side]["width_m"]-self.hand_status(side)["width_m"]) > 1e-6:
                raise RuntimeError("Hardware state changed during I/O proof")

    def capture(self):
        """Explicit live camera read. Lazy SDK import; never an arm/Hand operation."""
        self._unchanged()
        setting = self.config["camera"]
        if setting.get("type") != "realsense":
            raise RuntimeError("No commissioned adapter for actual camera type")
        calibration_path = resolve(self.base, setting["calibration"])
        if sha(calibration_path) != setting.get("calibration_sha256"):
            raise RuntimeError("Camera calibration hash changed or unspecified")
        driver = self._drivers()
        calibration = driver.module("field_demo", "calibration").load_config(calibration_path)
        if self._camera is None:
            self._camera = driver.module("field_demo", "camera").RealSenseCapture(
                calibration, width=setting["width"], height=setting["height"],
                fps=setting["fps"], max_age_s=setting["max_age_s"])
            self._camera.start()
        packet, metadata = self._camera.read()
        import numpy as np
        actual_K = np.asarray(metadata["intrinsics"], float)
        expected_K = np.asarray(setting["K"], float)
        tol = setting.get("intrinsic_tolerance_px")
        if not finite(tol) or not 0 <= tol <= 1e-6 or expected_K.shape != (3, 3) or not np.isfinite(expected_K).all() or np.max(abs(actual_K-expected_K)) > tol:
            raise RuntimeError("Actual measured intrinsics differ from pinned site camera")
        return packet, metadata

    def close(self):
        if self._camera is not None:
            self._camera.close()
            self._camera = None

    def register_stage(self, request):
        """Register an immutable exact field CSV, NOT a safety certificate.

        Request shape is deliberately public for task-owned ring adapter code.
        Registration is local-only and does not execute a device operation.
        """
        request = json.loads(canonical(request))
        required = {"request_id", "side", "part", "phase", "stage_key", "csv_path", "csv_sha256", "capture_id", "calibration_id", "T_tcp_part", "task_context"}
        if set(request) != required or request["side"] not in ("left", "right") or request["part"] not in ("ring1", "ring2"):
            raise ValueError("Exact hardware ring stage request schema required")
        from .config import rigid
        if request["phase"] not in ("perception", "grasp", "carry", "preinsert", "fixed_insert", "release", "withdraw", "final_check"):
            raise ValueError("Unknown ring phase")
        if not all(isinstance(request[k], str) and request[k] for k in ("request_id", "stage_key", "capture_id", "calibration_id")) or not rigid(request["T_tcp_part"]):
            raise ValueError("Measured grasp/capture identity required")
        path = resolve(self.base, request["csv_path"])
        if sha(path) != request["csv_sha256"]:
            raise ValueError("CSV changed before registration")
        with path.open() as stream:
            rows = list(csv.reader(stream))
        header = ["time_s"]+[name+str(j) for name in ("q", "dq", "ddq") for j in range(1, 8)]
        if not rows or rows[0] != header or len(rows) < 3 or len(rows) > 100001:
            raise ValueError("Original field CSV requires time_s + q/dq/ddq for seven joints")
        data = [[float(v) for v in row] for row in rows[1:]]
        if any(len(r) != 22 or not all(finite(x) for x in r) for r in data) or data[0][0] != 0 or any(b[0] <= a[0] for a, b in zip(data, data[1:])):
            raise ValueError("Finite strictly timed field polynomial knots required")
        if any(abs(x) > 1e-12 for r in (data[0], data[-1]) for x in r[8:]):
            raise ValueError("Field stage boundaries must be stopped")
        context = request["task_context"]
        fields = {"schema", "domain", "part", "side", "phase", "stage_key", "request_id", "csv_path", "csv_sha256", "csv_interpolation", "source_sha256", "site_config_sha256", "current_states", "observation", "world_part_goal", "observed_tcp_part", "canonical_end_q", "T_cell_tcp_target", "simulation_bias_enabled"}
        if not isinstance(context, dict) or set(context) != fields or context["schema"] != "capx-ring-hardware-stage/v1" or context["domain"] != "hardware" or context["simulation_bias_enabled"] is not False:
            raise ValueError("Complete exact measured task context required")
        if context["csv_interpolation"] != "original_fieldkit_quintic_1kHz":
            raise ValueError("Simulation replay cannot substitute for the hardware interpolant")
        for name in ("request_id", "side", "part", "phase", "stage_key", "csv_sha256"):
            if context[name] != request[name]:
                raise ValueError("Task context mismatch: "+name)
        if resolve(self.base, context["csv_path"]) != path or context["observed_tcp_part"] != request["T_tcp_part"]:
            raise ValueError("Task path/grasp differs from registered request")
        if not all(rigid(context[name]) for name in ("world_part_goal", "observed_tcp_part", "T_cell_tcp_target")):
            raise ValueError("Measured metric task transforms required")
        for value in (context["source_sha256"], context["site_config_sha256"]):
            if not isinstance(value, str) or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
                raise ValueError("Task source/site SHA256 required")
        if context["source_sha256"] != self.config.get("task_source_sha256"):
            raise ValueError("Task source differs from configured hardware task identity")
        W, G = context["world_part_goal"], context["observed_tcp_part"]
        inverse_G = [[G[j][i] for j in range(3)]+[-sum(G[j][i]*G[j][3] for j in range(3))] for i in range(3)]+[[0, 0, 0, 1]]
        expected_target = [[sum(W[i][k]*inverse_G[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
        if any(abs(expected_target[i][j]-context["T_cell_tcp_target"][i][j]) > 1e-9 for i in range(4) for j in range(4)):
            raise ValueError("Current measured grasp does not map to the SAME world part goal")
        site_config, site_path = self._site_config()
        if context["site_config_sha256"] != sha(site_path):
            raise ValueError("Task context site identity differs")
        states = context["current_states"]
        if not isinstance(states, dict) or set(states) != {"left", "right"}:
            raise ValueError("Both measured arm/Hand contexts required")
        for s in states.values():
            if any(not isinstance(s.get(k), list) or len(s[k]) != 7 or not all(finite(x) for x in s[k]) for k in ("q", "q_command")) or not finite(s.get("width_m")) or not 0 <= s["width_m"] <= .08 or not finite(s.get("captured_monotonic_s")):
                raise ValueError("Invalid measured arm/Hand context")
        obs = context["observation"]
        if not isinstance(obs, dict) or obs.get("part") != request["part"] or obs.get("domain") != "hardware" or obs.get("replay") is not False or obs.get("uses_oracle") is not False or obs.get("calibration_measured") is not True or obs.get("signed_class_valid") is not True:
            raise ValueError("Actual signed measured ring observation required")
        if obs.get("capture_id") != request["capture_id"] or obs.get("calibration_sha256") != request["calibration_id"] or not finite(obs.get("captured_monotonic_s")):
            raise ValueError("Task capture identity differs")
        if any(not isinstance(obs.get(k), str) or len(obs[k]) != 64 or any(c not in "0123456789abcdef" for c in obs[k]) for k in ("raw_sha256", "calibration_sha256")):
            raise ValueError("Raw/calibration SHA-256 required")
        q_end = context["canonical_end_q"]
        if not isinstance(q_end, list) or len(q_end) != 7 or not all(finite(x) for x in q_end) or any(abs(a-b) > 1e-9 for a, b in zip(q_end, data[-1][1:8])) or any(abs(a-b) > 1e-9 for a, b in zip(states[request["side"]]["q_command"], data[0][1:8])):
            raise ValueError("Current command start/canonical endpoint mismatch")
        # This source-reviewed registry is immutable on disk. Per-observation
        # digest is added ONLY to a private runtime copy, avoiding a circular
        # site-file-hash / request-hash dependency.
        row = site_config.get("stages", {}).get(request["stage_key"], {})
        if not row or row.get("side") != request["side"] or row.get("sha256") != request["csv_sha256"] or resolve(site_path.parent, row.get("csv")) != path or row.get("field_path_reviewed") is not True or row.get("tool_type") != "franka_hand":
            raise ValueError("Original reviewed site stage/side/CSV registration missing")
        key = (request["side"], request["stage_key"])
        if key in self._stages or request["request_id"] in self._registered_requests:
            raise ValueError("Stage/request ID cannot be reused")
        self._stages[key] = canonical(request)
        self._registered_requests.add(request["request_id"])
        digest = hashlib.sha256(self._stages[key].encode()).hexdigest()
        self._runtime_rows[key] = dict(row, capx_request_sha256=digest)
        return {"request_sha256": digest, "capx_request_sha256": digest,
                "side": request["side"], "stage_key": request["stage_key"], "csv_sha256": request["csv_sha256"],
                "registered": True, "collision_approved": False, "motion_issued": False}

    def execute_stage(self, side, stage_key):
        """Concrete driver path; current package/config has motion disabled.

        Missing hardware whole-world checker blocks, never falls back to sim.
        The checker consumes JSON on stdin and returns request-digest-bound JSON.
        """
        self._motion_gate()
        with self._lock:
            raw = self._stages.pop((side, stage_key))
            request = json.loads(raw)
            if request["request_id"] in self._consumed_requests:
                raise RuntimeError("Consumed motion request")
            self._consumed_requests.add(request["request_id"])
            path = resolve(self.base, request["csv_path"])
            if sha(path) != request["csv_sha256"]:
                raise RuntimeError("Field changed after registration")
            site = self._get_site()
            self._binary("motion", site.path(site.config["motion_binary"]))
            row = self._runtime_rows[(side, stage_key)]
            digest = hashlib.sha256(raw.encode()).hexdigest()
            if row.get("capx_request_sha256") != digest or row.get("side") != side:
                raise RuntimeError("Runtime request/side registration changed")
            if site.path(row["csv"]) != path or row.get("sha256") != request["csv_sha256"] or row.get("field_path_reviewed") is not True:
                raise RuntimeError("Stage is not in original site-reviewed field registry")
            checker = self.config["admission"]["executable"]
            executable = resolve(self.base, checker["path"])
            if sha(executable) != checker["sha256"]:
                raise RuntimeError("Hardware admission executable identity changed")
            with site.control_lock():
                site.require_ready()
                states = {s: self.read_state(s) for s in ("left", "right")}
                hands = {s: self.hand_status(s) for s in ("left", "right")}
                ctx = request["task_context"]
                now = time.monotonic()
                max_camera_age = self.config.get("max_sensor_age_s")
                max_state_age = self.config.get("max_state_age_s")
                if not finite(max_camera_age) or not 0 < max_camera_age <= 3 or not finite(max_state_age) or not 0 < max_state_age <= 1:
                    raise RuntimeError("Explicit actual observation/state age bounds required")
                if not 0 <= now-ctx["observation"]["captured_monotonic_s"] <= max_camera_age or any(not 0 <= now-v["captured_monotonic_s"] <= max_state_age for v in ctx["current_states"].values()):
                    raise RuntimeError("Task observation/state stale or from future")
                if any(abs(ctx["current_states"][s][k][j]-states[s]["state"][real][j]) > 1e-6 for s in states for k, real in (("q", "q"), ("q_command", "q_d")) for j in range(7)) or any(abs(ctx["current_states"][s]["width_m"]-hands[s]["width_m"]) > 1e-6 for s in hands):
                    raise RuntimeError("Actual current state differs from registered task context")
                evidence = {"schema": "capx-hardware-admission/v1", "stage": request,
                            "hardware_config_sha256": self.config_digest, "site_config_sha256": self.config["site_config_sha256"],
                            "states": states, "hands": hands, "geometry": self.config["geometry_record"],
                            "tools": self.config["tools"], "camera": self.config["camera"],
                            "interpolation": "original_fieldkit_quintic_1kHz", "domain": "hardware"}
                digest = hashlib.sha256(canonical(evidence).encode()).hexdigest()
                began = time.monotonic()
                response = subprocess.run([str(executable)], input=canonical(evidence), text=True,
                                          capture_output=True, check=True, timeout=30)
                proof = json.loads(response.stdout)
                required = ("full_world", "both_arms", "hands_all_widths", "payload", "installed_and_free_parts", "actual_command_bridge", "entire_interpolated_FR3", "fresh_camera_binding", "measured_frames")
                if proof.get("request_sha256") != digest or proof.get("domain") != "hardware" or any(proof.get(k) is not True for k in required):
                    raise RuntimeError("Hardware exact admission missing/current request differs")
                # Never reuse a slow plan's old state. Original native C++ also
                # rechecks actual command start in its first real-time callback.
                fresh = {s: self.read_state(s) for s in states}
                if time.monotonic()-began > 2 or any(abs(a-b) > 1e-6 for s in states for k in ("q", "q_d") for a, b in zip(states[s]["state"][k], fresh[s]["state"][k])):
                    raise RuntimeError("Current hardware state changed/approval expired")
                if any(abs(hands[s]["width_m"]-self.hand_status(s)["width_m"]) > 1e-6 for s in hands):
                    raise RuntimeError("Current hardware hand opening changed")
                if sha(path) != request["csv_sha256"]:
                    raise RuntimeError("Field changed immediately before driver execution")
                site.config["stages"][stage_key] = dict(row)
                return site.motion(side, "csv", [path])
