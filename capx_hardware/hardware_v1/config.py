"""Local configuration checks. No imports of device SDKs or network operations."""
import hashlib
import ipaddress
import json
import math
from pathlib import Path


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def rigid(value):
    if not isinstance(value, list) or len(value) != 4 or any(not isinstance(r, list) or len(r) != 4 for r in value):
        return False
    if not all(finite(v) for r in value for v in r):
        return False
    if any(abs(v-w) > 1e-9 for v, w in zip(value[3], (0, 0, 0, 1))):
        return False
    r = [row[:3] for row in value[:3]]
    if any(abs(sum(r[k][i]*r[k][j] for k in range(3))-(i == j)) > 1e-6 for i in range(3) for j in range(3)):
        return False
    det = sum(r[0][i]*(r[1][(i+1)%3]*r[2][(i+2)%3]-r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
    return abs(det-1) <= 1e-6 and sum(x[3]**2 for x in value[:3]) <= 25


def load(path):
    path = Path(path).resolve()
    config = json.loads(path.read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError("Nonfinite JSON: "+x)))
    if config.get("schema") != "capx-fr3-hardware/v1":
        raise ValueError("Expected capx-fr3-hardware/v1")
    return config, path


def resolve(base, value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError("Missing file path")
    p = Path(value)
    return p.resolve() if p.is_absolute() else (base/p).resolve()


def preflight(config, filename):
    """Report every missing site value without touching devices or importing drivers."""
    base = Path(filename).resolve().parent
    issues = []
    def check(ok, field, reason):
        if not ok:
            issues.append({"field": field, "reason": reason})
    def record(value, field):
        try:
            p = resolve(base, value["path"])
            good = p.is_file() and sha(p) == value["sha256"]
        except (ValueError, KeyError, TypeError, OSError):
            good = False
        check(good, field, "Measured record path and exact SHA-256 required")
        return good
    check(config.get("schema") == "capx-fr3-hardware/v1", "schema", "Unsupported schema")
    check(config.get("units") == "m_rad_s_N", "units", "Explicit metric units required")
    check(config.get("simulation_bias_enabled") is False, "simulation_bias_enabled", "Simulation bias cannot drive hardware")
    source = config.get("task_source_sha256")
    check(isinstance(source, str) and len(source) == 64 and all(c in "0123456789abcdef" for c in source),
          "task_source_sha256", "Pin the actual task package source identity")
    try:
        site_path = resolve(base, config.get("site_config"))
        site = json.loads(site_path.read_text())
    except (ValueError, OSError, TypeError):
        site = {}
    check(bool(site), "site_config", "Existing fieldkit site JSON required")
    check(site.get("schema_version") == 1, "site.schema_version", "Original fieldkit site schema required")
    ips = []
    for side in ("left", "right"):
        robot = site.get("robots", {}).get(side, {})
        try:
            ips.append(str(ipaddress.ip_address(robot.get("ip"))))
        except ValueError:
            check(False, f"robots.{side}.ip", "Measured robot address required")
        check(rigid(robot.get("T_cell_base")), f"robots.{side}.T_cell_base", "Measured proper cell-to-base SE3 required")
        for k in ("tool_mass_kg", "load_mass_kg"):
            check(finite(robot.get(k)) and robot[k] >= 0, f"robots.{side}.{k}", "Measured nonnegative mass required")
        hand = site.get("grippers", {}).get(side, {})
        check(hand.get("type") == "franka_hand", f"grippers.{side}.type", "Only inventoried Native Franka Hand adapter is supported in v1")
        tool = config.get("tools", {}).get(side, {})
        check(rigid(tool.get("T_flange_tcp")), f"tools.{side}.T_flange_tcp", "Measured actual TCP; never copy simulation palm offset")
        record(tool.get("measurement"), f"tools.{side}.measurement")
    check(len(ips) == 2 and len(set(ips)) == 2, "robots", "Two distinct valid addresses required")
    for k in ("workspace_checked", "stop_tested", "tool_and_payload_checked", "single_arm_test_area_clear"):
        check(site.get("commissioning", {}).get(k) is True, "commissioning."+k, "Site commissioning incomplete")
    for k in ("calibration_record", "commissioning_record", "motion_binary", "state_binary"):
        try:
            p = resolve(site_path.parent, site.get(k))
            ok = p.is_file() and p.stat().st_size > 0
        except (ValueError, UnboundLocalError):
            ok = False
        check(ok, "site."+k, "Existing original driver binary or actual field record required")
    for key in ("state", "motion", "hand"):
        record(config.get("binaries", {}).get(key), "binaries."+key)
    for name in ("T_cell_fixture",):
        check(rigid(config.get(name)), name, "Measured fixture frame required")
    record(config.get("geometry_record"), "geometry_record")
    camera = config.get("camera", {})
    check(camera.get("type") == "realsense", "camera.type", "Select installed real camera adapter; model not inferred")
    try:
        calibration = resolve(base, camera.get("calibration"))
        c = json.loads(calibration.read_text())
    except (ValueError, OSError, TypeError):
        c = {}
    check(bool(c.get("camera_serial")), "camera.camera_serial", "Actual measured serial required")
    check(rigid(c.get("T_cell_camera")), "camera.T_cell_camera", "Measured camera-to-cell transform required")
    check(c.get("measured", {}).get("verified") is True, "camera.measured", "Measured calibration evidence required")
    check(bool(c) and camera.get("calibration_sha256") == sha(calibration) if c else False,
          "camera.calibration_sha256", "Pin exact measured camera configuration bytes")
    K = camera.get("K")
    check(isinstance(K, list) and len(K) == 3 and all(isinstance(r, list) and len(r) == 3 and all(finite(v) for v in r) for r in K)
          and K[0][0] > 0 and K[1][1] > 0 and K[2] == [0, 0, 1], "camera.K", "Measured optical intrinsics required")
    if c:
        for side in ("left", "right"):
            check(c.get("robots", {}).get(side, {}).get("T_cell_base") == site.get("robots", {}).get(side, {}).get("T_cell_base"),
                  "camera.robots."+side+".T_cell_base", "Camera calibration and robot site base frames must agree exactly")
    endpoint = site.get("plc", {}).get("endpoint", {})
    check(bool(endpoint.get("host")), "plc.endpoint.host", "Actual PLC gateway required")
    check(type(endpoint.get("port")) is int and 1 <= endpoint["port"] <= 65535, "plc.endpoint.port", "Actual mapped gateway port required")
    for name in ("cell_ready", "cell_fault", "cell_stop", "ring1_clamped", "ring2_clamped", "ring1_clamp_command", "ring2_clamp_command"):
        s = site.get("plc", {}).get("signals", {}).get(name, {})
        check(type(s.get("address")) is int and 0 <= s["address"] <= 65535 and type(s.get("active")) is bool,
              "plc.signals."+name, "Verified register/coil address and polarity required")
    for part in ("ring1", "ring2"):
        grasp = site.get("gripper_actions", {}).get(part, {}).get("grasp", {})
        force = grasp.get("force_n")
        check(finite(force) and 5 <= force <= 20, f"gripper_actions.{part}.grasp.force_n", "Original hardware wrapper supports 5..20N; simulation 30N/pad cannot be copied or clipped")
    checker = config.get("admission", {})
    check(checker.get("hardware_qualified") is True, "admission.hardware_qualified", "Full current hardware-world admission has not been commissioned")
    record(checker.get("executable"), "admission.executable")
    record(config.get("driver_manifest"), "driver_manifest")
    try:
        from .drivers import DriverLoader
        m = config["driver_manifest"]
        DriverLoader(resolve(base, m["path"]), m["sha256"])
    except (OSError, ValueError, KeyError, TypeError):
        check(False, "driver_manifest.source_files", "Source-pinned original driver files must be present and unchanged")
    return {"schema": "capx-hardware-preflight/v1", "local_config_complete": not issues,
            "blockers": issues, "device_connected": False, "motion_issued": False,
            "hardware_ready": False, "scope": "Static file/config checks only; actual device/geometry validation is separate"}
