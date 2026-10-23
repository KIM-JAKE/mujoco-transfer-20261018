# FR3 hardware integration v1

This is NEW code outside the sealed common/simulation core. It calls existing
workspace drivers, rather than simulator command APIs or empty device callbacks.
No device was connected and no motion was tested while preparing this package.

## Entry points

From this adapter directory (or after the release packager installs it):

```sh
python -B -m hardware_v1 preflight --config hardware_configs/hardware.template.json
python -B -m unittest discover -s tests_hardware -v
```

`preflight` reads local files only. Exit 2 means required site values are absent;
the template intentionally reports missing calibration/devices/admission. Tests
use the REAL separate `capx_ring_tasks` request class and this backend. They
verify cross-package registration, immutable CSV/context and the disabled-motion
boundary while forbidding network/subprocess operations. They are not hardware
or physical collision tests.

Explicit, later **read-only** device probes, after filling and pinning site data:

```sh
python -m hardware_v1 status --config SITE/hardware.json --connect-read-only
python -m hardware_v1 camera --config SITE/hardware.json --connect-read-only --output NEW_CAPTURE.npz
```

These two commands really read FR3/Hand or start camera streaming; they are not
performed by preflight or installation. There is no arm, Hand, clamp, homing or
reset execution command in this CLI. `close()` stops the camera stream only.

## Existing implementations reused

- `fieldkit/read_state.cpp`: `m80_read_state IP`, actual libfranka `readOnce`.
- `fieldkit/hand.py` and `franka_hand.cpp`: status/move/grasp/stop. Homing is NOT
  called. Original validation enforces width<=80mm, speed<=20mm/s, grasp5..20N.
- `fieldkit/devices.py` and `modbus.py`: actual TCP PLC signal reads/writes and
  independently read clamp feedback with readiness monitoring.
- `fieldkit/site.py::Site.motion`: site-reviewed registered CSV, measured TCP
  verification, field records, cell lock, heartbeat, I/O interlocks and abort.
- `fieldkit/site_motion.cpp` / `libfranka_stage.cpp`: actual SDK dynamics,
  protocol10, 1kHz quintic joint interpolation, measured initial/start/tracking
  checks. The original executable is a 10%-envelope commissioning driver with
  external-wrench guards5N/1Nm. It is not a proven production insertion driver.
- `field_demo/camera.py::RealSenseCapture` and `calibration.py::load_config`:
  persistent measured-serial camera streaming, SDK deprojection, metadata-based
  exposure age, monotonic frame rejection and measured calibration provenance.

`drivers.local.json` pins the ORIGINAL Python/C++/header sources. Paths resolve
relative to the manifest, or can be absolute. The release packager may copy
exact sources to `vendor/`, generate a NEW relative manifest, and pin that new
manifest in a site config. Do not edit the original sources or old manifests.
Vendor source identity does not lock transitive SDK libraries or binary ABI.

## Site preparation order

1. Keep a new site directory. Copy the template JSONs; leave `motion_enabled`
   false. Record actual robot IPs, Native Hand models, PLC gateway and every
   signal's kind/address/polarity. Never infer sensors which are not installed.
2. Build the original fieldkit C++ tools against the field PC's libfranka and
   Eigen (see vendor `fieldkit/CMakeLists.txt`). Fill paths and SHA-256 for state,
   motion and Hand binaries. Do not use this server's Linux binaries blindly on
   another OS/architecture. No automatic firmware changes or homing.
3. Measure both `T_cell_base`, actual flange-to-TCP transforms, payload/tool
   mass, fixture geometry/frame and camera optical K/T/serial. Store measurement
   evidence and exact file hashes. Calibration must be assessed against the
   task's original tolerances; passing a generic calibration residual check
   does not prove insertion accuracy. The original camera calibration schema
   additionally needs measured provenance, frames and calibrated stock priors.
4. Run local preflight. Once addresses/calibration are available, explicitly
   read status and capture a new frame. Compare reported `F_T_EE`, serial/K,
   both-arm frames, clamp feedback and observed jig geometry with the measured
   configuration. A camera move creates a new calibration identity.
5. Generate real field trajectories from CURRENT measured state and grasp,
   through current cuRobo/IK proposals and a full current-world exact checker.
   Do not replay a successful simulation's 480Hz commands, body states, grasp
   attachments, virtual views, or command/actual bias.

## Task request and stage registration

Public import: `from hardware_v1 import HardwareBackend`.

`register_stage(request)` accepts the original keys `request_id, side, part,
phase, stage_key, csv_path, csv_sha256, capture_id, calibration_id, T_tcp_part`
and REQUIRED `task_context`, the complete `capx-ring-hardware-stage/v1` payload
from `capx_ring_tasks.StageRequest`. `calibration_id` here binds the exact
calibration SHA-256 from that payload. Requests are immutable/single-use.

CSV header is exactly `time_s,q1..q7,dq1..dq7,ddq1..ddq7`; values are polynomial
knots with strictly increasing time and stopped boundaries. Parsing checks
identity/input only. It does NOT approve continuous dynamic or collision limits.

The immutable site JSON must already register each field stage under `stages`:

```json
{
  "right.carry.ring1": {
    "side": "right", "csv": "MEASURED_FIELD_STAGE.csv", "sha256": "MEASURED_SHA256",
    "field_path_reviewed": true, "tool_type": "franka_hand",
    "robot_ip": "ACTUAL_SITE_IP", "F_T_EE_column_major": ["16 measured numbers required"]
  }
}
```

This is a schema illustration, not filled device data. Hardware config pins
the site file's bytes and `task_source_sha256`. Registration checks that row and creates a private
in-memory copy with `capx_request_sha256`, binding the exact complete request.
The disk site file stays unchanged, avoiding a circular site-hash/request-hash
dependency. Registration replies include digest/CSV/side/key and explicitly
`collision_approved:false,motion_issued:false`. Caller-supplied bindings alone
are never accepted as admission.

`execute_stage(side, stage_key)` requires a commissioned hardware config and
a source-pinned exact-admission executable. It passes JSON on stdin containing
the entire task request, fresh actual both-arm states, actual Hand openings,
current tools/world/camera/config IDs and the exact field CSV interpolation.
The reply must bind SHA-256 of this canonical JSON, `domain:hardware`, and
positive proof fields `full_world,both_arms,hands_all_widths,payload,
installed_and_free_parts,actual_command_bridge,entire_interpolated_FR3,
fresh_camera_binding,measured_frames`. After fresh state/opening/file rechecks,
the backend calls the ORIGINAL `Site.motion`. Hand/clamp transitions likewise
require continuous aperture/current-world and supported-release admission.

**That hardware full-world admission executable has not yet been provided or
commissioned.** Therefore current site templates intentionally block actuation.
It cannot be replaced by a simulation certificate, a cuRobo CLEAR result, a
manually written JSON approval, or a saved successful trajectory. No arbitrary
callback is installed which returns success. Task-owned direct native ring
motion is being migrated to these real method calls; complete physical task
coverage remains unqualified until commissioning and actual tests.

## Known transfer differences

- Simulation force30N/pad is incompatible with the existing Hand wrapper's
 5..20N grasp interface. No clipping, per-pad/total reinterpretation or force
 increase is performed. Qualify the actual Hand/contact policy separately.
- Sim 480Hz retimed arrays and actual field 1kHz quintic interpolation differ;
 recalculate and check the complete issued interpolant and actual→command start.
- Camera model/position/metal depth bias and calibration uncertainty are site
 data. The template uses nulls; no simulated K/T or palm offset is supplied.
- The exact checker must include both hands/arms, changing widths, held payload,
 jig, placed rings and remaining free stock under current calibration. cuRobo
 accelerates candidate generation; it is not the final admission authority.
- Offline tests prove source/config/I/O boundaries only. This package is code
 preparation, not a report of hardware success, throughput or stable variation.
