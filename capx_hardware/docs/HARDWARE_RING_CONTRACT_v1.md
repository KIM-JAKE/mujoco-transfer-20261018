# Ring pair and hardware integration contract v1

This new integration layer does not modify sealed common core `84ae8f7…`.
Root owns `hardware_v1/`, hardware configuration, entrypoint and preflight.
Skills owner owns its NEW task/native ring bridge and device-independent phase operations.

## Ring pair authority

Run actual SMALL then actual LARGE in one current scene/session. Use two real
`PartTask`s with the original eight phases and original checkpoint/recovery
budgets. `PipelineCoordinator.run_component` may drive each separately; do not
invent a third task or mark `full_task_success`. Existing native phase tokens,
single consumption, cancellation and main-thread barriers remain mandatory.
After LARGE release/withdraw/final_check, request a NEW acquisition of BOTH
rings, later than the last ring phase, with original precision and hand-clearance
gates. No installed SMALL surrogate qualifies as a pair. Preserve initial,
recovery and final observation IDs/capture clocks/calibration/source hashes.

Task-owned authority file: `native-ring-pair-session-final.json` with
`accepted_ring_pair`, `accepted_full3:false`, `domain`, run/source/config/scene
IDs, `cancelled`, `worker_alive_after_abort`, `terminal_failure`, all real phase
events, individual component outcomes and fresh final BOTH-ring evidence.
Acceptance requires 16 real phases (plus explicit bounded retries), both actual
component completions, uncancelled/stopped worker and fresh final evidence.
An earlier whole3 trace's two ring checkpoints remains retrospective partial
evidence, never an independent rings-only experiment. Do not fabricate PIN
phases. Keep original failure denominator and SMALL regression results.

## Hardware task/backend boundary

All poses use measured metric `T_cell_*`, explicit TCP/flange/base frames,
actual timestamped states and payloads. No simulation body handles, virtual
camera movement or simulation command/actual bias in hardware requests.
Task bridge sends side, part, phase, request ID, observed grasp transform,
fresh capture ID and immutable joint CSV `time_s,q1..7,dq1..7,ddq1..7`.
Hardware CSV interpolation is the original fieldkit quintic polynomial at
1kHz, NOT the simulator's 480Hz sample replay. Retime and recheck that exact
interpolant with the real measured TCP/load/world/other arm before motion.

`HardwareBackend.read_state(side)` uses the original `m80_read_state`.
`hand_status(side)` / `hand_move(side, action)` use original `FrankaHand`.
`plc_read(name)` / `clamp(part, asserted)` use original mapped Modbus PLC.
`capture()` uses original persistent `RealSenseCapture`, measured calibration,
SDK deprojection and original exposure-age checks. Actual camera selection is
site-specific; no camera model or serial is assumed by the template.
`execute_stage(side, stage_key)` uses the original registered/reviewed
`Site.motion(side, 'csv', [path])`, full interlocks and SDK dynamics checks.

cuRobo produces candidates ONLY. A site-provided, source-pinned hardware
admission executable must check the exact CSV/interpolant, actual→command
bridge, full two-arm/Hand/payload/installed/free stock geometry, all opening
contexts and FR3 limits using the current measured model. Its result is bound
to the complete request digest. No existing standalone hardware full-world
admission executable has yet been verified. Missing it blocks actuation;
simulation admission and successful source paths cannot substitute for it.

Current fieldkit CSV executable is commissioning-only at 10% envelope, external
wrench guards 5N/1Nm and protocol 10. Native Hand wrapper accepts grasp force
5..20N and width speed <=.02m/s. Simulation's 30N/pad does not map to a 30N
hardware command. Do not clip or silently substitute; require a separately
qualified actual Hand policy and record the mismatch.

This work authorizes CODE PREPARATION and offline/non-driving integration
verification, not robot/Hand/clamp motion. Default configuration has unknown
IPs/calibration/TCP/device mappings and motion disabled. Status/camera probes
are separate explicit read-only commands, never automatic preflight actions.
