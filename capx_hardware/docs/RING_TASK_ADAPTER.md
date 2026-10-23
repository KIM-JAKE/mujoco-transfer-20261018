# Ring task hardware bridge

This package prepares task requests for the concrete `hardware_v1.HardwareBackend`
provided alongside it. It does not connect a device at import or through its CLI.
Existing simulation results are preserved development evidence, not hardware
qualification. Robot/Hand/clamp motion is not authorized by this deliverable.

## Install and inspect without devices

From this directory: `python -m pip install .`.
In the combined release, install the parent release instead to include
`hardware_v1` and the source-pinned vendor drivers.

```
capx-ring-task preflight examples/rings.site-required.json
python -m unittest discover -s tests -v
```

The supplied template intentionally exits 2 and lists missing site measurements.
No addresses, K/T, TCP, force, PLC registers or jig coordinates are invented.
The common backend CLI provides explicit read-only status/camera probes. Its
preflight never connects automatically. Follow the combined release instructions
before choosing even read-only device access.

## Real connection path

`RingTaskBridge.read_context()` calls the concrete backend's original libfranka
`readOnce` adapter for both arms and original Native Hand status, retaining the
actual request/receive clock window. `capture`, `hand_move`, `clamp` and
`execute` delegate to the same concrete backend. No placeholder device callbacks
or simulation objects are used by this bridge.

An arm stage uses `StageRequest.create`: explicit side/part/phase, fresh hardware capture
ID/raw SHA/calibration SHA, current both-arm actual and command q, actual Hand
widths, current observed TCP-to-part G, unchanged measured world PART goal,
canonical end q and immutable CSV bytes. The current TCP goal is exactly
`T_cell_part_goal @ inverse(T_tcp_part)`. The arm is explicit: SMALL right pickup and left insertion are both supported; part IDs never infer the active arm. No simulator command/measured bias,
103.4mm palm assumption, old endpoint, synthetic camera or body pose is used.

`RingTaskBridge.execute` checks local ages and immutable bytes, then calls
`HardwareBackend.register_stage` with the complete `task_context`. It checks the
exact canonical registration digest returned by the real backend before calling
`execute_stage(side, stage_key)`. Registration is NOT collision approval. The
backend re-reads actual states/widths, checks exact hardware admission and its
expiry, then delegates to the original registered `Site.motion(..., 'csv', ...)`.
A registration error or partial move consumes the request; no automatic replay,
opening, reset or homing follows an error.

## Planning and remaining blocks

cuRobo IK/routes remain proposals. Convert a current measured-model proposal to
an independently retimed native field CSV with header:

```
time_s,q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7,ddq1,ddq2,ddq3,ddq4,ddq5,ddq6,ddq7
```

The original hardware driver evaluates quintic interpolation at 1kHz. The
simulator's 480Hz emitted arrays cannot be replayed as hardware permission.
Input parsing here checks finite values, increasing time, stopped boundaries,
current command start and canonical endpoint. It does NOT prove FR3 dynamics,
intermediate extrema, torque, payload geometry or continuous collision freedom.
The original native C++ and a separately commissioned, source-pinned full
hardware-world admission executable must check the exact interpolant, actual to
command bridge, both arms/Hands, every opening, payload, installed/free parts,
measured frames and current capture binding. That complete hardware admission
executable is not yet available; **actuation remains blocked**.

Original Native Hand wrapper accepts 5..20N and <=.02m/s. The simulation policy
uses 30N/pad; this bridge rejects that incompatible command rather than clipping
or substituting a force. A measured, separately qualified hardware grasp policy
is required. Original commissioning motion uses a 10% envelope and external
wrench guards 5N/1Nm. No hardware success, hardware speed, full-workflow safety or
simultaneous two-arm execution is claimed.

This package is a concrete motion/I/O boundary for the eight named ring phases,
not a new autonomous ring planner or complete hardware phase coordinator. Fresh
signed annular measurements must be produced and bound by the commissioned
camera/perception pipeline; a supplied `signed_class_valid` flag alone is not
admission. The backend's full checker must authenticate raw acquisition,
calibration and task context. Missing signed-class validation blocks execution.
SMALL/LARGE checkpoint and bounded-recovery rules must remain unchanged when
connecting the task coordinator. Existing full3 and component simulation
cohorts retain their original scope and failure denominators.

## Verification scope

Nine offline tests cover measured target composition, real backend registration
and digest agreement, motion-disabled refusal before drivers load, exact
read-state normalization, stale/future/replay/calibration/start/end/nonfinite
refusal, changed CSV, altered acknowledgement, single-use semantics and force
mismatch. The integration test imports the actual adjacent `hardware_v1`, never
calls a fake device or starts a simulator. No live hardware, network, GPU or
camera access is part of these tests.
