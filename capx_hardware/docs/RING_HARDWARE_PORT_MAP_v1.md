# Ring task motion / I-O port map

This is the code-preparation boundary for the installable task bridge and concrete
hardware backend. No simulation module below is imported by the hardware package.
The original simulation sources remain sealed and are retained as the development
reference. Their per-tick commands are not hardware trajectory files.

| Existing task source / function | Hardware connection | Remaining site qualification |
| --- | --- | --- |
| `loose_ring_pick._execute` / `move`, `line`: actual RIGHT pickup, preclose, breakaway, main lift and safe return | Explicit `StageRequest(side='right', part=..., phase='grasp', ...)` -> `RingTaskBridge.execute` -> real backend `register_stage` / `execute_stage` -> original `Site.motion('csv', ...)` | Native 1kHz quintic retiming from measured-model candidate; current approach/return and full gripper aperture geometry. Old lifted command is not a new proof. |
| `native_right_ring_insert.move`, `line`, `park_empty_left`: RIGHT loaded carry/insertion/correction, EMPTY LEFT park | Explicit active side and phase; exact current start/end + current G/world goal in the stage request | Full both-arm/Hand/payload/placed/free geometry for entire native interpolant and actual-to-command bridge; original planned correction refusal must stay refusal. |
| `loose_bin_task.move`, `trajectory`, `insert_program`: SMALL RIGHT donor -> LEFT receiver/insertion/withdraw | Both sides supported independently of part ID. Registered native CSV stages through the same task bridge | Transfer joint/Hand sequence with actual held-state transitions; paired concurrency has no hardware qualification and must not be inferred from simulation overlap. |
| `close_empty_left_before_large_pickup`: empty LEFT width change before LARGE pickup | `hand_status`, `hand_move` -> original Native Hand -> actual SDK acknowledgement | Continuous swept Hand geometry versus both current arms, installed rings and fresh free stock, including auxiliary PIN as obstacle only. Simulation's per-finger motor profile cannot be copied to real Hand commands. |
| Simulated clamp target writes in `run_realtime_recipe.close_scattered_clamp` | `RingTaskBridge.clamp(part, asserted)` -> real backend -> original mapped PLC coil + feedback wait | Actual verified command/feedback addresses, polarity, interlocks, jaw geometry and supported release transition. No invented clamp force/coil. |
| `Vision.request/poll` virtual fixed camera and renderer | `capture` -> source-pinned persistent RealSense adapter; commissioned signed-ring estimator produces measured evidence | Actual serial, measured K/T, timestamp/exposure registration, signed annular class/pose evidence, quality gates and fixed mount visibility. Simulated registered T/K is not site calibration. |
| `native_ring_component.capture_single` / final checkpoint | Fresh hardware raw capture and actual robot/Hand state; same original completion gates | A new after-withdraw acquisition and measured palm/tool clearance. A supplied bool, old photo or successful simulation final cannot approve hardware completion. |
| GPU IK / route candidates and finite saved robot knots | Current measured-model candidate only, then native CSV + exact hardware admission | Whole-world checker is missing in the current release. No GPU proposal or historical certificate enables movement. |

The task bridge is concrete request and I-O plumbing, not a declaration that all
ring generators have been automatically transformed into a hardware coordinator.
The remaining measured-model retimer/perception/orchestration and commissioned
whole-world admission must be integrated and qualified on site. The package
blocks actuation until those exist. Original simulation force30N/pad conflicts
with the actual Hand wrapper5..20N; no clipping, force substitution or claim of
an unchanged physical grasp policy is made.

The eight phase names remain perception/grasp/carry/preinsert/fixed_insert/
release/withdraw/final_check. Execution must preserve actual Common permission,
single-use action identity, original retry budgets and cancellation. Ring-pair
qualification requires actual SMALL then actual LARGE and NEW final BOTH-ring
observations; a LARGE component with authored installed SMALL is not a pair.
