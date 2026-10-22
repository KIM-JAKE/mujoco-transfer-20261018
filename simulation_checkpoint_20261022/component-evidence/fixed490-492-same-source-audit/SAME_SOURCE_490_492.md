# Same-source fixed-supply tests: 2 pass, 1 fail

All 414 Python source hashes match exactly across490/491/492 (canonical source-map SHA256 `7a7bbf82e49d35d9c0e6b13adaec12c5f2d9ec4682b3e4a58e02f80b244ca7f9`). The trial configuration differs only by initialization position/tilt variation.

| Case | Result | Whole wall time | Simulated physics |
|---|---|---:|---:|
|490 nominal|All3 inserted and retained|238.172s|48.167s|
|491 opposed10mm XY shifts|All3 inserted and retained|232.295s|46.787s|
|492 reverse10mm XY shifts|Stopped at ongoing SMALL grasp motion|62.907s|Incomplete|

Both successful complete cycles exceed the20/30second objective. Hardware and hard-realtime control remain untested. This is a small fixed-supply test set, not randomized bin-picking/stacked-ring reliability or a generalization percentage.

Requested±.04rad initial tilts physically settle before observation; the matrix includes the actual first accepted public RGB-D cap axes and centers. Those realized poses differ from requested tilts. The original physical settling policy, native model, force/speed caps and final precision gates remain in effect.

492 stopped correctly. Same-frame public RGB-D and SDK TCP transforms show SMALL relative-axis change31.253mrad exceeding the original25mrad gate, followed by19.343mrad exceeding the5mrad stationary reconfirmation gate. Centers changed130.64µm then65.04µm. CPU-only full-hypothesis image replay and pointcloud plane SVD reproduce the recorded GPU axes within3e-8rad, using over3,244cap points with14.6–15.4µm plane RMS. These are public images/encoders only; no privileged body poses were used. This establishes depth-plane consistency, not a force measurement or proof of hardware behavior.

The two successful runs retain the original caps:30N/pad effort and47.5mm/s finger target. Their issued command and encoder audits pass position-dependent95% velocity bounds and acceleration/jerk caps.492 failed before a final complete issued-field audit; its partial enforced encoder audit is included without substituting it for a completed-task audit.

Each audited profile records exact/width GPU receipt counts, conservative CPU fallbacks, immutable-scope reuse counters, saved-route admissions, instance timer totals, camera latency and hashes of source/config/report/public observations/physical trace. Nested timings overlap. Geometry-cache eviction causes a500-row original CPU fallback in full runs; unknowns never become CLEAR. The profiles do not claim zero fallback or a causal whole-cycle speed gain.

492 public packet archives are published separately under `experiments/fixed-cache492/rgb-perception/`; this directory contains hashes and compact analysis, without duplicated images or masks.
