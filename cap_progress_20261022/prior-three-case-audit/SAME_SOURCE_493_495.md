# Current416-Python fixed-supply build: three complete task passes

| Case | Whole wall time | Simulated physics | PIN corrections |
|---|---:|---:|---:|
|493 PLUS|212.540s|50.233s|6|
|494 nominal|196.697s|46.125s|3|
|495 MINUS|256.086s|57.929s|13|

All three completed both rings, the pin, release, final robot clearance and retention. All remain over20/30seconds; hardware and hard-realtime validation remain false.493 ran without another physical trial;494/495 ran concurrently on assigned GPUs3/2, sharing the GPU3 perception and GPU2 pair-proof services. These are actual case outcomes, not isolated latency benchmarks.

All416Python hashes match exactly (map SHA256 d513ce8359dea0b56ed9dcd0d8d244098e5fac39d63be7b02c4f9f9667ee6e8c). The published423fixed inputs, including seven JSON inputs, also match across all three archives; every actual file byte hash was checked (map SHA256 b4f367fe66d245b7a9a8a1cefee1a4645af119118289ff39b5b04d3bf55da5da). That is an archive hash verification, not an additional launcher/physics test.

This covers three fixed-shelf conditions: nominal and opposing10mm XY initialization shifts, with requested±.04rad initial tilts. Original physical settling remains; measured settled public RGB-D centers/axes are included and do not equal all requested initial tilts. This is not broad random-bin, stacked-ring, tilt-envelope or hardware reliability qualification. The older414-Python build's2PASS/1FAIL matrix is retained separately.

Saved-route reuse remains conditional. SMALL donor carry was admitted in all three, with5,345static/native/own certificate hits per admission. LARGE canonical carry was admitted in all three, with7,085static/native/own hits. The stored720-row SMALL LEFT middle was admitted in494/495.493 PLUS rejected it at the original current RIGHTlink4–LEFTpalm1mm pair guard, then completed using a fresh original replan. Do not claim every case reused every middle or checked only changed segments. Current payloads, opposite-arm states, widths, dynamic pairs and unknowns retain original guards.

493's balanced30° receiver achieved the fixed intended preclose grasp within76.89µm/1.356mrad after one original-bounded correction. Public postgiver→stable-hold relative-center change was2.30µm and original raw-axis comparison0.770mrad, passing the unchanged25mrad gate. Later measured relative grasp changed0.571mm/41.025mrad during carry, within the existing transport policy: handover stabilization does not establish that all slip/movement was eliminated. The raw public sequence is retained.

495 was slower descriptively. Ring completion occurred at194.875wall seconds versus157.040in494; PIN plus final retreat then took61.211versus39.657seconds.495 needed13PIN corrections/14fresh observations versus3/4in494, each correction0.35physics seconds, and executed1.40versus0.508physics seconds of encoder-window steps. Total physics was57.929versus46.125seconds. Accumulated wall time inside physics-step calls was119.908versus97.533seconds. Paths, observations and concurrency differ, so these are measured contributors rather than an isolated causal explanation.495 also remains slower than the earlier different414-source MINUS491 outcome232.295s.

All three original issued-command and enforced encoder speed audits pass.30N/pad effort and47.5mm/s finger target caps remain. No part attachments or post-initialization workpiece teleports are reported. Qualified preinsert PIN precision gates are stage-specific; raw post-release retention evaluator results are included without claiming a guaranteed final3mrad pin axis.

Public RGB-D capture-to-ready maxima were1.177/1.011/1.120seconds for493/494/495, with no3second deadline violations. The instance profiler restored its methods with zero errors and protected proof identities unchanged. Exact/width GPU receipts, conservative CPU fallbacks, local-only CAD intern counters, saved-route events, nested timings and all source/report/config/trace hashes are included in each audited profile. No new benchmarking, physics or hardware actuation was performed for this audit.
