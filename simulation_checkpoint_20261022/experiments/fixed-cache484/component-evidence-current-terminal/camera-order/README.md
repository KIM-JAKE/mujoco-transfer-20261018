# Fixed LARGE pickup camera-order candidate

Base: frozen479 loose_ring_pick.py. Apply narrow diff and whitelist/enable `fixed_large_pregrasp_preferred_views`. Direct PIN module is untouched.

For fixed LARGE pregrasp only, when the image-derived part-to-Hand translation is at most0.10m, first request the two existing views that exposed the annulus in480: −short×0.30+[0,0,0.30] and worldoffset[−0.28,−0.20,0.32]. All original views, including original primary, remain in the fallback bank. The distant initial pre-approach (about0.60m in480) retains its original order. Every request captures new pixels; original support and two-fresh-pose3mm/0.06rad consistency gates are unchanged. No image, mask, object pose or acceptance result is cached.

qualification.json replays the original decision rules on public480 packets/results. Four near-pickup groups had original support pattern[fail,fail,pass,fail,pass]. The reordered two qualifying views satisfy the same original consistency test and select exactly the same final measured pose. Saved camera extrinsics identify each original view about the directly measured center (within3mm). This is not a fresh physical execution or timing benchmark.

Measured480 readiness profile:73 images;1.867s capture,20.755s perception,0.526s queue. The12 failed captures avoided by this ordering used0.156s capture+2.665s perception. Separate44.215s aggregate ready-to-delivery delay mostly overlaps control/collision planning; it must not be reported as camera latency or assumed removable from total wall time.

A complete fresh IsaacSim test with original collision and physics guards is required. New camera order can change physical timing; no source-only success or end-to-end speed claim is made.
