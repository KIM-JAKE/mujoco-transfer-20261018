# Accepted357: guarded smooth GPU carry

Random seed446 completed both ring insertions, pin insertion, clamp retention and final empty-Hand retreat in470.50wall/59.07physics seconds. Two local small-ring rearrangements were required. This is a second pass on the same layout, not a new generalized-layout success.20/30seconds and actual hardware remain unverified.

Only the initial native RIGHT carry may use the planner-marked cubic through the same knots. Every float32 issued row plus actual starting encoders and all five native openings is checked against the original geometry, current LEFT arm and payload before movement. A pre-motion rejection retains the original polyline fallback. Actual large-ring carry took1.86875physics seconds rather than12.22292 with stops at each route knot. FR3 velocity, acceleration, jerk, native finger speed and force caps remain unchanged. Parallel total wall times are observed values, not a controlled timing comparison.

These are exact top-level Python sources. Server CAD/generated geometry, bin models/calibration, earlier class-only robot priors and Isaac runtime remain external; this is not a standalone scene. Controller uses simulated RGB-D and encoders. Trace actor poses are evaluation/replay data only. The standalone field package is separate and still has approach-admission blockers.

The report's nested `scattered_pick.full_task_pass` is provisional metadata emitted before final evaluation; the top-level `full_task_pass` and `vision_end_to_end_pass` are the final aggregate. The original report is preserved verbatim.

CPU planning calls pause simulation physics (`CPU_planning_advances_physics=false`), while all waits still count in the reported wall clock.59.07physics seconds is therefore not a real-hardware cycle-time prediction; physical continuous holds during long planning calls are not validated by this trial.
