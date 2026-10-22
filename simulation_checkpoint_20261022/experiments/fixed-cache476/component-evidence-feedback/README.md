# Bounded independent PIN feedback candidate476

Private patch against `/tmp/fixed475-full/overrides/direct_pin_skill.py`.
Apply `direct_pin_skill.py.diff` and merge `config-delta.json`; whitelist `pin_visual_independent_component_feedback` in the trial dispatcher. The opt-in flag leaves the old target exactly unchanged when false.

Actual474 completed both rings but blocked PIN insertion after16 fresh observations. Its first unit correction reduced physical-axis image error from6.192 to0.877mrad. Subsequent XY oscillation lowered the shared gain, including rotation, to0.3; physical tilt later remained4–8mrad. Independent postrun image-versus-physics evaluation found at most0.283mrad axis estimation bias, so perception changes are not indicated by this case.

The candidate starts center-XY feedback at0.5, retaining the existing0.3 floor and0.65 non-improvement reduction, triggered only by unqualified radial error. Axis correction retains original unit gain whenever its original3mrad gate is unqualified. Both original component deadbands remain. Independent gains are applied to measured physical-part center and axis before deriving the Hand pose from measured grasp and encoder tracking bias. This avoids translating the part around the grasp lever when center and orientation gains differ.

Unchanged:16-observation budget, final75µm center /3mrad axis /150µm complete-shaft gates, fresh RGB-D/SDK hold checks, original2mm/.01rad tracking-bias and8mm/.1rad per-correction bounds, actual payload/collision guards, FR3 caps, physical model, and robot controls.

`public474-replay-check.json` is ONLY a kinematic target invariant check using16 real public RGB-D observations and encoder FK. It checks exact legacy target preservation and desired part-center/axis behavior, including holding a qualified center during axis correction. It does not predict physical success or runtime. A new full IsaacSim run is required.
