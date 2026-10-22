# Qualified final PIN depth observation

Base: frozen512 direct_pin_skill.py SHA5244ae81. Merge only overrides/direct_pin_skill.py; new driver whitelist flag pin_full_depth_require_robust_circle=true. No new helper/import.

Both the initial full-depth observation and each existing axial-correction reobservation now request require_circle=True through the SAME original observe() and original13-view bank. Original class-radius/240degree/inlier/edgeRMS detector gates remain authoritative. The caller also rejects a depth result missing robust_cap_circle_from_pixels=True before any use of its Z. Failed views follow the original view fallback; backend/time/physics faults propagate. No new alignment budget, view set, delay, force or geometry change. Flag requires original visual-preinsert and robust circle settings.

Diagnostics pin_full_depth_qualified_observations record successful view index/timestamps and original cap-quality fields. Original depth/axis acceptance, all motion/proof functions, robot caps, fixed/pair/placed/native guards, full release/withdraw and physical retention checks are unchanged. Flag-off original behavior verified.

Actual514 final21point cap fragment reproduced bitwise with originalrobust=False and was rejected by originalrobust=True as lacking a240degree class-radius arc. Evaluator-only trace separately found realphysical PIN axial slip of about6.744mm and final depth-camera2.573mm Zbias; requiring qualified cap prevents a blind push from a low-quality image but does NOT fix or qualify the physical grasp. No engine/body poses enter this runtime patch.

Nine lifecycle fixtures passed using literal new observe() and actual514 public pixel-pose fixtures: rejected fragment then valid existing view, all13bad fail, originalflagoff result, firstvalid noextra frame/step, backend fault propagation, original3second timeout, unrelated observation invariance, alloriginal motion/proof/servo/retention functions ASTidentity, both initial/correction callsites qualified.

Optional collar-minus2-candidate-config-delta.json is an UNQUALIFIED contact experiment using the existing bounded−2mm target helper, NOT a default. Local originalCAD first100micron flat collar span improves from1.45/1.55mm(−1) to2.15/2.25mm(−2). At the fixture, however,−2 raises Hand~.766mm and reduces exact0554subset gap from1.820mm to1.054mm at514issuedendpoint. This positive subsetgap is not a full native-scene/issued-route admission; every original guard must still pass. Root owns all component/fulltask trials. No worker or main source was changed here.
