# 518 · PIN-only −30° grasp candidate: rejected

This diagnostic did not run the two rings and is **not a whole-task success**. It physically changed only the empty-Hand pickup target; the original CAD, mass, friction, motor forces, timing bounds, solver and collision guards were preserved. Source archive contains421Python/428source+fixed inputs with original hashes.

The −30° candidate passed the original300µm preclose gate and direct RGB-D/SDK postlift retention checks (44.11µm / .834mrad). During the subsequent physical carry, actual finger speed reached118.14mm/s,2.4871times the original47.5mm/s acceptance reserve. The original online guard stopped the trial at29.206472seconds. This candidate is rejected: the controller's bounded commands did not guarantee acceptable physical contact behavior. Postcarry, insertion, qualified full-depth and retention were not reached. No limits were relaxed or measured states clipped.

`tilt-target-qualification/` includes exact target geometry, invariants and the actual failed audit. Its positive static subset clearances were proposal evidence and did not qualify the physical grasp. The default successful historical checkpoint494 remains unchanged. Hardware execution and overall waiting-time reduction remain unverified/unmet.
