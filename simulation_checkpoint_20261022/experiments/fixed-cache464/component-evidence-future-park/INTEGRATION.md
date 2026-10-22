# Future whole-carry parking admission

Base: frozen463. Add future_saved_carry_guard.py and merge the narrow large_goal_aware_empty_left.py.diff. Fresh-load the new helper after fixed_saved_executor / serial_gpu_pair_scope and before executing the task. No driver flag, geometry, force, mass, timing cap, or clearance threshold changes.

For every candidate whose fresh public held G and BOTH current RIGHT width extrema match a compiled domain, the helper verifies the ENTIRE1416-row stored RIGHT field against the exact float32 planned future LEFT park q and ALL current5×5 apertures. It reuses only existing qualified static, own/SELF and exact GPU pair CLEAR proofs; original path_other_contexts still decides every uncertain/unproved state. The stricter existing terminal sampling resolution remains active. Whole-middle BLOCK rejects this candidate before LEFT motion, and cannot be repaired by a different terminal IK branch. No retry of that same blocked middle is saved as terminal-search fallback.

No matching G/width domain means the existing terminal-only park plus original fresh whole-carry planner remains the policy. No symmetry/gauge normalization is introduced. The future actual encoder pose is unknown before motion: this is explicitly a PLANNED ENDPOINT MODEL proof. Existing current-held-RIGHT protection during LEFT motion and fresh actual post-park whole-path/carry admission remain mandatory and unchanged.

Validation is read-only CAD/public-SDK MODEL replay, NOT an Isaac full task:
- Actual463 local2 planned park has6275 GPU-uncertain states in46825 tighter-resolution/all-width samples. Uncertainty is NOT interpreted as collision. Separate ORIGINAL SAT/LP replay confirms BLOCK near canonical row1171 for BOTH planned q and recorded post-park actual q (maximum tracking difference1.048mrad).
- Public444 and original local0/1/3/4 CPU-IK proposals each have46825/46825 conservative GPU CLEAR pair states with the actual463 fresh held CAD. This proves pair geometry only; original native fixed/SELF/current-held RIGHT safety and the actual LEFT route still must admit any candidate.
- Replayed native LEFT FK endpoint agrees with463's recorded geometric target within0.3µm (numerical model agreement, not hardware accuracy).
- Helper tests preserve full original middle, all width contexts, original guard collision rejection, fresh G attachment, logical empty LEFT, and domain-miss fallback. No actuation occurs in these tests.

Files: pair-replay-MODEL.json, CPU-block-replay.json, helper-tests.json, check_pair_replay.py / check_cpu_block.py. GPU batch costs~4.3s per candidate in this read-only replay; original35s candidate budget is unchanged. Parent owns actual464 test and final qualification.
