# Optional exact-row binary pair service

This independent service uses the BYTE-IDENTICAL original pair_kernel.py and binary protocol. It does not edit, restart, reconfigure or benchmark-register geometry in the existing GPU2 pair service.

Apply config-delta.json to choose the new socket. No controller Python change is needed.

- Container: isaac-jake-gpu-2-capacity-ac370e58.
- New container PID:121925 (verify ready.json and /proc/121925/cmdline before cleanup).
- Socket: /workspace/project/mujoco/.local/collision-gpu2-exact-dedup/pair-exact-dedup.sock.
- New service source fingerprint:79b4337c1fdf748b5555c92751b9a801ce80ad4422af724b89829588fbd52453.
- Original kernel SHA-256:39c740a824f8861038519dd08fc5a3e49fba0caf7cb65985dc7b2684cab57096.

Every original bank/index dtype, shape, bounds, geometry registration and margin check runs before deduplication. The helper deduplicates only literal concatenated int32 index rows within ONE request. Equal indices refer to identical transforms in that request's unchanged banks. First occurrence order is preserved. The unchanged original kernel evaluates unique rows; a complete inverse restores all original boolean outputs in their original order. Malformed kernel replies fail closed. No result cache crosses requests, and changed literal banks with the same indices are recomputed.

The original0.5mm per-shape /1mm pair margin, float64 arithmetic,1nm outward reserve, subset-normal policy and positive-CLEAR-only authority are unchanged. False still means unknown. No new OBB prefilter, approximate transform identity, pose tolerance, actor feedback or geometry relaxation is introduced.

`total_rows` retains the original expanded request count. `kernel_unique_rows` and `exact_*` diagnostics report reduced work. OBB/face counters describe unique rows actually evaluated by the unchanged kernel.

Qualification:
- rpc-qualification.json: all55,950 native fixture outputs match the separately evaluated unchanged kernel;28 touching/nanometre-boundary outputs also match. Invalid final original index, dtype, margin and used NaN transforms produce fallback_required with no partial mask. Changing a bank while keeping the same indices recomputes results.
- offline-native-reference.json: original native CAD, public483 donor command interpolation and recorded RGB-D grasp; original4.95s vs dedup2.51s, reverse order5.26s vs2.23s. This is a component model fixture, not a new task execution.
- warm-native-rpc-profile.json: new independent service55,950-row warm RPC2.506s;27,975 unique rows. The first cold integration RPC was3.775s. These are not end-to-end robot or hardware benchmarks.
- frozen-source-manifest.json records the four executable source files; the container's existing NumPy/PyTorch/CUDA environment is unchanged.

Cleanup: verify the new PID's cmdline identifies this directory's pair_binary_server.py, terminate ONLY that PID inside the above container, then remove ONLY this directory's pair-exact-dedup.sock after the process stops. Keep the original collision-gpu2/pair-binary.sock service untouched.
