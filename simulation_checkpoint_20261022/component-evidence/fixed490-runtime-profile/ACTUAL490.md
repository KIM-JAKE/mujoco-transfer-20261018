# Actual issued-guard and GPU transport profile

490 completed all three physical Isaac Sim insertions and final retention: 238.172242 seconds from first capture, including camera/perception/planning; 48.166667 seconds of simulated physics. The time target remains unmet, and real hardware remains untested.

The instance-only profiler detached with zero errors and retained protected proof method identities. Timings include diagnostic overhead; methods and scope intervals nest and must not be added as independent elapsed time.

| Measured method | 489 calls / seconds | 490 calls / seconds |
|---|---:|---:|
| Completed-part guard | 55,093 / 12.556 | 54,628 / 4.751 |
| RIGHT own-payload guard | 21,624 / 14.015 | 20,993 / 12.165 |
| LEFT own-payload guard | 13,105 / 16.870 | 13,272 / 17.148 |
| RIGHT USD/PhysX transforms + query | 21,744 / 9.042 | 21,114 / 8.240 |
| LEFT USD/PhysX transforms + query | 10,789 / 4.276 | 10,955 / 4.560 |

The observed completed-part mean fell from 0.228 to 0.087 ms/call. This is descriptive evidence on two actual task runs, not a controlled same-query speed benchmark. Original issued paths, PIN correction count, source and physical trace differ. Broadphase hit counters were not serialized by the frozen runtime, so no actual hit-rate claim is made.

490 co-moving scope counters report LEFT 5,656 fast attempts / 5,567 returns and RIGHT 1,528 / 1,326. These are immutable-scope positive CLEAR reuses; unknowns continue through original guards. The width-sweep normal path now includes these scopes. Own-payload checks and native USD/PhysX queries still account for substantial CPU work.

490 exact-state pair receipts: 124,124 input rows, 123,524 GPU CLEAR and 600 original CPU fallbacks. Of the fallbacks, 100 were conservative GPU unknowns and 500 were a safely rejected missing-registration receipt after server geometry-cache eviction. Three context misses also retain original guards. The 12 width-sweep scopes separately certified all 4,688 q samples. Successful replies use the expected frozen server source SHA, float64, unchanged 1 mm pair clearance, 1 nm outward reserve, and exact index-row deduplication. Total distinct pair client RPC time across exact-state and width-sweep calls was 9.042 seconds, original kernel time 6.086 seconds. Scope-inclusive wall intervals overlap guarded work and should not be summed as savings.

The earlier 488 experiment accidentally selected JSON because the client dispatcher requires a socket basename ending `pair-binary.sock`; all GPU requests safely fell back to original CPU checks. The corrected separate alias was qualified through the actual production dispatcher/client on the 55,950-row native fixture before 489. The 488 profile is retained as failure evidence, not an exact-dedup GPU benchmark.

490 RGB-D: 62 images; capture-to-ready median 0.245 s, p95 0.618 s, maximum 1.600 s; no 3-second deadline violations. Aggregate inference was 14.510 s. The 38.591 s from readiness to delivery mostly overlaps planning and must not be labeled camera latency or independently removable waiting.

Source/report/config/trace hashes are included in the audited profiles. No new physics, control inputs, source changes or GPU benchmark was performed to collect this report.
