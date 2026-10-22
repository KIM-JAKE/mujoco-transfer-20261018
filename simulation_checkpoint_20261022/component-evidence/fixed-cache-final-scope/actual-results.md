| Run | Actual outcome | Wall from capture | Simulated time | Saved RIGHT admitted | Native SELF / own payload query hits |
|---|---|---:|---:|---|---:|
| 450-precision-cached-variation-plus | PASS | 244.98s | 53.68s | no recorded admission | 0 / 0 |
| 460-cached-stable-nominal | PASS | 234.93s | 45.31s | yes | 14300 / 7085 |
| 461-three-domain-cached-variation-minus | PASS | 241.14s | 57.25s | yes | 14300 / 7085 |
| 462-three-domain-cached-variation-plus | FAIL | 177.89s | — | no recorded admission | 7215 / 0 |
| 463-live-entry-cached-variation-plus | FAIL | 199.84s | — | no recorded admission | 7215 / 0 |

Saved actual pipeline-results.json only. Query hits are operations, not independent trials. MODEL/preparation proof counts are not task passes. Missing final result is not success.

First-capture wall and simulated physical time reported separately. Offline preparation excluded from cycle. Jobs may run concurrently; differences are observations, not isolated causal benchmarks. GPU scope wall includes nested original CPU checks and is intentionally not called GPU compute.
