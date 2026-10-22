# Registration-cache miss: retain the current fallback

Existing493–496 each had one500-row missing-registration scope. The entire scope, including original CPU and other guards, took0.148–0.178seconds; the failed RPC itself took0.012–0.018seconds. On the three passing197–256second cycles, even removing that entire interval offers less than0.1%. Real retry savings would be smaller because registration, proof and original nonpair guards remain.

The server keeps eight geometry registrations using insertion-order FIFO eviction. The client remembers up to128registration keys. A missing registration safely returns all-false proof bits, discards the key and uses the original CPU guard; the next scope re-registers.

A safe default-off retry could be built, but requires exact structured-error validation, unchanged geometry/input confirmation, one recovery attempt and one remaining-time budget. Current timeouts are per RPC, so simply adding another full-timeout request would not meet the requested inclusive limit. Unknown masks and all other errors must never turn into CLEAR.

Recommendation: do not add this complexity for the measured workload. No implementation, server mutation, GPU benchmark, physics or hardware action was performed. Full source and actual diagnostic hashes are in evidence.json.
