# Cross-arm collision counterexample: seed450

The original pair checker rejects both the commanded and measured robot states at the failure: RIGHT link4 versus LEFT link7. The pose model agrees with actual recorded links/Hand to 0.477 micrometres and 1.090 microradians across early/late/final snapshots. LEFT stayed stationary. This was a donor-first plan, so a receiver-first context bug does not explain this example.

The original physical contact monitor stopped execution at22077N. The static diagnostic sends no robot commands and writes no physical actor poses. It only queries numeric FK and original pair geometry. Diagnostic body poses are evaluation-only. Hardware has not been tested.

The legacy carry executor lacked the exact issued-row/current-other-arm admission already used by the direct insertion skill. A clear check at route knots or sparse path samples is insufficient to approve unverified issued rows. The correction is to reject/replan before motion under the full original pair/payload guards, without reducing margins.

`reports/crossarm-counterexample.json` contains all original-checker answers and FK comparisons. The command state uses commanded width; the measured state uses actual width. Width does not change either offending rigid arm link, so this collision result is exactly invariant to using actual width for command FK.

The source files reproduce the read-only diagnostic. The driver diff only skips unused fixed receiver-catalog preparation. The remaining base is identified by the frozen source manifest. Large original recordings are referenced by hash and intentionally not duplicated.
