# Frozen 540 controller: preservation bundle

This directory preserves the existing successful executions 540 NOMINAL, 542 PLUS
and 543 MINUS. These are three execution records, not five repetitions per case.
No new physics run or policy modification was performed by the exporter.

- `experiments/fixed-cache540`: all 425 actual frozen Python sources, the original
  7 fixed JSON inputs, the exact NOMINAL configuration and full recorded data.
- `experiments/fixed-cache542` and `fixed-cache543`: independent exact run
  configurations and full recorded data, sharing the identical 540 code manifest.
- Every run includes all raw RGB-D packets/overlays, commands, actual encoders,
  PhysX trace, robot model, observation metadata and pipeline reports. Postrelease
  PIN packet identity and checksum are explicit. Evaluation-only fields in raw
  reports remain labelled; they are not public controller observations.
- `local-heavy-assets`: large camera/CAD data retained locally by content hash.
  Each run records its exact heavy asset location and SHA-256.

The original 425-Python aggregate is
`c90a297ee51bf2d8ba785c655137606abed751758e5e930e16168cc2a0c0a10c`.
The 432-input aggregate, also including the 7 original JSON files, is
`e2bdfd32b65fe5874c0bfeeb79fd54b07b46344cd4c75d4fbd2b247192ec619c`.
Both use SHA-256 over sorted-key compact JSON mapping original repository paths
to SHA-256 values; the Python-only hash does not include the JSON inputs.

The active physical target remains the original collar offset -1 mm. Broad-tail
and -55 degree collar proposals are not active in these successes. Controller
changes, force/friction changes, slower task settings, or failed candidates must
not be folded into this frozen cohort.

`export_cohort.py --versions 544 545 ...` preserves future completed runs only if
the frozen source and fixed JSON maps match. Existing experiments are verified
and never overwritten. `archive-SHA256.json` authenticates each preserved file;
`exact-source-manifest.json` and `SOURCE_SHA256SUMS` authenticate primary code.
The exporter is a preservation tool and never starts a simulator or robot.

The first three wall times are 255.01 / 228.84 / 228.33 seconds. They do not meet
20 or 30 seconds. No hardware readiness, real calibration robustness, random bin
generalization, or repeated-case success rate is established by this bundle.
