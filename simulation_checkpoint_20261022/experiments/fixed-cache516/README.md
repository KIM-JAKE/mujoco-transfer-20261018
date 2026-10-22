# Actual516 PIN-only component failure: −2mm target and qualified-depth flag

Actual516 is a **PIN-only diagnostic**, with no ring pickup/insertion tasks. It failed at **15.547647s wall**, before closure/lift/transport, because fresh holder preclose alignment did not satisfy the original **300µm** position gate within the existing three corrections. Errors were659.7→537.8→411.5→352.4µm. The gate and correction budget were unchanged; the hand did not blindly close. This is not whole-task success or a speedup measurement. Hardware remains untested.

The configured physical collar target was−2mm. The `pin_full_depth_require_robust_circle` flag was enabled, but **full-depth observation was never reached**. Its source/lifecycle proof in `qualified-depth-component-evidence/` is not physical insertion validation. The archived candidate direct_pin_skill.py SHA matches the actual516 source exactly. The later517 preclose-gain source is not included. `trial-config.json` retains its original inherited experiment note verbatim; that stale prose does not override the actual516−2mm config or failure record.

`early-prime-component-evidence/` is a shared-source lifecycle component only; this PIN-only run performed no ring macro/early-prime hit. `code/` and `support-code/` contain427 exact frozen input files, verified by `exact-source-manifest.json`; archived495 service sources are under `service-sources/`. Original PIN RGB-D packets are included with hashes.

The qualified-depth diagnostic folder may contain clearly labelled prior514 evaluator-only materials: these are post-run comparisons and never runtime control, sensing, or516 success evidence. No runtime source, live worker, physics job, global guide, Git index or commit was changed to create this archive.
