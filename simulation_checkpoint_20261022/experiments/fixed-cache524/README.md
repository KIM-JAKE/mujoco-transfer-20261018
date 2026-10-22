# Historical actual524 whole-scene PASS; PIN postrelease sensing gap remains

The original524 evaluator reported ALL3 physical/vision completion at **213.256445s wall /49.272917s physics**, with original robot limits and no attachments or post-initialization teleports. This is the historical source's result for the nominal layout, not a current sensor-only qualification, a three-case guarantee, a time-target success or a hardware test.

Important completion limitation: original524 used fresh RGB-D for perception, grasp/ENTRY and pre-release insertion-depth confirmation, but after PIN release/withdrawal/retention it read simulator `truth['pin'].get_transforms()` to set its PIN pass/fail. **There is no fresh postrelease PIN RGB-D check in this source.** The selected packet is explicitly PRE-RELEASE full-depth evidence. The separate newly staged sensor-authoritative postrelease patch was not present or physically validated in524; this archive does not retroactively claim it was.

The421Python/7JSON frozen input files are byte-verified against exact-source-manifest.json; authoritative source lives undercode/support-code. Original495 service sources are copied unchanged. The24mm precontact-entry geometry patch and original tests are preserved as component evidence. Raw command/encoder and original scene-trace files remain available for auditing or camera replay; simulator body poses are rendering/evaluation data, not proof of a new public sensor observation.

A prepared video replays all213.26s of original wall time at1x, including planning/sensing waits, from all34 recorded body poses. It is Warp CAD ray rendering, not RTX or a new physical simulation. Its overlay/manifest explicitly labels this historical postrelease completion gap. No render job was launched while physical tests were active.

No global guide, RESULTS, Git index/commit, live job or controller source was modified to create this archive.
