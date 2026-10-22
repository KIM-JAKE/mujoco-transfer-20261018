# Actual525 whole-task PLUS failure: optional PIN depth push was blocked

Actual525 failed at **231.393459s wall** after the final PIN insert and fresh pre-release full-depth observation. Its directly measured center error was **1.320546mm** and axis error **14.744839mrad**, already within the original final1.5mm/50mrad acceptance limits. The original optional extra depth-correction branch nevertheless proposed another push; its original geometry preflight rejected RIGHT link4 against M80 `instance_0216`. The blocked correction was not issued.

This is a whole-task FAILURE, not three-part completion, speed improvement or hardware validation. Release, withdrawal and retention completion were not reached. Existing original physical motor/contact/precision gates were retained. The later combined change that skips an unnecessary push after already-qualified final depth and adds public postrelease retention was NOT present in525. It cannot be treated as tested by this historical failure.

`code/` and `support-code/` contain428 byte-verified frozen input files, including421Python sources. Service sources are copied from the unchanged495 archive. Six last actual public PIN RGB-D packets63–68 include an occluded failed view, final FAR/ENTRY measurements and qualified pre-release full-depth. All68 observation records and original commands/trace are preserved. No new image was generated or substituted.

The active526/527 successor experiments are separate source/results. No main guide, RESULTS, Git index/commit, live job, controller source or worker process was modified to archive525.
