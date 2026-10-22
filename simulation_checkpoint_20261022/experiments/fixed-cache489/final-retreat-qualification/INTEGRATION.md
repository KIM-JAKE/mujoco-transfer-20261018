# Final empty RIGHT completion

Only `final_empty_clearance.py` changes against frozen488. No perception, control caps, force, physical actor, clamp, mass, friction or contact-gate changes.

Public488 SDK reconstruction shows the old planned endpoint was172.509mm from its nearest calibrated insertion goal, while the immediately sampled actual endpoint was160.727mm. Native-root tracking error was33.706mm; joint6 lagged command by218.326mrad and was still moving at760.836mrad/s. A constant-velocity extrapolation reaches only164.84mm after100ms. This is a genuine incomplete retreat, not a floating-point threshold issue.

Two bounded farther proposals continue the first original lateral direction (2× and1.75×), followed by all original alternatives. Cartesian proposal AND actual float32 IK/retimed endpoint must clear every original calibrated goal by180mm. The actual completion gate remains165mm. On the recorded488 start, the new2× endpoint clears all goals by182.942mm and passes the original FR3 array retiming limits at0.420833s. This is endpoint/limits MODEL evidence only; the unchanged complete world/native/self/current-other/placed-part checks still qualify the actual selected issued field before any motion.

The final field is checked from current actual/command starts, both current LEFT actual and command poses, and all original widths. All18 actual/command DOFs must remain unchanged between proof and first issued row.

If actual encoders already satisfy165mm after issuance, return immediately. Otherwise, permit at most100ms of original controller steps with the command unchanged ONLY if encoders are still moving. Before any extra step, fully check the freshly measured actual-to-existing-command bridge at<=5mrad, all current RIGHT widths and both current LEFT actual/command poses at all widths. Blocked, stale, malformed or stationary-short states stop without another step. Original per-step motor/contact/abort checks remain. There is no recovery/replanning after a partial actuation error.

Seven focused lifecycle cases pass: immediate success, moving completion after full proof, stationary-short rejection, obstructed bridge, stale context,100ms cap and propagation of original step faults. Actual physical qualification is pending root489.
