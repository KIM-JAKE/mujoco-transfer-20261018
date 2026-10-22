Opt-in canonical PIN pickup branch priority

Merge only overrides/direct_pin_skill.py atop frozen506/507/508/509 direct skill.
Add driver whitelist flag `pin_prefer_canonical_pickup_branch`; set true only with `fixed_supply_mode=true`. No new runtime helper/import.

The original fresh RGB-D retarget, physical −1/−2mm proposal, original calibrated IK and original target remain unchanged. The THREE EXISTING seed sources (taught pin_tail_approach, catalogue pickup_q, already-solved catalogue preq) solve that same newpre target and are tried first, deduplicated by the original .05rad criterion and ordered by proximity to the existing taught seed. Every endpoint still runs original checker.clear and checker.route(seconds=.75); actual issued motion, descend, preshape, retention, full terminal/precision/caps and gain law unchanged. All original GPU and CPU fallback groups remain after the preferred group. No catch is added around proof/motion errors and no retry after partial motion.

New report `pin_canonical_pickup_branch` records literal seed source/requested q, solved q, original endpoint/route admission or rejection, selection/fallback and actual+command q after completed approach. No actor poses are used. The opt-in rejects non-fixed mode before accessing robot context.

Read-only exact-public numeric IK qualification: existing taught seed gives q6=1.58716/q7=−1.22259 at prepose and q6=1.71612/q7=−1.32031 after original seven-knot descend; original bounds/tolerances hold. Catalogue seed alternatives are in public-all-canonical-ik.json. This is NOT whole-scene or physical qualification. Runtime original full guards must admit; original fallback may still be selected and is reported honestly.

10 lifecycle fixtures passed, including disabled behavior, blocked/unavailable preferred route fallback, guard faults, partial-movement fault, all-blocked no-motion, fixed-mode requirement and AST identity of ALL original nested motion/sensor/proof/control functions. No force, mass, geometry, speed, acceptance or iteration budget changes.
