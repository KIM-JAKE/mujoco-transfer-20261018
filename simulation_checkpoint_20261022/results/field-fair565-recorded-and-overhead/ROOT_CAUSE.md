# First retained UNKNOWN rejection

The first new-overhead candidate fails on **right link5 column7_3**. Its unchanged CAD enclosure radius is13.359mm, plus the offline fixture's2mm cable allowance. Stopped endpoint sampling adds no temporal sweep. The original checker adds19.268mm for a5mrad joint-tracking budget, then a1mm collision margin. Default20mrad would add77.074mm to this same column.

The nearest **actual UNKNOWN4mm cell**, [1,72,29], is centered at [0.428331,−0.628524,1.193343]m. Exact cell distance and merged-box distance both equal33.1656mm, so merging has not filled a free gap. All110 pixels in its projected footprint are valid and within the overhead image. The nearest observed basket-side surface is at camera-Z1.074131m; the farthest voxel corner is1.073491m. Adding the unchanged3mm depth bound prevents a whole-cell free certificate by2.3598mm. This is a real retained uncertainty boundary near the basket side, rather than absent FOV or a depth hole.

Nominal column+cable+1mm margin clears this cell by16.806mm. Tracking inflation reaches it: at5mrad clearance is−2.462mm. The single-column threshold would be4.361mrad, which is **only nonexecutable model sensitivity**, not a proposed approved limit. No commissioning evidence or runtime setting has been changed. A CAD enclosure overlapping an uncertain cell does not prove actual robot CAD collision. It explains the conservative rejection. This first case does not establish that every target is blocked solely by tracking uncertainty.

The JSON includes the genuine saved robot FK state, exact capsule, ray footprint and both baseline20mrad/opt-in5mrad first failures. No robot/Hand command was sent. No additional view or posture grid was run.
