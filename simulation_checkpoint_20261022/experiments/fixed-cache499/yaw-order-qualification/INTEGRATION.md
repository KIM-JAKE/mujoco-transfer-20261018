Opt-in existing PIN yaw candidate reordering ONLY

Narrow direct_pin_skill.py.diff against417/current+3mm experimentaltarget. Flag pin_tangent_flip_yaw_first=true; rootwhitelist. No newmodule. Onlychanges candidateorder when originalpin_axial_yaw_free=true:180,90,-90,0. Samecalibratedgoalcenter/signedaxis/samecandidatevalues, samefirstadmittedloop (ASTidentical), all originalcurrentG/fullterminal/IK/currentgeometry/route/issued/native/retime/hardwaregates remain. Defaultflagfalse preservesoriginalorder.

Why: public494/497 SDK endpoints imply near180degflip aboutHandclosingY(absaxisprojection.99991/.999996), a physically weak rollingdirection betweenopposed contacts. Equivalentgoalaxialyaw180 hypothesizedflipaboutpad-tangent. Actualjointpath cancontainotherrotations; originalpathintegralYfraction.529/.540, so endpointreordering alone doesNOTestablishactualmotion/retentionimprovement. Component498 yaw0 baseline vs499 candidateexpected; explicitcomponent-only label, no whole-taskclaim.

New+3mm target497 previouslyFAILEDretention197.4s; mustnot replacequalified416checkpoint. Thisstage onlyexperimentalcomparison. Everyadmissionfailuremustretainoriginalstop/fallback, no force/speed/gate/geometry changes. Originaloldcandidate fallback remainsif180notadmissible.
