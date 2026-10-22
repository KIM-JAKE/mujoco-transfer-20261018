"""Additional independently compiled public camera/encoder grasp domains.

No symmetry or pose normalization is assumed. The original CAD is compiled
at EACH literal nominal G before any physical task motion; unmatched G and
unproven rows retain original full live geometry checks.
"""
RIGHT_EXTRA_DOMAINS = [{'label': 'public459_six_mm_chord_RGBD_domain', 'nominal_TCP_object_T': [[-0.8658128129897771, 0.500338364477621, 0.0054815559223657245, -0.005555387648454604], [-0.5003458033583742, -0.8658257774129677, 5.739094788844864e-06, -2.983497148889569e-06], [0.004748945267826223, -0.0027376684981748503, 0.99998511029418, 0.005632705051284548], [0.0, 0.0, 0.0, 1.0]], 'provenance': {'source_run': 'native-roas-table-fixed-front-v459-closure-cached-variation-minus', 'event': 'outer_lifted_fresh_opposed_contact', 'camera_control_source': 'Fresh RGB-D pose transformed by actual robot encoder FK, no free-body actor coordinates', 'source_event_wall_s': 145.80004216800444, 'source_event_sha256': 'eeb012da279d16fee5b93a0d2602ea2eb424b51c7df51aff23a183056a5395c3', 'measured_TCP_object_T': [[-0.8658128129897771, 0.500338364477621, 0.0054815559223657245, -0.005555387648454604], [-0.5003458033583742, -0.8658257774129677, 5.739094788844864e-06, -2.983497148889569e-06], [0.004748945267826223, -0.0027376684981748503, 0.99998511029418, 0.005632705051284548], [0.0, 0.0, 0.0, 1.0]]}}]
