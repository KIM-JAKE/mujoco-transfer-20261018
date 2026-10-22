"""Public RGB-D corrections to a saved fixed-pickup branch.

The catalog stores sensor estimates and robot encoder q only. These are priors,
never collision certificates. Every changed pickup/bridge still uses current
native/world/payload/observed-stock/paired/issued guards in loose_ring_pick.
"""
from pathlib import Path
import hashlib,json,numpy as np
from fixed_saved_routes import DATA as ROBOT
DATA=json.loads(r'''{
  "source_run": "native-roas-table-fixed-front-v432-phase-offset-paired-native-all3",
  "source_report_sha256": "e856466876974d23647075b46cf34bd88cb74ea910a9ff5148ecf5c001965949",
  "uses_simulator_body_poses": false,
  "source_inputs": [
    "selected initial RGB-D center/signed axis",
    "fresh lifted RGB-D + capture SDK Hand FK",
    "actual right joint encoder trace"
  ],
  "parts": {
    "ring2": {
      "nominal_RGBD_pose": {
        "part": "ring2",
        "center_m": [
          0.19879948405072131,
          -0.7145313616184643,
          0.8898728296081854
        ],
        "axis": [
          0.050999623449118694,
          6.154181245257211e-06,
          0.9986986724584018
        ],
        "front_back_from_RGBD": true,
        "front_back_orientation_margin_m": 0.0010593313620214893
      },
      "yaw_rad": -1.5707963267948966,
      "approach_offset_m": [
        -0.08,
        0.0,
        0.0
      ],
      "contact_offset_m": 0.0045,
      "nominal_lifted_grasp": [
        [
          0.9988605711826041,
          -6.381472053693104e-06,
          0.04772731305445061,
          -0.00030451652614194025
        ],
        [
          2.0756285638549508e-05,
          -0.9999999589996943,
          -0.0005674163138495123,
          -3.7184516670563106e-05
        ],
        [
          0.047727278205642376,
          0.0005676836168221319,
          -0.998860364930135,
          0.005245828214441972
        ],
        [
          0.0,
          0.0,
          0.0,
          1.0
        ]
      ],
      "pregrasp_q": [
        1.4738483428955078,
        0.5041953325271606,
        1.032783031463623,
        -2.002089500427246,
        -2.733006715774536,
        4.427253723144531,
        2.0952372550964355
      ],
      "pregrasp_q_source_wall_s": 8.934651697054505,
      "canonical_join_q": [
        1.5210565328598022,
        0.43911710381507874,
        1.1344223022460938,
        -2.001211404800415,
        -2.5884838104248047,
        4.401438236236572,
        2.180384874343872
      ],
      "canonical_join_q_source_wall_s": 20.05365191504825
    },
    "ring1": {
      "nominal_RGBD_pose": {
        "part": "ring1",
        "center_m": [
          0.22999599704830434,
          -0.5750528571616497,
          0.8920611478820013
        ],
        "axis": [
          -1.209547188470976e-06,
          -8.718016642000339e-06,
          -0.9999999999612665
        ],
        "front_back_from_RGBD": true,
        "front_back_orientation_margin_m": 0.002072383935253927
      },
      "yaw_rad": 1.5707963267948966,
      "approach_offset_m": [
        0.0,
        0.0,
        0.0
      ],
      "contact_offset_m": 0.005,
      "nominal_lifted_grasp": [
        [
          -0.9999860450881956,
          0.00014443184711579002,
          0.005310189302399735,
          -0.0001512882121375292
        ],
        [
          -0.0001491586049523154,
          -0.999999698739415,
          -0.0009041250346640591,
          1.0596082461344913e-05
        ],
        [
          0.0053100619788468825,
          -0.0009048760278310622,
          0.9999856296040281,
          0.005992141215172442
        ],
        [
          0.0,
          0.0,
          0.0,
          1.0
        ]
      ],
      "pregrasp_q": [
        0.4039134681224823,
        0.41672655940055847,
        2.2156200408935547,
        -2.2982349395751953,
        -2.5823450088500977,
        4.514933109283447,
        -0.8679608702659607
      ],
      "pregrasp_q_source_wall_s": 110.82434351101983,
      "canonical_join_q": [
        0.4181712567806244,
        0.41778960824012756,
        2.236109495162964,
        -2.356355905532837,
        -2.559419631958008,
        4.447268962860107,
        -0.8786002993583679
      ],
      "canonical_join_q_source_wall_s": 164.61793596798088
    }
  }
}
''')
def _enabled(c):
    if not(c['config'].get('fixed_supply_mode') and c['config'].get('fixed_pickup_canonical_priors')):return False
    arm=c['arm'];g={k:np.round(np.asarray(getattr(arm,k)),9).tolist()for k in ['base','tip','frames','axes','bounds']}
    if hashlib.sha256(json.dumps(g,sort_keys=True).encode()).hexdigest()!=ROBOT['robot_geometry_sha256']:return False
    here=Path(__file__).resolve().parent
    return all(hashlib.sha256((here/n).read_bytes()).hexdigest()==h for n,h in ROBOT['native_geometry_sha256'].items())
def pickup_hint(c,part,pose):
    if not _enabled(c) or part not in DATA['parts']:return None
    d=DATA['parts'][part];base=d['nominal_RGBD_pose'];p=np.asarray(pose['center_m'],float);n=np.asarray(pose['axis'],float)
    if not pose.get('front_back_from_RGBD') or pose.get('front_back_orientation_margin_m',0)<=.0005:return None
    if not(np.isfinite(p).all() and np.isfinite(n).all()) or abs(np.linalg.norm(n)-1)>.001:return None
    shift=float(np.linalg.norm(p-np.asarray(base['center_m'])));angle=float(np.arccos(np.clip(n@np.asarray(base['axis']),-1,1)))
    if shift>.030 or angle>.15:return None
    return dict(d,RGBD_center_shift_m=shift,RGBD_signed_axis_change_rad=angle,source=DATA['source_run'],uses_simulator_body_poses=False)
def grasp_admission(c,part,G):
    if not _enabled(c) or part not in DATA['parts']:return None
    G=np.asarray(G,float);nominal=np.asarray(DATA['parts'][part]['nominal_lifted_grasp']);translation=float(np.linalg.norm(G[:3,3]-nominal[:3,3]));angle=float(np.arccos(np.clip(G[:3,2]@nominal[:3,2],-1,1)))
    # Limits select the canonical path family; they never relax native contact,
    # retention or SMALL's original6mm axial-depth gate. A miss uses full guards.
    return dict(eligible=bool(np.isfinite(G).all() and translation<=.0015 and angle<=.06),translation_error_m=translation,signed_axis_error_rad=angle,translation_limit_m=.0015,signed_axis_limit_rad=.06,uses_simulator_body_poses=False,transport_collision_certificate=False)
def join_target(c,part,G):
    admission=grasp_admission(c,part,G)
    if admission is None or not admission['eligible']:return None,admission
    value=c['config'].get('fixed_pickup_canonical_join_q',{}).get(part,DATA['parts'][part]['canonical_join_q']);q=np.asarray(value,float)
    if q.shape!=(7,)or not np.isfinite(q).all()or np.any(q<c['arm'].bounds[:,0])or np.any(q>c['arm'].bounds[:,1]):raise ValueError('Invalid canonical pickup join robot q')
    return q,admission
