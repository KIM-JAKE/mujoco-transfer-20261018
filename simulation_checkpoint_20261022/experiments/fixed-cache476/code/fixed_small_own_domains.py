"""Additional tight own-only grasp domains; all misses use original guards."""
import copy
import numpy as np
DOMAINS=[{'name': 'public460_SMALL_LEFT_stable_RGBD', 'TCP_grasp': [[-0.43698802123976627, -0.7577681538662638, 0.48459182434239084, 0.0044224062996056634], [0.8662666756217525, -0.49958210925406266, -3.970460964778094e-05, -3.466904591020725e-05], [0.24212338146683796, 0.41976833342762393, 0.8747405645029263, 0.0038535831637835916], [0.0, 0.0, 0.0, 1.0]], 'source': 'native-roas-table-fixed-front-v460-cached-stable-nominal', 'RGBD_job_index': 17, 'translation_m': 0.00025, 'rotation_rad': 0.005}, {'name': 'public461_SMALL_LEFT_stable_RGBD', 'TCP_grasp': [[-0.4299802749406265, -0.7451885159999884, 0.5097169611390782, 0.003910742825576019], [0.8661431588762707, -0.4997962254867571, -3.516320299070693e-05, -3.4672296456844665e-05], [0.2547807033471386, 0.4414726709047419, 0.8603423236865789, 0.003919419553620296], [0.0, 0.0, 0.0, 1.0]], 'source': 'native-roas-table-fixed-front-v461-three-domain-cached-variation-minus', 'RGBD_job_index': 17, 'translation_m': 0.00025, 'rotation_rad': 0.005}, {'name': 'public468_SMALL_LEFT_stable_RGBD', 'TCP_grasp': [[-0.43358340376910975, -0.7489210295488035, 0.5011218060317517, 0.003861192239616562], [0.8659960359719132, -0.500047236746765, 0.001966840685360566, -3.119675250859433e-05], [0.24911145416650832, 0.4348222187831477, 0.8653746935148767, 0.003890050048593663], [0.0, 0.0, 0.0, 1.0]], 'source': 'native-roas-table-fixed-front-v468-saved-small-donor-nominal', 'RGBD_event': 'ring2_receiver_final_stationary_contact', 'translation_m': 0.00025, 'rotation_rad': 0.005}, {'name': 'public470_SMALL_LEFT_stable_RGBD', 'TCP_grasp': [[-0.4323712282912961, -0.7488841946006878, 0.5022229844133226, 0.004461269854772576], [0.8661839791373097, -0.4997251741284024, 0.0005510653393023995, -3.2572579615419706e-05], [0.2505606716848535, 0.43525570068739966, 0.864738148351331, 0.003835808360546089], [0.0, 0.0, 0.0, 1.0]], 'source': 'native-roas-table-fixed-front-v470-dense-proof-serial-park-nominal', 'RGBD_event': 'ring2_receiver_final_stationary_contact', 'translation_m': 0.00025, 'rotation_rad': 0.005}]

def prepare(checker,qs,static_certificate,cache_dir):
    from isaac_kinematics import frame
    from fixed_static_route_certificate import StaticRows
    from fixed_own_route_certificate import prepare_like_static
    results=[];summaries=[]
    for domain in DOMAINS:
        metadata=copy.deepcopy(static_certificate.metadata)
        metadata.update(nominal_hand_object_T=(frame([0,0,.1034],np.eye(3))@np.asarray(domain['TCP_grasp'])).tolist(),translation_m=domain['translation_m'],rotation_rad=domain['rotation_rad'])
        # These copied static row bits are NEVER installed as a static proof.
        # Only existing own-collision compilation consumes their metadata.
        proposal=StaticRows(metadata,static_certificate.row_keys)
        own=prepare_like_static(checker,qs,proposal,cache_dir)
        results.append(dict(name=domain['name'],certificate=own['certificate']))
        summaries.append(dict(name=domain['name'],source=domain['source'],RGBD_job_index=domain.get('RGBD_job_index'),RGBD_event=domain.get('RGBD_event'),translation_m=domain['translation_m'],rotation_rad=domain['rotation_rad'],**own['summary']))
    return dict(domains=results,summary=dict(own_only=True,original_full_width_interval_m=static_certificate.metadata['width_interval_m'],original_proximal_contact_clipping_and_1mm_margin_preserved=True,domains=summaries))

def select(checker,state):
    from fixed_static_route_certificate import grasp_in_envelope
    current=getattr(checker,'payload_relative',None)
    if current is not None and checker.payload_path:
        for entry in state.get('additional_own_domains',[]):
            certificate=entry['certificate'];m=certificate.metadata
            if grasp_in_envelope(np.asarray(m['nominal_hand_object_T']),np.asarray(current),m['translation_m'],m['rotation_rad']):
                return certificate,entry['name']
    return state.get('own_certificate'),'original_broad_domain'
