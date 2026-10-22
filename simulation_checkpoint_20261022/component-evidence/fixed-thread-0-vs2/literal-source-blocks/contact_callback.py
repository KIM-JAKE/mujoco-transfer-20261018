empty_donor_contact_guard={'left':False,'right':False};empty_donor_carried_part={}
empty_donor_payload_peak={'force_n':0.}
def set_empty_donor_contact_guard(side,enabled,part=None):
    if side not in empty_donor_contact_guard:raise ValueError('Unknown empty donor side')
    empty_donor_contact_guard[side]=bool(enabled) and bool(startup_config.get('empty_donor_payload_contact_guard',True))
    empty_donor_carried_part[side]=part;empty_donor_payload_peak.clear();empty_donor_payload_peak['force_n']=0.
    report.setdefault('empty_donor_contact_guard_events',[]).append(dict(side=side,armed=empty_donor_contact_guard[side],carried_part=part,physics_s=globals().get('physical'),force_threshold_n=12.))
contact_side_cache={};ring_contact_debug=[]
def actor_side(encoded):
    if encoded not in contact_side_cache:
        path=str(PhysicsSchemaTools.intToSdfPath(encoded));contact_side_cache[encoded]=0 if('left_arm_'in path or 'left_NativeHand'in path)else 1 if('right_arm_'in path or 'right_NativeHand'in path)else -1
    return contact_side_cache[encoded]
def contacts(headers,data):
    global peak
    for h in headers:
        if startup_config.get('ring_contact_debug') and round(physical/DT)%4==0:
            p0=str(PhysicsSchemaTools.intToSdfPath(h.actor0));p1=str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            if any(paths['workpiece_'+part] in [p0,p1] for part in ['ring1','ring2']):
                # Evaluator-only manifold evidence, never a policy input.
                rows=[data[i] for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data)]
                force=sum(float(np.linalg.norm(row.impulse))/(DT/active_physics_substeps) for row in rows)
                if force>.1:
                    points=[]
                    for row in rows:
                        item={}
                        for field in ['position','normal','impulse']:
                            value=getattr(row,field,None)
                            if value is not None:item[field]=np.asarray(value,float).tolist()
                        points.append(item)
                    ring_contact_debug.append(dict(physics_s=physical,actor0=p0,actor1=p1,force_n=force,contacts=points,evaluator_only=True))
        a=actor_side(h.actor0);b=actor_side(h.actor1)
        if (a==0 and b==1)or(a==1 and b==0):peak=max(peak,sum(float(np.linalg.norm(data[i].impulse))/(DT/active_physics_substeps) for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data)))
        if a in [0,1] and b==-1 or b in [0,1] and a==-1:
            robot_side='left' if (a if a in [0,1] else b)==0 else 'right'
            if empty_donor_contact_guard[robot_side]:
                other_encoded=h.actor1 if a in [0,1] else h.actor0
                other_path=str(PhysicsSchemaTools.intToSdfPath(other_encoded))
                # All ring-stock bodies are prohibited after a verified
                # transfer and empty-donor withdrawal. The selected actor
                # from the evaluator is deliberately never consulted.
                is_ring=('/World/BinStock/' in other_path and 'ring' in other_path) or other_path.endswith(('workpiece_ring1','workpiece_ring2'))
                if is_ring:
                    force=sum(float(np.linalg.norm(data[i].impulse))/(DT/active_physics_substeps) for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
                    if force>empty_donor_payload_peak['force_n']:
                        empty_donor_payload_peak.update(force_n=force,empty_robot_side=robot_side,carried_part=empty_donor_carried_part.get(robot_side),actor0=str(PhysicsSchemaTools.intToSdfPath(h.actor0)),actor1=str(PhysicsSchemaTools.intToSdfPath(h.actor1)),physics_s=physical)
subscription=sim.subscribe_contact_report_events(contacts)
