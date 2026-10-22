"""Visible annular faces from RGB-D only, including stacked and tilted rings.

Fixture bounds and the two known CAD diameters are inputs. No object IDs,
renderer geometry, simulator transforms, or random scene seed are accepted.
"""
import time
import numpy as np
from scipy.ndimage import binary_erosion
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .rgbd_geometry import cloud,axis_frame
from .rgbd_annular_cap_gate import annular_cap_gate

def detect(packet,calibration,estimator,desired_part=None,seed=None):
    started=time.perf_counter();P=cloud(packet);rgb=packet['rgb'];floor=np.asarray(calibration['floor_origin_world_m']);normal=np.asarray(calibration['basket_to_world_R'])[:,2];bounds=np.asarray(calibration['outer_bounds_world_m']);height=(P-floor)@normal;min_cos=.05 if calibration.get('allow_sideways_faces') else .45
    gray=rgb.mean(2);metal=np.isfinite(P).all(2)&(np.ptp(rgb.astype(float),axis=2)<45)&(gray>35)&(gray<180)&(height>.004)&(height<.14)&np.all((P>bounds[0]-.003)&(P<bounds[1]+.003),axis=2)
    if calibration.get('allow_sideways_faces') and seed is not None:metal&=np.linalg.norm(P-seed,axis=2)<.048
    if 'fixture_depth_m' in packet:
        fixture=packet['fixture_depth_m'];metal&=(~np.isfinite(fixture))|(packet['depth_m']<fixture-.0003)
    dx=np.roll(P,-1,1)-np.roll(P,1,1);dy=np.roll(P,-1,0)-np.roll(P,1,0);N=np.cross(dx,dy);norm=np.linalg.norm(N,axis=2);N/=np.maximum(norm[:,:,None],1e-14);N*=np.where((N@normal)>=0,1.,-1.)[:,:,None]
    valid=metal&(norm>1e-12)&(np.linalg.norm(dx,axis=2)<.006)&(np.linalg.norm(dy,axis=2)<.006)&((N@normal)>min_cos);ids=np.full(valid.shape,-1,int);ids[valid]=np.arange(valid.sum());rows=[];cols=[]
    for a,b in [(np.s_[:-1,:],np.s_[1:,:]),(np.s_[:,:-1],np.s_[:,1:]),(np.s_[:-1,:-1],np.s_[1:,1:]),(np.s_[:-1,1:],np.s_[1:,:-1])]:
        good=valid[a]&valid[b]&(np.einsum('ijk,ijk->ij',N[a],N[b])>.995)&(abs(np.einsum('ijk,ijk->ij',P[a]-P[b],N[a]))<.00015)
        rows.extend(ids[a][good]);cols.extend(ids[b][good])
    if not valid.any():raise RuntimeError('No visible annular surface in bin RGB-D')
    regions=[];instances=[];masks=[];rng=np.random.default_rng(71);cap_veto_rejections=0
    if not (calibration.get('allow_sideways_faces') or calibration.get('global_plane_faces')):
        graph=coo_matrix((np.ones(len(rows)),(rows,cols)),shape=(valid.sum(),valid.sum())).tocsr();count,labels=connected_components(graph,directed=False);label_image=np.full(valid.shape,-1,int);label_image[valid]=labels;instances=[];masks=[];rng=np.random.default_rng(71)
        counts=np.bincount(labels);regions=[]
        for label in np.flatnonzero(counts>=10)[np.argsort(counts[counts>=10])[::-1]]:
            mask=label_image==label;pts=P[mask];mean=pts.mean(0);_,_,V=np.linalg.svd(pts-mean,full_matrices=False);n=V[-1];n*=1 if n@normal>0 else -1;level=float(mean@n)
            if np.std(pts@n)>.0001 or n@normal<min_cos:continue
            for group in regions:
                if n@group[1]>.9995 and abs(float(mean@group[1])-group[2])<.00015:
                    group[0]|=mask;break
            else:regions.append([mask,n,level])
    if calibration.get('allow_sideways_faces') or calibration.get('global_plane_faces'):
        # After lift, a ring may rotate by tens of degrees. Global observed
        # planes join disconnected annular face fragments without an axis prior.
        regions=[];remaining=metal.copy()
        for plane_index in range(14):
            locations=np.argwhere(valid&remaining)
            if len(locations)<30:break
            sample=locations[np.linspace(0,len(locations)-1,min(160,len(locations))).astype(int)];normals=N[sample[:,0],sample[:,1]];anchors=P[sample[:,0],sample[:,1]];points=P[remaining];points=points[::max(1,len(points)//2400)];levels=np.einsum('ij,ij->i',normals,anchors);score=(abs(points@normals.T-levels)<.00007).sum(0);best=int(np.argmax(score));n=normals[best];level=levels[best];mask=remaining&(abs(P@n-level)<.00009)
            if mask.sum()<65:break
            pts=P[mask];mean=pts.mean(0);_,_,V=np.linalg.svd(pts-mean,full_matrices=False);n=V[-1];n*=1 if n@normal>0 else -1;level=float(mean@n);mask=remaining&(abs(P@n-level)<.00009);remaining&=~mask;regions.append([mask,n,level])
    for mask,_,_ in regions:
        pts=P[mask]
        if len(pts)<65:continue
        mean=pts.mean(0);_,_,V=np.linalg.svd(pts-mean,full_matrices=False);n=V[-1];n*=1 if n@normal>0 else -1
        if n@normal<min_cos or np.std(pts@n)>.0002:continue
        basis=axis_frame(n)[:,:2];boundary=mask&~binary_erosion(mask);edge=P[boundary]@basis;local=pts@basis
        if len(edge)<25:continue
        edge=edge[np.linspace(0,len(edge)-1,min(600,len(edge))).astype(int)];candidates=[]
        pair=rng.integers(0,len(edge),size=(350,2));a,b=edge[pair[:,0]],edge[pair[:,1]];delta=b-a;dist=np.linalg.norm(delta,axis=1);perp=np.c_[-delta[:,1],delta[:,0]]/np.maximum(dist[:,None],1e-12)
        for part,radius,half in [('ring1',.0375,.00775),('ring2',.031,.007)]:
            if desired_part and part!=desired_part:continue
            edge_radius=radius-float(np.nanmedian(packet['depth_m'])/packet['K'][0,0])*1.4
            good=(dist>edge_radius*.5)&(dist<edge_radius*1.98);mid=(a[good]+b[good])/2;side=np.sqrt(np.maximum(0,edge_radius**2-(dist[good]/2)**2))[:,None]*perp[good];hyp=np.r_[mid+side,mid-side]
            if not len(hyp):continue
            error=abs(np.linalg.norm(edge[None,:,:]-hyp[:,None,:],axis=2)-edge_radius);scores=(error<.0012).sum(1)
            for ix in np.argsort(scores)[-12:][::-1]:
                c=hyp[ix];keep=abs(np.linalg.norm(edge-c,axis=1)-edge_radius)<.0013
                if keep.sum()<25:continue
                q=edge[keep];coef=np.linalg.lstsq(np.c_[2*q,np.ones(len(q))],np.einsum('ij,ij->i',q,q),rcond=None)[0];c=coef[:2];measured=float(np.sqrt(max(0,coef[2]+c@c)));keep=abs(np.linalg.norm(edge-c,axis=1)-measured)<.001
                angles=np.arctan2(*(edge[keep]-c).T[::-1]);coverage=len(np.unique(((angles+np.pi)/(2*np.pi)*24).astype(int)))
                if abs(measured-radius)>.003 or coverage<13:continue
                if (part=='ring1' and measured<.0345) or (part=='ring2' and measured>.032):continue
                radial=np.linalg.norm(local-c,axis=1);fraction=float((radial<radius+.0015).mean())
                if np.sum(radial<radius+.0015)<100:continue
                if np.sum(radial<radius-.009)>np.sum(radial<radius+.0015)*.20:continue
                visible_side=1 if n@(packet['T_cell_camera'][:3,3]-pts.mean(0))>=0 else -1
                center=basis@c+n*(float(pts.mean(0)@n)-visible_side*half);initial_center=center.copy();initial_axis=n.copy();r=axis_frame(n);obs=pts[radial<radius+.0015];obs=obs[np.linspace(0,len(obs)-1,min(700,len(obs))).astype(int)]
                # The measured planar face fully fixes axis and axial depth.
                # Preserve it: unconstrained ICP on a thin annulus can drift
                # to a different CAD face under partial visibility.
                rms=float(np.sqrt(np.mean((np.linalg.norm(edge[keep]-c,axis=1)-measured)**2)))
                if rms>.0008:continue
                face_radial=radial[(radial<radius+.0015)&(radial>(.029 if part=='ring1' else .023))]
                if len(face_radial)<40:continue
                inner_face_radius=float(np.percentile(face_radial,5))
                # A large ring's ~32 mm bore can resemble a small ring's
                # 31 mm outside edge. Require observed material INSIDE the
                # hypothesized outside edge; an inner edge has its material
                # on the opposite side. No class/actor labels enter this gate.
                interior=(radial>radius-.009)&(radial<radius-.0012)
                interior_angles=np.arctan2(*(local[interior]-c).T[::-1])
                interior_sectors=len(np.unique(np.floor((interior_angles+np.pi)/(2*np.pi)*24)))
                if inner_face_radius>radius-.0007 or interior.sum()<35 or interior_sectors<7:continue
                threshold=.034 if part=='ring1' else .0275
                local_face_sign=1 if inner_face_radius>threshold else -1
                oriented_axis=n*visible_side*local_face_sign
                orientation_margin=abs(inner_face_radius-threshold)
                instance_mask=mask.copy();instance_mask[mask]=radial<radius+.002
                pose=dict(part=part,initial_arc_center_m=initial_center.tolist(),initial_plane_axis=initial_axis.tolist(),center_m=center.tolist(),axis=oriented_axis.tolist(),visible_face_inner_radius_m=inner_face_radius,visible_CAD_end_sign=local_face_sign,front_back_orientation_margin_m=orientation_margin,front_back_from_RGBD=True,rms_m=rms,depth_points=len(pts),measured_classification_diameter_m=measured*2,visible_arc_coverage_deg=coverage*15,size_classification='RGB-D observed annular arc metric diameter',uses_depth_sensor=True,uses_simulator_body_poses=False)
                if not calibration.get('allow_sideways_faces'):
                    # Initial proposals must not predict a CAD cap where the
                    # current depth camera directly observes empty space.
                    # Apply before candidate ranking and centre deduplication;
                    # an invalid arc must not suppress a valid neighbouring fit.
                    accepted,proof=annular_cap_gate(packet,pose)
                    if not accepted:cap_veto_rejections+=1;continue
                    pose['annular_cap_free_space_gate']=proof
                candidates.append((coverage+len(pts)/5000-rms*500,pose,instance_mask))
        for _,pose,mask in sorted(candidates,key=lambda x:-x[0]):
            if any(np.linalg.norm(np.array(pose['center_m'])-p['center_m'])<.020 for p in instances):continue
            instances.append(pose);masks.append(mask)
    # Exposed, high faces are preferred, independent of original actor identity.
    order=sorted(range(len(instances)),key=lambda i:np.linalg.norm(np.array(instances[i]['center_m'])-seed) if seed is not None else -(np.array(instances[i]['center_m'])-floor)@normal)
    selected=[];selected_masks=[]
    for i in order:
        p=instances[i]
        if any(p['part']==q['part'] for q in selected):continue
        selected.append(p);selected_masks.append(masks[i])
    result=dict(poses=selected,instances=instances,segmentation=dict(backend='RGB-D observed planes and annular diameter; asymmetric taper face identifies signed axis',initial_cap_free_space_veto=not calibration.get('allow_sideways_faces'),cap_veto_rejected_candidates=cap_veto_rejections,SAM_used=False,cached_image_masks_used=False,processing_ms=(time.perf_counter()-started)*1000),depth_sensor_input=True,simulator_body_pose_input=False)
    required={desired_part} if desired_part else {'ring1','ring2'}
    result['both_sizes_visible']=required<={p['part'] for p in selected}
    return result,selected_masks if selected_masks else [np.zeros(metal.shape,bool)]
