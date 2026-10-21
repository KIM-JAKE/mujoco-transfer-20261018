"""Measured loaded Hand geometry, distinct from a finite-force spring target.

This SIM-only certificate changes ONLY carrier-finger/held-CAD query geometry.
All fixed, other-arm, placed-part and robot-self checks retain their requested
aperture sweeps. Actual finger encoders must stay inside the checked interval,
including a full control-step reach at the unchanged 47.5 mm/s per-finger cap.
"""
import numpy as np


class LoadedContactAperture:
    def __init__(self, *, side, part, payload_path, encoder_samples,
                 sample_times_s, contact, measured_grasp, control_dt_s,
                 capture_age_physics_s, half_interval_m=.00025):
        samples=np.asarray(encoder_samples,float)
        times=np.asarray(sample_times_s,float)
        grasp=np.asarray(measured_grasp,float)
        if side not in ('left','right') or part not in ('ring1','ring2'):
            raise RuntimeError('Loaded-contact certificate requires a native ring carrier')
        if samples.ndim!=2 or samples.shape[1]!=2 or len(samples)<5 or times.shape!=(len(samples),):
            raise RuntimeError('Loaded-contact certificate requires a measured encoder window')
        if not np.isfinite(samples).all() or not np.isfinite(times).all() or np.any(np.diff(times)<=0):
            raise RuntimeError('Invalid loaded-contact encoder window')
        if times[-1]-times[0]<.099 or np.max(np.ptp(samples,axis=0))>.00015 or np.ptp(samples.sum(1))>.00015:
            raise RuntimeError('Loaded-contact aperture has no stationary 100 ms encoder window')
        if np.any(samples<0) or np.any(samples>.04) or np.max(abs(np.diff(samples,axis=1)))>.0002:
            raise RuntimeError('Loaded-contact native coupled encoder geometry is invalid')
        if not contact.get('opposed_contact_compatible') or not contact.get('both_hulls_near_CAD'):
            raise RuntimeError('Loaded-contact geometry requires fresh opposed native pad evidence')
        if grasp.shape!=(4,4) or not np.isfinite(grasp).all() or not payload_path:
            raise RuntimeError('Loaded-contact geometry lacks its image-derived carried transform')
        if not 0<=capture_age_physics_s<=.25:
            raise RuntimeError('Loaded-contact image is too old at binding')
        if not .00025<=half_interval_m<=.0005 or not 0<control_dt_s<=.01:
            raise RuntimeError('Invalid conservative loaded-contact geometry interval')
        self.side=side;self.part=part;self.payload_path=str(payload_path)
        self.lower=np.maximum(0,samples.min(0)-half_interval_m)
        self.upper=np.minimum(.04,samples.max(0)+half_interval_m)
        self.speed_cap_m_s=.0475;self.dt=float(control_dt_s);self.active=True;self._proxy_cache={}
        self.record=dict(side=side,part=part,measured_grasp=grasp.tolist(),
            measured_finger_samples_m=samples.tolist(),sample_times_s=times.tolist(),
            finger_lower_bounds_m=self.lower.tolist(),finger_upper_bounds_m=self.upper.tolist(),
            aperture_bounds_m=[float(self.lower.sum()),float(self.upper.sum())],
            minimum_total_aperture_interval_m=.001,encoder_lookahead_s=self.dt,
            per_finger_speed_cap_m_s=self.speed_cap_m_s,
            fresh_bilateral_CAD_contact=contact,capture_age_physics_s=float(capture_age_physics_s),
            geometry_scope='carrier native fingers against current held CAD only',
            spring_target_is_not_measured_geometry=True,uses_free_body_poses=False)
        self.check(samples[-1])

    def check(self, actual_fingers):
        """Reject BEFORE a next control step could leave the verified interval."""
        if not self.active:return
        q=np.asarray(actual_fingers,float)
        reach=self.speed_cap_m_s*self.dt
        if q.shape!=(2,) or not np.isfinite(q).all() or np.any(q-reach<self.lower) or np.any(q+reach>self.upper):
            raise RuntimeError('Loaded-contact actual encoder aperture could exit its checked geometry interval '+str(dict(actual_m=q.tolist(),lower_m=self.lower.tolist(),upper_m=self.upper.tolist(),next_step_reach_m=reach)))
        if abs(float(q[0]-q[1]))>.0002:
            raise RuntimeError('Loaded-contact coupled finger encoder difference exceeds 0.2 mm')

    def finger_swept_proxy(self, checker, body, geometry):
        """Exact convex extrusion covering the ENTIRE per-finger interval.

        No samples approximate the interval. The convex hull of both endpoint
        copies encloses every native CAD point at every allowed translation.
        The returned proxy is in Hand-root coordinates, not finger coordinates.
        """
        if not self.active or checker.payload_path!=self.payload_path:return None
        names=[checker.arm.hand_path+'/'+self.side+'_left_finger',
               checker.arm.hand_path+'/'+self.side+'_right_finger']
        if body not in names:return None
        key=(body,id(geometry))
        if key in self._proxy_cache:return self._proxy_cache[key][1]
        index=names.index(body);sign=1 if index==0 else -1
        rotation=np.eye(3) if sign==1 else np.diag([-1.,-1.,1.])
        vertices=np.asarray(geometry[3],float)@rotation.T
        points=np.concatenate([vertices+[0,sign*position,.0584]
                               for position in [self.lower[index],self.upper[index]]])
        from predictive_pair_clearance import proxy
        value=proxy(points);self._proxy_cache[key]=(geometry,value);return value

    def release(self):
        self.active=False
