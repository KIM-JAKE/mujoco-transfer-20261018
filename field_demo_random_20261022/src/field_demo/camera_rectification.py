"""Rectify aligned RealSense RGB-D using the actual SDK lens projection.

The map is built once per stream profile. Frames retain authentic organized
SDK XYZ; no synthetic depth, hole filling or simulator state is accepted.
Nearest-pixel ray quantization is explicitly bounded for occupancy queries.
"""
import hashlib
import json
import time
import numpy as np


class RealSenseRectifier:
    def __init__(self, sdk, intrinsics, *, camera_serial, depth_uncertainty_m):
        self.width, self.height = int(intrinsics.width), int(intrinsics.height)
        self.K = np.array([[intrinsics.fx, 0., intrinsics.ppx],
                           [0., intrinsics.fy, intrinsics.ppy], [0., 0., 1.]])
        if min(self.width, self.height) < 2 or not np.isfinite(self.K).all() or min(self.K[0,0], self.K[1,1]) <= 0:
            raise ValueError('Actual finite SDK intrinsics required')
        if not camera_serial or not np.isfinite(depth_uncertainty_m) or not 0 < depth_uncertainty_m <= .05:
            raise ValueError('Actual serial and explicit conservative depth uncertainty required')
        self.serial, self.uncertainty = str(camera_serial), float(depth_uncertainty_m)
        self.source = dict(width=self.width, height=self.height, K=self.K.tolist(),
                           distortion_model=str(intrinsics.model),
                           distortion_coefficients=[float(v) for v in intrinsics.coeffs],
                           camera_serial=self.serial)
        if not np.isfinite(self.source['distortion_coefficients']).all():
            raise ValueError('Finite actual SDK distortion coefficients required')
        self.u, self.v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        # SDK projection implements the actual distortion convention. Unsupported
        # SDK models fail here before the first live exposure is acquired.
        coordinates = np.empty((self.height, self.width, 2), dtype=float)
        try:
            for y in range(self.height):
                for x in range(self.width):
                    ray = [(x-self.K[0,2])/self.K[0,0], (y-self.K[1,2])/self.K[1,1], 1.]
                    coordinates[y,x] = sdk.rs2_project_point_to_pixel(intrinsics, ray)
        except Exception as exc:
            raise ValueError('SDK cannot project this lens profile; no pinhole fallback: '+str(exc)) from exc
        finite = np.isfinite(coordinates).all(axis=2)
        safe = np.where(finite[...,None], coordinates, 0.)
        pixels = np.rint(safe).astype(np.int64)
        self.inside = finite & (pixels[:,:,0] >= 0) & (pixels[:,:,0] < self.width) & (pixels[:,:,1] >= 0) & (pixels[:,:,1] < self.height)
        self.x = np.clip(pixels[:,:,0], 0, self.width-1)
        self.y = np.clip(pixels[:,:,1], 0, self.height-1)
        self.map_sha256 = hashlib.sha256(json.dumps(self.source, sort_keys=True, separators=(',',':')).encode()+pixels.tobytes()+self.inside.tobytes()).hexdigest()

    def apply(self, packet, metadata):
        started = time.perf_counter()
        if str(metadata.get('camera_serial')) != self.serial:
            raise ValueError('Rectification profile serial mismatch')
        if metadata.get('rectified_registered_depth') is True:
            raise ValueError('Frame already rectified; refusing a second remap')
        if metadata.get('aligned_to') != 'RGB optical frame' or metadata.get('units') != 'm':
            raise ValueError('Aligned RGB optical-frame measured depth in metres required')
        if not np.allclose(packet['K'], self.K, atol=1e-10, rtol=0):
            raise ValueError('Actual frame intrinsics differ from cached lens map')
        rgb = np.asarray(packet['rgb'])
        depth = np.asarray(packet['depth_m'], dtype=float)
        xyz = np.asarray(packet.get('points_camera_image_m'), dtype=float)
        shape = (self.height, self.width)
        if rgb.shape != (*shape,3) or depth.shape != shape or xyz.shape != (*shape,3):
            raise ValueError('Matching organized SDK XYZ and aligned RGB-D required')
        output_depth = depth[self.y,self.x].copy()
        output_xyz = xyz[self.y,self.x].copy()
        valid = self.inside & np.isfinite(output_depth) & (output_depth > .05) & (output_depth < 5.)
        valid &= np.isfinite(output_xyz).all(axis=2) & (output_xyz[:,:,2] > .05)
        # Organized cloud Z and raw aligned depth must describe the same frame.
        agreement = np.abs(output_xyz[:,:,2]-output_depth)
        if np.any(valid & (agreement > max(2*float(metadata.get('depth_scale_m',0.)), 1e-5))):
            raise ValueError('SDK XYZ and aligned depth disagree: frame/registration mismatch')
        if not valid.any():
            raise ValueError('No valid measured rays remain after rectification')
        projected_u = self.K[0,0]*output_xyz[:,:,0]/np.where(valid,output_xyz[:,:,2],1.)+self.K[0,2]
        projected_v = self.K[1,1]*output_xyz[:,:,1]/np.where(valid,output_xyz[:,:,2],1.)+self.K[1,2]
        error = np.maximum(np.abs(projected_u-self.u), np.abs(projected_v-self.v))
        max_error = float(error[valid].max())
        padding = max(1, int(np.ceil(max_error))+1)
        if padding > 4:
            raise ValueError('SDK measured rays disagree with rectified projection beyond supported four-pixel footprint')
        output_depth[~valid] = np.nan
        output_xyz[~valid] = np.nan
        output_rgb = rgb[self.y,self.x].copy()
        output_rgb[~self.inside] = 0
        result = dict(packet, rgb=output_rgb, depth_m=output_depth.astype(np.float32),
                      K=self.K.copy(), points_camera_image_m=output_xyz.astype(np.float32),
                      points_camera_m=output_xyz[valid].astype(np.float32))
        info = dict(metadata, rectified_registered_depth=True, projection_kind='rectified_pinhole',
                    intrinsics=self.K.tolist(), distortion_model='none', distortion_coefficients=[0.]*5,
                    source_lens_profile=self.source, rectification_map_sha256=self.map_sha256,
                    rectification_method='actual SDK lens projection; nearest measured depth and SDK XYZ; holes retained',
                    depth_uncertainty_m=self.uncertainty,
                    ray_footprint_padding_px=padding, maximum_measured_ray_pixel_offset=max_error,
                    deprojection='Authentic SDK organized XYZ remapped with RGB-D; no depth interpolation',
                    rectification_elapsed_s=time.perf_counter()-started)
        return result, info
