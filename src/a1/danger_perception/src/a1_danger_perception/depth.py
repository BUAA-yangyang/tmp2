from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo

from .detector import Candidate


@dataclass
class DepthParams:
    min_depth_m: float
    max_depth_m: float
    depth_percentile: float
    min_depth_valid_ratio: float
    max_depth_std_m: float
    estimate_sphere_center: bool
    sphere_radius_m: float
    depth_mask_erode: int
    fallback_camera_frame: str


@dataclass
class DepthEstimate:
    point: Optional[PointStamped]
    status: str
    depth_median: float
    depth_valid_ratio: float
    depth_std: float


class DepthLocalizer:
    """Estimate a 3D point for a 2D image candidate using depth and intrinsics."""

    def __init__(self, params: DepthParams) -> None:
        self.params = params

    def estimate(
        self,
        candidate: Candidate,
        depth_m: np.ndarray,
        camera_info: CameraInfo,
        stamp,
    ) -> DepthEstimate:
        depth = np.asarray(depth_m, dtype=np.float32)
        if depth.ndim == 3:
            depth = np.squeeze(depth)
        if depth.ndim != 2:
            return DepthEstimate(None, "invalid_depth_image", 0.0, 0.0, 0.0)

        mask = np.zeros(depth.shape, dtype=np.uint8)
        cv2.drawContours(mask, [candidate.contour], -1, 255, thickness=-1)

        erode_size = int(self.params.depth_mask_erode)
        if erode_size > 0:
            kernel_size = 2 * erode_size + 1
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)

        object_pixels = mask > 0
        object_count = int(np.count_nonzero(object_pixels))
        if object_count == 0:
            return DepthEstimate(None, "empty_depth_mask", 0.0, 0.0, 0.0)

        object_depths = depth[object_pixels]
        valid = (
            np.isfinite(object_depths)
            & (object_depths >= self.params.min_depth_m)
            & (object_depths <= self.params.max_depth_m)
        )
        valid_depths = object_depths[valid]
        valid_ratio = float(valid_depths.size) / float(object_count)
        if valid_depths.size == 0 or valid_ratio < self.params.min_depth_valid_ratio:
            return DepthEstimate(None, "insufficient_valid_depth", 0.0, valid_ratio, 0.0)

        depth_surface = float(np.percentile(valid_depths, self.params.depth_percentile))
        depth_std = float(np.std(valid_depths))
        if depth_std > self.params.max_depth_std_m:
            return DepthEstimate(None, "depth_too_noisy", depth_surface, valid_ratio, depth_std)

        z = depth_surface
        if self.params.estimate_sphere_center and candidate.class_name == "red_sphere":
            z += self.params.sphere_radius_m

        fx = float(camera_info.K[0])
        fy = float(camera_info.K[4])
        cx = float(camera_info.K[2])
        cy = float(camera_info.K[5])
        if abs(fx) < 1e-6 or abs(fy) < 1e-6:
            return DepthEstimate(None, "invalid_camera_info", depth_surface, valid_ratio, depth_std)

        u, v = candidate.center_uv
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = camera_info.header.frame_id or self.params.fallback_camera_frame
        point.point.x = (float(u) - cx) * z / fx
        point.point.y = (float(v) - cy) * z / fy
        point.point.z = z

        return DepthEstimate(point, "ok", depth_surface, valid_ratio, depth_std)
