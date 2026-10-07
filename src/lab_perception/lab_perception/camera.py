"""
Pinhole model of a camera at a fixed pose in the world.

Pure Python and NumPy (no ROS). Depth images hold the distance along the
optical axis in metres, as Gazebo's RGB-D camera publishes them. The camera
link frame looks along +x with +z up; the optical frame looks along +z with
+x right and +y down in the image.
"""

import math
from typing import Sequence

import numpy as np


def _rotation(yaw: float, pitch: float) -> np.ndarray:
    """Return the world rotation of the camera link: yaw about z, then pitch about y."""
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return rz @ ry


class CameraModel:
    """Intrinsics plus the camera link pose (position, yaw, pitch; no roll)."""

    def __init__(self, fx: float, fy: float, cx: float, cy: float, width: int, height: int,
                 position: Sequence[float], yaw: float, pitch: float):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.width, self.height = width, height
        self.position = np.asarray(position, dtype=float)
        self.rotation = _rotation(yaw, pitch)

    @classmethod
    def from_info(cls, k: Sequence[float], width: int, height: int,
                  position: Sequence[float], yaw: float, pitch: float) -> 'CameraModel':
        """Build from a sensor_msgs/CameraInfo K matrix (row-major 3x3)."""
        return cls(k[0], k[4], k[2], k[5], width, height, position, yaw, pitch)

    def rays(self) -> np.ndarray:
        """
        Return per-pixel ray directions in the world, shape (H, W, 3).

        Each ray is scaled so that one unit along it is one metre of depth
        along the optical axis: a pixel's point is position + depth * ray.
        """
        v, u = np.mgrid[0:self.height, 0:self.width].astype(float)
        x_opt = (u - self.cx) / self.fx
        y_opt = (v - self.cy) / self.fy
        link = np.stack([np.ones_like(x_opt), -x_opt, -y_opt], axis=-1)
        return link @ self.rotation.T

    def pixels_to_world(self, depth: np.ndarray) -> np.ndarray:
        """Return the world point of every pixel, (H, W, 3); NaN where depth is not finite."""
        depth = np.where(np.isfinite(depth), depth, np.nan)
        return self.position + depth[..., None] * self.rays()

    def project(self, points: np.ndarray) -> np.ndarray:
        """Return the pixel coordinates (u, v) of world points, shape (N, 2)."""
        link = (np.atleast_2d(points) - self.position) @ self.rotation
        x_opt, y_opt, z_opt = -link[:, 1], -link[:, 2], link[:, 0]
        return np.stack([self.fx * x_opt / z_opt + self.cx,
                         self.fy * y_opt / z_opt + self.cy], axis=-1)
