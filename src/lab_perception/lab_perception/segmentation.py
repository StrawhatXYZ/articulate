"""
Find objects on a table from a depth image.

Pure Python, NumPy and SciPy (no ROS). Anything standing on the table within
the arm's workspace band shows up as pixels above the table plane; connected
groups of them are objects. Positions are taken from each object's top
surface, which the camera sees from above without the bias of its sides.
"""

import math
from typing import List, NamedTuple, Sequence, Tuple

import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull, QhullError

from .camera import CameraModel

TOP_SURFACE_M = 0.004       # points this close below the highest one form the top


class Blob(NamedTuple):
    """An object on the table, as seen from depth."""

    centroid: Tuple[float, float, float]    # centre of its volume, world frame
    top_height: float                       # height of its top above the table
    yaw: float                              # long axis, rad (mod pi; mod pi/2 if square)
    pixels: int
    bbox: Tuple[int, int, int, int]          # image box: u0, v0, u1, v1 (inclusive)


def footprint_yaw(xy: np.ndarray) -> float:
    """
    Return the direction of the long side of the smallest rectangle around xy.

    Rotating calipers over the convex hull: the minimum-area rectangle has a
    side along one hull edge. The result is in [-pi/2, pi/2).
    """
    try:
        hull = xy[ConvexHull(xy).vertices]
    except (QhullError, ValueError):
        return 0.0
    best_area, best_yaw = math.inf, 0.0
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        angle = math.atan2(b[1] - a[1], b[0] - a[0])
        c, s = math.cos(angle), math.sin(angle)
        local = xy @ np.array([[c, -s], [s, c]])
        span = local.max(axis=0) - local.min(axis=0)
        area = span[0] * span[1]
        if area < best_area:
            best_area = area
            best_yaw = angle if span[0] >= span[1] else angle + math.pi / 2
    return (best_yaw + math.pi / 2) % math.pi - math.pi / 2


def find_blobs(depth: np.ndarray, camera: CameraModel, table_top: float,
               axis_xy: Sequence[float], radius: Tuple[float, float] = (0.10, 0.30),
               height: Tuple[float, float] = (0.005, 0.07),
               exclude: Sequence[Tuple[Sequence[float], float]] = (),
               min_pixels: int = 80) -> List[Blob]:
    """
    Return the objects standing on the table in a depth image.

    Only points within radius (min, max) of axis_xy and more than height[0]
    above the table count. A group reaching higher than height[1] is not a
    piece (the arm, or something it holds) and is dropped whole. exclude
    lists (xy, r) discs to ignore, such as fixed bins. Groups smaller than
    min_pixels are dropped. Pixels connect diagonally too, so scattered
    missing depth does not split an object.
    """
    world = camera.pixels_to_world(depth)
    with np.errstate(invalid='ignore'):
        above = world[..., 2] - table_top
        reach = np.hypot(world[..., 0] - axis_xy[0], world[..., 1] - axis_xy[1])
        mask = (above > height[0]) & (reach > radius[0]) & (reach < radius[1])
        for (x, y), r in exclude:
            mask &= np.hypot(world[..., 0] - x, world[..., 1] - y) > r
    labels, count = ndimage.label(mask, structure=np.ones((3, 3)))
    blobs = []
    for index in range(1, count + 1):
        region = labels == index
        pixels = int(region.sum())
        if pixels < min_pixels:
            continue
        points, heights = world[region], above[region]
        top = float(heights.max())
        if top > height[1]:
            continue
        surface = points[heights >= top - TOP_SURFACE_M]
        x, y = surface[:, :2].mean(axis=0)
        vs, us = np.nonzero(region)
        blobs.append(Blob(
            centroid=(float(x), float(y), table_top + top / 2),
            top_height=top,
            yaw=footprint_yaw(surface[:, :2]),
            pixels=pixels,
            bbox=(int(us.min()), int(vs.min()), int(us.max()), int(vs.max()))))
    return blobs
