"""Shared test fixtures: the lab camera and a ray-cast depth/colour renderer."""

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lab_perception.camera import CameraModel  # noqa: E402, I100

TABLE_TOP = 0.75
# The overhead camera of the lab world (spike, 7 October 2026): 640x480,
# horizontal field of view 0.9 rad.
WIDTH, HEIGHT, HFOV = 640, 480, 0.9
FOCAL = WIDTH / 2 / math.tan(HFOV / 2)
CAMERA = {'position': (0.55, -0.30, 1.30), 'yaw': 2.481, 'pitch': 0.767}


@pytest.fixture
def camera():
    return CameraModel(FOCAL, FOCAL, WIDTH / 2, HEIGHT / 2, WIDTH, HEIGHT, **CAMERA)


def box(x, y, size, yaw=0.0, colour=(200, 30, 30)):
    """Return a box resting on the table: centre xy, size (sx, sy, sz)."""
    return {'kind': 'box', 'xy': (x, y), 'size': size, 'yaw': yaw, 'colour': colour}


def cylinder(x, y, radius, height, colour=(30, 30, 200)):
    """Return an upright cylinder resting on the table."""
    return {'kind': 'cylinder', 'xy': (x, y), 'radius': radius, 'height': height,
            'colour': colour}


def _hit_box(origin, rays, obj):
    """Return the ray parameter of the first hit on an upright box, inf for none."""
    c, s = math.cos(-obj['yaw']), math.sin(-obj['yaw'])
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    sx, sy, sz = obj['size']
    centre = np.array([obj['xy'][0], obj['xy'][1], TABLE_TOP + sz / 2])
    o = rot @ (origin - centre)
    d = rays @ rot.T
    half = np.array([sx, sy, sz]) / 2
    with np.errstate(divide='ignore', invalid='ignore'):
        t1 = (-half - o) / d
        t2 = (half - o) / d
    near = np.nanmax(np.minimum(t1, t2), axis=-1)
    far = np.nanmin(np.maximum(t1, t2), axis=-1)
    return np.where((near <= far) & (far > 0), near, np.inf)


def _hit_cylinder(origin, rays, obj):
    """Return the ray parameter of the first hit on an upright cylinder, inf for none."""
    r, h = obj['radius'], obj['height']
    ox, oy = origin[0] - obj['xy'][0], origin[1] - obj['xy'][1]
    dx, dy, dz = rays[..., 0], rays[..., 1], rays[..., 2]
    # Top cap.
    t_top = (TABLE_TOP + h - origin[2]) / dz
    on_top = (ox + t_top * dx) ** 2 + (oy + t_top * dy) ** 2 <= r * r
    hit = np.where(on_top & (t_top > 0), t_top, np.inf)
    # Side.
    a = dx * dx + dy * dy
    b = 2 * (ox * dx + oy * dy)
    cc = ox * ox + oy * oy - r * r
    disc = b * b - 4 * a * cc
    with np.errstate(invalid='ignore'):
        t_side = (-b - np.sqrt(disc)) / (2 * a)
    z = origin[2] + t_side * dz
    side = (disc >= 0) & (t_side > 0) & (z >= TABLE_TOP) & (z <= TABLE_TOP + h)
    return np.minimum(hit, np.where(side, t_side, np.inf))


def render(camera, objects, table_colour=(150, 110, 70)):
    """Return (rgb uint8 HxWx3, depth float32 HxW) of the table and objects."""
    rays = camera.rays()
    origin = camera.position
    t_table = (TABLE_TOP - origin[2]) / rays[..., 2]
    depth = np.where(t_table > 0, t_table, np.inf)
    rgb = np.zeros(rays.shape, dtype=np.uint8)
    rgb[...] = table_colour
    for obj in objects:
        t = _hit_box(origin, rays, obj) if obj['kind'] == 'box' else \
            _hit_cylinder(origin, rays, obj)
        closer = t < depth
        depth = np.where(closer, t, depth)
        rgb[closer] = obj['colour']
    return rgb, depth.astype(np.float32)
