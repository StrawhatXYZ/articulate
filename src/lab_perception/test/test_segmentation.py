"""Finding objects on the table from depth, against ray-cast scenes."""

import math

from conftest import box, cylinder, render, TABLE_TOP
from lab_perception.segmentation import find_blobs
import numpy as np
import pytest

AXIS = (0.0388353, 0.0)
SCENE = [box(0.20, -0.05, (0.03, 0.03, 0.03), yaw=0.3),
         cylinder(0.12, -0.15, 0.015, 0.05),
         box(0.15, 0.08, (0.05, 0.03, 0.03), yaw=-0.6, colour=(30, 180, 50))]


def find(camera, objects, **kwargs):
    _, depth = render(camera, objects)
    return find_blobs(depth, camera, TABLE_TOP, AXIS, **kwargs)


def nearest(blobs, xy):
    return min(blobs, key=lambda b: math.dist(b.centroid[:2], xy))


def test_round_trip_through_the_camera(camera):
    _, depth = render(camera, [])
    world = camera.pixels_to_world(depth)
    v, u = 240, 320
    assert camera.project(world[v, u][None])[0] == pytest.approx([u, v], abs=0.5)
    assert world[v, u, 2] == pytest.approx(TABLE_TOP, abs=1e-6)


def test_every_piece_is_found_at_its_centre(camera):
    blobs = find(camera, SCENE)
    assert len(blobs) == 3
    for obj in SCENE:
        blob = nearest(blobs, obj['xy'])
        assert math.dist(blob.centroid[:2], obj['xy']) < 0.004
        height = obj['size'][2] if obj['kind'] == 'box' else obj['height']
        assert blob.top_height == pytest.approx(height, abs=0.003)
        assert blob.centroid[2] == pytest.approx(TABLE_TOP + height / 2, abs=0.003)


def wrapped(angle, period):
    return (angle + period / 2) % period - period / 2


def test_block_yaw_is_its_long_axis(camera):
    blob = nearest(find(camera, SCENE), SCENE[2]['xy'])
    assert abs(wrapped(blob.yaw - (-0.6), math.pi)) < math.radians(3)


def test_cube_yaw_is_known_up_to_a_quarter_turn(camera):
    blob = nearest(find(camera, SCENE), SCENE[0]['xy'])
    assert abs(wrapped(blob.yaw - 0.3, math.pi / 2)) < math.radians(4)


def test_bounding_box_covers_the_piece_in_the_image(camera):
    blob = nearest(find(camera, SCENE), SCENE[0]['xy'])
    u, v = camera.project(np.array([[0.20, -0.05, TABLE_TOP + 0.03]]))[0]
    u0, v0, u1, v1 = blob.bbox
    assert u0 <= u <= u1 and v0 <= v <= v1
    assert blob.pixels >= 80


def test_missing_depth_is_ignored(camera):
    _, depth = render(camera, SCENE)
    holes = np.random.default_rng(0).random(depth.shape)
    depth[holes < 0.05] = np.nan
    depth[holes > 0.97] = np.inf
    blobs = find_blobs(depth, camera, TABLE_TOP, AXIS)
    assert len(blobs) == 3


def test_excluded_regions_are_dropped(camera):
    blobs = find(camera, SCENE, exclude=[(SCENE[1]['xy'], 0.05)])
    assert len(blobs) == 2
    assert all(math.dist(b.centroid[:2], SCENE[1]['xy']) > 0.05 for b in blobs)


def test_small_blobs_are_dropped(camera):
    speck = box(0.18, -0.20, (0.004, 0.004, 0.01))
    assert len(find(camera, [speck])) == 0


def test_only_the_workspace_band_counts(camera):
    far = box(0.40, 0.0, (0.03, 0.03, 0.03))       # beyond the reach band
    near = box(0.06, 0.0, (0.03, 0.03, 0.03))      # at the arm's base
    tall = box(0.2, -0.05, (0.03, 0.03, 0.12))     # taller than any piece
    assert find(camera, [far, near, tall]) == []


def test_empty_table_has_no_blobs(camera):
    assert find(camera, []) == []
