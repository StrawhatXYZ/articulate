"""The overhead camera's Gazebo model and the perception settings it implies."""

import math
import xml.etree.ElementTree as ET

from lab_simulation import layout, overhead_camera as oc
import pytest


def test_sdf_describes_an_rgbd_camera_at_the_low_rate():
    model = ET.fromstring(oc.sdf()).find('model')
    assert model.get('name') == oc.NAME and model.find('static').text == 'true'
    sensor = model.find('link/sensor')
    assert sensor.get('type') == 'rgbd_camera'
    assert float(sensor.find('update_rate').text) == oc.RATE_HZ
    assert sensor.find('topic').text == oc.TOPIC
    image = sensor.find('camera/image')
    size = (int(image.find('width').text), int(image.find('height').text))
    assert size == (oc.WIDTH, oc.HEIGHT)


def test_spawn_places_the_camera_at_its_pose():
    args = oc.spawn_arguments()
    value = {args[i]: args[i + 1] for i in range(0, len(args), 2)}
    assert [float(value[k]) for k in ('-x', '-y', '-z')] == pytest.approx(list(oc.POSITION))
    assert float(value['-P']) == oc.PITCH and float(value['-Y']) == oc.YAW
    assert float(value['-R']) == 0.0


def test_camera_looks_at_the_workspace():
    # The optical axis meets the table inside the sampling annulus.
    x, y, z = oc.POSITION
    reach = (z - layout.TABLE_TOP) / math.tan(oc.PITCH)
    hit = (x + reach * math.cos(oc.YAW), y + reach * math.sin(oc.YAW))
    assert math.dist(hit, layout.PAN_AXIS_XY) < layout.MAX_RADIUS


def test_perception_parameters_cover_every_piece_and_mask_every_bin():
    params = oc.perception_parameters()
    assert params['labels'] == sorted(layout.PIECES)
    for label, prompt in zip(params['labels'], params['prompts']):
        assert layout.PIECES[label].description in prompt
    assert len(params['exclude_xy']) == 2 * len(layout.BINS)
    lo, hi = params['radius']
    assert lo < layout.MIN_RADIUS and hi > layout.MAX_RADIUS
    assert params['camera_position'] == list(oc.POSITION)
    assert 0 < params['min_score'] and 0 < params['min_margin']


def test_bin_mask_covers_a_bin_but_spares_the_nearest_piece_positions():
    for bin_ in layout.BINS.values():
        assert oc.BIN_MASK_RADIUS >= bin_.outer / 2 * math.sqrt(2) - 0.001
        assert oc.BIN_MASK_RADIUS < layout.BIN_CLEARANCE - 0.015


def test_piece_descriptions_are_distinct():
    descriptions = [p.description for p in layout.PIECES.values()]
    assert len(set(descriptions)) == len(descriptions)
