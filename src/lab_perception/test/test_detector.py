"""Detecting and naming pieces in a frame, and the ROS message round trip."""

import math

from conftest import box, cylinder, render, TABLE_TOP
from lab_perception.detector import crop, detect, Detection
from lab_perception.messages import detections_from_msg, detections_to_msg
from lab_perception.naming import ColourEmbedder, LabelMatcher
import numpy as np
import pytest
from std_msgs.msg import Header

AXIS = (0.0388353, 0.0)
PROMPTS = {'red_cube': 'a red cube', 'blue_cylinder': 'a blue cylinder',
           'green_block': 'a green block'}
SCENE = [box(0.20, -0.05, (0.03, 0.03, 0.03), yaw=0.3, colour=(200, 30, 30)),
         cylinder(0.12, -0.15, 0.015, 0.05, colour=(30, 30, 200)),
         box(0.15, 0.08, (0.05, 0.03, 0.03), yaw=-0.6, colour=(30, 180, 50))]


def run(camera, scene, table_colour=(150, 110, 70)):
    rgb, depth = render(camera, scene, table_colour)
    matcher = LabelMatcher(ColourEmbedder(), PROMPTS, 0.5, 0.1)
    return detect(rgb, depth, camera, matcher, table_top=TABLE_TOP, axis_xy=AXIS)


def test_every_piece_is_detected_and_named(camera):
    kept, dropped = run(camera, SCENE)
    assert dropped == []
    by_label = {d.label: d for d in kept}
    assert set(by_label) == set(PROMPTS)
    for obj, label in zip(SCENE, ('red_cube', 'blue_cylinder', 'green_block')):
        assert math.dist(by_label[label].centroid[:2], obj['xy']) < 0.004


def test_duplicate_labels_keep_the_best_match(camera):
    pale_red = box(0.10, -0.20, (0.03, 0.03, 0.03), colour=(150, 90, 90))
    kept, dropped = run(camera, SCENE + [pale_red])
    reds = [d for d in kept if d.label == 'red_cube']
    assert len(reds) == 1 and math.dist(reds[0].centroid[:2], (0.20, -0.05)) < 0.004
    assert [d.label for d in dropped] == ['red_cube']


def test_unknown_objects_are_reported_but_not_kept(camera):
    # On a neutral table, a grey object matches no colour prompt. (The colour
    # stand-in averages the crop's margin of table in with the object.)
    grey = box(0.10, -0.20, (0.03, 0.03, 0.03), colour=(120, 120, 120))
    kept, dropped = run(camera, SCENE + [grey], table_colour=(128, 128, 128))
    assert None not in {d.label for d in kept}
    assert [d.label for d in dropped] == [None]


def test_crop_is_clipped_to_the_image():
    rgb = np.zeros((10, 10, 3), dtype=np.uint8)
    assert crop(rgb, (0, 0, 3, 3), margin=8).shape == (10, 10, 3)
    assert crop(rgb, (4, 4, 5, 5), margin=1).shape == (4, 4, 3)


def test_message_round_trip():
    detections = [Detection('red_cube', 0.9, (0.2, -0.05, 0.765), 0.3, (1, 2, 3, 4)),
                  Detection(None, 0.1, (0.1, 0.1, 0.77), -0.2, (5, 6, 7, 8))]
    header = Header(frame_id='world')
    header.stamp.sec = 12
    msg = detections_to_msg(detections, header)
    assert msg.header.stamp.sec == 12 and len(msg.detections) == 2
    assert msg.detections[1].results[0].hypothesis.class_id == 'unknown'
    back = detections_from_msg(msg)
    assert [d.label for d in back] == ['red_cube', None]
    assert back[0].centroid == pytest.approx((0.2, -0.05, 0.765))
    assert back[0].yaw == pytest.approx(0.3)
    assert back[0].score == pytest.approx(0.9)
