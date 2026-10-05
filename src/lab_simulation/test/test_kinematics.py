"""Tests for the closed-form arm kinematics."""

import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

from lab_simulation import kinematics
from lab_simulation.kinematics import (end_effector_rotation, forward, inverse, is_reachable,
                                       rotate, Unreachable)
import pytest

URDF = ET.parse(Path(__file__).resolve().parents[1] / 'urdf' / 'simple_arm.urdf').getroot()


def joint(name):
    return next(j for j in URDF.findall('joint') if j.get('name') == name)


def origin_z(name):
    return float(joint(name).find('origin').get('xyz').split()[2])


def test_constants_match_urdf():
    assert kinematics.BASE_Z == pytest.approx(origin_z('world_joint'))
    assert kinematics.SHOULDER_HEIGHT == pytest.approx(
        origin_z('joint_1') + origin_z('joint_2'))
    assert kinematics.UPPER_ARM == pytest.approx(origin_z('joint_3'))
    assert kinematics.FOREARM == pytest.approx(origin_z('end_effector_joint'))
    tip = next(link for link in URDF.findall('link') if link.get('name') == 'end_effector')
    assert kinematics.TIP_RADIUS == pytest.approx(
        float(tip.find('visual/geometry/sphere').get('radius')))
    for name, (lo, hi) in zip(('joint_1', 'joint_2', 'joint_3'), kinematics.JOINT_LIMITS):
        limit = joint(name).find('limit')
        assert (lo, hi) == pytest.approx((float(limit.get('lower')), float(limit.get('upper'))))


def test_forward_home_points_straight_up():
    assert forward((0.0, 0.0, 0.0)) == pytest.approx((0.0, 0.0, 0.75 + 0.29 + 0.35))


def test_round_trip_over_random_configurations():
    rng = random.Random(0)
    for _ in range(2000):
        q = tuple(rng.uniform(lo, hi) for lo, hi in kinematics.JOINT_LIMITS)
        p = forward(q)
        if math.hypot(p[0], p[1]) < 1e-3:
            continue
        solution = inverse(p)
        assert kinematics.within_limits(solution)
        assert forward(solution) == pytest.approx(p, abs=1e-9)


def test_inverse_prefers_the_higher_elbow():
    q = inverse((0.2, 0.0, 0.825))
    elbow_z = 0.75 + 0.29 + 0.20 * math.cos(q.q2)
    assert elbow_z > 0.9


@pytest.mark.parametrize('xyz', [(1.0, 0.0, 1.0), (0.0, 0.0, 0.5), (0.0, 0.0, 2.0)])
def test_out_of_workspace_raises(xyz):
    with pytest.raises(Unreachable):
        inverse(xyz)


def test_is_reachable_applies_margin():
    near_edge = forward((0.0, 1.4, 0.3))  # about 4 mm inside the outer boundary
    assert is_reachable(near_edge)
    assert not is_reachable(near_edge, margin=0.01)


def test_point_directly_behind_the_base_is_reachable():
    q = inverse((-0.2, 0.0, 0.9))
    assert kinematics.within_limits(q)
    assert forward(q) == pytest.approx((-0.2, 0.0, 0.9), abs=1e-5)


def test_end_effector_rotation_is_identity_at_home():
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    for row, expected in zip(end_effector_rotation((0.0, 0.0, 0.0)), identity):
        assert row == pytest.approx(expected)


def test_end_effector_z_axis_points_along_the_forearm():
    rng = random.Random(1)
    for _ in range(200):
        q = tuple(rng.uniform(lo, hi) for lo, hi in kinematics.JOINT_LIMITS)
        tip = forward(q)
        r = kinematics.UPPER_ARM * math.sin(q[1])
        elbow = (r * math.cos(q[0]), r * math.sin(q[0]),
                 kinematics.BASE_Z + kinematics.SHOULDER_HEIGHT
                 + kinematics.UPPER_ARM * math.cos(q[1]))
        along = tuple((t - e) / kinematics.FOREARM for t, e in zip(tip, elbow))
        assert rotate(end_effector_rotation(q), (0.0, 0.0, 1.0)) == pytest.approx(along, abs=1e-9)


def test_end_effector_rotation_is_orthonormal():
    m = end_effector_rotation((0.7, -1.1, 1.9))
    for i in range(3):
        for j in range(3):
            dot = sum(m[k][i] * m[k][j] for k in range(3))
            assert dot == pytest.approx(1.0 if i == j else 0.0, abs=1e-12)
