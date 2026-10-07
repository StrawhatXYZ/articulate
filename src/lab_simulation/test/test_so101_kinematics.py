"""Forward and inverse kinematics of the SO-101, against its URDF."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from lab_simulation.so101_kinematics import ARM_JOINTS, Kinematics, Unreachable
import numpy as np
import pytest
import xacro

STANDALONE = Path(get_package_share_directory('so101_description')) / 'urdf' / 'so101.urdf.xacro'


@pytest.fixture(scope='module')
def kin():
    return Kinematics(xacro.process_file(str(STANDALONE)).toxml())


def approach_angle_deg(transform):
    """Angle between the gripper's approach axis (frame z) and straight down."""
    return math.degrees(math.acos(max(-1.0, min(1.0, -transform[2, 2]))))


def yaw_error_deg(transform, yaw):
    """Closing-axis yaw error modulo half a turn (the jaws grip both ways)."""
    actual = math.atan2(transform[1, 0], transform[0, 0])
    error = (actual - yaw + math.pi / 2) % math.pi - math.pi / 2
    return abs(math.degrees(error))


def test_arm_joints_and_limits_come_from_the_urdf(kin):
    assert kin.joints == ARM_JOINTS
    assert kin.limits['shoulder_pan'] == pytest.approx((-1.91986, 1.91986))
    assert kin.limits['wrist_roll'] == pytest.approx((-2.74385, 2.84121))


def test_forward_at_zero_matches_the_upstream_chain(kin):
    # Reference values computed independently from so101_new_calib.urdf.
    frame = kin.forward((0, 0, 0, 0, 0), link='gripper_frame_link')
    assert frame[:3, 3] == pytest.approx([0.3914, 0.0, 0.2265], abs=2e-4)
    assert frame[:3, 2] == pytest.approx([1.0, 0.0, 0.0], abs=1e-3)


def test_grasp_point_is_offset_inside_the_jaws(kin):
    frame = kin.forward((0, 0, 0, 0, 0), link='gripper_frame_link')
    grasp = kin.forward((0, 0, 0, 0, 0))
    offset = np.linalg.inv(frame) @ grasp
    assert offset[:3, 3] == pytest.approx([-0.022, 0.0, -0.004], abs=1e-9)


def test_pan_axis_position(kin):
    assert kin.pan_axis_xy == pytest.approx((0.0388353, 0.0), abs=1e-6)


@pytest.mark.parametrize('radius', [0.10, 0.14, 0.18, 0.22, 0.26])
@pytest.mark.parametrize('bearing', [-1.6, -0.7, 0.0, 0.9, 1.7])
@pytest.mark.parametrize('height', [0.015, 0.06])
def test_inverse_round_trip_with_vertical_approach(kin, radius, bearing, height):
    if height > 0.05 and radius > 0.24:
        pytest.skip('outside the hover reach map')
    px, py = kin.pan_axis_xy
    target = (px + radius * math.cos(bearing), py + radius * math.sin(bearing), height)
    yaw = bearing + 0.4
    q = kin.inverse(target, yaw)
    transform = kin.forward(q)
    assert np.linalg.norm(transform[:3, 3] - target) < 5e-4
    assert approach_angle_deg(transform) < 1.0
    assert yaw_error_deg(transform, yaw) < 1.0
    for name, value in zip(ARM_JOINTS, q):
        lower, upper = kin.limits[name]
        assert lower - 1e-9 <= value <= upper + 1e-9


def test_inverse_without_yaw_keeps_the_wrist_roll(kin):
    px, py = kin.pan_axis_xy
    q = kin.inverse((px + 0.18, py, 0.05), None)
    assert q[4] == 0.0
    assert approach_angle_deg(kin.forward(q)) < 1.0


@pytest.mark.parametrize('target', [(0.6, 0.0, 0.1), (0.05, 0.0, 0.0), (0.2, 0.0, 0.5),
                                    (-0.25, 0.0, 0.02)])
def test_unreachable_targets_raise(kin, target):
    with pytest.raises(Unreachable):
        kin.inverse(target, 0.0)
    assert not kin.is_reachable(target, 0.0)


def test_reachable_matches_inverse(kin):
    px, py = kin.pan_axis_xy
    assert kin.is_reachable((px + 0.18, py, 0.02), 0.3)


def test_inverse_is_deterministic(kin):
    px, py = kin.pan_axis_xy
    target = (px + 0.15, py + 0.05, 0.03)
    assert kin.inverse(target, 0.2) == kin.inverse(target, 0.2)


def test_grasp_yaw_reports_the_closing_axis(kin):
    px, py = kin.pan_axis_xy
    q = kin.inverse((px + 0.18, py + 0.02, 0.02), 0.5)
    assert yaw_error_deg(kin.forward(q), kin.grasp_yaw(q)) < 1e-6
    assert abs(((kin.grasp_yaw(q) - 0.5 + math.pi / 2) % math.pi) - math.pi / 2) < 0.02
