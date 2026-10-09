"""Tests for arm motion requests and position-based settle detection."""

import math

from lab_simulation.arm import HOME, plan_request, PLANNING_FAILURES, SettleDetector
from lab_simulation.so101_kinematics import ARM_JOINTS
from moveit_msgs.msg import MoveItErrorCodes
import pytest

STEP = 0.01  # simulation seconds between joint-state samples


def first_settle(detector, trajectory, duration=3.0):
    """Feed trajectory(t) -> joint positions; return (time settled, still_since)."""
    t = 0.0
    while t <= duration:
        still_since = detector.update(t, trajectory(t))
        if still_since is not None:
            return t, still_since
        t = round(t + STEP, 6)
    return None, None


def test_joints_already_at_rest_settle_after_min_move_and_hold():
    now, since = first_settle(SettleDetector(start=0.0), lambda t: (0.0, 0.0, 0.0))
    assert since == pytest.approx(0.3)
    assert now == pytest.approx(0.6)


def test_settles_once_the_motion_stops():
    def ramp_then_stop(t):
        return (min(t, 1.0), 0.0, 0.0)

    now, since = first_settle(SettleDetector(start=0.0), ramp_then_stop)
    assert since == pytest.approx(1.0, abs=STEP)
    assert now == pytest.approx(1.3, abs=STEP)


def test_slow_drift_below_the_still_speed_counts_as_settled():
    now, _ = first_settle(SettleDetector(start=0.0), lambda t: (0.0, 0.0, -0.01 + 0.0025 * t))
    assert now is not None


def test_oscillation_never_settles():
    def hunting(t):
        return (0.02 * math.sin(2 * math.pi * t / 0.4), 0.0, 0.0)

    now, _ = first_settle(SettleDetector(start=0.0), hunting)
    assert now is None


def test_start_offset_is_respected():
    now, since = first_settle(SettleDetector(start=10.0),
                              lambda t: (0.0, 0.0, 0.0), duration=0.0)
    assert now is None
    detector = SettleDetector(start=10.0)
    for i in range(61):
        result = detector.update(10.0 + i * STEP, (0.0, 0.0, 0.0))
    assert result == pytest.approx(10.3)


def test_plan_request_targets_every_arm_joint_from_the_current_state():
    goal = (0.1, -0.2, 0.3, 0.4, -0.5)
    request = plan_request(goal)
    assert request.group_name == 'arm'
    assert request.start_state.is_diff
    [constraints] = request.goal_constraints
    assert [c.joint_name for c in constraints.joint_constraints] == list(ARM_JOINTS)
    assert [c.position for c in constraints.joint_constraints] == pytest.approx(goal)
    assert all(0 < c.tolerance_above <= 0.01 for c in constraints.joint_constraints)
    assert 0 < request.max_velocity_scaling_factor <= 1
    # Explicit planner and planning volume (MoveIt warns on every request otherwise).
    assert request.pipeline_id == 'ompl' and request.planner_id
    assert request.workspace_parameters.header.frame_id == 'world'
    assert request.workspace_parameters.min_corner.z <= 0.75 < \
        request.workspace_parameters.max_corner.z


def test_planning_failures_exclude_execution_failures():
    assert MoveItErrorCodes.PLANNING_FAILED in PLANNING_FAILURES
    assert MoveItErrorCodes.GOAL_IN_COLLISION in PLANNING_FAILURES
    for code in (MoveItErrorCodes.SUCCESS, MoveItErrorCodes.CONTROL_FAILED,
                 MoveItErrorCodes.TIMED_OUT, MoveItErrorCodes.PREEMPTED):
        assert code not in PLANNING_FAILURES


def test_home_matches_the_srdf():
    import xml.etree.ElementTree as ET
    from ament_index_python.packages import get_package_share_directory
    srdf = ET.parse(get_package_share_directory('so101_moveit_config')
                    + '/config/so101.srdf').getroot()
    state = {j.get('name'): float(j.get('value'))
             for j in srdf.find("group_state[@name='home']").findall('joint')}
    assert tuple(state[name] for name in ARM_JOINTS) == pytest.approx(HOME)


def test_every_planning_side_error_counts_as_a_planning_failure():
    """GOAL_STATE_INVALID (goal in collision) was once reported as a timeout."""
    for code in (MoveItErrorCodes.GOAL_STATE_INVALID, MoveItErrorCodes.START_STATE_INVALID,
                 MoveItErrorCodes.INVALID_LINK_NAME, MoveItErrorCodes.INVALID_OBJECT_NAME,
                 MoveItErrorCodes.FRAME_TRANSFORM_FAILURE):
        assert code in PLANNING_FAILURES


def test_plan_request_scales_velocity_and_acceleration_by_speed():
    default = plan_request(HOME)
    fast = plan_request(HOME, speed=0.8)
    assert default.max_velocity_scaling_factor == 0.5
    assert fast.max_velocity_scaling_factor == 0.8 and fast.max_acceleration_scaling_factor == 0.8
