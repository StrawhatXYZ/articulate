"""Tests for arm motion: smooth setpoint ramps and position-based settle detection."""

import math

from lab_simulation.arm import interpolate, motion_duration, SettleDetector
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


def test_motion_duration_scales_with_the_largest_joint_move():
    # Cubic ease-in/out peaks at 1.5x the average speed.
    assert motion_duration((0.0, 0.0, 0.0), (1.0, 0.2, -0.5), max_speed=1.0) == pytest.approx(1.5)
    assert motion_duration((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), max_speed=1.0) == pytest.approx(0.2)


def test_interpolate_starts_and_ends_exactly():
    start, goal = (0.0, 1.0, -1.0), (1.0, 0.0, 1.0)
    assert interpolate(start, goal, 0.0, 2.0) == pytest.approx(start)
    assert interpolate(start, goal, 2.0, 2.0) == pytest.approx(goal)
    assert interpolate(start, goal, 5.0, 2.0) == pytest.approx(goal)
    assert interpolate(start, goal, 1.0, 2.0) == pytest.approx((0.5, 0.5, 0.0))


def test_interpolate_eases_in_and_out_monotonically():
    start, goal, duration, dt = (0.0,), (1.0,), 1.0, 0.001
    path = [interpolate(start, goal, i * dt, duration)[0] for i in range(1001)]
    assert all(b >= a for a, b in zip(path, path[1:]))
    assert (path[1] - path[0]) / dt < 0.01      # starts from rest
    assert (path[-1] - path[-2]) / dt < 0.01    # arrives at rest
    peak = max((b - a) / dt for a, b in zip(path, path[1:]))
    assert peak == pytest.approx(1.5, rel=0.01)
