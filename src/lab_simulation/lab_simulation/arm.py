"""Move the simulated arm smoothly between joint targets and wait for it to settle."""

import time
from typing import Callable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from .kinematics import Joints
from .ros_util import sim_now
from .tasks import MoveResult, SimulationUnresponsive

JOINT_NAMES = ('joint_1', 'joint_2', 'joint_3')
MIN_MOVE_S = 0.3        # do not judge "settled" before motion has started
STILL_SPEED = 0.02      # rad/s: slower than this a joint counts as still
HOLD_S = 0.3            # joints must stay still this long to count as settled
FEEDBACK_TIMEOUT_S = 5.0
MAX_JOINT_SPEED = 1.0   # rad/s peak setpoint speed; half the 2.0 rad/s joint limit
MIN_MOTION_S = 0.2      # shortest setpoint ramp


def motion_duration(start: Sequence[float], goal: Sequence[float],
                    max_speed: float = MAX_JOINT_SPEED) -> float:
    """Return the ramp time that keeps every joint at or below max_speed."""
    largest = max(abs(g - s) for s, g in zip(start, goal))
    # The cubic ease below peaks at 1.5x the average speed.
    return max(MIN_MOTION_S, 1.5 * largest / max_speed)


def interpolate(start: Sequence[float], goal: Sequence[float], elapsed: float,
                duration: float) -> Tuple[float, ...]:
    """
    Return the setpoint elapsed seconds into a ramp from start to goal.

    A cubic ease (3u^2 - 2u^3) starts and ends at zero speed, so a held piece
    is not swung by abrupt starts and stops.
    """
    u = min(max(elapsed / duration, 0.0), 1.0)
    blend = u * u * (3.0 - 2.0 * u)
    return tuple(s + (g - s) * blend for s, g in zip(start, goal))


class SettleDetector:
    """
    Decide when the joints have stopped moving, from measured positions only.

    Reported joint velocities are deliberately not used: after a large base
    rotation Gazebo's joint state publisher can keep reporting the velocity
    limit (2.0 rad/s) for a joint whose position is constant. A joint counts
    as still when its position spread over the last HOLD_S is within
    STILL_SPEED * HOLD_S. Samples from the first MIN_MOVE_S after a command
    are ignored, so a move is never judged before it has started.
    """

    def __init__(self, start: float, min_move_s: float = MIN_MOVE_S,
                 hold_s: float = HOLD_S, still_speed: float = STILL_SPEED):
        self._judge_from = start + min_move_s
        self._hold_s = hold_s
        self._tolerance = still_speed * hold_s
        self._samples: List[Tuple[float, Tuple[float, ...]]] = []

    def update(self, now: float, positions: Sequence[float]) -> Optional[float]:
        """Record a sample; return the time the joints became still, else None."""
        if now < self._judge_from - 1e-9:
            return None
        self._samples.append((now, tuple(positions)))
        # Keep the newest sample that is at least hold_s old as the window start.
        while len(self._samples) > 1 and now - self._samples[1][0] >= self._hold_s - 1e-9:
            self._samples.pop(0)
        oldest = self._samples[0][0]
        if now - oldest < self._hold_s - 1e-9:
            return None
        spread = max(max(column) - min(column) for column in zip(*(q for _, q in self._samples)))
        return oldest if spread <= self._tolerance else None


class Arm:
    """Joint position interface to the Gazebo controllers through the bridge."""

    def __init__(self, node: Node):
        self._node = node
        self._publishers = {
            name: node.create_publisher(Float64, f'/simple_arm/{name}/cmd_pos', 10)
            for name in JOINT_NAMES
        }
        self._positions = {}
        self._last_feedback = None
        node.create_subscription(
            JointState, '/joint_states', self._on_joint_states, qos_profile_sensor_data)

    def _on_joint_states(self, msg: JointState) -> None:
        self._positions.update(zip(msg.name, msg.position))
        self._last_feedback = time.monotonic()

    def joint_positions(self) -> Optional[Joints]:
        """Return the latest measured joint angles, or None before feedback arrives."""
        if not all(name in self._positions for name in JOINT_NAMES):
            return None
        return Joints(*(self._positions[name] for name in JOINT_NAMES))

    def wall_seconds_since_feedback(self) -> float:
        """Return wall seconds since the last /joint_states message."""
        if self._last_feedback is None:
            return float('inf')
        return time.monotonic() - self._last_feedback

    def _publish(self, q: Sequence[float]) -> None:
        for name, value in zip(JOINT_NAMES, q):
            self._publishers[name].publish(Float64(data=float(value)))

    def _max_error(self, q: Sequence[float]) -> float:
        current = self.joint_positions()
        return max(abs(a - b) for a, b in zip(current, q)) if current else float('inf')

    def move_to(self, q: Sequence[float], timeout_s: float,
                monitor: Optional[Callable[[], bool]] = None) -> MoveResult:
        """
        Ramp the joints smoothly to angles q and wait until the arm settles.

        The setpoint follows interpolate() from the measured start for
        motion_duration(); timeout_s counts from the end of that ramp.
        Settling is judged by SettleDetector from measured positions after
        the ramp. A monitor returning False aborts the move. Times are
        simulation seconds.
        """
        start = sim_now(self._node)
        origin = self.joint_positions() or tuple(q)
        duration = motion_duration(origin, q)
        detector = SettleDetector(start + duration)
        while True:
            now = sim_now(self._node)
            self._publish(interpolate(origin, q, now - start, duration))
            if self.wall_seconds_since_feedback() > FEEDBACK_TIMEOUT_S:
                raise SimulationUnresponsive(
                    f'no /joint_states for {FEEDBACK_TIMEOUT_S:.0f} s of wall time')
            if monitor is not None and not monitor():
                return MoveResult(False, True, now - start, self._max_error(q))
            current = self.joint_positions()
            still_since = detector.update(now, current) if current is not None else None
            if still_since is not None:
                return MoveResult(True, False, still_since - start, self._max_error(q))
            if now - start >= duration + timeout_s:
                return MoveResult(False, False, now - start, self._max_error(q))
            rclpy.spin_once(self._node, timeout_sec=0.01)
