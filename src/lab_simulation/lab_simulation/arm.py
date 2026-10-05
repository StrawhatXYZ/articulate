"""Command the simulated arm's joints and wait for it to settle."""

import time
from typing import Callable, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from .kinematics import Joints
from .ros_util import sim_now
from .tasks import MoveResult, SimulationUnresponsive

JOINT_NAMES = ('joint_1', 'joint_2', 'joint_3')
REPUBLISH_S = 0.1       # resend targets in case a message was lost
MIN_MOVE_S = 0.3        # do not judge "settled" before motion has started
STILL_SPEED = 0.02      # rad/s: below this a joint counts as still
HOLD_S = 0.3            # joints must stay still this long to count as settled
FEEDBACK_TIMEOUT_S = 5.0


class Arm:
    """Joint position interface to the Gazebo controllers through the bridge."""

    def __init__(self, node: Node):
        self._node = node
        self._publishers = {
            name: node.create_publisher(Float64, f'/simple_arm/{name}/cmd_pos', 10)
            for name in JOINT_NAMES
        }
        self._positions = {}
        self._velocities = {}
        self._last_feedback = None
        node.create_subscription(
            JointState, '/joint_states', self._on_joint_states, qos_profile_sensor_data)

    def _on_joint_states(self, msg: JointState) -> None:
        self._positions.update(zip(msg.name, msg.position))
        self._velocities.update(zip(msg.name, msg.velocity))
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

    def _max_speed(self) -> float:
        return max((abs(self._velocities.get(name, 0.0)) for name in JOINT_NAMES), default=0.0)

    def move_to(self, q: Sequence[float], timeout_s: float,
                monitor: Optional[Callable[[], bool]] = None) -> MoveResult:
        """
        Command joint angles q and wait until the arm has settled.

        Settled means every joint is slower than STILL_SPEED for HOLD_S, judged
        only after MIN_MOVE_S. A monitor returning False aborts the move.
        Times are simulation seconds.
        """
        start = sim_now(self._node)
        last_publish = None
        still_since = None
        while True:
            now = sim_now(self._node)
            if last_publish is None or now - last_publish >= REPUBLISH_S:
                self._publish(q)
                last_publish = now
            if self.wall_seconds_since_feedback() > FEEDBACK_TIMEOUT_S:
                raise SimulationUnresponsive(
                    f'no /joint_states for {FEEDBACK_TIMEOUT_S:.0f} s of wall time')
            if monitor is not None and not monitor():
                return MoveResult(False, True, now - start, self._max_error(q))
            if now - start >= MIN_MOVE_S and self._max_speed() < STILL_SPEED:
                still_since = now if still_since is None else still_since
                if now - still_since >= HOLD_S:
                    return MoveResult(True, False, still_since - start, self._max_error(q))
            else:
                still_since = None
            if now - start >= timeout_s:
                return MoveResult(False, False, now - start, self._max_error(q))
            rclpy.spin_once(self._node, timeout_sec=0.01)
