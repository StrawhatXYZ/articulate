"""Open and close the SO-101's jaws through ros2_control and report where they stop."""

from typing import Optional

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .arm import JointFeedback, SettleDetector
from .ros_util import sim_now, spin_until

JOINT = 'gripper'
OPEN = 0.6          # rad: ~45 mm between the jaws
CLOSED = -0.17      # rad: fully closed. Commanded to grip; the jaw stalls on a piece
OPEN_TOLERANCE = 0.05
SETTLE_TIMEOUT_S = 3.0
COMMAND_TOPIC = '/gripper_controller/commands'


class Gripper:
    """
    Command the gripper joint and wait until the jaw stops moving.

    The controller keeps commanding the target, so a jaw closed on a piece
    keeps squeezing at the servo's torque limit until it is opened.
    """

    def __init__(self, node: Node, feedback: JointFeedback):
        self._node = node
        self._feedback = feedback
        self._publisher = node.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)

    def ready(self) -> bool:
        """Return True once the controller is subscribed and the joint reports."""
        return (self._publisher.get_subscription_count() > 0
                and self._feedback.get((JOINT,)) is not None)

    def position(self) -> Optional[float]:
        """Return the jaw angle in radians."""
        value = self._feedback.get((JOINT,))
        return None if value is None else value[0]

    def _command(self, target: float) -> Optional[float]:
        """Command target; return the angle the jaw settles at, None on timeout."""
        start = sim_now(self._node)
        detector = SettleDetector(start)
        settled = []

        def still() -> bool:
            self._feedback.check_alive()
            # Re-sent each cycle: cheap, and immune to a lost first message.
            self._publisher.publish(Float64MultiArray(data=[target]))
            position = self.position()
            if position is not None and detector.update(sim_now(self._node), (position,)):
                settled.append(position)
            return bool(settled)

        if not spin_until(self._node, still, sim_timeout_s=SETTLE_TIMEOUT_S):
            return None
        return settled[0]

    def release(self, angle: float = OPEN) -> bool:
        """Open the jaws to angle (default fully open); True once they are there."""
        position = self._command(angle)
        return position is not None and abs(position - angle) <= OPEN_TOLERANCE

    def grip(self) -> Optional[float]:
        """Close the jaws; return the angle they stall at, or None on timeout."""
        return self._command(CLOSED)

    def hold(self, angle: float) -> None:
        """
        Stop squeezing and hold the jaw at angle.

        Used before latching a piece: once latched, the piece is fixed to the
        gripper, and a jaw still pushing against it would fight the latch
        joint until the piece is squeezed out.
        """
        self._publisher.publish(Float64MultiArray(data=[angle]))
