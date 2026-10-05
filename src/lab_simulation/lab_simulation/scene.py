"""Piece poses, scene reset and suction, through the Gazebo bridge."""

from functools import partial
import math
from typing import Dict, Optional, Sequence, Tuple

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Empty
from std_msgs.msg import String

from . import layout
from .ros_util import spin_for, spin_until

# One pose topic per piece, bridged from each piece's PosePublisher.
POSE_TOPIC = '/lab/pieces/{name}/pose'
SET_POSE_SERVICE = '/world/table_world/set_pose'
SETTLE_SPEED = 0.001    # m/s: below this a piece counts as at rest
SETTLE_SAMPLE_S = 0.1
SUCTION_ACK_S = 2.0     # simulation seconds to wait for a suction state change
GRIP_SETTLE_S = 0.2     # let an attached piece settle into the grip
RESET_ATTEMPTS = 5      # teleports per reset while a piece is still moving
RESET_SETTLE_S = 1.0    # wait per attempt for the pieces to come to rest

XYZ = Tuple[float, float, float]


class Scene:
    """Read piece poses, reset the layout, and engage or release suction."""

    def __init__(self, node: Node):
        self._node = node
        self._poses: Dict[str, XYZ] = {}
        for name in layout.PIECES:
            node.create_subscription(PoseStamped, POSE_TOPIC.format(name=name),
                                     partial(self._on_pose, name), qos_profile_sensor_data)
        self._set_pose = node.create_client(SetEntityPose, SET_POSE_SERVICE)
        self._attach = {n: node.create_publisher(Empty, f'/simple_arm/suction/{n}/attach', 10)
                        for n in layout.PIECES}
        self._detach = {n: node.create_publisher(Empty, f'/simple_arm/suction/{n}/detach', 10)
                        for n in layout.PIECES}
        # Last state each suction plugin reported; None until it reports.
        self._suction: Dict[str, Optional[str]] = {n: None for n in layout.PIECES}
        for name in layout.PIECES:
            node.create_subscription(String, f'/simple_arm/suction/{name}/state',
                                     partial(self._on_suction, name), 10)

    def _on_pose(self, name: str, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._poses[name] = (p.x, p.y, p.z)

    def _on_suction(self, name: str, msg: String) -> None:
        self._suction[name] = msg.data

    def _set_suction(self, names: Sequence[str], state: str) -> bool:
        """
        Command suction on names and wait until each confirms state.

        A command is not assumed to have taken effect after a fixed delay: with
        the simulation running faster than real time, a late message would
        engage suction after the arm has already moved on.
        """
        topics = self._attach if state == 'attached' else self._detach
        pending = [n for n in names if self._suction[n] != state]
        for name in pending:
            topics[name].publish(Empty())
        return spin_until(self._node, lambda: all(self._suction[n] == state for n in pending),
                          sim_timeout_s=SUCTION_ACK_S)

    def has_poses(self) -> bool:
        """Return True once every piece's pose has been received."""
        return all(name in self._poses for name in layout.PIECES)

    def pose(self, name: str) -> Optional[XYZ]:
        """Return the latest world position of a piece's centre."""
        return self._poses.get(name)

    def grasp_gap(self, name: str, tip_xyz: Sequence[float]) -> float:
        """Return the measured gap between the tip sphere and the top of a piece."""
        return layout.grasp_gap(layout.PIECES[name], self._poses[name], tip_xyz)

    def attach(self, name: str, tip_xyz: Sequence[float]) -> bool:
        """Engage suction on a piece if the tip is within GRASP_GAP_MAX of it."""
        if self.grasp_gap(name, tip_xyz) > layout.GRASP_GAP_MAX:
            return False
        if not self._set_suction([name], 'attached'):
            return False
        # The piece shifts slightly as the joint takes hold; return only once
        # it has settled, so callers measure the carry offset of the real grip.
        spin_for(self._node, GRIP_SETTLE_S)
        return True

    def detach(self, name: str) -> None:
        """Release suction on a piece and wait for the confirmation."""
        self._set_suction([name], 'detached')

    def detach_all(self) -> None:
        """Release suction on every piece and wait for the confirmations."""
        self._set_suction(list(layout.PIECES), 'detached')

    def _place(self, name: str, x: float, y: float) -> bool:
        request = SetEntityPose.Request()
        request.entity = Entity(name=name, type=Entity.MODEL)
        request.pose = Pose()
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = layout.PIECES[name].centre_z
        request.pose.orientation.w = 1.0
        if not self._set_pose.wait_for_service(timeout_sec=5.0):
            return False
        future = self._set_pose.call_async(request)
        spin_until(self._node, future.done, sim_timeout_s=10.0)
        return future.done() and future.result() is not None and future.result().success

    def reset(self, positions: Dict[str, Tuple[float, float]]) -> bool:
        """
        Teleport every piece to positions, upright, and wait until they rest.

        Teleporting keeps a body's velocity, so a piece knocked into rolling
        arrives still rolling. Rolling does die out through friction, so the
        pieces are teleported back to their targets until they come to rest.
        """
        for _ in range(RESET_ATTEMPTS):
            for name, (x, y) in positions.items():
                if not self._place(name, x, y):
                    return False
            if self.wait_settled(RESET_SETTLE_S):
                return True
        return False

    def wait_settled(self, timeout_s: float = 3.0) -> bool:
        """Wait until every piece moves slower than SETTLE_SPEED."""
        waited = 0.0
        previous = dict(self._poses)
        while waited < timeout_s:
            spin_for(self._node, SETTLE_SAMPLE_S)
            waited += SETTLE_SAMPLE_S
            current = dict(self._poses)
            if all(math.dist(current[n], previous[n]) <= SETTLE_SPEED * SETTLE_SAMPLE_S
                   for n in layout.PIECES if n in current and n in previous):
                return True
            previous = current
        return False
