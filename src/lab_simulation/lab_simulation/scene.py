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

from . import layout
from .ros_util import spin_for, spin_until

# One pose topic per piece, bridged from each piece's PosePublisher.
POSE_TOPIC = '/lab/pieces/{name}/pose'
SET_POSE_SERVICE = '/world/table_world/set_pose'
SETTLE_SPEED = 0.001    # m/s: below this a piece counts as at rest
SETTLE_SAMPLE_S = 0.1

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

    def _on_pose(self, name: str, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._poses[name] = (p.x, p.y, p.z)

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
        self._attach[name].publish(Empty())
        spin_for(self._node, 0.05)
        return True

    def detach(self, name: str) -> None:
        """Release suction on a piece."""
        self._detach[name].publish(Empty())
        spin_for(self._node, 0.05)

    def detach_all(self) -> None:
        """Release suction on every piece."""
        for name in layout.PIECES:
            self._detach[name].publish(Empty())
        spin_for(self._node, 0.05)

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

    def reset(self, positions: Dict[str, Tuple[float, float]], timeout_s: float = 3.0) -> bool:
        """Teleport every piece to positions, upright, and wait until they rest."""
        for name, (x, y) in positions.items():
            if not self._place(name, x, y):
                return False
        return self.wait_settled(timeout_s)

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
