"""Publish RViz markers for the table and the pieces at their live poses."""

from functools import partial
from typing import Dict

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

from . import layout
from .scene import POSE_TOPIC

TOPIC = '/lab/piece_markers'

# (marker type, scale x, y, z, colour r, g, b), from layout.PIECES.
SHAPES = {
    name: (Marker.CYLINDER if piece.shape == 'cylinder' else Marker.CUBE,
           2 * piece.half_x, 2 * piece.half_y, 2 * piece.half_height, *piece.colour)
    for name, piece in layout.PIECES.items()
}


class ObjectMarkers(Node):
    """Republish piece poses from Gazebo as a MarkerArray, plus the table."""

    def __init__(self):
        super().__init__('object_markers')
        self._poses: Dict[str, Pose] = {}
        self._publisher = self.create_publisher(MarkerArray, TOPIC, 10)
        for name in layout.PIECES:
            self.create_subscription(PoseStamped, POSE_TOPIC.format(name=name),
                                     partial(self._on_pose, name), qos_profile_sensor_data)
        self.create_timer(0.1, self._publish)

    def _on_pose(self, name: str, msg: PoseStamped) -> None:
        self._poses[name] = msg.pose

    def _marker(self, marker_id, kind, scale, colour):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lab'
        marker.id = marker_id
        marker.type = kind
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b = colour
        marker.color.a = 1.0
        return marker

    def _publish(self) -> None:
        markers = MarkerArray()
        table = self._marker(0, Marker.CUBE, (1.0, 1.0, 0.05), (0.6, 0.4, 0.2))
        table.pose.position.z = 0.725
        table.pose.orientation.w = 1.0
        table.color.a = 0.6
        markers.markers.append(table)
        for marker_id, name in enumerate(sorted(SHAPES), start=1):
            if name not in self._poses:
                continue
            kind, sx, sy, sz, r, g, b = SHAPES[name]
            marker = self._marker(marker_id, kind, (sx, sy, sz), (r, g, b))
            marker.pose = self._poses[name]
            markers.markers.append(marker)
        self._publisher.publish(markers)


def main() -> None:
    """Run the marker publisher until shutdown."""
    rclpy.init()
    node = ObjectMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C can shut the context down while spin() waits; that is a
        # normal exit, anything else is not.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
