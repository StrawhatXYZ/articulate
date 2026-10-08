#!/usr/bin/env python3
"""
Detect and name the pieces on the table from the overhead RGB-D camera.

Subscribes to synchronised colour and depth frames and the camera info. Each
message on /lab/perception/request (std_msgs/Empty) asks for one detection:
the first frame taken after the request is processed and its named objects
are published on /lab/perception/objects (vision_msgs/Detection3DArray),
stamped with the frame's own time. Between requests no frame is processed,
so the models do not compete with the simulation for the CPU.

Runs with the /opt/lab-ai interpreter (PyTorch); launch with
lab_simulation's arm_world.launch.py perception:=true.
"""

import time

from lab_perception.camera import CameraModel
from lab_perception.detector import detect
from lab_perception.messages import detections_to_msg
from lab_perception.naming import ColourEmbedder, LabelMatcher, SiglipEmbedder
import message_filters
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty
from vision_msgs.msg import Detection3DArray

TOPIC = '/lab/perception/objects'
REQUEST_TOPIC = '/lab/perception/request'


def image_array(msg: Image) -> np.ndarray:
    """Return an rgb8 or 32FC1 image message as a NumPy array."""
    if msg.encoding == 'rgb8':
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == '32FC1':
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
    raise ValueError(f'unsupported image encoding {msg.encoding}')


class PerceptionNode(Node):
    """Turns RGB-D frames into named detections."""

    def __init__(self):
        super().__init__('perception')
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('camera_position', [0.0, 0.0, 1.0]), ('camera_yaw', 0.0), ('camera_pitch', 0.0),
            ('table_top', 0.75), ('axis_xy', [0.0, 0.0]), ('radius', [0.10, 0.30]),
            ('height', [0.005, 0.07]), ('exclude_xy', [0.0]), ('exclude_radius', 0.0),
            ('labels', ['']), ('prompts', ['']), ('embedder', 'siglip'),
            ('model_id', 'google/siglip2-base-patch16-224'),
            ('min_score', 0.0), ('min_margin', 0.0))}
        self._params = p
        xy = p['exclude_xy']
        self._exclude = [((xy[i], xy[i + 1]), p['exclude_radius'])
                         for i in range(0, len(xy) - 1, 2)] if p['exclude_radius'] > 0 else []
        embedder = ColourEmbedder() if p['embedder'] == 'colour' else SiglipEmbedder(
            p['model_id'])
        self._matcher = LabelMatcher(embedder, dict(zip(p['labels'], p['prompts'])),
                                     p['min_score'], p['min_margin'])
        self._camera = None
        self._frame = None
        self._requested = None      # time of the pending request, or None
        self._publisher = self.create_publisher(Detection3DArray, TOPIC, 10)
        self.create_subscription(CameraInfo, '/lab/camera/camera_info', self._on_info, 10)
        sync = message_filters.ApproximateTimeSynchronizer(
            [message_filters.Subscriber(self, Image, '/lab/camera/image'),
             message_filters.Subscriber(self, Image, '/lab/camera/depth_image')],
            queue_size=5, slop=0.02)
        sync.registerCallback(self._on_frame)
        self.create_subscription(Empty, REQUEST_TOPIC, self._on_request, 10)
        self.create_timer(0.02, self._process)
        self.get_logger().info(f'perception ready: {len(self._matcher.labels)} labels, '
                               f"{p['embedder']} embedder")

    def _on_info(self, msg: CameraInfo) -> None:
        if self._camera is None:
            p = self._params
            self._camera = CameraModel.from_info(msg.k, msg.width, msg.height,
                                                 p['camera_position'], p['camera_yaw'],
                                                 p['camera_pitch'])

    def _on_request(self, _msg: Empty) -> None:
        # Keep the first pending request: repeats (the client re-asks while
        # it waits) must not push the deadline for a fresh frame forward.
        if self._requested is None:
            self._requested = self.get_clock().now().nanoseconds * 1e-9

    def _on_frame(self, rgb: Image, depth: Image) -> None:
        self._frame = (rgb, depth)

    def _process(self) -> None:
        if self._requested is None or self._frame is None or self._camera is None:
            return
        rgb, depth = self._frame
        stamp = depth.header.stamp.sec + depth.header.stamp.nanosec * 1e-9
        if stamp <= self._requested:
            return      # taken before the request: wait for a newer frame
        self._frame = None
        self._requested = None
        start = time.monotonic()
        p = self._params
        kept, dropped = detect(image_array(rgb), image_array(depth), self._camera,
                               self._matcher, table_top=p['table_top'],
                               axis_xy=p['axis_xy'], radius=tuple(p['radius']),
                               height=tuple(p['height']), exclude=self._exclude)
        header = depth.header
        header.frame_id = 'world'
        self._publisher.publish(detections_to_msg(kept + dropped, header))
        self.get_logger().debug(f'{len(kept)} named, {len(dropped)} dropped in '
                                f'{1000 * (time.monotonic() - start):.0f} ms')


def main() -> None:
    """Run the perception node until shutdown."""
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
