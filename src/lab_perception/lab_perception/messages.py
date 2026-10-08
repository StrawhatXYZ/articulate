"""Convert detections to and from vision_msgs/Detection3DArray."""

import math
from typing import List

from std_msgs.msg import Header
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from .detector import Detection

UNKNOWN = 'unknown'


def detections_to_msg(detections: List[Detection], header: Header) -> Detection3DArray:
    """Return the message for detections; header carries the source frame stamp."""
    msg = Detection3DArray(header=header)
    for d in detections:
        item = Detection3D(header=header)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = d.label or UNKNOWN
        hypothesis.hypothesis.score = float(d.score)
        pose = item.bbox.center
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in d.centroid)
        pose.orientation.z = math.sin(d.yaw / 2)
        pose.orientation.w = math.cos(d.yaw / 2)
        hypothesis.pose.pose = pose
        item.results.append(hypothesis)
        # The image box is not carried; the 3D box size is left unset.
        msg.detections.append(item)
    return msg


def detections_from_msg(msg: Detection3DArray) -> List[Detection]:
    """Return the detections in a message (image boxes are not carried)."""
    detections = []
    for item in msg.detections:
        result = item.results[0]
        pose = item.bbox.center
        label = result.hypothesis.class_id
        detections.append(Detection(
            None if label == UNKNOWN else label, result.hypothesis.score,
            (pose.position.x, pose.position.y, pose.position.z),
            2 * math.atan2(pose.orientation.z, pose.orientation.w), (0, 0, 0, 0)))
    return detections
