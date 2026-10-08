"""
Detect and name the pieces in one RGB-D frame.

Pure Python and NumPy (no ROS): depth segmentation finds the objects, the
label matcher names their colour crops, and each label is kept at most once.
"""

from typing import List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

from .camera import CameraModel
from .naming import LabelMatcher
from .segmentation import find_blobs

CROP_MARGIN_PX = 8      # context around an object for the embedding model


class Detection(NamedTuple):
    """A named object: label None when the namer was not confident."""

    label: Optional[str]
    score: float
    centroid: Tuple[float, float, float]
    yaw: float
    bbox: Tuple[int, int, int, int]


def crop(rgb: np.ndarray, bbox: Sequence[int], margin: int = CROP_MARGIN_PX) -> np.ndarray:
    """Return the image region of bbox (u0, v0, u1, v1 inclusive) plus margin, clipped."""
    u0, v0, u1, v1 = bbox
    height, width = rgb.shape[:2]
    return rgb[max(0, v0 - margin):min(height, v1 + margin + 1),
               max(0, u0 - margin):min(width, u1 + margin + 1)]


def detect(rgb: np.ndarray, depth: np.ndarray, camera: CameraModel, matcher: LabelMatcher,
           **find_blobs_args) -> Tuple[List[Detection], List[Detection]]:
    """
    Return (kept, dropped) detections for one frame.

    Kept holds at most one detection per label, the best-scoring one.
    Dropped holds unknown objects and lower-scoring duplicates, for logging:
    they are never acted on.
    """
    blobs = find_blobs(depth, camera, **find_blobs_args)
    names = matcher.name([crop(rgb, b.bbox) for b in blobs])
    detections = [Detection(label, score, b.centroid, b.yaw, b.bbox)
                  for b, (label, score) in zip(blobs, names)]
    best = {}
    for d in detections:
        if d.label is not None and (d.label not in best or d.score > best[d.label].score):
            best[d.label] = d
    kept = list(best.values())
    dropped = [d for d in detections if all(d is not k for k in kept)]
    return kept, dropped
