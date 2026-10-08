"""
Score what perception saw against where the pieces really are.

Pure Python (no ROS). Used only for reporting: in vision mode the robot acts
on the detections, and these scores record how right they were.
"""

import math
from typing import Dict, List, Optional, Sequence, Tuple

from . import metrics

MATCH_RADIUS_M = 0.03       # a detection this close to a piece is "of" that piece

XY = Tuple[float, float]


def evaluate(detections: Sequence[Tuple[Optional[str], XY]], truth: Dict[str, XY],
             radius: float = MATCH_RADIUS_M) -> dict:
    """
    Match detections (label, xy) to true piece positions, nearest first.

    Each detection matches at most one piece. Returns, per piece, the label
    of its match (None if missed), whether it is right, and the position
    error; plus the counts of missed pieces and of detections matching none.
    """
    pairs = sorted((math.dist(xy, pos), i, name)
                   for i, (_, xy) in enumerate(detections) for name, pos in truth.items())
    pieces = {name: {'label': None, 'correct': False, 'error_m': None} for name in truth}
    used = set()
    for distance, i, name in pairs:
        if distance > radius or i in used or pieces[name]['label'] is not None:
            continue
        label = detections[i][0]
        if label is None:
            continue
        used.add(i)
        pieces[name] = {'label': label, 'correct': label == name, 'error_m': distance}
    missed = sum(1 for p in pieces.values() if p['label'] is None)
    false = sum(1 for i in range(len(detections)) if i not in used)
    return {'pieces': pieces, 'missed': missed, 'false': false}


def summarize(evaluations: List[dict], latencies_s: Sequence[float]) -> dict:
    """Return detection quality over many observations."""
    found = [p for e in evaluations for p in e['pieces'].values() if p['label'] is not None]
    correct = sum(1 for p in found if p['correct'])
    errors_mm = [1000 * p['error_m'] for p in found]
    return {
        'observations': len(evaluations),
        'pieces': sum(len(e['pieces']) for e in evaluations),
        'named_correctly': correct,
        'naming_accuracy': correct / len(found) if found else None,
        'missed': sum(e['missed'] for e in evaluations),
        'false': sum(e['false'] for e in evaluations),
        'position_error_mm': metrics.stats(errors_mm),
        'latency_ms': metrics.stats([1000 * t for t in latencies_s]),
    }
