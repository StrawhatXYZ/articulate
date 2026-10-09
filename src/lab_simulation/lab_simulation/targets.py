"""
Resolve a plan's place target to a safe table position.

Pure Python (no ROS). A safe position is reachable with a vertical grasp,
inside the free table area, clear of the bins and of every other piece.
Targets that are not safe move to the nearest position that is.
From PR #3 by MunuriGangadhar.
"""

import math
from typing import Dict, Optional, Sequence, Tuple

from lab_simulation import layout

XY = Tuple[float, float]
EDGE_MARGIN = 0.005
NEAR_GAP = layout.MIN_SEPARATION + 0.01
GRID_STEP = 0.005
# Unit directions, world frame, as the robot sees them: front = away (+x).
SIDES: Dict[str, XY] = {'left': (0.0, 1.0), 'right': (0.0, -1.0),
                        'front': (1.0, 0.0), 'back': (-1.0, 0.0)}
_REGION_ANCHORS = {'left': (-0.33, 0.18), 'right': (-1.45, 0.18), 'front': (-0.89, 0.205),
                   'back': (-0.89, 0.155), 'center': (-0.89, 0.18)}   # (bearing, radius)


def is_valid(xy: Sequence[float], others: Sequence[Sequence[float]]) -> bool:
    """Return True if xy is reachable, in the free area and clear of bins and pieces."""
    dx, dy = xy[0] - layout.PAN_AXIS_XY[0], xy[1] - layout.PAN_AXIS_XY[1]
    bearing = math.atan2(dy, dx)
    return (layout.MIN_RADIUS + EDGE_MARGIN <= math.hypot(dx, dy)
            <= layout.MAX_RADIUS - EDGE_MARGIN
            and layout.FREE_BEARINGS[0] <= bearing <= layout.FREE_BEARINGS[1]
            and layout.clear_of_bins(xy)
            and all(math.dist(xy, other) >= layout.MIN_SEPARATION for other in others))


def _grid():
    steps = int(2 * layout.MAX_RADIUS / GRID_STEP) + 1
    x0 = layout.PAN_AXIS_XY[0] - layout.MAX_RADIUS
    y0 = layout.PAN_AXIS_XY[1] - layout.MAX_RADIUS
    for i in range(steps):
        for j in range(steps):
            yield (x0 + i * GRID_STEP, y0 + j * GRID_STEP)


def nearest_valid(desired: Sequence[float], others, region: Optional[str] = None
                  ) -> Optional[XY]:
    """Return the valid position closest to desired (inside region if given), or None."""
    best, best_distance = None, math.inf
    for xy in _grid():
        distance = math.dist(xy, desired)
        if (distance < best_distance and is_valid(xy, others)
                and (region is None or layout.in_region(xy, region))):
            best, best_distance = xy, distance
    return best


def region_point(region: str, others) -> Optional[XY]:
    """Return the valid position nearest the middle of a region, or None."""
    bearing, radius = _REGION_ANCHORS[region]
    anchor = (layout.PAN_AXIS_XY[0] + radius * math.cos(bearing),
              layout.PAN_AXIS_XY[1] + radius * math.sin(bearing))
    return nearest_valid(anchor, others, region)


def resolve(target: dict, piece: str, positions: Dict[str, XY],
            starts: Dict[str, XY]) -> Tuple[Optional[XY], str]:
    """
    Return (xy, note) for a target of piece, or (None, reason).

    positions are the other pieces' current table positions (pieces in bins
    are absent) and starts where they began. A bin target returns that bin's
    centre unchecked (the arm aims its jaws there).
    """
    kind = target.get('kind')
    others = [xy for name, xy in positions.items() if name != piece]
    if kind == 'bin':
        return layout.BINS[piece].xy, 'bin'
    if kind == 'start':
        desired = starts.get(piece)
        if desired is None:
            return None, f'no start position known for {piece}'
    elif kind == 'near':
        other, side = target.get('piece'), target.get('side')
        if other not in positions or other == piece:
            return None, f'cannot place {piece} near {other}'
        if side is None:
            xy = nearest_valid(positions[other], others)
            return (xy, 'near') if xy else (None, f'no free spot near {other}')
        if side not in SIDES:
            return None, f'unknown side {side}'
        d = SIDES[side]
        desired = (positions[other][0] + NEAR_GAP * d[0], positions[other][1] + NEAR_GAP * d[1])
    elif kind == 'region':
        if target.get('region') not in layout.REGIONS:
            return None, f'unknown region {target.get("region")}'
        xy = region_point(target['region'], others)
        return (xy, 'region') if xy else (None, f'no free reachable spot for {piece}')
    elif kind == 'xy':
        desired = tuple(target['xy'])
    else:
        return None, f'unknown target kind {kind}'
    if is_valid(desired, others):
        return tuple(desired), 'as asked'
    xy = nearest_valid(desired, others)
    if xy is None:
        return None, f'no free reachable spot for {piece}'
    return xy, 'moved to the nearest free reachable spot'
