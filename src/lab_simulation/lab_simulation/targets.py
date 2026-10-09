"""
Resolve a plan's place target to a safe table position.

Pure Python (no ROS). A safe position lies in the annulus around the arm's pan
axis where a vertical grasp is reachable (layout.in_workspace; test_layout.py
checks that annulus against the kinematics), clear of every other piece and of
the bins. Targets that are not safe are moved to the nearest position that is.
"""

import math
from typing import Dict, Optional, Sequence, Tuple

from lab_simulation import layout

XY = Tuple[float, float]
Result = Tuple[Optional[XY], str]

EDGE_MARGIN = 0.005
NEAR_GAP = layout.MIN_SEPARATION + 0.01
GRID_STEP = 0.005
SIDES = {'left': (0.0, 1.0), 'right': (0.0, -1.0), 'front': (1.0, 0.0), 'back': (-1.0, 0.0)}
REGIONS = {'left': (0.0, 1.0), 'right': (0.0, -1.0), 'front': (1.0, 0.0),
           'back': (-1.0, 0.0), 'center': None}
CENTER = (layout.PAN_AXIS_XY[0] + 0.18, layout.PAN_AXIS_XY[1])


def is_valid(xy: Sequence[float], others: Sequence[Sequence[float]]) -> bool:
    """Return True if xy is reachable, clear of the bins and of every other piece."""
    dx, dy = xy[0] - layout.PAN_AXIS_XY[0], xy[1] - layout.PAN_AXIS_XY[1]
    return (layout.MIN_RADIUS + EDGE_MARGIN <= math.hypot(dx, dy)
            <= layout.MAX_RADIUS - EDGE_MARGIN
            and abs(math.atan2(dy, dx)) <= layout.MAX_BEARING - 0.02
            and layout.clear_of_bins(xy)
            and all(math.dist(xy, other) >= layout.MIN_SEPARATION for other in others))


def region_point(region: str, others: Sequence[Sequence[float]]) -> Optional[XY]:
    """Return the valid position that best represents a region, or None."""
    if REGIONS[region] is None:
        return nearest_valid(CENTER, others)
    direction = REGIONS[region]
    best, best_score = None, -math.inf
    steps = int(2 * layout.MAX_RADIUS / GRID_STEP) + 1
    x0 = layout.PAN_AXIS_XY[0] - layout.MAX_RADIUS
    y0 = layout.PAN_AXIS_XY[1] - layout.MAX_RADIUS
    for i in range(steps):
        for j in range(steps):
            xy = (x0 + i * GRID_STEP, y0 + j * GRID_STEP)
            along = ((xy[0] - layout.PAN_AXIS_XY[0]) * direction[0]
                     + (xy[1] - layout.PAN_AXIS_XY[1]) * direction[1])
            across = ((xy[0] - layout.PAN_AXIS_XY[0]) * direction[1]
                      - (xy[1] - layout.PAN_AXIS_XY[1]) * direction[0])
            score = along - 0.5 * abs(across)
            if score > best_score and is_valid(xy, others):
                best, best_score = xy, score
    return best


def nearest_valid(desired: Sequence[float], others: Sequence[Sequence[float]]) -> Optional[XY]:
    """Return the valid position closest to desired, or None if there is none."""
    best, best_distance = None, math.inf
    steps = int(2 * layout.MAX_RADIUS / GRID_STEP) + 1
    x0 = layout.PAN_AXIS_XY[0] - layout.MAX_RADIUS
    y0 = layout.PAN_AXIS_XY[1] - layout.MAX_RADIUS
    for i in range(steps):
        for j in range(steps):
            xy = (x0 + i * GRID_STEP, y0 + j * GRID_STEP)
            distance = math.dist(xy, desired)
            if distance < best_distance and is_valid(xy, others):
                best, best_distance = xy, distance
    return best


def resolve(target: dict, piece: str, positions: Dict[str, XY],
            starts: Dict[str, XY]) -> Result:
    """
    Return (xy, note) for a plan target of piece, or (None, reason).

    positions are the pieces' current xy and starts where they began. A bin
    target returns that bin's centre unchecked (the arm aims its jaws there).
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
        other = target.get('piece')
        side = target.get('side')
        if other not in positions or other == piece:
            return None, f'cannot place {piece} near {other}'
        if side not in SIDES:
            return None, f'unknown side {side}'
        direction = SIDES[side]
        desired = (positions[other][0] + NEAR_GAP * direction[0],
                   positions[other][1] + NEAR_GAP * direction[1])
    elif kind == 'region':
        if target.get('region') not in REGIONS:
            return None, f'unknown region {target.get("region")}'
        xy = region_point(target['region'], others)
        if xy is None:
            return None, f'no free reachable spot for {piece}'
        return xy, 'region'
    elif kind == 'xy':
        desired = tuple(target['xy'])
    else:
        return None, f'unknown target kind {kind}'
    if is_valid(desired, others):
        return desired, 'as asked'
    xy = nearest_valid(desired, others)
    if xy is None:
        return None, f'no free reachable spot for {piece}'
    return xy, 'moved to the nearest free reachable spot'
