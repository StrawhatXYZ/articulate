"""
Piece geometry and seeded sampling of layouts and targets.

Pure Python (no ROS). Piece sizes mirror worlds/table_world.sdf; the
consistency test checks them against the file.
"""

import math
import random
from typing import Dict, NamedTuple, Sequence, Tuple

from .kinematics import is_reachable, TIP_RADIUS

TABLE_TOP = 0.75
HOVER = 0.05            # approach height above the grasp point
LIFT = 0.05             # carry height above the grasp point
MARGIN = 0.01           # IK margin around every sampled point
MIN_SEPARATION = 0.08   # piece centre to centre
MIN_RADIUS = 0.12       # clear of the arm base (radius 0.08)
MAX_RADIUS = 0.35       # beyond the arm's reach
MIN_TRAVEL = 0.05       # a place target must differ from the start
GRASP_GAP_MAX = 0.005   # suction engages only within this gap
# The grasp approach stops this far above the piece instead of pressing on
# it: a sagging arm that presses on the top slides off it. Must stay below
# GRASP_GAP_MAX so the suction can still engage.
GRASP_STANDOFF = 0.003

XY = Tuple[float, float]


class Piece(NamedTuple):
    """A graspable piece: half its height and the grasp footprint half-width."""

    name: str
    half_height: float
    footprint: float

    @property
    def centre_z(self) -> float:
        """Return the centre height of the piece resting on the table."""
        return TABLE_TOP + self.half_height

    @property
    def grasp_z(self) -> float:
        """Return the tip-centre height that touches the top of the resting piece."""
        return TABLE_TOP + 2 * self.half_height + TIP_RADIUS


PIECES: Dict[str, Piece] = {
    piece.name: piece for piece in (
        Piece('red_cube', 0.025, 0.025),
        Piece('blue_cylinder', 0.05, 0.03),
        Piece('green_block', 0.02, 0.02),
    )
}


def grasp_gap(piece: Piece, piece_xyz: Sequence[float], tip_xyz: Sequence[float]) -> float:
    """
    Return the gap between the tip sphere and the top of the piece.

    The gap is infinite when the tip is not above the piece's footprint.
    """
    if math.dist(piece_xyz[:2], tip_xyz[:2]) > piece.footprint:
        return math.inf
    top = piece_xyz[2] + piece.half_height
    return abs((tip_xyz[2] - TIP_RADIUS) - top)


def _sample_xy(rng: random.Random) -> XY:
    r = rng.uniform(MIN_RADIUS, MAX_RADIUS)
    theta = rng.uniform(-3.0, 3.0)
    return (r * math.cos(theta), r * math.sin(theta))


def graspable(piece: Piece, xy: XY) -> bool:
    """Return True if the piece can be grasped and lifted at xy, with margin."""
    return all(
        is_reachable((xy[0], xy[1], z), MARGIN)
        for z in (piece.grasp_z, piece.grasp_z + max(HOVER, LIFT)))


def sample_layout(rng: random.Random, min_separation: float = MIN_SEPARATION,
                  max_attempts: int = 10000) -> Dict[str, XY]:
    """Return a seeded random resting position for every piece."""
    positions: Dict[str, XY] = {}
    for name in sorted(PIECES):
        for _ in range(max_attempts):
            xy = _sample_xy(rng)
            if graspable(PIECES[name], xy) and all(
                    math.dist(xy, other) >= min_separation for other in positions.values()):
                positions[name] = xy
                break
        else:
            raise RuntimeError(f'could not place {name} after {max_attempts} attempts')
    return positions


def sample_place_target(rng: random.Random, name: str, positions: Dict[str, XY],
                        max_attempts: int = 10000) -> XY:
    """Return a seeded random place target for piece name, clear of the others."""
    piece = PIECES[name]
    for _ in range(max_attempts):
        xy = _sample_xy(rng)
        clear = all(
            math.dist(xy, pos) >= (MIN_TRAVEL if other == name else MIN_SEPARATION)
            for other, pos in positions.items())
        if clear and graspable(piece, xy):
            return xy
    raise RuntimeError(f'could not find a place target for {name}')


def sample_reach_target(rng: random.Random,
                        max_attempts: int = 10000) -> Tuple[float, float, float]:
    """Return a seeded random reachable point at least 5 cm above the table."""
    for _ in range(max_attempts):
        x, y = _sample_xy(rng)
        z = rng.uniform(TABLE_TOP + 0.05, TABLE_TOP + 0.65)
        if is_reachable((x, y, z), MARGIN):
            return (x, y, z)
    raise RuntimeError('could not find a reachable target')
