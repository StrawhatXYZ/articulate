"""
Piece geometry and seeded sampling of layouts and targets.

Pure Python (no ROS). Piece sizes mirror worlds/table_world.sdf; the
consistency test checks them against the file. Positions are in the world
frame; the SO-101 stands at the origin on the table top, facing +x.

Every position is sampled in an annulus around the arm's pan axis where a
vertical grasp and the hover above it are reachable (spec section 2);
test_layout.py checks that against the arm's kinematics.
"""

import math
import random
from typing import Dict, List, NamedTuple, Optional, Tuple

TABLE_TOP = 0.75
PAN_AXIS_XY = (0.0388353, 0.0)   # shoulder_pan axis in the world frame
MIN_RADIUS = 0.14       # closer, the gripper meets the arm's own shoulder
MAX_RADIUS = 0.22       # further, the hover above a piece is out of reach
MAX_BEARING = 1.75      # rad either side of straight ahead (pan limit 1.92)
HOVER = 0.05            # approach height above the grasp point
LIFT = 0.05             # carry height above the grasp point
MIN_SEPARATION = 0.08   # piece centre to centre: room for the open jaws
MIN_TRAVEL = 0.05       # a place target must differ from the start
REACH_CLEARANCE = 0.06  # reach targets stay this far from every piece (xy)
REACH_HEIGHT = (0.03, 0.08)   # reach targets, above the table top

XY = Tuple[float, float]
XYZ = Tuple[float, float, float]


class Piece(NamedTuple):
    """A graspable piece. Sizes are half extents in its own frame."""

    name: str
    shape: str              # 'box' or 'cylinder'
    half_x: float           # box half length, or cylinder radius
    half_y: float           # box half width, or cylinder radius
    half_height: float
    mass: float

    @property
    def centre_z(self) -> float:
        """Return the centre height of the piece resting upright on the table."""
        return TABLE_TOP + self.half_height

    @property
    def grasp_z(self) -> float:
        """Return the grasp point height: the jaws close around the centre."""
        return self.centre_z

    @property
    def grasp_width(self) -> float:
        """Return the width the jaws close on."""
        return 2 * min(self.half_x, self.half_y)

    def closing_yaws(self, piece_yaw: float) -> Optional[List[float]]:
        """
        Return the world yaws the jaws can close along, or None if any works.

        The jaws must close across the piece's narrow side: either side of a
        square box, the short side of a block, any side of a cylinder.
        """
        if self.shape == 'cylinder':
            return None
        if math.isclose(self.half_x, self.half_y):
            return [piece_yaw, piece_yaw + math.pi / 2]
        across_y = self.half_y < self.half_x
        return [piece_yaw + (math.pi / 2 if across_y else 0.0)]


PIECES: Dict[str, Piece] = {
    piece.name: piece for piece in (
        Piece('red_cube', 'box', 0.015, 0.015, 0.015, 0.030),
        Piece('blue_cylinder', 'cylinder', 0.015, 0.015, 0.025, 0.035),
        Piece('green_block', 'box', 0.025, 0.015, 0.015, 0.045),
    )
}


def _sample_xy(rng: random.Random) -> XY:
    # Uniform over the annulus area, not over the radius.
    r = math.sqrt(rng.uniform(MIN_RADIUS ** 2, MAX_RADIUS ** 2))
    bearing = rng.uniform(-MAX_BEARING, MAX_BEARING)
    return (PAN_AXIS_XY[0] + r * math.cos(bearing), PAN_AXIS_XY[1] + r * math.sin(bearing))


def in_workspace(xy: XY) -> bool:
    """Return True if xy lies in the sampling annulus."""
    dx, dy = xy[0] - PAN_AXIS_XY[0], xy[1] - PAN_AXIS_XY[1]
    return (MIN_RADIUS - 1e-9 <= math.hypot(dx, dy) <= MAX_RADIUS + 1e-9
            and abs(math.atan2(dy, dx)) <= MAX_BEARING + 1e-9)


def sample_layout(rng: random.Random, min_separation: float = MIN_SEPARATION,
                  max_attempts: int = 10000) -> Dict[str, XY]:
    """Return a seeded random resting position for every piece."""
    positions: Dict[str, XY] = {}
    for name in sorted(PIECES):
        for _ in range(max_attempts):
            xy = _sample_xy(rng)
            if all(math.dist(xy, other) >= min_separation for other in positions.values()):
                positions[name] = xy
                break
        else:
            raise RuntimeError(f'could not place {name} after {max_attempts} attempts')
    return positions


def sample_place_target(rng: random.Random, name: str, positions: Dict[str, XY],
                        max_attempts: int = 10000) -> XY:
    """Return a seeded random place target for piece name, clear of the others."""
    for _ in range(max_attempts):
        xy = _sample_xy(rng)
        if all(math.dist(xy, pos) >= (MIN_TRAVEL if other == name else MIN_SEPARATION)
               for other, pos in positions.items()):
            return xy
    raise RuntimeError(f'could not find a place target for {name}')


def sample_reach_target(rng: random.Random, positions: Dict[str, XY],
                        max_attempts: int = 10000) -> XYZ:
    """Return a seeded random reach target clear of the pieces."""
    for _ in range(max_attempts):
        xy = _sample_xy(rng)
        z = TABLE_TOP + rng.uniform(*REACH_HEIGHT)
        if all(math.dist(xy, pos) >= REACH_CLEARANCE for pos in positions.values()):
            return (xy[0], xy[1], z)
    raise RuntimeError('could not find a reach target')
