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
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple

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
# Sorting bins: open boxes standing on the table, one per piece colour.
BIN_RADIUS = 0.20       # bin centres, from the pan axis
BIN_INNER = 0.09        # inside width: room for the open jaws (~7 cm span)
BIN_WALL = 0.005        # wall thickness
BIN_WALL_HEIGHT = 0.02
# Pieces and targets stay this far from bin centres: the open jaws (4.8 cm
# from their centre) must clear a bin wall (5 cm out).
BIN_CLEARANCE = 0.12

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
    colour: Tuple[float, float, float]   # RGB, as in the world file
    description: str                     # what the camera's namer looks for

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


class Bin(NamedTuple):
    """An open sorting bin: square, walls only, standing on the table."""

    name: str
    piece: str              # the piece that belongs in it
    bearing: float          # rad from straight ahead, around the pan axis

    @property
    def xy(self) -> XY:
        """Return the bin's centre on the table."""
        return (PAN_AXIS_XY[0] + BIN_RADIUS * math.cos(self.bearing),
                PAN_AXIS_XY[1] + BIN_RADIUS * math.sin(self.bearing))

    @property
    def outer(self) -> float:
        """Return the outside width."""
        return BIN_INNER + 2 * BIN_WALL

    def contains(self, xy: Sequence[float], margin: float = 0.0) -> bool:
        """Return True if xy lies inside the walls, at least margin from them."""
        dx, dy = xy[0] - self.xy[0], xy[1] - self.xy[1]
        c, s = math.cos(self.bearing), math.sin(self.bearing)
        # Bins are turned to face the arm: local x is radial.
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        half = BIN_INNER / 2 - margin
        return abs(lx) <= half and abs(ly) <= half


PIECES: Dict[str, Piece] = {
    piece.name: piece for piece in (
        Piece('red_cube', 'box', 0.015, 0.015, 0.015, 0.030, (0.8, 0.1, 0.1), 'red cube'),
        Piece('blue_cylinder', 'cylinder', 0.015, 0.015, 0.025, 0.035, (0.1, 0.1, 0.8),
              'blue cylinder'),
        Piece('green_block', 'box', 0.025, 0.015, 0.015, 0.045, (0.1, 0.7, 0.2),
              'green rectangular block'),
    )
}


# On the arm's left (positive bearings), side by side; the pieces use the rest
# of the workspace.
BINS: Dict[str, Bin] = {
    b.piece: b for b in (
        Bin('green_bin', 'green_block', 0.55),
        Bin('red_bin', 'red_cube', 1.07),
        Bin('blue_bin', 'blue_cylinder', 1.59),
    )
}


def bin_walls(bin_: Bin) -> List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
    """Return (centre xyz, size xyz) of a bin's four walls, in the bin's frame."""
    reach = BIN_INNER / 2 + BIN_WALL / 2
    long = BIN_INNER + 2 * BIN_WALL
    z = BIN_WALL_HEIGHT / 2
    return [((reach, 0.0, z), (BIN_WALL, long, BIN_WALL_HEIGHT)),
            ((-reach, 0.0, z), (BIN_WALL, long, BIN_WALL_HEIGHT)),
            ((0.0, reach, z), (BIN_INNER, BIN_WALL, BIN_WALL_HEIGHT)),
            ((0.0, -reach, z), (BIN_INNER, BIN_WALL, BIN_WALL_HEIGHT))]


def clear_of_bins(xy: Sequence[float]) -> bool:
    """Return True if xy is at least BIN_CLEARANCE from every bin centre."""
    return all(math.dist(xy, b.xy) >= BIN_CLEARANCE for b in BINS.values())


def _sample_xy(rng: random.Random) -> XY:
    """Return a workspace point clear of the bins."""
    while True:
        # Uniform over the annulus area, not over the radius.
        r = math.sqrt(rng.uniform(MIN_RADIUS ** 2, MAX_RADIUS ** 2))
        bearing = rng.uniform(-MAX_BEARING, MAX_BEARING)
        xy = (PAN_AXIS_XY[0] + r * math.cos(bearing), PAN_AXIS_XY[1] + r * math.sin(bearing))
        if clear_of_bins(xy):
            return xy


def in_workspace(xy: XY) -> bool:
    """Return True if xy lies in the sampling annulus."""
    dx, dy = xy[0] - PAN_AXIS_XY[0], xy[1] - PAN_AXIS_XY[1]
    return (MIN_RADIUS - 1e-9 <= math.hypot(dx, dy) <= MAX_RADIUS + 1e-9
            and abs(math.atan2(dy, dx)) <= MAX_BEARING + 1e-9)


# Areas of the table for instructions ("put it in the front"). The bins fill
# the robot's left, so the free table is a thin curved strip: the annulus
# sector between these bearings (rad), about 7 cm deep. Along the strip, as
# the robot sees it, are left (the third nearest the bins), center and right;
# across it, front (the far half, away from the robot) and back (the near
# half). A spot has a side and a depth, so it is, say, both front and left.
REGIONS = ('left', 'right', 'front', 'back', 'center')
FREE_BEARINGS = (-1.73, -0.05)
_SIDE_BEARINGS = (-1.17, -0.61)
_DEPTH_RADIUS = 0.18


def side_of(xy: Sequence[float]) -> str:
    """Return 'left', 'center' or 'right': where xy lies along the free strip."""
    bearing = math.atan2(xy[1] - PAN_AXIS_XY[1], xy[0] - PAN_AXIS_XY[0])
    if bearing > _SIDE_BEARINGS[1]:
        return 'left'
    return 'right' if bearing < _SIDE_BEARINGS[0] else 'center'


def depth_of(xy: Sequence[float]) -> str:
    """Return 'front' (away from the robot) or 'back': where xy lies across the strip."""
    radius = math.hypot(xy[0] - PAN_AXIS_XY[0], xy[1] - PAN_AXIS_XY[1])
    return 'front' if radius > _DEPTH_RADIUS else 'back'


def in_region(xy: Sequence[float], region: str) -> bool:
    """Return True if xy lies in a table area (a side or a depth)."""
    return region in (side_of(xy), depth_of(xy))


def area_of(xy: Sequence[float]) -> str:
    """Describe where xy is, e.g. 'front left'."""
    return f'{depth_of(xy)} {side_of(xy)}'


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


def sample_yaws(rng: random.Random) -> Dict[str, float]:
    """Return a seeded random resting yaw for every piece."""
    return {name: rng.uniform(-math.pi, math.pi) for name in sorted(PIECES)}


def sample_light(rng: random.Random) -> Tuple[XYZ, float]:
    """Return a seeded random sun direction and brightness (0.4-1.0)."""
    direction = (rng.uniform(-0.8, 0.8), rng.uniform(-0.8, 0.8), -1.0)
    return direction, rng.uniform(0.4, 1.0)


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
