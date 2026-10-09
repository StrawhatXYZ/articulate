"""
Placement memory: what the robot believes about each piece.

Pure Python. The camera cannot see inside the bins, so the robot remembers
the pieces it put there. Anything the camera sees on the table wins over
memory. A piece's start is where the camera first saw it after a reset.
"""

from typing import Dict, NamedTuple, Optional, Tuple

from lab_simulation import layout
from lab_simulation.tasks import PiecePose

TABLE, BOX, UNKNOWN = 'table', 'box', 'unknown'
XY = Tuple[float, float]


class PieceState(NamedTuple):
    """One piece as the robot believes it to be."""

    name: str
    status: str                       # TABLE, BOX or UNKNOWN
    pose: Optional[PiecePose]         # seen, or estimated when the robot placed it (BOX)
    start_xy: Optional[XY]
    placed_by_me: bool = False


class PlacementMemory:
    """Combine camera sightings with the robot's own placements."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        """Forget everything (a new scene)."""
        self._status = {name: UNKNOWN for name in layout.PIECES}
        self._pose: Dict[str, Optional[PiecePose]] = {name: None for name in layout.PIECES}
        self._start: Dict[str, Optional[XY]] = {name: None for name in layout.PIECES}
        self._placed = {name: False for name in layout.PIECES}

    def observe(self, seen: Dict[str, PiecePose]) -> None:
        """Apply one camera observation (pieces seen on the table)."""
        for name in layout.PIECES:
            if name in seen and layout.BINS[name].contains(seen[name].xy):
                # Seen inside its bin (ground truth sees into the bins).
                self._status[name], self._pose[name] = BOX, seen[name]
            elif name in seen:
                self._status[name], self._pose[name] = TABLE, seen[name]
                self._placed[name] = False
                if self._start[name] is None:
                    self._start[name] = seen[name].xy
            elif self._status[name] == TABLE:
                self._status[name], self._pose[name] = UNKNOWN, None

    def placed_in_box(self, name: str, pose: PiecePose) -> None:
        """Record that the robot put name into its bin, resting at pose."""
        self._status[name], self._pose[name], self._placed[name] = BOX, pose, True

    def moved(self, name: str) -> None:
        """Forget where name is: it was picked up (the camera will find it)."""
        self._status[name], self._pose[name], self._placed[name] = UNKNOWN, None, False

    def states(self) -> Dict[str, PieceState]:
        """Return every piece's current state."""
        return {name: PieceState(name, self._status[name], self._pose[name],
                                 self._start[name], self._placed[name])
                for name in layout.PIECES}
