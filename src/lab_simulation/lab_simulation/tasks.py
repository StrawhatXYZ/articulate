"""
Benchmark tasks: reach, and suction pick-and-place.

Pure Python (no ROS). Tasks drive the arm and scene through small interfaces
(arm.Arm and scene.Scene implement them), so they can be unit-tested with
fakes. All times are simulation seconds.
"""

from dataclasses import asdict, dataclass, field
import math
import random
from typing import Callable, Dict, NamedTuple, Optional, Sequence, Tuple

from . import layout
from .kinematics import forward, inverse, Unreachable

REACH_TOLERANCE_M = 0.01
PLACE_TOLERANCE_M = 0.02
# A held piece hangs ~1 cm off its grip (suction joint compliance); a dropped
# piece separates from the tip. Placement accuracy is judged separately.
CARRY_TOLERANCE_M = 0.03
DISTURB_THRESHOLD_M = 0.01
PLACE_CLEARANCE_M = 0.003
MOVE_TIMEOUT_S = 5.0
HOME = (0.0, 0.0, 0.0)

XYZ = Tuple[float, float, float]


class SimulationUnresponsive(RuntimeError):
    """Raised when the simulation stops publishing feedback."""


class MoveResult(NamedTuple):
    """Outcome of one arm move: settled, aborted by a monitor, timings, error."""

    reached: bool
    aborted: bool
    settle_time_s: float
    max_error: float


@dataclass
class TrialResult:
    """One trial; error_m is tip error (reach) or placement error (pick_place)."""

    task: str
    index: int
    success: bool
    failure: Optional[str]
    cycle_time_s: float
    error_m: Optional[float] = None
    disturbed: bool = False
    details: Dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        """Return the result as a JSON-serialisable dict."""
        return asdict(self)


def _add(a: Sequence[float], b: Sequence[float]) -> XYZ:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Sequence[float], b: Sequence[float]) -> XYZ:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


class ReachTask:
    """Move the tip to a random reachable point and measure the error."""

    name = 'reach'

    def __init__(self, arm, now: Callable[[], float]):
        self._arm = arm
        self._now = now

    def run(self, rng: random.Random, index: int) -> TrialResult:
        """Run one reach trial."""
        target = layout.sample_reach_target(rng)
        details = {'target': list(target)}
        start = self._now()
        try:
            q = inverse(target)
        except Unreachable:
            return TrialResult(self.name, index, False, 'unreachable', 0.0, details=details)
        move = self._arm.move_to(q, MOVE_TIMEOUT_S)
        elapsed = self._now() - start
        error = math.dist(forward(self._arm.joint_positions()), target)
        details['settle_time_s'] = move.settle_time_s
        details['max_joint_error_rad'] = move.max_error
        if not move.reached:
            return TrialResult(self.name, index, False, 'timeout', elapsed, error,
                               details=details)
        ok = error <= REACH_TOLERANCE_M
        return TrialResult(self.name, index, ok, None if ok else 'placement_error',
                           elapsed, error, details=details)


class PickPlaceTask:
    """Pick a random piece with suction and place it at a random target."""

    name = 'pick_place'

    def __init__(self, arm, scene, now: Callable[[], float]):
        self._arm = arm
        self._scene = scene
        self._now = now

    def run(self, rng: random.Random, index: int) -> TrialResult:
        """Run one pick-and-place trial from the current scene."""
        names = sorted(layout.PIECES)
        start = {name: self._scene.pose(name) for name in names}
        name = rng.choice(names)
        target = layout.sample_place_target(
            rng, name, {n: (p[0], p[1]) for n, p in start.items()})
        details = {'piece': name, 'start': list(start[name]), 'target': list(target)}
        t0 = self._now()
        try:
            failure = self._execute(layout.PIECES[name], start[name], target, details)
        finally:
            self._scene.detach(name)
        elapsed = self._now() - t0
        if failure is not None:
            return TrialResult(self.name, index, False, failure, elapsed, details=details)
        self._scene.wait_settled()
        final = self._scene.pose(name)
        details['final'] = list(final)
        error = math.dist(final[:2], target)
        disturbed = any(
            math.dist(self._scene.pose(other), start[other]) > DISTURB_THRESHOLD_M
            for other in names if other != name)
        ok = error <= PLACE_TOLERANCE_M
        return TrialResult(self.name, index, ok, None if ok else 'placement_error',
                           elapsed, error, disturbed, details)

    def _move(self, xyz: XYZ, monitor=None) -> Optional[str]:
        try:
            q = inverse(xyz)
        except Unreachable:
            return 'unreachable'
        result = self._arm.move_to(q, MOVE_TIMEOUT_S, monitor)
        if result.aborted:
            return 'dropped'
        if not result.reached:
            return 'timeout'
        return None

    def _tip(self) -> XYZ:
        return forward(self._arm.joint_positions())

    def _execute(self, piece, start: XYZ, target, details: dict) -> Optional[str]:
        x, y, z = start
        grasp_z = z + piece.half_height + layout.TIP_RADIUS + layout.GRASP_STANDOFF
        for xyz in ((x, y, grasp_z + layout.HOVER), (x, y, grasp_z)):
            failure = self._move(xyz)
            if failure:
                return failure

        tip = self._tip()
        if not self._scene.attach(piece.name, tip):
            details['grasp_gap_m'] = self._scene.grasp_gap(piece.name, tip)
            return 'grasp_miss'
        offset = _sub(self._scene.pose(piece.name), tip)

        details['carry_deviation_m'] = 0.0

        def carrying() -> bool:
            deviation = math.dist(self._scene.pose(piece.name), _add(self._tip(), offset))
            details['carry_deviation_m'] = max(details['carry_deviation_m'], deviation)
            return deviation <= CARRY_TOLERANCE_M

        # Aim the tip so that the piece, not the tip, lands on the target.
        tx, ty = target[0] - offset[0], target[1] - offset[1]
        place_z = piece.centre_z - offset[2] + PLACE_CLEARANCE_M
        for xyz in ((tip[0], tip[1], tip[2] + layout.LIFT),
                    (tx, ty, place_z + layout.LIFT),
                    (tx, ty, place_z)):
            failure = self._move(xyz, carrying)
            if failure:
                return failure

        self._scene.detach(piece.name)
        failure = self._move((tx, ty, place_z + layout.HOVER))
        if failure:
            return failure
        home = self._arm.move_to(HOME, MOVE_TIMEOUT_S)
        return None if home.reached else 'timeout'
