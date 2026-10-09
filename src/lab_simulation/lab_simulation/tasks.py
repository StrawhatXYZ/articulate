"""
Benchmark tasks for the SO-101: reach, two-finger pick-and-place, and colour sorting.

Pure Python (no ROS). Tasks drive the robot through small interfaces
(arm.Arm, gripper.Gripper and scene.Scene implement them; the tests use
fakes) and use so101_kinematics for goals. All times are simulation seconds.

Tasks plan from scene.observe(): the simulator's poses in ground-truth
mode, the camera's detections in vision mode. Scoring always uses the true
poses (scene.pose), and so do the simulation's referee checks (the grasp
gate below, drop detection), which decide nothing about where to move.

A grasp counts only if the jaws really closed on the piece: the gripper
stalled open (something is between the jaws) and the piece sits between
them. In the default "latched" grasp mode the piece is then locked to the
gripper, because Gazebo cannot hold a friction grasp indefinitely (spec
section 4.4); in "friction" mode nothing is added.
"""

from dataclasses import asdict, dataclass, field
import math
import random
from typing import Callable, Dict, List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

from . import layout
from .so101_kinematics import Unreachable

GRASP_MODES = ('latched', 'friction')
REACH_TOLERANCE_M = 0.01
PLACE_TOLERANCE_M = 0.02
# A placed piece must rest upright on the table: not on the floor, not on its side.
PLACE_HEIGHT_TOLERANCE_M = 0.01
UPRIGHT_TOLERANCE_DEG = 15.0
# A held piece stays at its grip position; further than this it has slipped
# or fallen out of the jaws.
CARRY_TOLERANCE_M = 0.01
# The latch joint is slightly compliant: a held piece sways after the arm
# stops. Let it come to rest (up to this long) before checking the grip.
CARRY_SETTLE_S = 0.5
DISTURB_THRESHOLD_M = 0.01
PLACE_CLEARANCE_M = 0.003
# The jaw stalls above this angle only if something is between the jaws
# (fully closed is -0.17 rad; a 3 cm piece stops it near 0.19 rad).
HOLDING_MIN_ANGLE = 0.05
# After closing, the gripped piece's centre must be this close to grasp_link
# (the moving jaw pushes it ~9 mm towards the fixed jaw).
GRIP_MAX_OFFSET_M = 0.015
PLACE_SOLVE_ITERATIONS = 5
# Before latching, ease the squeeze by this much and let the piece settle:
# at full servo torque it is pressed a few millimetres into the pads, and a
# latch frozen in that overlap kicks the piece away when it is released.
LATCH_EASE_RAD = 0.05
LATCH_SETTLE_S = 0.3
# Inside a bin the jaws open only this far to let go: fully open (0.6 rad)
# the moving jaw reaches 48 mm from the grasp point, past the 45 mm to a wall.
# At 0.5 rad it reaches 41 mm and clears the piece by ~11 mm; the slide off
# the piece is shortened to match, so the jaw does not brush it over.
BIN_RELEASE_RAD = 0.5
BIN_RELEASE_CLEARANCE_M = 0.005
# Where a gripped piece sits in the grasp frame: pressed ~9 mm against the
# fixed jaw. In vision mode placements are aimed with this, not with the
# measured grip, which only the simulator knows.
NOMINAL_GRIP = np.array([0.009, 0.0, 0.0])
# After release, slide the jaws this far along the closing axis, moving the
# fixed jaw off the piece, before lifting: the gripped piece was pressed
# against the fixed jaw (grasp frame +x), and lifting straight up would drag
# it along. The open moving jaw has ~17 mm to spare on the other side.
RELEASE_CLEARANCE_M = 0.008

XYZ = Tuple[float, float, float]


class SimulationUnresponsive(RuntimeError):
    """Raised when the simulation stops publishing feedback."""


class MoveResult(NamedTuple):
    """Outcome of one arm move: planned, reached, duration and final joint error."""

    planned: bool
    reached: bool
    duration_s: float
    max_error: float


class Observation(NamedTuple):
    """What the robot believes about the pieces at one moment."""

    poses: Dict[str, 'PiecePose']   # pieces seen, by label
    source: str                     # 'ground_truth' or 'vision'
    evaluation: Optional[dict] = None   # vision only: scored against the truth
    wait_s: float = 0.0             # simulation seconds spent waiting for it
    latency_s: float = 0.0          # wall seconds spent waiting for it

    @property
    def vision(self) -> bool:
        return self.source == 'vision'


def record_observation(observation: Observation, details: dict) -> None:
    """Add an observation's detection scores to a trial's details."""
    if observation.evaluation is not None:
        details.setdefault('perception', []).append(observation.evaluation)
        details.setdefault('perception_latency_s', []).append(observation.latency_s)


class PiecePose(NamedTuple):
    """A piece's centre, its yaw and how far it is tilted from upright."""

    x: float
    y: float
    z: float
    yaw: float = 0.0
    tilt_deg: float = 0.0

    @property
    def xyz(self) -> XYZ:
        return (self.x, self.y, self.z)

    @property
    def xy(self) -> Tuple[float, float]:
        return (self.x, self.y)


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


def _move_failure(result: MoveResult) -> Optional[str]:
    if not result.planned:
        return 'planning_failed'
    if not result.reached:
        return 'timeout'
    return None


def _approach_yaws(yaws: Optional[List[float]], piece_yaw: float) -> List[List[float]]:
    """
    Return closing yaws to try one by one when picking out of a bin.

    Each allowed yaw (any quarter turn for a cylinder) and its half turn,
    which puts the fixed and the moving jaw on opposite sides. From PR #3.
    """
    base = yaws if yaws is not None else [piece_yaw + k * math.pi / 2 for k in range(2)]
    return [[yaw + turn] for yaw in base for turn in (0.0, math.pi)]


def vertical_rotation(yaw: float) -> np.ndarray:
    """Return the grasp frame rotation pointing straight down, closing along yaw."""
    x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    z = np.array([0.0, 0.0, -1.0])
    return np.column_stack([x, np.cross(z, x), z])


class ReachTask:
    """Move the grasp point, pointing down, to a random point clear of the pieces."""

    name = 'reach'

    def __init__(self, arm, scene, kinematics, now: Callable[[], float]):
        self._arm = arm
        self._scene = scene
        self._kin = kinematics
        self._now = now

    def run(self, rng: random.Random, index: int) -> TrialResult:
        """Run one reach trial."""
        observation = self._scene.observe()
        positions = {n: p.xy for n, p in sorted(observation.poses.items())}
        target = layout.sample_reach_target(rng, positions)
        details = {'target': list(target)}
        record_observation(observation, details)
        start = self._now()
        try:
            q = self._kin.inverse(target, None)
        except Unreachable:
            return TrialResult(self.name, index, False, 'unreachable', 0.0, details=details)
        self._scene.update_planning_scene()
        move = self._arm.move_to(q)
        elapsed = self._now() - start
        details['max_joint_error_rad'] = move.max_error
        failure = _move_failure(move)
        if failure == 'planning_failed':
            return TrialResult(self.name, index, False, failure, elapsed, details=details)
        error = math.dist(self._kin.forward(self._arm.joint_positions())[:3, 3], target)
        if failure is None and error > REACH_TOLERANCE_M:
            failure = 'placement_error'
        return TrialResult(self.name, index, failure is None, failure, elapsed, error,
                           details=details)


class PickPlaceTask:
    """Pick a random piece with the jaws and place it at a random target."""

    name = 'pick_place'

    def __init__(self, arm, gripper, scene, kinematics, now: Callable[[], float],
                 grasp_mode: str = 'latched'):
        if grasp_mode not in GRASP_MODES:
            raise ValueError(f'grasp_mode must be one of {GRASP_MODES}')
        self._arm = arm
        self._gripper = gripper
        self._scene = scene
        self._kin = kinematics
        self._now = now
        self._latched = grasp_mode == 'latched'

    def run(self, rng: random.Random, index: int) -> TrialResult:
        """Run one pick-and-place trial from the current scene."""
        names = sorted(layout.PIECES)
        start = {name: self._scene.pose(name) for name in names}     # for scoring
        observation = self._scene.observe()
        seen = observation.poses
        name = rng.choice(names)
        target = layout.sample_place_target(rng, name, {n: p.xy for n, p in seen.items()})
        details = {'piece': name, 'start': list(start[name].xyz), 'target': list(target)}
        record_observation(observation, details)
        if name not in seen:
            return TrialResult(self.name, index, False, 'not_found', 0.0, details=details)
        t0 = self._now()
        try:
            failure = self.pick_and_place(layout.PIECES[name], seen[name], target, details,
                                          nominal_grip=observation.vision)
        finally:
            self._scene.unlatch(name)
            self._gripper.release()
            self._scene.update_planning_scene()
        elapsed = self._now() - t0
        if failure is not None:
            return TrialResult(self.name, index, False, failure, elapsed, details=details)
        self._scene.wait_settled()
        final = self._scene.pose(name)
        details['final'] = list(final.xyz)
        details['final_tilt_deg'] = final.tilt_deg
        error = math.dist(final.xy, target)
        details['final_height_error_m'] = final.z - layout.PIECES[name].centre_z
        upright_on_table = (abs(details['final_height_error_m']) <= PLACE_HEIGHT_TOLERANCE_M
                            and final.tilt_deg <= UPRIGHT_TOLERANCE_DEG)
        disturbed = any(
            math.dist(self._scene.pose(other).xyz, start[other].xyz) > DISTURB_THRESHOLD_M
            for other in names if other != name)
        ok = error <= PLACE_TOLERANCE_M and upright_on_table
        return TrialResult(self.name, index, ok, None if ok else 'placement_error',
                           elapsed, error, disturbed, details)

    def _move(self, q) -> Optional[str]:
        return _move_failure(self._arm.move_to(q))

    def _solve(self, xyz: Sequence[float], yaws: Optional[List[float]],
               seed=None) -> Tuple[Tuple[float, ...], Optional[float]]:
        """Return joint angles for a vertical grasp at xyz with one of yaws."""
        for yaw in (yaws or [None]):
            try:
                return self._kin.inverse(xyz, yaw, seed), yaw
            except Unreachable:
                continue
        raise Unreachable(f'no grasp at {tuple(xyz)}')

    def _grip_offset(self, name: str) -> np.ndarray:
        """Return the piece centre in the grasp frame, from the measured state."""
        tcp = self._kin.forward(self._arm.joint_positions())
        return (np.linalg.inv(tcp) @ np.array([*self._scene.pose(name).xyz, 1.0]))[:3]

    def _carrying(self, name: str, grip: np.ndarray, details: dict) -> bool:
        self._scene.wait_settled(CARRY_SETTLE_S)
        expected = (self._kin.forward(self._arm.joint_positions()) @ np.array([*grip, 1.0]))[:3]
        deviation = math.dist(self._scene.pose(name).xyz, expected)
        details['carry_deviation_m'] = max(details.get('carry_deviation_m', 0.0), deviation)
        return deviation <= CARRY_TOLERANCE_M

    def _tcp_for_piece(self, piece_xyz: Sequence[float], grip: np.ndarray,
                       yaw: float, seed) -> Tuple[float, ...]:
        """
        Return joint angles that put a piece held with grip at piece_xyz.

        The grasp frame points straight down, so its rotation depends only on
        the closing-axis yaw. The IK may turn the jaws half a turn instead (it
        grips either way round), which moves the held piece; re-aim with the
        yaw actually achieved until it stops changing.
        """
        q = None
        for _ in range(PLACE_SOLVE_ITERATIONS):
            tcp = np.asarray(piece_xyz) - vertical_rotation(yaw) @ grip
            q = self._kin.inverse(tcp, yaw, seed)
            achieved = self._kin.grasp_yaw(q)
            if math.isclose(math.cos(achieved - yaw), 1.0, abs_tol=1e-6):
                return q
            yaw = achieved
        return q

    def pick_and_place(self, piece, start: PiecePose, target, details: dict,
                       aim_jaws: bool = False, nominal_grip: bool = False,
                       from_bin: bool = False) -> Optional[str]:
        """
        Pick piece from start and set it down at target (xy); return a failure or None.

        With aim_jaws the open jaws, rather than the piece, are centred on
        target: the piece is gripped off-centre (pressed against the fixed
        jaw), and centring the jaws keeps them clear of a bin's walls. With
        from_bin the piece is picked out of a bin: the jaws open only partly
        and every closing direction is tried until one clears the walls.
        On success details['placed'] is the piece's estimated resting pose
        [x, y, z, yaw], from the release pose and the grip used.
        """
        # details['stage'] records how far the pick got, for failure analysis.
        # Approach: hover over the piece with the jaws across its narrow side,
        # then descend around it. The piece itself is not an obstacle.
        details['stage'] = 'approach'
        yaws = piece.closing_yaws(start.yaw)
        grasp_xyz = (start.x, start.y, start.z)
        if not (self._gripper.release(BIN_RELEASE_RAD) if from_bin else self._gripper.release()):
            return 'timeout'
        self._scene.update_planning_scene(exclude=piece.name)
        # Inside a bin a jaw can meet a wall in one orientation and not in
        # another: try each closing direction, and each jaw on either side.
        # A planning failure leaves the arm where it was. (From PR #3.)
        failure, q_grasp = 'unreachable', None
        for attempt in (_approach_yaws(yaws, start.yaw) if from_bin else [yaws]):
            try:
                q_grasp, yaw = self._solve(grasp_xyz, attempt)
                q_hover, _ = self._solve((start.x, start.y, start.z + layout.HOVER),
                                         None if yaw is None else [yaw], q_grasp)
            except Unreachable:
                continue
            for q in (q_hover, q_grasp):
                failure = self._move(q)
                if failure:
                    break
            if failure != 'planning_failed':
                break
        if failure:
            return failure

        # Close, and accept the grasp only if the jaws closed on this piece.
        details['stage'] = 'grasp'
        stalled_at = self._gripper.grip()
        if stalled_at is None:
            return 'timeout'
        details['gripper_stall_rad'] = stalled_at
        grip = self._grip_offset(piece.name)
        details['grip_offset_m'] = float(np.linalg.norm(grip))
        if stalled_at < HOLDING_MIN_ANGLE or np.linalg.norm(grip) > GRIP_MAX_OFFSET_M:
            return 'grasp_miss'
        if self._latched:
            self._gripper.hold(stalled_at + LATCH_EASE_RAD)
            self._scene.wait_settled(LATCH_SETTLE_S)
            if not self._scene.latch(piece.name):
                return 'timeout'
        measured_grip = self._grip_offset(piece.name)     # referee only
        grip = NOMINAL_GRIP if nominal_grip else measured_grip
        self._scene.update_planning_scene(held=piece.name)

        # Carry: lift, move over the target, lower until the piece just
        # clears the table.
        details['stage'] = 'carry'
        held_yaw = self._kin.grasp_yaw(self._arm.joint_positions())
        place_xyz = (target[0], target[1], piece.centre_z + PLACE_CLEARANCE_M)
        try:
            q_lift, _ = self._solve(
                (start.x, start.y, start.z + layout.LIFT), [held_yaw], self._arm.joint_positions())
            if aim_jaws:
                # Centre the jaws (grasp_link) on target; the piece lands
                # beside it, offset by the grip. Solved for the jaws directly,
                # so a half turn of the wrist in the IK cannot move them.
                jaws_z = place_xyz[2] + grip[2]
                q_place = self._kin.inverse((target[0], target[1], jaws_z), held_yaw, q_lift)
                q_above = self._kin.inverse((target[0], target[1], jaws_z + layout.LIFT),
                                            self._kin.grasp_yaw(q_place), q_place)
            else:
                q_place = self._tcp_for_piece(place_xyz, grip, held_yaw, q_lift)
                q_above = self._tcp_for_piece(
                    (place_xyz[0], place_xyz[1], place_xyz[2] + layout.LIFT), grip,
                    self._kin.grasp_yaw(q_place), q_place)
        except Unreachable:
            return 'unreachable'
        details['carry_deviation_m'] = 0.0
        for q in (q_lift, q_above, q_place):
            failure = self._move(q)
            if failure:
                return failure
            if not self._carrying(piece.name, measured_grip, details):
                return 'dropped'

        # Release and retreat; the placed piece is not an obstacle while the
        # open jaws leave it. A latched piece stays put while the jaws open,
        # then drops the last few millimetres straight down: unlatching
        # first would let the opening jaw brush it over.
        details['stage'] = 'release'
        opened = self._gripper.release(BIN_RELEASE_RAD) if aim_jaws else self._gripper.release()
        self._scene.unlatch(piece.name)
        if not opened:
            return 'timeout'
        self._scene.update_planning_scene(exclude=piece.name)
        q_now = self._arm.joint_positions()
        tcp = self._kin.forward(q_now)
        slide = BIN_RELEASE_CLEARANCE_M if aim_jaws else RELEASE_CLEARANCE_M
        clear = tcp[:3, 3] + slide * tcp[:3, 0]
        release_yaw = self._kin.grasp_yaw(q_now)
        # Where the piece now rests, from the release pose and the grip used
        # (nominal in vision mode): picking it out of a bin later starts here.
        centre = tcp[:3, 3] + tcp[:3, :3] @ grip
        placed = [float(centre[0]), float(centre[1]), piece.centre_z,
                  start.yaw + release_yaw - self._kin.grasp_yaw(q_grasp)]
        try:
            q_clear = self._kin.inverse(clear, release_yaw, q_now)
            q_retreat = self._kin.inverse(clear + (0.0, 0.0, layout.HOVER), release_yaw,
                                          q_clear)
        except Unreachable:
            return 'unreachable'
        details['stage'] = 'retreat'
        for q in (q_clear, q_retreat):
            failure = self._move(q)
            if failure:
                return failure
        self._scene.update_planning_scene()
        details['placed'] = placed
        details['stage'] = 'home'
        home = self._arm.home()
        return _move_failure(home)


class SortTask:
    """Put every piece into the bin of its colour."""

    name = 'sort'

    def __init__(self, arm, gripper, scene, kinematics, now: Callable[[], float],
                 grasp_mode: str = 'latched'):
        self._gripper = gripper
        self._scene = scene
        self._now = now
        self._pick = PickPlaceTask(arm, gripper, scene, kinematics, now, grasp_mode)

    def run(self, rng: random.Random, index: int) -> TrialResult:
        """
        Sort the current layout, one piece after another.

        A failed pick does not end the trial: the remaining pieces are still
        sorted, and every piece is checked at the end, so one that was knocked
        out of its bin later also counts. The trial succeeds only if every
        piece rests upright inside its own bin.
        """
        names = sorted(layout.PIECES)
        start = {name: self._scene.pose(name) for name in names}     # for scoring
        pieces: Dict[str, dict] = {}
        trial_details: Dict = {'pieces': pieces}
        failure = None
        waited = 0.0
        t0 = self._now()
        for name in names:
            bin_ = layout.BINS[name]
            details = {'bin': bin_.name, 'start': list(start[name].xyz)}
            pieces[name] = details
            # Look again before every pick: earlier picks may have moved things.
            observation = self._scene.observe()
            waited += observation.wait_s
            record_observation(observation, trial_details)
            if name not in observation.poses:
                details['failure'] = 'not_found'
                failure = failure or 'not_found'
                continue
            try:
                piece_failure = self._pick.pick_and_place(
                    layout.PIECES[name], observation.poses[name], bin_.xy, details,
                    aim_jaws=True, nominal_grip=observation.vision)
            finally:
                # A failed pick may have stopped inside the bin: open partly.
                self._scene.unlatch(name)
                self._gripper.release(BIN_RELEASE_RAD)
                self._scene.update_planning_scene()
            details['failure'] = piece_failure
            failure = failure or piece_failure
        # Time spent waiting for the camera is reported as perception latency.
        elapsed = self._now() - t0 - waited
        self._scene.wait_settled()
        errors = []
        for name in names:
            piece, bin_, details = layout.PIECES[name], layout.BINS[name], pieces[name]
            final = self._scene.pose(name)
            details['final'] = list(final.xyz)
            details['final_tilt_deg'] = final.tilt_deg
            sorted_ok = (bin_.contains(final.xy, margin=min(piece.half_x, piece.half_y))
                         and abs(final.z - piece.centre_z) <= PLACE_HEIGHT_TOLERANCE_M
                         and final.tilt_deg <= UPRIGHT_TOLERANCE_DEG)
            details['in_bin'] = sorted_ok
            if details['failure'] is None:
                errors.append(math.dist(final.xy, bin_.xy))
                if not sorted_ok:
                    details['failure'] = 'placement_error'
                    failure = failure or 'placement_error'
        ok = all(d['in_bin'] for d in pieces.values())
        if ok:
            failure = None
        return TrialResult(self.name, index, ok, failure, elapsed,
                           max(errors) if errors else None, details=trial_details)
