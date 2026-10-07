"""
Tests for the benchmark tasks against a fake robot and world.

The fakes use the real SO-101 kinematics: the arm jumps to commanded joint
angles, the gripper closes on a piece only if it is between the jaws, and a
held piece moves with the grasp frame.
"""

import math
import random

from lab_simulation import layout
from lab_simulation.tasks import (
    CARRY_TOLERANCE_M, GRIP_MAX_OFFSET_M, LATCH_EASE_RAD, MoveResult, PickPlaceTask, PiecePose,
    ReachTask, vertical_rotation)
import numpy as np
import pytest

HOME = (0.0, -1.3, 0.6, 1.6, 0.0)
OPEN, CLOSED, STALL = 0.6, -0.17, 0.19
# Where the moving jaw pushes a gripped piece, in the grasp frame.
PRESSED = np.array([0.009, 0.0, 0.0])


class FakeWorld:
    """Arm, gripper and scene in one, sharing state."""

    def __init__(self, kin, positions=None):
        self.kin = kin
        self.q = HOME
        self.gripper = OPEN
        self.time = 0.0
        self.held = None
        self.grip_offset = None
        self.latched = set()
        self.latch_calls = []
        self.scene_updates = []
        self.moves = []
        self.fail_plan_at = None      # move index that fails to plan
        self.drop_on_move = None      # move index after which a held piece falls out
        self.miss = False             # piece slides away as the jaws close
        self.held_at = None
        positions = positions or layout.sample_layout(random.Random(1))
        self.pieces = {name: PiecePose(x, y, layout.PIECES[name].centre_z)
                       for name, (x, y) in positions.items()}

    # arm
    def joint_positions(self):
        return self.q

    def move_to(self, q):
        index = len(self.moves)
        self.moves.append(tuple(q))
        self.time += 1.0
        if index == self.fail_plan_at:
            return MoveResult(False, False, 0.0, math.inf)
        self.q = tuple(q)
        if self.held is not None:
            if index == self.drop_on_move and self.held not in self.latched:
                self._drop()
            else:
                tcp = self.kin.forward(self.q)
                x, y, z = (tcp @ np.array([*self.grip_offset, 1.0]))[:3]
                self.pieces[self.held] = PiecePose(x, y, z)
        return MoveResult(True, True, 1.0, 0.0)

    def home(self):
        return self.move_to(HOME)

    # gripper
    def release(self):
        self.gripper = OPEN
        if self.held is not None and self.held not in self.latched:
            self._drop()
        return True

    def grip(self):
        tcp = self.kin.forward(self.q)
        for name, pose in self.pieces.items():
            local = (np.linalg.inv(tcp) @ np.array([*pose.xyz, 1.0]))[:3]
            if np.linalg.norm(local) < 0.02 and not self.miss:
                self.held, self.grip_offset = name, PRESSED
                x, y, z = (tcp @ np.array([*PRESSED, 1.0]))[:3]
                self.pieces[name] = PiecePose(x, y, z)
                self.gripper = STALL
                return STALL
        self.gripper = CLOSED
        return CLOSED

    def hold(self, angle):
        self.held_at = angle

    def _drop(self):
        p = self.pieces[self.held]
        self.pieces[self.held] = PiecePose(p.x, p.y, layout.PIECES[self.held].centre_z)
        self.held = None

    # scene
    def pose(self, name):
        return self.pieces[name]

    def latch(self, name):
        self.latch_calls.append(name)
        self.latched.add(name)
        return True

    def unlatch(self, name):
        self.latched.discard(name)
        if self.held == name and self.gripper == OPEN:
            self._drop()

    def update_planning_scene(self, exclude=None, held=None):
        self.scene_updates.append((exclude, held))

    def wait_settled(self, timeout_s=3.0):
        return True

    def now(self):
        return self.time


@pytest.fixture
def world(lab_kinematics):
    return FakeWorld(lab_kinematics)


def pick_task(world, mode='latched'):
    return PickPlaceTask(world, world, world, world.kin, world.now, mode)


def test_vertical_rotation_points_down_and_closes_along_yaw():
    r = vertical_rotation(0.7)
    assert r[:, 2] == pytest.approx([0, 0, -1])
    assert r[:, 0] == pytest.approx([math.cos(0.7), math.sin(0.7), 0])
    assert np.linalg.det(r) == pytest.approx(1.0)


@pytest.mark.parametrize('seed', range(5))
def test_reach_succeeds_and_measures_error(world, seed):
    result = ReachTask(world, world, world.kin, world.now).run(random.Random(seed), 0)
    assert result.success, result
    assert result.error_m < 1e-3
    assert world.scene_updates == [(None, None)]


def test_reach_planning_failure(world):
    world.fail_plan_at = 0
    result = ReachTask(world, world, world.kin, world.now).run(random.Random(0), 0)
    assert not result.success and result.failure == 'planning_failed'


@pytest.mark.parametrize('seed', range(8))
def test_pick_place_succeeds_and_latches(world, seed):
    result = pick_task(world).run(random.Random(seed), 0)
    assert result.success, result
    piece = result.details['piece']
    assert world.latch_calls == [piece]
    # Eases the squeeze before latching, then holds there.
    assert world.held_at == pytest.approx(STALL + LATCH_EASE_RAD)
    assert result.error_m < 0.005
    assert result.details['gripper_stall_rad'] == STALL
    assert world.pieces[piece].xy == pytest.approx(result.details['target'], abs=0.005)
    assert world.q == HOME and not world.latched


def test_pick_place_plans_around_the_other_pieces(world):
    result = pick_task(world).run(random.Random(3), 0)
    piece = result.details['piece']
    # Approach and retreat ignore the target piece; the carry moves it with
    # the gripper; going home sees every piece again.
    assert (piece, None) in world.scene_updates
    assert (None, piece) in world.scene_updates
    assert world.scene_updates[-1] == (None, None)


def test_friction_mode_never_latches(world):
    result = pick_task(world, 'friction').run(random.Random(2), 0)
    assert result.success, result
    assert world.latch_calls == []
    assert world.held_at is None       # keeps squeezing


def test_closing_on_nothing_is_a_miss(world):
    world.miss = True
    result = pick_task(world).run(random.Random(0), 0)
    assert result.failure == 'grasp_miss'
    assert result.details['gripper_stall_rad'] == CLOSED
    assert world.latch_calls == []


def test_a_stalled_jaw_away_from_the_piece_is_a_miss(world, monkeypatch):
    """Jaws blocked by something else (e.g. a neighbour) must not latch."""
    task = pick_task(world)

    def pushed_away_then_stalled():
        # The descending jaw shoved the piece 3 cm aside, then stalled anyway.
        for name, pose in world.pieces.items():
            tcp = world.kin.forward(world.q)
            if math.dist(pose.xyz, tcp[:3, 3]) < 0.02:
                world.pieces[name] = pose._replace(x=pose.x + 0.03)
        return STALL
    monkeypatch.setattr(world, 'grip', pushed_away_then_stalled)
    result = task.run(random.Random(0), 0)
    assert result.failure == 'grasp_miss'
    assert result.details['grip_offset_m'] > GRIP_MAX_OFFSET_M
    assert world.latch_calls == []


def test_friction_slip_during_carry_is_dropped(world):
    world.drop_on_move = 3      # hover, grasp, lift: falls out on the lift
    result = pick_task(world, 'friction').run(random.Random(0), 0)
    assert result.failure == 'dropped'
    assert result.details['carry_deviation_m'] > CARRY_TOLERANCE_M


def test_latched_piece_does_not_slip(world):
    world.drop_on_move = 3
    assert pick_task(world).run(random.Random(0), 0).success


@pytest.mark.parametrize('move', range(4))
def test_planning_failure_fails_the_trial_and_releases(world, move):
    world.fail_plan_at = move
    result = pick_task(world).run(random.Random(0), 0)
    assert result.failure == 'planning_failed'
    assert not world.latched and world.gripper == OPEN


def test_placement_error_when_the_piece_lands_tilted(world, monkeypatch):
    task = pick_task(world)
    original = world.pose

    def tilted(name):
        pose = original(name)
        return pose._replace(tilt_deg=40.0) if world.held is None and name in world.pieces \
            and world.moves else pose
    monkeypatch.setattr(world, 'pose', tilted)
    result = task.run(random.Random(0), 0)
    assert result.failure == 'placement_error'
    assert result.details['final_tilt_deg'] == 40.0


def test_unknown_grasp_mode_rejected(world):
    with pytest.raises(ValueError):
        pick_task(world, 'suction')


def test_block_is_gripped_across_its_narrow_side(world):
    """The jaws close along the block's y axis (3 cm), not its 5 cm length."""
    rng = random.Random(0)
    for _ in range(50):
        if pick_task(world).run(rng, 0).details['piece'] == 'green_block':
            break
        world.__init__(world.kin)
    yaw = world.kin.grasp_yaw(world.moves[1])
    piece_yaw = 0.0
    assert abs(math.sin(yaw - piece_yaw)) == pytest.approx(1.0, abs=1e-3)


def test_jaws_slide_off_the_placed_piece_before_lifting(world):
    """Lifting straight up would drag the piece pressed against the fixed jaw."""
    from lab_simulation.tasks import RELEASE_CLEARANCE_M
    assert pick_task(world).run(random.Random(0), 0).success
    place, clear, retreat = (world.kin.forward(q) for q in world.moves[4:7])
    shift = clear[:3, 3] - place[:3, 3]
    # The piece was pressed against the fixed jaw, on the grasp frame's +x
    # side: moving +x takes that jaw away from it.
    assert PRESSED[0] > 0
    assert shift @ place[:3, 0] == pytest.approx(RELEASE_CLEARANCE_M, abs=1e-4)
    assert shift[2] == pytest.approx(0.0, abs=1e-4)
    assert retreat[2, 3] - clear[2, 3] == pytest.approx(layout.HOVER, abs=1e-4)
