"""
Tests for the benchmark tasks against a fake robot and world.

The fakes use the real SO-101 kinematics: the arm jumps to commanded joint
angles, the gripper closes on a piece only if it is between the jaws, and a
held piece moves with the grasp frame.
"""

import math
import random

from fakes import CLOSED, FakeWorld, HOME, OPEN, PRESSED, STALL
from lab_simulation import layout
from lab_simulation.tasks import (
    _approach_yaws, BIN_RELEASE_RAD, CARRY_TOLERANCE_M, GRIP_MAX_OFFSET_M, LATCH_EASE_RAD,
    PickPlaceTask, ReachTask, SortTask, vertical_rotation)
import numpy as np
import pytest


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
    world.drop_on_move = 2      # hover, grasp, above target: falls out on the way
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
    place, clear, retreat = (world.kin.forward(q) for q in world.moves[3:6])
    shift = clear[:3, 3] - place[:3, 3]
    # The piece was pressed against the fixed jaw, on the grasp frame's +x
    # side: moving +x takes that jaw away from it.
    assert PRESSED[0] > 0
    assert shift @ place[:3, 0] == pytest.approx(RELEASE_CLEARANCE_M, abs=1e-4)
    assert shift[2] == pytest.approx(0.0, abs=1e-4)
    assert retreat[2, 3] - clear[2, 3] == pytest.approx(layout.HOVER, abs=1e-4)


def sort_task(world, mode='latched'):
    return SortTask(world, world, world, world.kin, world.now, mode)


@pytest.mark.parametrize('seed', range(4))
def test_sort_puts_every_piece_in_its_own_bin(lab_kinematics, seed):
    world = FakeWorld(lab_kinematics, layout.sample_layout(random.Random(seed)))
    result = sort_task(world).run(random.Random(seed), 0)
    assert result.success, result
    for name, details in result.details['pieces'].items():
        assert details['in_bin'] and details['failure'] is None
        assert layout.BINS[name].contains(world.pieces[name].xy, margin=0.015)
    assert world.latch_calls == sorted(layout.PIECES)
    assert result.error_m < layout.BIN_INNER / 2


def test_sort_opens_the_jaws_only_partly_inside_a_bin(world, monkeypatch):
    from lab_simulation.tasks import BIN_RELEASE_RAD
    angles = []
    real = world.release
    monkeypatch.setattr(world, 'release', lambda angle=OPEN: angles.append(angle) or real(angle))
    assert sort_task(world).run(random.Random(0), 0).success
    # Per piece: fully open to approach it on the table, partly to let go in
    # the bin, and partly again afterwards in case the pick failed in a bin.
    assert angles == [OPEN, BIN_RELEASE_RAD, BIN_RELEASE_RAD] * len(layout.PIECES)
    assert BIN_RELEASE_RAD < OPEN


def test_sort_centres_the_jaws_not_the_piece(world):
    """The gripped piece sits ~9 mm off the jaw centre; the jaws aim at the bin."""
    result = sort_task(world).run(random.Random(0), 0)
    assert result.success
    for name, details in result.details['pieces'].items():
        offset = math.dist(world.pieces[name].xy, layout.BINS[name].xy)
        assert offset == pytest.approx(np.linalg.norm(PRESSED), abs=0.002)


def test_sort_continues_after_a_miss_and_fails_the_trial(world, monkeypatch):
    real_grip = world.grip
    calls = []

    def miss_first():
        calls.append(1)
        if len(calls) == 1:
            world.gripper = CLOSED
            return CLOSED
        return real_grip()
    monkeypatch.setattr(world, 'grip', miss_first)
    result = sort_task(world).run(random.Random(0), 0)
    assert not result.success and result.failure == 'grasp_miss'
    pieces = result.details['pieces']
    first, *rest = sorted(layout.PIECES)
    assert pieces[first]['failure'] == 'grasp_miss' and not pieces[first]['in_bin']
    assert all(pieces[n]['in_bin'] for n in rest)


def test_sort_checks_pieces_at_the_end(world, monkeypatch):
    """A piece knocked out of its bin after being sorted fails the trial."""
    task = sort_task(world)
    original = world.wait_settled

    def knock_red_out(timeout_s=3.0):
        if len(world.moves) >= 21:     # after the last piece's trip home (7 moves each)
            p = world.pieces['red_cube']
            world.pieces['red_cube'] = p._replace(x=p.x + 0.08)
        return original(timeout_s)
    monkeypatch.setattr(world, 'wait_settled', knock_red_out)
    result = task.run(random.Random(0), 0)
    assert not result.success and result.failure == 'placement_error'
    assert not result.details['pieces']['red_cube']['in_bin']


def test_hidden_piece_is_not_found(world):
    world.source = 'vision'
    world.hidden = set(layout.PIECES)
    result = pick_task(world).run(random.Random(0), 0)
    assert result.failure == 'not_found'
    assert world.moves == [] and world.latch_calls == []
    assert result.details['perception'][0]['missed'] == 3


def test_vision_plans_from_the_detection_not_the_truth(world):
    """Detections 6 mm off: the jaws go where the camera says."""
    world.source = 'vision'
    world.seen_offset = (0.006, 0.0)
    result = pick_task(world).run(random.Random(0), 0)
    truth = result.details['start']
    grasp = world.kin.forward(world.moves[1])[:3, 3]
    assert grasp[:2] == pytest.approx([truth[0] + 0.006, truth[1]], abs=5e-4)


def test_vision_aims_placement_with_the_nominal_grip(lab_kinematics):
    """The true grip (12 mm) differs from the nominal one (9 mm) by 3 mm."""
    results = {}
    for source in ('ground_truth', 'vision'):
        world = FakeWorld(lab_kinematics)
        world.source = source
        world.pressed = np.array([0.012, 0.0, 0.0])
        results[source] = pick_task(world).run(random.Random(0), 0)
    assert results['ground_truth'].error_m < 5e-4
    assert results['vision'].error_m == pytest.approx(0.003, abs=5e-4)


def test_observations_are_recorded_only_in_vision_mode(world):
    assert 'perception' not in pick_task(world).run(random.Random(0), 0).details
    world.__init__(world.kin)
    world.source = 'vision'
    details = pick_task(world).run(random.Random(0), 0).details
    assert len(details['perception']) == 1 and details['perception_latency_s'] == [0.2]


def test_sort_looks_before_every_pick_and_skips_unseen_pieces(world):
    world.source = 'vision'
    world.hidden = {'green_block'}
    result = sort_task(world).run(random.Random(0), 0)
    assert world.observations == len(layout.PIECES)
    assert result.failure == 'not_found' and not result.success
    pieces = result.details['pieces']
    assert pieces['green_block']['failure'] == 'not_found'
    assert all(pieces[n]['in_bin'] for n in ('blue_cylinder', 'red_cube'))
    assert len(result.details['perception']) == len(layout.PIECES)


def test_sort_cycle_time_excludes_waiting_for_the_camera(world):
    result = sort_task(world).run(random.Random(0), 0)
    moves = len(world.moves)
    assert result.cycle_time_s == pytest.approx(moves * 1.0)


@pytest.mark.parametrize('seed', range(6))
def test_sort_centres_the_jaws_on_the_bin_whatever_the_wrist_does(lab_kinematics, seed):
    """Random piece yaws make the IK pick either wrist half turn; the jaws stay centred."""
    world = FakeWorld(lab_kinematics, layout.sample_layout(random.Random(seed)))
    rng = random.Random(seed)
    for name, yaw in layout.sample_yaws(rng).items():
        world.pieces[name] = world.pieces[name]._replace(yaw=yaw)
    places = []
    real = world.move_to

    def record(q):
        if world.held is not None and len(places) < 10:
            places.append((world.held, tuple(q)))
        return real(q)
    world.move_to = record
    result = sort_task(world).run(rng, 0)
    assert result.success, result
    for name in layout.PIECES:
        # The lowest carried pose of each piece is its place pose.
        poses = [q for held, q in places if held == name]
        place = min(poses, key=lambda q: world.kin.forward(q)[2, 3])
        jaws = world.kin.forward(place)[:3, 3]
        assert math.dist(jaws[:2], layout.BINS[name].xy) < 1e-3


def test_pick_and_place_records_where_the_piece_was_put(world):
    task = pick_task(world)
    details = {}
    assert task.pick_and_place(layout.PIECES['red_cube'], world.pose('red_cube'),
                               layout.BINS['red_cube'].xy, details, aim_jaws=True,
                               nominal_grip=True) is None
    placed = details['placed']
    assert math.dist(placed[:2], world.pose('red_cube').xy) < 0.005
    assert placed[2] == pytest.approx(layout.PIECES['red_cube'].centre_z)


def test_approach_yaws_try_each_jaw_on_either_side():
    assert [round(y[0], 3) for y in _approach_yaws([0.3], 0.3)] == [0.3, round(0.3 + math.pi, 3)]
    assert len(_approach_yaws(None, 0.0)) == 4


def test_picking_from_a_bin_opens_the_jaws_only_partly(world, monkeypatch):
    angles = []
    original = world.release
    monkeypatch.setattr(world, 'release',
                        lambda angle=OPEN: angles.append(angle) or original(angle))
    pick_task(world).pick_and_place(layout.PIECES['red_cube'], world.pose('red_cube'),
                                    layout.BINS['red_cube'].xy, {}, aim_jaws=True, from_bin=True)
    assert angles[0] == BIN_RELEASE_RAD


def test_picking_from_a_bin_tries_another_approach_after_a_planning_failure(world):
    world.fail_plan_at = 0                     # the first hover cannot be planned
    details = {}
    failure = pick_task(world).pick_and_place(
        layout.PIECES['red_cube'], world.pose('red_cube'), layout.BINS['red_cube'].xy, details,
        aim_jaws=True, from_bin=True)
    assert failure is None and details['stage'] == 'home'


def test_after_the_grasp_the_arm_goes_straight_to_above_the_target(world):
    details = {}
    target = layout.BINS['red_cube'].xy
    assert pick_task(world).pick_and_place(layout.PIECES['red_cube'], world.pose('red_cube'),
                                           target, details, aim_jaws=True) is None
    # hover, grasp, above the target, place, slide clear, retreat, home
    assert len(world.moves) == 7
    above = world.kin.forward(world.moves[2])[:3, 3]
    place = world.kin.forward(world.moves[3])[:3, 3]
    assert math.dist(above[:2], target) < 0.03 and above[2] > place[2]
