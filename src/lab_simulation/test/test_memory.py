"""Placement memory: what the robot believes about pieces it cannot see."""

from lab_simulation.memory import BOX, PlacementMemory, TABLE, UNKNOWN
from lab_simulation.tasks import PiecePose

RED = PiecePose(0.20, -0.08, 0.765)
BOXED = PiecePose(0.135, 0.176, 0.765)


def test_everything_is_unknown_until_seen():
    memory = PlacementMemory()
    assert {s.status for s in memory.states().values()} == {UNKNOWN}


def test_first_sighting_sets_the_start():
    memory = PlacementMemory()
    memory.observe({'red_cube': RED})
    memory.observe({'red_cube': PiecePose(0.15, -0.10, 0.765)})
    state = memory.states()['red_cube']
    assert state.status == TABLE and state.start_xy == RED.xy
    assert state.pose.xy == (0.15, -0.10)


def test_placed_in_box_survives_not_being_seen():
    memory = PlacementMemory()
    memory.observe({'red_cube': RED})
    memory.moved('red_cube')
    memory.placed_in_box('red_cube', BOXED)
    memory.observe({})
    state = memory.states()['red_cube']
    assert state.status == BOX and state.placed_by_me and state.pose == BOXED


def test_camera_wins_over_memory():
    memory = PlacementMemory()
    memory.placed_in_box('red_cube', BOXED)
    memory.observe({'red_cube': RED})
    assert memory.states()['red_cube'].status == TABLE


def test_table_piece_not_seen_becomes_unknown():
    memory = PlacementMemory()
    memory.observe({'red_cube': RED})
    memory.observe({})
    assert memory.states()['red_cube'].status == UNKNOWN


def test_moved_forgets_the_pose_until_seen_again():
    memory = PlacementMemory()
    memory.observe({'red_cube': RED})
    memory.moved('red_cube')
    assert memory.states()['red_cube'].status == UNKNOWN
    assert memory.states()['red_cube'].start_xy == RED.xy


def test_reset_clears_states_and_starts():
    memory = PlacementMemory()
    memory.observe({'red_cube': RED})
    memory.reset()
    state = memory.states()['red_cube']
    assert state.status == UNKNOWN and state.start_xy is None


def test_a_piece_seen_inside_its_box_is_in_its_box():
    memory = PlacementMemory()
    memory.observe({'red_cube': BOXED})              # ground truth sees into the bins
    state = memory.states()['red_cube']
    assert state.status == BOX and state.pose == BOXED and not state.placed_by_me
