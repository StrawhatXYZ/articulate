"""The executor runs checked steps on the (fake) robot and keeps memory honest."""

import math

from fakes import FakeWorld
from lab_simulation import layout
from lab_simulation.executor import Executor
from lab_simulation.memory import BOX, PlacementMemory, TABLE
from lab_simulation.plan_check import Step
from lab_simulation.tasks import MoveResult, PickPlaceTask
import pytest


@pytest.fixture
def setup(lab_kinematics):
    world = FakeWorld(lab_kinematics)
    pick = PickPlaceTask(world, world, world, lab_kinematics, world.now, 'latched')
    return world, Executor(pick, world, world, world, PlacementMemory())


def run(executor, steps, cancel_after=None):
    seen = []
    done = executor.run(steps, lambda stage, msg: seen.append((stage, msg)),
                        lambda: cancel_after is not None and len(seen) >= cancel_after)
    return done, seen


def test_move_to_bin_updates_memory(setup):
    world, ex = setup
    ex.observe()
    outcome, _ = run(ex, [Step('move', 'red_cube', {'kind': 'bin'})])
    assert outcome[0] == 'done'
    assert ex.memory.states()['red_cube'].status == BOX
    assert layout.BINS['red_cube'].contains(world.pose('red_cube').xy)


def test_box_pick_uses_remembered_pose_and_failure_homes(setup, monkeypatch):
    world, ex = setup
    ex.observe()
    run(ex, [Step('move', 'red_cube', {'kind': 'bin'})])
    remembered = ex.memory.states()['red_cube'].pose
    assert math.dist(remembered.xy, world.pose('red_cube').xy) < 0.005
    homes = []
    monkeypatch.setattr(world, 'move_to', lambda q: MoveResult(False, False, 0.0, math.inf))
    monkeypatch.setattr(world, 'home', lambda: homes.append(1) or MoveResult(True, True, 1, 0))
    outcome, _ = run(ex, [Step('move', 'red_cube', {'kind': 'start'})])
    assert outcome[0] == 'failed' and 'step 1/1' in outcome[1] and homes
    assert ex.memory.states()['red_cube'].status == BOX       # never left the box


def test_steps_report_progress_in_order(setup):
    _, ex = setup
    ex.observe()
    _, seen = run(ex, [Step('move', 'red_cube', {'kind': 'bin'}), Step('home')])
    assert [s for s, _ in seen] == ['step', 'step']
    assert seen[0][1].startswith('step 1/2')


def test_cancel_takes_effect_between_steps(setup):
    _, ex = setup
    ex.observe()
    outcome, _ = run(ex, [Step('move', 'red_cube', {'kind': 'bin'}),
                          Step('move', 'blue_cylinder', {'kind': 'bin'})], cancel_after=1)
    assert outcome[0] == 'cancelled'
    assert ex.memory.states()['red_cube'].status == BOX
    assert ex.memory.states()['blue_cylinder'].status == TABLE


def test_a_step_no_longer_valid_stops_the_plan(setup):
    world, ex = setup
    ex.observe()

    def lose_green(stage, message):             # the camera loses it after step 1
        if message.startswith('step 2/2'):
            world.hidden = {'green_block'}

    outcome = ex.run([Step('move', 'red_cube', {'kind': 'bin'}),
                      Step('move', 'green_block', {'kind': 'bin'})], lose_green, lambda: False)
    assert outcome[0] == 'failed' and 'not visible' in outcome[1]


def test_a_missed_grasp_in_the_box_keeps_the_piece_remembered_in_its_box(setup):
    world, ex = setup
    ex.observe()
    run(ex, [Step('move', 'red_cube', {'kind': 'bin'})])
    world.miss = True                               # the jaws close on nothing
    world.hidden = {'red_cube'}                     # vision cannot see into the box
    outcome, _ = run(ex, [Step('move', 'red_cube', {'kind': 'start'})])
    assert outcome[0] == 'failed' and 'grasp' in outcome[1]
    assert ex.memory.states()['red_cube'].status == BOX


def test_a_placement_in_the_box_is_remembered_even_if_going_home_fails(setup, monkeypatch):
    world, ex = setup
    ex.observe()
    monkeypatch.setattr(world, 'home', lambda: MoveResult(False, False, 0.0, math.inf))
    outcome, _ = run(ex, [Step('move', 'red_cube', {'kind': 'bin'})])
    assert outcome[0] == 'failed'
    assert ex.memory.states()['red_cube'].status == BOX


def test_the_look_before_planning_is_reused_for_the_first_step_only(setup):
    world, ex = setup
    ex.observe()                                     # the look the plan was made from
    looks = world.observations
    run(ex, [Step('move', 'red_cube', {'kind': 'bin'}),
             Step('move', 'blue_cylinder', {'kind': 'bin'})])
    assert world.observations == looks + 1          # only the second step looks again
