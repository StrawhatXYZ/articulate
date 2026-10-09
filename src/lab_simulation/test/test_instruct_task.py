"""The end-to-end instruction task, on the fake robot with a scripted planner."""

import random

from fakes import FakeWorld
from lab_simulation import instruction_cases as C
from lab_simulation.executor import Executor
from lab_simulation.instruct_task import InstructTask
from lab_simulation.instructor import Instructor
from lab_simulation.memory import PlacementMemory
from lab_simulation.plan_check import ToolCall
from lab_simulation.planner import PlannerReply
from lab_simulation.tasks import PickPlaceTask
import pytest


class Scripted:
    """Answers every instruction with the same reply."""

    def __init__(self, calls=(), error=None):
        self.calls, self.error, self.texts = list(calls), error, []

    def plan(self, text, states):
        self.texts.append(text)
        return PlannerReply(self.calls, self.error, {}, 0.3)


def make_task(kin, planner):
    world = FakeWorld(kin)
    pick = PickPlaceTask(world, world, world, kin, world.now, 'latched')
    executor = Executor(pick, world, world, world, PlacementMemory())
    return world, InstructTask(Instructor(planner, executor), executor, world)


def test_a_correct_plan_succeeds(lab_kinematics):
    assert C.E2E[0].text == 'put the red cube in its box'
    planner = Scripted([ToolCall('move_piece', {'piece': 'red_cube', 'target': 'bin'})])
    _, task = make_task(lab_kinematics, planner)
    result = task.run(random.Random(0), 0)
    assert result.success, result.details
    assert result.details['case'] == 0 and result.details['outcome'] == 'done'


def test_unavailable_planner_is_a_planner_failure_and_the_next_trial_runs(lab_kinematics):
    _, task = make_task(lab_kinematics, Scripted(error='unavailable'))
    first = task.run(random.Random(0), 0)
    second = task.run(random.Random(0), 1)
    assert first.failure == 'planner' and second.failure == 'planner'
    assert second.details['text'] == C.E2E[1].text


def test_a_refusal_by_the_check_is_attributed_to_the_check(lab_kinematics):
    planner = Scripted([ToolCall('move_piece', {'piece': 'red_cube', 'target': 'sideways'})])
    _, task = make_task(lab_kinematics, planner)
    assert task.run(random.Random(0), 0).failure == 'check'


def test_setup_runs_before_the_instruction_and_memory_is_reset(lab_kinematics):
    index = next(i for i, c in enumerate(C.E2E) if c.setup)
    planner = Scripted([ToolCall('move_piece', {'piece': 'red_cube', 'target': 'start'})])
    world, task = make_task(lab_kinematics, planner)
    result = task.run(random.Random(0), index)
    assert result.success, result.details
    assert task.memory_resets == 1


def test_a_failing_setup_fails_the_trial(lab_kinematics, monkeypatch):
    index = next(i for i, c in enumerate(C.E2E) if c.setup)
    world, task = make_task(lab_kinematics, Scripted())
    monkeypatch.setattr(world, 'grip', lambda: -0.17)       # closes on nothing
    assert task.run(random.Random(0), index).failure == 'setup'


@pytest.mark.parametrize('expected,outcome', [(C.REFUSE, 'refused'), (C.HOME, 'done')])
def test_refusal_and_home_cases_score(lab_kinematics, expected, outcome):
    index = next(i for i, c in enumerate(C.E2E) if c.expected == expected and not c.setup)
    call = (ToolCall('refuse', {'reason': 'no'}) if expected == C.REFUSE
            else ToolCall('go_home', {}))
    _, task = make_task(lab_kinematics, Scripted([call]))
    result = task.run(random.Random(0), index)
    assert result.success and result.details['outcome'] == outcome


def test_blame_separates_a_full_table_from_a_check_rejection():
    from lab_simulation.instructor import InstructResult
    full = InstructResult('refused', 'Refused: step 1: no free spot left of red_cube', [],
                          {'verdict_source': 'check'})
    bad = InstructResult('refused', 'Refused: unknown target sideways', [],
                         {'verdict_source': 'check'})
    unsafe_done = InstructResult('done', 'Done.', ['go home'], {})
    assert InstructTask._blame({C.R: 'bin'}, full) == 'infeasible'
    assert InstructTask._blame({C.R: 'bin'}, bad) == 'check'
    assert InstructTask._blame(C.REFUSE, unsafe_done) == 'planner'
