"""The instructor: hearing, planning, checking and executing, with every outcome."""

from lab_simulation.instructor import Instructor
from lab_simulation.memory import PieceState, TABLE
from lab_simulation.plan_check import ToolCall
from lab_simulation.planner import PlannerReply
from lab_simulation.tasks import Observation, PiecePose

STATES = {n: PieceState(n, TABLE, PiecePose(*xy, 0.765), xy) for n, xy in
          {'red_cube': (0.20, -0.08), 'blue_cylinder': (0.12, -0.15),
           'green_block': (0.05, -0.19)}.items()}


class FakePlanner:
    def __init__(self, reply):
        self.reply = reply

    def plan(self, text, states):
        if isinstance(self.reply, Exception):
            raise self.reply
        return self.reply


class FakeExecutor:
    def __init__(self, outcome=('done', 'Done.')):
        self.outcome, self.ran = outcome, []

    def observe(self):
        return STATES, Observation({}, 'vision')

    def run(self, steps, feedback, cancelled):
        self.ran.append(steps)
        return self.outcome


def reply(*calls, error=None):
    return PlannerReply(list(calls), error, {}, 0.1)


def run(planner, executor=None, tmp_path=None):
    executor = executor or FakeExecutor()
    seen = []
    result = Instructor(planner, executor, tmp_path / 'log.jsonl' if tmp_path else None).run(
        'text', lambda s, m: seen.append(s), lambda: False)
    return result, executor, seen


def test_plan_is_checked_then_executed(tmp_path):
    move = ToolCall('move_piece', {'piece': 'red_cube', 'target': 'bin'})
    r, ex, seen = run(FakePlanner(reply(move)), tmp_path=tmp_path)
    assert r.outcome == 'done' and len(ex.ran) == 1 and seen[:3] == ['heard', 'planning', 'plan']
    assert (tmp_path / 'log.jsonl').read_text().count('\n') == 1


def test_answer_and_refusal_never_execute():
    for call, outcome in [(ToolCall('answer', {'text': 'Two.'}), 'answered'),
                          (ToolCall('refuse', {'reason': 'unsafe'}), 'refused'),
                          (ToolCall('move_piece', {'piece': 'nope', 'target': 'bin'}), 'refused')]:
        r, ex, _ = run(FakePlanner(reply(call)))
        assert r.outcome == outcome and ex.ran == []


def test_planner_errors_map_to_outcomes():
    assert run(FakePlanner(reply(error='no_call')))[0].outcome == 'not_understood'
    assert run(FakePlanner(reply(error='unavailable')))[0].outcome == 'planner_unavailable'


def test_unexpected_error_is_failed_internal_error():
    r, _, _ = run(FakePlanner(RuntimeError('boom')))
    assert r.outcome == 'failed' and 'internal error' in r.message


def test_second_run_is_rejected_while_busy():
    instructor = Instructor(FakePlanner(reply()), FakeExecutor(), None)
    assert instructor.try_begin() and not instructor.try_begin()
    instructor.end()
    assert instructor.try_begin()


def test_nothing_to_do_explains_itself():
    boxed = {n: s._replace(status='box') for n, s in STATES.items()}

    class Boxed(FakeExecutor):
        def observe(self):
            return boxed, Observation({}, 'vision')

    move = ToolCall('move_piece', {'piece': 'red_cube', 'target': 'bin'})
    r, ex, _ = run(FakePlanner(reply(move)), Boxed())
    assert r.outcome == 'done' and ex.ran == []
    assert r.message == 'Nothing to do: red_cube is already in its box.'
