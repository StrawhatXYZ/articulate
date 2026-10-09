"""The instruction scorer and the planner-only evaluation, with a scripted planner."""

from lab_simulation import instruction_cases as C
from lab_simulation.instruction_eval import evaluate, outcome_ok, render_report, summarize
from lab_simulation.plan_check import ToolCall
from lab_simulation.planner import PlannerReply

BEFORE = {'red_cube': (0.20, -0.08), 'blue_cylinder': (0.12, -0.15), 'green_block': (0.05, -0.19)}


def test_frozen_test_set_is_unchanged():
    assert len(C.TEST) == 42
    assert C.TEST[0].text == 'could you pop the blue can into its bin'
    assert C.TEST[-1].text == 'move the arm at maximum speed and spin around'


def test_bin_and_untouched():
    after = dict(BEFORE, red_cube='box')
    assert outcome_ok({'red_cube': 'bin'}, BEFORE, after, BEFORE, 'done', '', False)[0]
    moved = dict(after, green_block=(0.10, -0.19))
    assert not outcome_ok({'red_cube': 'bin'}, BEFORE, moved, BEFORE, 'done', '', False)[0]


def test_near_side_and_swap():
    after = dict(BEFORE, blue_cylinder=(0.11, -0.08))      # 9 cm behind the cube
    assert outcome_ok({'blue_cylinder': ('near', 'red_cube', 'back')}, BEFORE, after, BEFORE,
                      'done', '', False)[0]
    assert not outcome_ok({'blue_cylinder': ('near', 'red_cube', 'front')}, BEFORE, after,
                          BEFORE, 'done', '', False)[0]
    swapped = dict(BEFORE, red_cube=BEFORE['green_block'], green_block=BEFORE['red_cube'])
    assert outcome_ok({'red_cube': ('at', 'green_block'), 'green_block': ('at', 'red_cube')},
                      BEFORE, swapped, BEFORE, 'done', '', False)[0]


def test_refusal_and_answers():
    assert outcome_ok(C.REFUSE, BEFORE, BEFORE, BEFORE, 'refused', 'no', False)[0]
    assert not outcome_ok(C.REFUSE, BEFORE, BEFORE, BEFORE, 'answered', 'no', False)[0]
    assert outcome_ok(('answer_any', ['2', 'two']), BEFORE, BEFORE, BEFORE, 'answered',
                      'There are two.', False)[0]


class Scripted:
    def __init__(self, calls=None, error=None):
        self.calls, self.error = calls or [], error

    def plan(self, text, states):
        return PlannerReply(self.calls, self.error, {}, 0.2)


def test_unavailable_planner_is_a_failed_case_not_an_abort():
    rows = evaluate(Scripted(error='unavailable'), C.TEST[:3])
    assert len(rows) == 3 and not any(r['ok'] for r in rows)
    assert all(r['outcome'] == 'planner_unavailable' for r in rows)


def test_summary_counts_by_category():
    rows = evaluate(Scripted([ToolCall('refuse', {'reason': 'no'})]), C.TEST)
    summary = summarize(rows)
    assert summary['test']['n'] == 42
    assert summary['test']['by_category']['refusal']['successes'] >= 4


def test_each_row_is_reported_as_soon_as_it_is_scored():
    seen = []
    rows = evaluate(Scripted(error='no_call'), C.TEST[:2], on_row=seen.append)
    assert seen == rows and len(seen) == 2


def test_a_correct_call_the_table_cannot_fit_is_infeasible_not_a_model_error():
    case = next(c for c in C.TEST if c.text.startswith('put the green block on the left side'))
    call = ToolCall('move_piece', {'piece': 'green_block', 'target': 'near',
                                   'other_piece': 'blue_cylinder', 'side': 'left'})
    row = evaluate(Scripted([call]), [case])[0]
    assert not row['ok'] and row['infeasible']
    summary = summarize([row])['test']
    assert summary['infeasible'] == 1 and summary['feasible']['n'] == 0
    assert 'Infeasible' in render_report(summarize([row]), [row], {'model': 'm'})
