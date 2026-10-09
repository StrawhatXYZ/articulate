"""The safety check: every malformed, unknown or impossible plan is refused."""

import random

from lab_simulation import layout
from lab_simulation.memory import BOX, PieceState, TABLE, UNKNOWN
from lab_simulation.plan_check import check, MAX_STEPS, Step, ToolCall
from lab_simulation.tasks import PiecePose

XY = {'red_cube': (0.20, -0.08), 'blue_cylinder': (0.12, -0.15), 'green_block': (0.05, -0.19)}


def states(**override):
    out = {n: PieceState(n, TABLE, PiecePose(*xy, 0.765), xy) for n, xy in XY.items()}
    for name, status in override.items():
        pose = PiecePose(*layout.BINS[name].xy, 0.765) if status == BOX else None
        out[name] = PieceState(name, status, pose, XY[name], status == BOX)
    return out


def call(name, **args):
    return ToolCall(name, args)


def test_move_to_bin_is_one_step():
    v = check([call('move_piece', piece='red_cube', target='bin')], states())
    assert v.kind == 'plan' and v.steps == (Step('move', 'red_cube', {'kind': 'bin'}),)
    assert v.final['red_cube'] == 'box'


def test_colour_word_names_a_piece_and_ambiguity_refuses():
    assert check([call('move_piece', piece='the red one', target='bin')],
                 states()).steps[0].piece == 'red_cube'
    assert check([call('move_piece', piece='Red Cube', target='bin')], states()).kind == 'plan'
    v = check([call('move_piece', piece='piece', target='bin')], states())
    assert v.kind == 'refuse' and v.source == 'check'


def test_refuse_and_answer_pass_through_alone():
    assert check([call('refuse', reason='unsafe')], states()).kind == 'refuse'
    v = check([call('answer', text='Two pieces.')], states())
    assert v.kind == 'answer' and v.message == 'Two pieces.'


def test_refuse_mixed_with_moves_is_refused_as_contradictory():
    v = check([call('refuse', reason='x'), call('go_home')], states())
    assert v.kind == 'refuse' and 'contradict' in v.message


def test_unknown_piece_refuses():
    v = check([call('move_piece', piece='green_block', target='bin')],
              states(green_block=UNKNOWN))
    assert v.kind == 'refuse' and 'not visible' in v.message


def test_near_needs_the_other_piece_on_the_table():
    v = check([call('move_piece', piece='red_cube', target='near', other_piece='blue_cylinder',
                    side='left')], states(blue_cylinder=BOX))
    assert v.kind == 'refuse'


def test_swap_expands_to_three_moves_and_ends_swapped():
    v = check([call('swap_pieces', piece_a='red_cube', piece_b='green_block')], states())
    assert v.kind == 'plan' and len(v.steps) == 3
    assert v.final['red_cube'] == XY['green_block'] and v.final['green_block'] == XY['red_cube']


def test_swap_needs_both_on_the_table():
    v = check([call('swap_pieces', piece_a='red_cube', piece_b='green_block')],
              states(red_cube=BOX))
    assert v.kind == 'refuse'


def test_sort_all_moves_only_table_pieces():
    v = check([call('sort_all')], states(red_cube=BOX))
    assert [s.piece for s in v.steps] == ['blue_cylinder', 'green_block']


def test_moving_a_boxed_piece_to_its_box_is_a_no_op():
    v = check([call('move_piece', piece='red_cube', target='bin')], states(red_cube=BOX))
    assert v.kind == 'plan' and v.steps == ()


def test_too_many_steps_refuses():
    calls = [call('go_home')] * (MAX_STEPS + 1)
    assert check(calls, states()).kind == 'refuse'


def test_no_calls_is_not_a_plan():
    assert check([], states()).kind == 'refuse'


def test_random_junk_never_raises():
    rng = random.Random(7)
    junk = [None, 3, 2.5, [], {}, {'a': [1]}, 'x' * 10000, ['red_cube'], {'kind': 'bin'},
            'red_cube', 'bin', 'near', 'left', '', '\x00', '🙂']
    names = ['move_piece', 'swap_pieces', 'sort_all', 'go_home', 'refuse', 'answer', 'rm', None,
             3, []]
    keys = ['piece', 'target', 'other_piece', 'side', 'region', 'piece_a', 'piece_b', 'reason',
            'text', 'xy', 'kind']
    for _ in range(5000):
        calls = []
        for _ in range(rng.randint(0, 4)):
            args = {rng.choice(keys): rng.choice(junk) for _ in range(rng.randint(0, 4))}
            calls.append(ToolCall(rng.choice(names), rng.choice([args, args, None, 'x', 5])))
        verdict = check(calls, states())
        assert verdict.kind in ('plan', 'refuse', 'answer')
        for step in verdict.steps:
            assert step.action in ('move', 'home')
