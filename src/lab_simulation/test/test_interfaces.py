"""The Instruct action interface exists with the agreed fields."""

from lab_interfaces.action import Instruct


def test_instruct_fields_round_trip():
    goal = Instruct.Goal(text='put the red cube in its box')
    result = Instruct.Result(outcome='done', message='ok', steps=['move red_cube to bin'])
    feedback = Instruct.Feedback(stage='planning', message='')
    assert goal.text.startswith('put')
    assert result.steps == ['move red_cube to bin'] and result.outcome == 'done'
    assert feedback.stage == 'planning'
