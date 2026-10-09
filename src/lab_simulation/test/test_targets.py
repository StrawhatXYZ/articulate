"""Resolving plan targets to safe table positions."""

import math

from lab_simulation import layout, targets

POSITIONS = {'red_cube': (0.20, -0.08), 'blue_cylinder': (0.12, -0.15),
             'green_block': (0.05, -0.19)}
STARTS = dict(POSITIONS)


def _others(piece, positions=POSITIONS):
    return [xy for name, xy in positions.items() if name != piece]


def _resolve(piece, target, positions=POSITIONS):
    return targets.resolve(target, piece, positions, STARTS)


def test_bin_target_is_the_bin_centre():
    assert _resolve('red_cube', {'kind': 'bin'})[0] == layout.BINS['red_cube'].xy


def test_start_is_used_when_free():
    assert _resolve('red_cube', {'kind': 'start'})[0] == STARTS['red_cube']


def test_start_occupied_moves_to_a_valid_nearby_point():
    positions = dict(POSITIONS, blue_cylinder=STARTS['red_cube'])
    xy, note = targets.resolve({'kind': 'start'}, 'red_cube', positions, STARTS)
    assert xy != STARTS['red_cube'] and note.startswith('moved')
    assert targets.is_valid(xy, _others('red_cube', positions))


def test_near_puts_the_piece_on_the_requested_side():
    xy, _ = _resolve('blue_cylinder', {'kind': 'near', 'piece': 'red_cube', 'side': 'back'})
    other = POSITIONS['red_cube']
    assert math.dist(xy, other) >= layout.MIN_SEPARATION
    assert math.dist(xy, layout.PAN_AXIS_XY) < math.dist(other, layout.PAN_AXIS_XY)


def test_near_without_a_side_takes_any_free_side():
    xy, _ = _resolve('green_block', {'kind': 'near', 'piece': 'red_cube', 'side': None})
    assert xy is not None and targets.is_valid(xy, _others('green_block'))


def test_near_itself_unknown_or_bad_side_is_refused():
    assert _resolve('red_cube', {'kind': 'near', 'piece': 'red_cube', 'side': 'left'})[0] is None
    assert _resolve('red_cube', {'kind': 'near', 'piece': 'nope', 'side': 'left'})[0] is None
    assert _resolve('red_cube', {'kind': 'near', 'piece': 'green_block',
                                 'side': 'up'})[0] is None


def test_every_region_resolves_in_that_region():
    positions = {}                                      # an empty table
    for region in layout.REGIONS:
        xy, _ = targets.resolve({'kind': 'region', 'region': region}, 'green_block',
                                positions, STARTS)
        assert xy is not None and layout.in_region(xy, region)
        assert targets.is_valid(xy, positions.values())


def test_a_full_region_is_refused():
    xy, reason = _resolve('green_block', {'kind': 'region', 'region': 'center'})
    assert xy is None and 'no free' in reason


def test_unreachable_xy_is_moved_into_the_workspace():
    xy, _ = _resolve('red_cube', {'kind': 'xy', 'xy': (2.0, 2.0)})
    assert targets.is_valid(xy, _others('red_cube'))


def test_xy_inside_a_bin_is_moved_clear_of_it():
    xy, _ = _resolve('red_cube', {'kind': 'xy', 'xy': layout.BINS['green_block'].xy})
    assert layout.clear_of_bins(xy)


def test_unknown_kind_is_refused():
    xy, reason = _resolve('red_cube', {'kind': 'sideways'})
    assert xy is None and reason
