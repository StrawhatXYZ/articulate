"""Tests for resolving plan targets to safe table positions."""

import math

from lab_simulation import layout, targets

POSITIONS = {'red_cube': (0.20, -0.08), 'blue_cylinder': (0.12, 0.0), 'green_block': (0.15, -0.15)}
STARTS = dict(POSITIONS)


def _others(piece):
    return [xy for name, xy in POSITIONS.items() if name != piece]


def _resolve(piece, target):
    return targets.resolve(target, piece, POSITIONS, STARTS)


def test_bin_target_is_the_bin_centre():
    xy, _ = _resolve('red_cube', {'kind': 'bin'})
    assert xy == layout.BINS['red_cube'].xy


def test_start_is_used_when_free():
    xy, _ = _resolve('red_cube', {'kind': 'start'})
    assert xy == STARTS['red_cube']


def test_start_occupied_moves_to_a_valid_nearby_point():
    positions = dict(POSITIONS, blue_cylinder=STARTS['red_cube'])
    xy, _ = targets.resolve({'kind': 'start'}, 'red_cube', positions, STARTS)
    assert xy != STARTS['red_cube']
    assert targets.is_valid(xy, [positions['blue_cylinder'], positions['green_block']])


def test_near_puts_the_piece_on_the_requested_side():
    xy, _ = _resolve('blue_cylinder', {'kind': 'near', 'piece': 'green_block', 'side': 'front'})
    other = POSITIONS['green_block']
    assert xy[0] > other[0]
    assert math.dist(xy, other) >= layout.MIN_SEPARATION
    assert targets.is_valid(xy, _others('blue_cylinder'))


def test_near_itself_or_unknown_is_refused():
    assert _resolve('red_cube', {'kind': 'near', 'piece': 'red_cube', 'side': 'left'})[0] is None
    assert _resolve('red_cube', {'kind': 'near', 'piece': 'nope', 'side': 'left'})[0] is None
    assert _resolve('red_cube', {'kind': 'near', 'piece': 'green_block', 'side': 'up'})[0] is None


def test_every_region_is_valid():
    for region in targets.REGIONS:
        for piece in POSITIONS:
            xy, _ = _resolve(piece, {'kind': 'region', 'region': region})
            assert xy is not None and targets.is_valid(xy, _others(piece))


def test_regions_are_ordered_from_the_robots_view():
    positions = {'red_cube': (0.15, -0.15), 'blue_cylinder': (0.15, 0.15),
                 'green_block': (0.2, 0.0)}
    xy = {r: targets.resolve({'kind': 'region', 'region': r}, 'green_block', positions,
                             positions)[0] for r in targets.REGIONS}
    assert xy['front'][0] > xy['back'][0]
    assert xy['left'][1] > xy['right'][1]


def test_unreachable_xy_is_moved_into_the_workspace():
    xy, _ = _resolve('red_cube', {'kind': 'xy', 'xy': [2.0, 2.0]})
    assert layout.in_workspace(xy) and targets.is_valid(xy, _others('red_cube'))


def test_xy_inside_a_bin_is_moved_clear_of_it():
    xy, _ = _resolve('red_cube', {'kind': 'xy', 'xy': list(layout.BINS['green_block'].xy)})
    assert layout.clear_of_bins(xy)


def test_unknown_kind_is_refused():
    xy, reason = _resolve('red_cube', {'kind': 'sideways'})
    assert xy is None and reason
