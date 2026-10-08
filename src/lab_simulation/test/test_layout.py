"""Tests for piece geometry and seeded sampling."""

import math
import random

from lab_simulation import layout
import pytest


def test_same_seed_gives_same_layout():
    assert layout.sample_layout(random.Random('42')) == layout.sample_layout(random.Random('42'))


def test_pan_axis_matches_the_arm(lab_kinematics):
    assert layout.PAN_AXIS_XY == pytest.approx(lab_kinematics.pan_axis_xy, abs=1e-6)


@pytest.mark.parametrize('name', sorted(layout.PIECES))
@pytest.mark.parametrize('radius', [layout.MIN_RADIUS, 0.18, layout.MAX_RADIUS])
@pytest.mark.parametrize('bearing', [-layout.MAX_BEARING, -0.8, 0.0, 0.8, layout.MAX_BEARING])
def test_every_workspace_point_is_graspable(lab_kinematics, name, radius, bearing):
    """Grasp, hover and carry heights are reachable across the annulus."""
    piece = layout.PIECES[name]
    x = layout.PAN_AXIS_XY[0] + radius * math.cos(bearing)
    y = layout.PAN_AXIS_XY[1] + radius * math.sin(bearing)
    yaws = piece.closing_yaws(0.0) or [None]
    for z in (piece.grasp_z, piece.grasp_z + layout.HOVER, piece.grasp_z + layout.LIFT):
        assert any(lab_kinematics.is_reachable((x, y, z), yaw) for yaw in yaws)


@pytest.mark.parametrize('radius', [layout.MIN_RADIUS, layout.MAX_RADIUS])
@pytest.mark.parametrize('bearing', [-layout.MAX_BEARING, 0.0, layout.MAX_BEARING])
@pytest.mark.parametrize('height', layout.REACH_HEIGHT)
def test_reach_heights_are_reachable(lab_kinematics, radius, bearing, height):
    x = layout.PAN_AXIS_XY[0] + radius * math.cos(bearing)
    y = layout.PAN_AXIS_XY[1] + radius * math.sin(bearing)
    assert lab_kinematics.is_reachable((x, y, layout.TABLE_TOP + height), None)


@pytest.mark.parametrize('seed', range(50))
def test_layout_constraints(seed):
    positions = layout.sample_layout(random.Random(seed))
    assert set(positions) == set(layout.PIECES)
    xys = list(positions.values())
    for i, a in enumerate(xys):
        assert layout.in_workspace(a)
        for b in xys[i + 1:]:
            assert math.dist(a, b) >= layout.MIN_SEPARATION


@pytest.mark.parametrize('seed', range(50))
def test_place_target_constraints(seed):
    rng = random.Random(seed)
    positions = layout.sample_layout(rng)
    target = layout.sample_place_target(rng, 'red_cube', positions)
    assert layout.in_workspace(target)
    assert math.dist(target, positions['red_cube']) >= layout.MIN_TRAVEL
    for name in ('blue_cylinder', 'green_block'):
        assert math.dist(target, positions[name]) >= layout.MIN_SEPARATION


@pytest.mark.parametrize('seed', range(50))
def test_reach_target_is_in_the_workspace_and_clear_of_pieces(seed):
    rng = random.Random(seed)
    positions = layout.sample_layout(rng)
    x, y, z = layout.sample_reach_target(rng, positions)
    assert layout.in_workspace((x, y))
    low, high = layout.REACH_HEIGHT
    assert layout.TABLE_TOP + low <= z <= layout.TABLE_TOP + high
    assert all(math.dist((x, y), p) >= layout.REACH_CLEARANCE for p in positions.values())


def test_impossible_layout_raises():
    with pytest.raises(RuntimeError):
        layout.sample_layout(random.Random(0), min_separation=1.0, max_attempts=50)


def test_every_piece_fits_the_jaws():
    """The jaw pads are angled for a 3 cm piece (spec section 2)."""
    for piece in layout.PIECES.values():
        assert piece.grasp_width == pytest.approx(0.03)


def test_closing_yaws():
    cube, block = layout.PIECES['red_cube'], layout.PIECES['green_block']
    assert cube.closing_yaws(0.3) == pytest.approx([0.3, 0.3 + math.pi / 2])
    # The block is 5 cm along x and 3 cm along y: close across y.
    assert block.closing_yaws(0.3) == pytest.approx([0.3 + math.pi / 2])
    assert layout.PIECES['blue_cylinder'].closing_yaws(0.3) is None


def test_open_jaws_fit_between_neighbours():
    """MIN_SEPARATION leaves room for the open jaws (~45 mm) beside a piece."""
    widest = max(max(p.half_x, p.half_y) for p in layout.PIECES.values())
    assert layout.MIN_SEPARATION - 2 * widest >= 0.03


def test_one_bin_per_piece():
    assert sorted(layout.BINS) == sorted(layout.PIECES)
    assert {b.piece for b in layout.BINS.values()} == set(layout.PIECES)


def test_bins_are_in_the_workspace_and_apart():
    bins = list(layout.BINS.values())
    for i, a in enumerate(bins):
        assert layout.in_workspace(a.xy)
        for b in bins[i + 1:]:
            assert math.dist(a.xy, b.xy) >= a.outer


def test_open_jaws_fit_in_a_bin():
    """The open jaws span ~7 cm along the closing axis (fixed to moving jaw)."""
    assert layout.BIN_INNER >= 0.085


@pytest.mark.parametrize('piece', sorted(layout.PIECES))
@pytest.mark.parametrize('offset', [(0.0, 0.0), (0.01, 0.0), (-0.01, 0.0), (0.0, 0.01),
                                    (0.0, -0.01)])
def test_every_bin_is_reachable_for_its_piece(lab_kinematics, piece, offset):
    bin_ = layout.BINS[piece]
    p = layout.PIECES[piece]
    x, y = bin_.xy[0] + offset[0], bin_.xy[1] + offset[1]
    for z in (p.centre_z + 0.003, p.centre_z + 0.003 + layout.LIFT):
        assert any(lab_kinematics.is_reachable((x, y, z), yaw) for yaw in (0.0, math.pi / 2))


def test_bin_contains():
    bin_ = layout.BINS['red_cube']
    x, y = bin_.xy
    assert bin_.contains((x, y))
    assert bin_.contains((x + 0.04, y), margin=0.0)
    assert not bin_.contains((x + 0.04, y), margin=0.015)
    assert not bin_.contains((x + 0.06, y))


@pytest.mark.parametrize('seed', range(50))
def test_samples_stay_clear_of_the_bins(seed):
    rng = random.Random(seed)
    positions = layout.sample_layout(rng)
    target = layout.sample_place_target(rng, 'red_cube', positions)
    reach = layout.sample_reach_target(rng, positions)
    for xy in (*positions.values(), target, reach[:2]):
        assert layout.clear_of_bins(xy)


def test_yaws_and_light_are_seeded_and_in_range():
    a, b = random.Random(5), random.Random(5)
    assert layout.sample_yaws(a) == layout.sample_yaws(b)
    assert layout.sample_light(a) == layout.sample_light(b)
    yaws = layout.sample_yaws(random.Random(1))
    assert sorted(yaws) == sorted(layout.PIECES)
    assert all(-math.pi <= y < math.pi for y in yaws.values())
    for seed in range(20):
        (dx, dy, dz), brightness = layout.sample_light(random.Random(seed))
        assert dz < 0 and abs(dx) <= 0.8 and abs(dy) <= 0.8
        assert 0.4 <= brightness <= 1.0
