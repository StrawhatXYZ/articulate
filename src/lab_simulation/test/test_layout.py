"""Tests for piece geometry and seeded sampling."""

import math
import random

from lab_simulation import layout
from lab_simulation.kinematics import is_reachable
import pytest


def test_same_seed_gives_same_layout():
    assert layout.sample_layout(random.Random('42')) == layout.sample_layout(random.Random('42'))


@pytest.mark.parametrize('seed', range(50))
def test_layout_constraints(seed):
    positions = layout.sample_layout(random.Random(seed))
    assert set(positions) == set(layout.PIECES)
    xys = list(positions.values())
    for i, a in enumerate(xys):
        assert math.hypot(*a) >= layout.MIN_RADIUS
        for b in xys[i + 1:]:
            assert math.dist(a, b) >= layout.MIN_SEPARATION
    for name, (x, y) in positions.items():
        piece = layout.PIECES[name]
        assert is_reachable((x, y, piece.grasp_z), layout.MARGIN)
        assert is_reachable((x, y, piece.grasp_z + layout.HOVER), layout.MARGIN)


@pytest.mark.parametrize('seed', range(50))
def test_place_target_constraints(seed):
    rng = random.Random(seed)
    positions = layout.sample_layout(rng)
    target = layout.sample_place_target(rng, 'red_cube', positions)
    assert math.dist(target, positions['red_cube']) >= layout.MIN_TRAVEL
    for name in ('blue_cylinder', 'green_sphere'):
        assert math.dist(target, positions[name]) >= layout.MIN_SEPARATION
    assert is_reachable((*target, layout.PIECES['red_cube'].grasp_z), layout.MARGIN)


@pytest.mark.parametrize('seed', range(50))
def test_reach_target_is_reachable_and_above_table(seed):
    x, y, z = layout.sample_reach_target(random.Random(seed))
    assert z >= layout.TABLE_TOP + 0.05
    assert is_reachable((x, y, z), layout.MARGIN)


def test_impossible_layout_raises():
    with pytest.raises(RuntimeError):
        layout.sample_layout(random.Random(0), min_separation=1.0, max_attempts=50)


def test_grasp_gap():
    cube = layout.PIECES['red_cube']
    centre = (0.2, 0.0, cube.centre_z)
    touching = (0.2, 0.0, cube.grasp_z)
    assert layout.grasp_gap(cube, centre, touching) == pytest.approx(0.0)
    assert layout.grasp_gap(cube, centre, (0.2, 0.0, cube.grasp_z + 0.004)) == pytest.approx(
        0.004)
    assert layout.grasp_gap(cube, centre, (0.25, 0.0, cube.grasp_z)) == math.inf
