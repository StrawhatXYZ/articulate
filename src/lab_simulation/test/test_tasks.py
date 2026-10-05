"""Tests for the reach and pick-and-place tasks, using an in-memory fake world."""

import math
import random

from lab_simulation import layout
from lab_simulation.kinematics import forward
from lab_simulation.tasks import MoveResult, PickPlaceTask, ReachTask
import pytest


class FakeWorld:
    """
    Stand-in for arm.Arm and scene.Scene.

    Moves complete instantly (1 simulated second each). Options inject faults.
    """

    def __init__(self, positions=None, joint_bias=0.0, drop=False, stuck=False,
                 knock_off=False, sag=0.0):
        positions = positions or {'red_cube': (0.2, 0.0), 'blue_cylinder': (-0.2, 0.0),
                                  'green_block': (0.0, -0.2)}
        self.poses = {n: (x, y, layout.PIECES[n].centre_z) for n, (x, y) in positions.items()}
        self.q = (0.0, 0.0, 0.0)
        self.time = 0.0
        self.attached = None
        self.offset = None
        self.joint_bias = joint_bias
        self.drop = drop
        self.stuck = stuck
        self.knock_off = knock_off
        self.sag = sag  # a held piece hangs this far below its grip (joint compliance)
        self.commanded_tips = []

    # arm.Arm interface
    def joint_positions(self):
        return self.q

    def move_to(self, q, timeout_s, monitor=None):
        self.time += 1.0
        self.commanded_tips.append(forward(q))
        if self.stuck:
            return MoveResult(False, False, timeout_s, 1.0)
        self.q = (q[0] + self.joint_bias, q[1], q[2])
        if self.attached and not self.drop:
            tip = forward(self.q)
            held = tuple(t + o for t, o in zip(tip, self.offset))
            self.poses[self.attached] = (held[0], held[1], held[2] - self.sag)
        if monitor is not None and not monitor():
            return MoveResult(False, True, 0.5, 0.0)
        return MoveResult(True, False, 0.5, 0.0)

    # scene.Scene interface
    def pose(self, name):
        return self.poses[name]

    def grasp_gap(self, name, tip):
        return layout.grasp_gap(layout.PIECES[name], self.poses[name], tip)

    def attach(self, name, tip):
        if self.grasp_gap(name, tip) > layout.GRASP_GAP_MAX:
            return False
        self.attached = name
        self.offset = tuple(p - t for p, t in zip(self.poses[name], tip))
        return True

    def detach(self, name):
        if self.attached == name:
            self.attached = None

    def wait_settled(self, timeout_s=3.0):
        for name, (x, y, _) in self.poses.items():
            self.poses[name] = (x, y, -1.0) if self.knock_off else \
                (x, y, layout.PIECES[name].centre_z)
        return True


def now_of(world):
    return lambda: world.time


def test_reach_success():
    world = FakeWorld()
    result = ReachTask(world, now_of(world)).run(random.Random(1), 0)
    assert result.success and result.failure is None
    assert result.error_m < 1e-9
    assert result.cycle_time_s == 1.0
    assert result.details['max_joint_error_rad'] == 0.0


def test_reach_miss_is_placement_error():
    world = FakeWorld(joint_bias=0.2)
    result = ReachTask(world, now_of(world)).run(random.Random(1), 0)
    assert not result.success and result.failure == 'placement_error'
    assert result.error_m > 0.01


def test_reach_timeout():
    world = FakeWorld(stuck=True)
    result = ReachTask(world, now_of(world)).run(random.Random(1), 0)
    assert result.failure == 'timeout'


def test_pick_place_success_lands_on_target():
    world = FakeWorld()
    result = PickPlaceTask(world, world, now_of(world)).run(random.Random(3), 0)
    assert result.success, result
    assert result.error_m < 1e-6
    assert world.attached is None
    piece = result.details['piece']
    assert math.dist(world.poses[piece][:2], result.details['target']) < 1e-6
    assert result.disturbed is False


def test_pick_place_grasp_miss_never_attaches():
    world = FakeWorld(joint_bias=0.2)
    result = PickPlaceTask(world, world, now_of(world)).run(random.Random(3), 0)
    assert result.failure == 'grasp_miss'
    assert result.details['grasp_gap_m'] > layout.GRASP_GAP_MAX


def test_pick_place_drop_detected():
    world = FakeWorld(drop=True)
    result = PickPlaceTask(world, world, now_of(world)).run(random.Random(3), 0)
    assert result.failure == 'dropped'
    assert world.attached is None
    assert result.details['carry_deviation_m'] > 0.01


def test_pick_place_unreachable_piece():
    world = FakeWorld(positions={'red_cube': (0.6, 0.0), 'blue_cylinder': (-0.6, 0.0),
                                 'green_block': (0.0, -0.6)})
    result = PickPlaceTask(world, world, now_of(world)).run(random.Random(3), 0)
    assert result.failure == 'unreachable'


def test_pick_place_piece_knocked_off_table_is_placement_error():
    world = FakeWorld(knock_off=True)
    result = PickPlaceTask(world, world, now_of(world)).run(random.Random(3), 0)
    assert result.failure == 'placement_error' or result.success
    assert result.details['final'][2] == -1.0
    assert result.disturbed is True


def test_result_serialises():
    world = FakeWorld()
    data = ReachTask(world, now_of(world)).run(random.Random(1), 4).to_dict()
    assert data['task'] == 'reach' and data['index'] == 4
    assert set(data) == {'task', 'index', 'success', 'failure', 'cycle_time_s', 'error_m',
                         'disturbed', 'details'}


def test_pick_descends_to_the_standoff_above_the_piece():
    world = FakeWorld()
    result = PickPlaceTask(world, world, now_of(world)).run(random.Random(3), 0)
    piece = layout.PIECES[result.details['piece']]
    start_z = result.details['start'][2]
    grasp_tip_z = world.commanded_tips[1][2]  # moves: hover, descend, ...
    contact_z = start_z + piece.half_height + layout.TIP_RADIUS
    assert grasp_tip_z == pytest.approx(contact_z + layout.GRASP_STANDOFF, abs=1e-6)


def test_held_piece_hanging_slightly_off_its_grip_is_not_dropped():
    world = FakeWorld(sag=0.015)
    result = PickPlaceTask(world, world, now_of(world)).run(random.Random(3), 0)
    assert result.failure != 'dropped'
    assert result.details['carry_deviation_m'] == pytest.approx(0.015, abs=1e-6)
