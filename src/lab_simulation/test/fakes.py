"""
A fake robot and world for the task, executor and benchmark tests.

The fakes use the real SO-101 kinematics: the arm jumps to commanded joint
angles, the gripper closes on a piece only if it is between the jaws, and a
held piece moves with the grasp frame.
"""

import math
import random

from lab_simulation import layout
from lab_simulation.tasks import MoveResult, Observation, PiecePose
import numpy as np

HOME = (0.0, -1.3, 0.6, 1.6, 0.0)
OPEN, CLOSED, STALL = 0.6, -0.17, 0.19
# Where the moving jaw pushes a gripped piece, in the grasp frame.
PRESSED = np.array([0.009, 0.0, 0.0])


class FakeWorld:
    """Arm, gripper and scene in one, sharing state."""

    def __init__(self, kin, positions=None):
        self.kin = kin
        self.q = HOME
        self.gripper = OPEN
        self.time = 0.0
        self.held = None
        self.grip_offset = None
        self.latched = set()
        self.latch_calls = []
        self.scene_updates = []
        self.moves = []
        self.fail_plan_at = None      # move index that fails to plan
        self.drop_on_move = None      # move index after which a held piece falls out
        self.miss = False             # piece slides away as the jaws close
        self.held_at = None
        self.source = 'ground_truth'  # or 'vision'
        self.seen_offset = (0.0, 0.0)  # vision: detections this far off the truth
        self.hidden = set()           # vision: pieces the camera does not see
        self.pressed = PRESSED        # where the jaws push a gripped piece
        self.observations = 0
        positions = positions or layout.sample_layout(random.Random(1))
        self.pieces = {name: PiecePose(x, y, layout.PIECES[name].centre_z)
                       for name, (x, y) in positions.items()}

    # arm
    def joint_positions(self):
        return self.q

    def move_to(self, q):
        index = len(self.moves)
        self.moves.append(tuple(q))
        self.time += 1.0
        if index == self.fail_plan_at:
            return MoveResult(False, False, 0.0, math.inf)
        self.q = tuple(q)
        if self.held is not None:
            if index == self.drop_on_move and self.held not in self.latched:
                self._drop()
            else:
                tcp = self.kin.forward(self.q)
                x, y, z = (tcp @ np.array([*self.grip_offset, 1.0]))[:3]
                self.pieces[self.held] = PiecePose(x, y, z)
        return MoveResult(True, True, 1.0, 0.0)

    def home(self):
        return self.move_to(HOME)

    # gripper
    def release(self, angle=OPEN):
        self.released_to = angle
        self.gripper = OPEN
        if self.held is not None and self.held not in self.latched:
            self._drop()
        return True

    def grip(self):
        tcp = self.kin.forward(self.q)
        for name, pose in self.pieces.items():
            local = (np.linalg.inv(tcp) @ np.array([*pose.xyz, 1.0]))[:3]
            if np.linalg.norm(local) < 0.02 and not self.miss:
                self.held, self.grip_offset = name, self.pressed
                x, y, z = (tcp @ np.array([*self.pressed, 1.0]))[:3]
                self.pieces[name] = PiecePose(x, y, z)
                self.gripper = STALL
                return STALL
        self.gripper = CLOSED
        return CLOSED

    def hold(self, angle):
        self.held_at = angle

    def _drop(self):
        p = self.pieces[self.held]
        self.pieces[self.held] = PiecePose(p.x, p.y, layout.PIECES[self.held].centre_z)
        self.held = None

    # scene
    def pose(self, name):
        return self.pieces[name]

    def observe(self):
        self.observations += 1
        self.time += 0.5
        dx, dy = self.seen_offset
        poses = {n: p._replace(x=p.x + dx, y=p.y + dy) for n, p in self.pieces.items()
                 if n not in self.hidden}
        evaluation = {'pieces': {}, 'missed': len(self.hidden), 'false': 0} \
            if self.source == 'vision' else None
        return Observation(poses, self.source, evaluation, wait_s=0.5, latency_s=0.2)

    def latch(self, name):
        self.latch_calls.append(name)
        self.latched.add(name)
        return True

    def unlatch(self, name):
        self.latched.discard(name)
        if self.held == name and self.gripper == OPEN:
            self._drop()

    def update_planning_scene(self, exclude=None, held=None, bins=True):
        self.scene_updates.append((exclude, held))

    def unlatch_all(self):
        for name in list(self.latched):
            self.unlatch(name)

    def wait_settled(self, timeout_s=3.0):
        return True

    def now(self):
        return self.time
