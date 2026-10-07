"""
Forward and inverse kinematics of the SO-101 arm, built from its URDF.

Pure Python and NumPy (no ROS). Positions are in the URDF root frame in
metres, angles in radians. Task goals use a vertical approach: the gripper
points straight down and its closing axis (grasp_link x) is turned to a
requested yaw, which a 5-DOF arm can always satisfy where it can reach.
"""

import math
from typing import Dict, List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
TIP = 'grasp_link'
POSITION_TOLERANCE_M = 1e-4
ANGLE_TOLERANCE = 1e-3          # residual of the unit-vector orientation terms
MAX_ITERATIONS = 200
STALL_WINDOW = 10           # give up if the error shrinks less than
STALL_IMPROVEMENT = 0.05    # this fraction over a window of iterations
DAMPING = 1e-3
FD_STEP = 1e-6
# Elbow-up starting guesses (shoulder_lift, elbow_flex, wrist_flex); pan and
# roll are seeded from the target.
SEEDS = ((0.0, 0.0, 1.2), (0.5, 0.5, 1.0), (-0.5, 1.0, 1.0), (0.3, -0.3, 1.2),
         (-1.0, 1.5, 1.2), (1.0, -0.5, 1.4))

ROLL_SEEDS = (0.0, 1.57, -1.57, 2.7)
# Horizontal distance from the pan axis outside which no vertical-approach
# solution exists (generous bounds; the solver decides inside them).
MIN_RADIUS_M = 0.04
MAX_RADIUS_M = 0.33

Joints = Tuple[float, float, float, float, float]
XYZ = Tuple[float, float, float]


class Unreachable(ValueError):
    """Raised when no joint configuration within limits reaches a goal."""


def _floats(text: Optional[str], default: str = '0 0 0') -> List[float]:
    return [float(v) for v in (text or default).split()]


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _transform(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = _rpy(*rpy)
    m[:3, 3] = xyz
    return m


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    m = np.eye(4)
    m[:3, :3] = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)
    return m


class Kinematics:
    """Kinematic chain from the URDF root to a tip link."""

    def __init__(self, urdf_xml: str, tip: str = TIP):
        robot = ET.fromstring(urdf_xml)
        by_child = {j.find('child').get('link'): j for j in robot.findall('joint')}
        self.limits: Dict[str, Tuple[float, float]] = {}
        for joint in robot.findall('joint'):
            limit = joint.find('limit')
            if joint.get('name') in ARM_JOINTS and limit is not None:
                self.limits[joint.get('name')] = (float(limit.get('lower')),
                                                  float(limit.get('upper')))
        self.joints = ARM_JOINTS
        self._chains = {}
        self._by_child = by_child
        self._tip = tip
        self._chain(tip)
        pan = self.forward_partial('shoulder_pan')
        self.pan_axis_xy = (float(pan[0, 3]), float(pan[1, 3]))
        # +1 if a positive pan angle turns the arm anticlockwise seen from above.
        self._pan_sign = 1.0 if self._bearing(self.forward((0.1, 0, 0, 0, 0))[:3, 3]) > 0 else -1.0

    def _chain(self, link: str):
        """Return [(fixed transform, axis or None, joint name)] from the root to link."""
        if link not in self._chains:
            chain = []
            current = link
            while current in self._by_child:
                joint = self._by_child[current]
                origin = joint.find('origin')
                fixed = _transform(_floats(origin.get('xyz') if origin is not None else None),
                                   _floats(origin.get('rpy') if origin is not None else None))
                axis = None
                if joint.get('type') in ('revolute', 'continuous'):
                    axis = np.array(_floats(joint.find('axis').get('xyz'), '1 0 0'))
                    axis = axis / np.linalg.norm(axis)
                chain.append((fixed, axis, joint.get('name')))
                current = joint.find('parent').get('link')
            self._chains[link] = list(reversed(chain))
        return self._chains[link]

    def forward(self, q: Sequence[float], link: Optional[str] = None) -> np.ndarray:
        """Return the 4x4 pose of link (default: the tip) for arm joint angles q."""
        angles = dict(zip(ARM_JOINTS, q))
        m = np.eye(4)
        for fixed, axis, name in self._chain(link or self._tip):
            m = m @ fixed
            if axis is not None:
                m = m @ _axis_rotation(axis, angles.get(name, 0.0))
        return m

    def forward_partial(self, joint_name: str) -> np.ndarray:
        """Return the pose of a joint's frame (before its rotation) at zero angles."""
        child = next(c for c, j in self._by_child.items() if j.get('name') == joint_name)
        m = np.eye(4)
        for fixed, axis, name in self._chain(child):
            m = m @ fixed
            if name == joint_name:
                return m
        return m

    def _bearing(self, xyz: Sequence[float]) -> float:
        return math.atan2(xyz[1] - self.pan_axis_xy[1], xyz[0] - self.pan_axis_xy[0])

    def grasp_yaw(self, q: Sequence[float]) -> float:
        """Return the world yaw of the closing axis (tip x) for joint angles q."""
        m = self.forward(q)
        return math.atan2(m[1, 0], m[0, 0])

    def _residual(self, q: np.ndarray, target: np.ndarray,
                  yaw: Optional[float]) -> np.ndarray:
        m = self.forward(q)
        terms = [m[:3, 3] - target, m[:3, 2] - (0.0, 0.0, -1.0)]
        if yaw is not None:
            terms.append(m[:2, 0] - (math.cos(yaw), math.sin(yaw)))
        return np.concatenate(terms)

    def _converged(self, residual: np.ndarray) -> bool:
        return (np.linalg.norm(residual[:3]) < POSITION_TOLERANCE_M
                and np.max(np.abs(residual[3:])) < ANGLE_TOLERANCE)

    def _solve(self, seed: np.ndarray, target: np.ndarray, yaw: Optional[float],
               active: Sequence[int]) -> Optional[np.ndarray]:
        lower = np.array([self.limits[n][0] for n in ARM_JOINTS])
        upper = np.array([self.limits[n][1] for n in ARM_JOINTS])
        q = np.clip(seed, lower, upper)
        best = math.inf
        for iteration in range(MAX_ITERATIONS):
            residual = self._residual(q, target, yaw)
            if self._converged(residual):
                return q
            error = float(np.linalg.norm(residual))
            if iteration % STALL_WINDOW == 0:
                if error > best * (1.0 - STALL_IMPROVEMENT):
                    return None     # stuck against a limit or in a local minimum
                best = error
            jacobian = np.zeros((residual.size, len(active)))
            for column, index in enumerate(active):
                step = np.zeros(5)
                step[index] = FD_STEP
                jacobian[:, column] = (self._residual(q + step, target, yaw) - residual) / FD_STEP
            jjt = jacobian @ jacobian.T + DAMPING * np.eye(residual.size)
            delta = jacobian.T @ np.linalg.solve(jjt, residual)
            q[list(active)] = np.clip(q[list(active)] - delta, lower[list(active)],
                                      upper[list(active)])
        return None

    def inverse(self, target: Sequence[float], yaw: Optional[float],
                seed: Optional[Sequence[float]] = None) -> Joints:
        """
        Return joint angles that put the tip at target, pointing straight down.

        With yaw, the closing axis is turned to that world yaw (either way
        round: the jaws grip both ways). With yaw None the wrist roll stays
        at zero. Raises Unreachable if no solution within limits exists.
        """
        target = np.asarray(target, dtype=float)
        pan = self._pan_sign * self._bearing(target)
        lower, upper = self.limits['shoulder_pan']
        radius = math.dist(target[:2], self.pan_axis_xy)
        if not (MIN_RADIUS_M <= radius <= MAX_RADIUS_M) or not lower <= pan <= upper:
            # Cheap rejection; the solver would only fail slowly here.
            raise Unreachable(f'{tuple(target)} is out of reach')
        active = (0, 1, 2, 3) if yaw is None else (0, 1, 2, 3, 4)
        rolls = (0.0,) if yaw is None else ROLL_SEEDS
        seeds = [] if seed is None else [np.asarray(seed, dtype=float)]
        for roll in rolls:
            for lift, elbow, flex in SEEDS:
                seeds.append(np.array([pan, lift, elbow, flex, roll]))
        for start in seeds:
            for goal_yaw in ([None] if yaw is None else [yaw, yaw + math.pi]):
                if goal_yaw is None:
                    start = start.copy()
                    start[4] = 0.0
                q = self._solve(start.copy(), target, goal_yaw, active)
                if q is not None:
                    return tuple(float(v) for v in q)
        raise Unreachable(f'no vertical-approach solution for {tuple(target)}')

    def is_reachable(self, target: Sequence[float], yaw: Optional[float]) -> bool:
        """Return True if inverse(target, yaw) has a solution."""
        try:
            self.inverse(target, yaw)
        except Unreachable:
            return False
        return True
