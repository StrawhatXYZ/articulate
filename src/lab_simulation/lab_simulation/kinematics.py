"""
Closed-form kinematics for the 3-DOF lab arm.

Pure Python (no ROS). Positions are in the world frame in metres, angles in
radians. The constants mirror urdf/simple_arm.urdf; test_kinematics.py checks
them against the file.
"""

import math
from typing import NamedTuple, Sequence, Tuple

Matrix = Tuple[Tuple[float, float, float], ...]

BASE_Z = 0.75            # world_joint: base_link on the table top
SHOULDER_HEIGHT = 0.29   # joint_1 (0.04) + link_1 (0.25), above base_link
UPPER_ARM = 0.20         # joint_2 -> joint_3
FOREARM = 0.15           # joint_3 -> end_effector
TIP_RADIUS = 0.025       # end_effector sphere

JOINT_LIMITS = (
    (-3.14159, 3.14159),
    (-2.356, 2.356),
    (-2.356, 2.356),
)


class Unreachable(ValueError):
    """Raised when no joint configuration within limits reaches a point."""


class Joints(NamedTuple):
    """Joint angles in radians: base yaw, shoulder pitch, elbow pitch."""

    q1: float
    q2: float
    q3: float


def forward(q: Sequence[float]) -> Tuple[float, float, float]:
    """Return the end-effector centre for joint angles q."""
    q1, q2, q3 = q
    r = UPPER_ARM * math.sin(q2) + FOREARM * math.sin(q2 + q3)
    h = UPPER_ARM * math.cos(q2) + FOREARM * math.cos(q2 + q3)
    return (r * math.cos(q1), r * math.sin(q1), BASE_Z + SHOULDER_HEIGHT + h)


def within_limits(q: Sequence[float]) -> bool:
    """Return True if every joint angle is inside its limit."""
    return all(lo <= v <= hi for v, (lo, hi) in zip(q, JOINT_LIMITS))


def inverse(xyz: Sequence[float]) -> Joints:
    """
    Return joint angles that put the end-effector centre at xyz.

    Of the two elbow solutions, the one with the higher elbow is preferred,
    which keeps the forearm away from the table.
    """
    x, y, z = xyz
    r = math.hypot(x, y)
    h = z - BASE_Z - SHOULDER_HEIGHT
    # atan2 can return exactly +/-pi, a hair outside the +/-3.14159 limit.
    lo, hi = JOINT_LIMITS[0]
    q1 = min(max(math.atan2(y, x), lo), hi) if r > 1e-9 else 0.0
    c3 = (r * r + h * h - UPPER_ARM ** 2 - FOREARM ** 2) / (2 * UPPER_ARM * FOREARM)
    if not -1.0 <= c3 <= 1.0:
        raise Unreachable(f'{tuple(xyz)} is outside the arm workspace')
    solutions = []
    for q3 in (math.acos(c3), -math.acos(c3)):
        q2 = math.atan2(r, h) - math.atan2(
            FOREARM * math.sin(q3), UPPER_ARM + FOREARM * math.cos(q3))
        q = Joints(q1, q2, q3)
        if within_limits(q):
            solutions.append(q)
    if not solutions:
        raise Unreachable(f'{tuple(xyz)} needs joint angles outside the limits')
    return max(solutions, key=lambda q: math.cos(q.q2))


def is_reachable(xyz: Sequence[float], margin: float = 0.0) -> bool:
    """Return True if xyz and points margin away radially and vertically are reachable."""
    x, y, z = xyz
    r = math.hypot(x, y)
    ux, uy = (x / r, y / r) if r > 1e-9 else (1.0, 0.0)
    probes = (
        (0.0, 0.0, 0.0),
        (margin * ux, margin * uy, 0.0),
        (-margin * ux, -margin * uy, 0.0),
        (0.0, 0.0, margin),
        (0.0, 0.0, -margin),
    )
    for dx, dy, dz in probes:
        try:
            inverse((x + dx, y + dy, z + dz))
        except Unreachable:
            return False
    return True


def end_effector_rotation(q: Sequence[float]) -> Matrix:
    """
    Return the world orientation of the end-effector frame for joint angles q.

    The frame yaws with joint_1 and pitches with joint_2 + joint_3:
    Rz(q1) * Ry(q2 + q3). Its z axis points along the forearm. A piece held
    by suction is fixed in this frame, so it rotates with the arm.
    """
    q1, q2, q3 = q
    c1, s1 = math.cos(q1), math.sin(q1)
    cp, sp = math.cos(q2 + q3), math.sin(q2 + q3)
    return ((c1 * cp, -s1, c1 * sp),
            (s1 * cp, c1, s1 * sp),
            (-sp, 0.0, cp))


def rotate(matrix: Matrix, vector: Sequence[float]) -> Tuple[float, float, float]:
    """Return matrix * vector."""
    return tuple(sum(row[k] * vector[k] for k in range(3)) for row in matrix)


def unrotate(matrix: Matrix, vector: Sequence[float]) -> Tuple[float, float, float]:
    """Return transpose(matrix) * vector (the inverse of rotate for a rotation)."""
    return tuple(sum(matrix[k][i] * vector[k] for k in range(3)) for i in range(3))
