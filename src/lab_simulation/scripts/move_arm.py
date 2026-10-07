#!/usr/bin/env python3
"""
Run a scripted joint-space check on the simulated SO-101.

For each pose in DEMO_SEQUENCE the node sends the arm joints to
arm_controller (a ros2_control JointTrajectoryController) as a one-point
trajectory, and the gripper to gripper_controller. After the hold time it
compares the commanded pose with /joint_states and reports the tracking
error. It moves the arm only; it does not plan with MoveIt or pick.

The node exits with status 0 if every pose was reached within TOLERANCE_RAD
and 1 otherwise, so it can also be used as a smoke test.

Usage:
  ros2 run lab_simulation move_arm.py
  ros2 run lab_simulation move_arm.py --ros-args -p use_sim_time:=true
"""

import sys

from builtin_interfaces.msg import Duration as DurationMsg
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
JOINT_NAMES = ARM_JOINTS + ('gripper',)

# (lower, upper) position limits in radians. Must match the SO-101 URDF;
# test/test_robot_description.py checks this.
JOINT_LIMITS = {
    'shoulder_pan': (-1.91986, 1.91986),
    'shoulder_lift': (-1.74533, 1.74533),
    'elbow_flex': (-1.69, 1.69),
    'wrist_flex': (-1.65806, 1.65806),
    'wrist_roll': (-2.74385, 2.84121),
    'gripper': (-0.174533, 1.74533),
}

# Each entry is (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
# wrist_roll, gripper) in radians plus a hold time in seconds.
DEMO_SEQUENCE = [
    ((0.0, -1.3, 0.6, 1.6, 0.0, 0.6), 3.0),     # home, jaws open
    ((0.0, 0.0, 0.0, 0.0, 0.0, 0.6), 3.0),      # straight out
    ((0.0, -0.4, 0.4, 1.6, 0.0, 0.6), 3.0),     # gripper down, 8 cm over the table
    ((0.8, -0.4, 0.4, 1.6, 1.2, 0.0), 3.0),     # turn left, roll the wrist, close
    ((-0.8, -0.5, 0.4, 1.6, -1.2, 0.6), 3.0),   # turn right, open
    ((0.0, -1.3, 0.6, 1.6, 0.0, 0.6), 3.0),     # home
]

# Maximum per-joint error accepted as "pose reached".
TOLERANCE_RAD = 0.05
MOVE_TIME_S = 2.0

# Trajectories are sent once per pose; a modest timer rate is enough to
# check the hold time.
PUBLISH_RATE_HZ = 10.0


def clamp(value, lower, upper):
    """Limit value to the closed interval [lower, upper]."""
    return max(lower, min(upper, value))


class ArmDemo(Node):
    """Steps the arm through DEMO_SEQUENCE and verifies each pose."""

    def __init__(self):
        super().__init__('arm_demo')
        self._arm = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._gripper = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, qos_profile_sensor_data)

        self._positions = {}
        self._step = -1
        self._step_end = None
        self._targets = None
        self._failures = 0
        self.done = False

        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)
        self.get_logger().info('waiting for /joint_states and the controllers')

    @property
    def succeeded(self):
        """Return True when every pose was reached within tolerance."""
        return self.done and self._failures == 0

    def _on_joint_states(self, msg):
        self._positions.update(zip(msg.name, msg.position))

    def _ready(self):
        has_feedback = all(name in self._positions for name in JOINT_NAMES)
        connected = all(p.get_subscription_count() > 0 for p in (self._arm, self._gripper))
        return has_feedback and connected

    def _tick(self):
        if self.done:
            return
        if self._step < 0:
            if self._ready():
                self.get_logger().info('simulation ready, starting demonstration')
                self._start_step(0)
            return

        if self.get_clock().now() >= self._step_end:
            self._check_pose()
            if self._step + 1 < len(DEMO_SEQUENCE):
                self._start_step(self._step + 1)
            else:
                self._finish()

    def _start_step(self, index):
        pose, hold = DEMO_SEQUENCE[index]
        self._step = index
        self._targets = {}
        for name, value in zip(JOINT_NAMES, pose):
            lower, upper = JOINT_LIMITS[name]
            target = clamp(value, lower, upper)
            if target != value:
                self.get_logger().warning(
                    f'{name} target {value:.3f} outside limits, clamped to {target:.3f}')
            self._targets[name] = target
        self._step_end = self.get_clock().now() + Duration(seconds=hold)

        self.get_logger().info('pose {}/{}: ({}) rad, holding {:.1f} s'.format(
            index + 1, len(DEMO_SEQUENCE),
            ', '.join(f'{v:.2f}' for v in self._targets.values()), hold))
        self._publish_targets()

    def _publish_targets(self):
        trajectory = JointTrajectory(joint_names=list(ARM_JOINTS))
        trajectory.points = [JointTrajectoryPoint(
            positions=[self._targets[name] for name in ARM_JOINTS],
            time_from_start=DurationMsg(sec=int(MOVE_TIME_S),
                                        nanosec=int(MOVE_TIME_S % 1 * 1e9)))]
        self._arm.publish(trajectory)
        self._gripper.publish(Float64MultiArray(data=[self._targets['gripper']]))

    def _check_pose(self):
        errors = {
            name: abs(self._positions[name] - target)
            for name, target in self._targets.items()
        }
        worst = max(errors, key=errors.get)
        if errors[worst] <= TOLERANCE_RAD:
            self.get_logger().info(
                f'  reached, max error {errors[worst]:.3f} rad ({worst})')
        else:
            self._failures += 1
            self.get_logger().error(
                f'  not reached: {worst} error {errors[worst]:.3f} rad '
                f'exceeds tolerance {TOLERANCE_RAD:.3f} rad')

    def _finish(self):
        self.done = True
        self._timer.cancel()
        if self._failures:
            self.get_logger().error(
                f'demonstration finished, {self._failures} pose(s) not reached')
        else:
            self.get_logger().info('demonstration complete, all poses reached')


def main():
    rclpy.init()
    node = ArmDemo()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        succeeded = node.succeeded
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if succeeded else 1


if __name__ == '__main__':
    sys.exit(main())
