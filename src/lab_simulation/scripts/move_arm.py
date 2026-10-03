#!/usr/bin/env python3
"""
Run a scripted joint-space demonstration on the simulated 3-DOF arm.

For each pose in DEMO_SEQUENCE the node publishes a std_msgs/msg/Float64
position target (radians) on /simple_arm/<joint>/cmd_pos. ros_gz_bridge
forwards these to Gazebo's JointPositionController, which latches the target
and holds the joint there. After the hold time the node compares the commanded
pose with /joint_states and reports the tracking error.

The node exits with status 0 if every pose was reached within TOLERANCE_RAD
and 1 otherwise, so it can also be used as a smoke test.

Usage:
  ros2 run lab_simulation move_arm.py
  ros2 run lab_simulation move_arm.py --ros-args -p use_sim_time:=true
"""

import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINT_NAMES = ('joint_1', 'joint_2', 'joint_3')

# (lower, upper) position limits in radians. Must match simple_arm.urdf;
# test/test_robot_description.py checks this.
JOINT_LIMITS = {
    'joint_1': (-3.14159, 3.14159),
    'joint_2': (-1.5708, 1.5708),
    'joint_3': (-1.5708, 1.5708),
}

# Each entry is (joint_1, joint_2, joint_3) in radians plus a hold time in
# seconds. joint_1 is base yaw, joint_2 shoulder pitch, joint_3 elbow pitch.
DEMO_SEQUENCE = [
    ((0.0, 0.0, 0.0), 3.0),   # home
    ((0.0, 0.9, 0.0), 3.0),   # raise the shoulder
    ((0.0, 0.9, 0.8), 3.0),   # bend the elbow
    ((0.8, 0.9, 0.8), 3.0),   # rotate the base to the left
    ((0.0, 0.0, 0.0), 3.0),   # return home
]

# Maximum per-joint error accepted as "pose reached".
TOLERANCE_RAD = 0.05

# Targets are latched by the controller, so republishing is not required to
# hold a pose. A modest rate guards against messages lost during discovery.
PUBLISH_RATE_HZ = 10.0


def clamp(value, lower, upper):
    """Limit value to the closed interval [lower, upper]."""
    return max(lower, min(upper, value))


class ArmDemo(Node):
    """Steps the arm through DEMO_SEQUENCE and verifies each pose."""

    def __init__(self):
        super().__init__('arm_demo')
        self._cmd_publishers = {
            name: self.create_publisher(Float64, f'/simple_arm/{name}/cmd_pos', 10)
            for name in JOINT_NAMES
        }
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, qos_profile_sensor_data)

        self._positions = {}
        self._step = -1
        self._step_end = None
        self._targets = None
        self._failures = 0
        self.done = False

        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)
        self.get_logger().info('waiting for /joint_states and the Gazebo bridge')

    @property
    def succeeded(self):
        """Return True when every pose was reached within tolerance."""
        return self.done and self._failures == 0

    def _on_joint_states(self, msg):
        self._positions.update(zip(msg.name, msg.position))

    def _ready(self):
        has_feedback = all(name in self._positions for name in JOINT_NAMES)
        bridged = all(p.get_subscription_count() > 0 for p in self._cmd_publishers.values())
        return has_feedback and bridged

    def _tick(self):
        if self.done:
            return
        if self._step < 0:
            if self._ready():
                self.get_logger().info('simulation ready, starting demonstration')
                self._start_step(0)
            return

        self._publish_targets()
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

        self.get_logger().info(
            'pose {}/{}: ({:.2f}, {:.2f}, {:.2f}) rad, holding {:.1f} s'.format(
                index + 1, len(DEMO_SEQUENCE), *self._targets.values(), hold))
        self._publish_targets()

    def _publish_targets(self):
        for name, target in self._targets.items():
            self._cmd_publishers[name].publish(Float64(data=target))

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
