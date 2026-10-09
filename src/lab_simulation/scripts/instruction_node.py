#!/usr/bin/env python3
"""
Serve /lab/instruct: a typed or spoken instruction in, robot actions out.

Builds the Instructor from the live robot (arm, gripper, scene, pick skill),
the placement memory and the Ollama planner. One goal at a time: a goal sent
while busy is rejected. Cancelling takes effect between steps.

Two nodes: the robot node is spun only by the robot code (the tasks spin it
while they wait, as in the benchmark); the action server has its own node
and multi-threaded executor, so goals, cancel requests and feedback are
handled while a plan runs, without spinning the robot node re-entrantly.
"""

from pathlib import Path
import sys
import threading

from lab_interfaces.action import Instruct
from lab_simulation.arm import Arm, JointFeedback
from lab_simulation.description import robot_description
from lab_simulation.executor import Executor
from lab_simulation.gripper import Gripper
from lab_simulation.instructor import Instructor
from lab_simulation.memory import PlacementMemory
from lab_simulation.planner import DEFAULT_MODEL, DEFAULT_URL, Planner
from lab_simulation.ros_util import sim_now, spin_until
from lab_simulation.scene import Scene
from lab_simulation.so101_kinematics import Kinematics
from lab_simulation.tasks import PickPlaceTask
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter


def main() -> int:
    """Start the node and serve goals until shutdown."""
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('instruction',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    perception = node.declare_parameter('perception', 'vision').value
    planner = Planner(node.declare_parameter('ollama_url', DEFAULT_URL).value,
                      node.declare_parameter('model', DEFAULT_MODEL).value)
    log_path = Path(node.declare_parameter('log', '/workspace/results/instructions.jsonl').value)
    feedback = JointFeedback(node)
    arm, gripper, scene = Arm(node, feedback), Gripper(node, feedback), Scene(node, perception)
    kinematics = Kinematics(robot_description())
    pick = PickPlaceTask(arm, gripper, scene, kinematics, lambda: sim_now(node), 'latched')
    executor = Executor(pick, scene, gripper, arm, PlacementMemory())
    instructor = Instructor(planner, executor, log_path)
    log = node.get_logger()

    ready = spin_until(node, lambda: sim_now(node) > 0 and arm.joint_positions() is not None
                       and gripper.ready() and scene.has_poses() and scene.has_detections(),
                       sim_timeout_s=float('inf'), wall_timeout_s=180.0) and arm.wait_for_server()
    if not ready:
        log.error('simulation not running')
        return 1
    executor.reset_arm()
    executor.observe()
    problem = planner.check_ready()
    if problem:
        log.warning(problem)
    else:
        threading.Thread(target=planner.plan, args=('go home', executor.memory.states()),
                         daemon=True).start()      # warm up: loads the model

    def on_goal(_request):
        return GoalResponse.ACCEPT if instructor.try_begin() else GoalResponse.REJECT

    def execute(handle):
        try:
            def send(stage, message):
                handle.publish_feedback(Instruct.Feedback(stage=stage, message=message))
                log.info(f'{stage}: {message}')
            result = instructor.run(handle.request.text, send,
                                    lambda: handle.is_cancel_requested)
        finally:
            instructor.end()
        if result.outcome == 'cancelled':
            handle.canceled()
        else:
            handle.succeed()
        log.info(f'{result.outcome}: {result.message}')
        return Instruct.Result(outcome=result.outcome, message=result.message,
                               steps=result.steps)

    server_node = rclpy.create_node('instruction_server')
    ActionServer(server_node, Instruct, '/lab/instruct', execute_callback=execute,
                 goal_callback=on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                 callback_group=ReentrantCallbackGroup())
    server = MultiThreadedExecutor()
    server.add_node(server_node)
    log.info('ready')
    try:
        server.spin()
    except KeyboardInterrupt:
        pass
    server.shutdown()
    server_node.destroy_node()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
