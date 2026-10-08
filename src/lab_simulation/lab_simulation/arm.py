"""Move the SO-101 arm through MoveIt, and judge when joints have settled."""

import time
from typing import List, Optional, Sequence, Tuple

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .ros_util import sim_now, spin_until
from .so101_kinematics import ARM_JOINTS
from .tasks import MoveResult, SimulationUnresponsive

HOME = (0.0, -1.3, 0.6, 1.6, 0.0)   # SRDF "home"
GROUP = 'arm'
PIPELINE = 'ompl'
PLANNER = 'RRTConnect'
# Planning volume, world frame: the table and well above it.
WORKSPACE = ((-1.0, -1.0, 0.0), (1.0, 1.0, 2.0))
PLANNING_TIME_S = 2.0
VELOCITY_SCALING = 0.5
ACCELERATION_SCALING = 0.5
GOAL_TOLERANCE_RAD = 0.001     # planner goal tolerance per joint
REACHED_TOLERANCE_RAD = 0.02   # measured joint error that counts as arrived
MOVE_TIMEOUT_S = 30.0          # simulation seconds for planning plus execution
# Wall seconds without /joint_states that count as a frozen simulation. Camera
# rendering plus model inference on a loaded CPU can pause the simulation for
# a few seconds without anything being wrong.
FEEDBACK_TIMEOUT_S = 15.0
SERVER_TIMEOUT_S = 60.0        # wall seconds to wait for move_group

# MoveIt error codes that mean no motion was executed because no valid plan
# exists (as opposed to a failure while executing one).
PLANNING_FAILURES = {
    MoveItErrorCodes.PLANNING_FAILED,
    MoveItErrorCodes.INVALID_MOTION_PLAN,
    MoveItErrorCodes.START_STATE_IN_COLLISION,
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS,
    MoveItErrorCodes.GOAL_IN_COLLISION,
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS,
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED,
    MoveItErrorCodes.INVALID_GROUP_NAME,
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS,
    MoveItErrorCodes.INVALID_ROBOT_STATE,
    MoveItErrorCodes.NO_IK_SOLUTION,
}

MIN_MOVE_S = 0.3        # do not judge "settled" before motion has started
STILL_SPEED = 0.02      # rad/s: slower than this a joint counts as still
HOLD_S = 0.3            # joints must stay still this long to count as settled


class SettleDetector:
    """
    Decide when joints have stopped moving, from measured positions only.

    Reported joint velocities are not used: Gazebo has been seen to report a
    stale velocity for a joint whose position is constant. A joint counts as
    still when its position spread over the last HOLD_S is within
    STILL_SPEED * HOLD_S. Samples from the first MIN_MOVE_S after a command
    are ignored, so a move is never judged before it has started.
    """

    def __init__(self, start: float, min_move_s: float = MIN_MOVE_S,
                 hold_s: float = HOLD_S, still_speed: float = STILL_SPEED):
        self._judge_from = start + min_move_s
        self._hold_s = hold_s
        self._tolerance = still_speed * hold_s
        self._samples: List[Tuple[float, Tuple[float, ...]]] = []

    def update(self, now: float, positions: Sequence[float]) -> Optional[float]:
        """Record a sample; return the time the joints became still, else None."""
        if now < self._judge_from - 1e-9:
            return None
        self._samples.append((now, tuple(positions)))
        # Keep the newest sample that is at least hold_s old as the window start.
        while len(self._samples) > 1 and now - self._samples[1][0] >= self._hold_s - 1e-9:
            self._samples.pop(0)
        oldest = self._samples[0][0]
        if now - oldest < self._hold_s - 1e-9:
            return None
        spread = max(max(column) - min(column) for column in zip(*(q for _, q in self._samples)))
        return oldest if spread <= self._tolerance else None


def plan_request(goal: Sequence[float]) -> MotionPlanRequest:
    """Return a MoveIt request to move the arm group to joint angles goal."""
    request = MotionPlanRequest()
    request.group_name = GROUP
    request.pipeline_id = PIPELINE
    request.planner_id = PLANNER
    request.workspace_parameters.header.frame_id = 'world'
    (request.workspace_parameters.min_corner.x, request.workspace_parameters.min_corner.y,
     request.workspace_parameters.min_corner.z) = WORKSPACE[0]
    (request.workspace_parameters.max_corner.x, request.workspace_parameters.max_corner.y,
     request.workspace_parameters.max_corner.z) = WORKSPACE[1]
    request.num_planning_attempts = 1
    request.allowed_planning_time = PLANNING_TIME_S
    request.max_velocity_scaling_factor = VELOCITY_SCALING
    request.max_acceleration_scaling_factor = ACCELERATION_SCALING
    request.start_state.is_diff = True      # start from the current state
    request.goal_constraints = [Constraints(joint_constraints=[
        JointConstraint(joint_name=name, position=float(value),
                        tolerance_above=GOAL_TOLERANCE_RAD,
                        tolerance_below=GOAL_TOLERANCE_RAD, weight=1.0)
        for name, value in zip(ARM_JOINTS, goal)])]
    return request


class JointFeedback:
    """Latest /joint_states, with the wall time it last arrived."""

    def __init__(self, node: Node):
        self.positions = {}
        self._last = None
        node.create_subscription(JointState, '/joint_states', self._on_joint_states,
                                 qos_profile_sensor_data)

    def _on_joint_states(self, msg: JointState) -> None:
        self.positions.update(zip(msg.name, msg.position))
        self._last = time.monotonic()

    def get(self, names: Sequence[str]) -> Optional[Tuple[float, ...]]:
        """Return the named joints' positions, or None before feedback arrives."""
        if not all(name in self.positions for name in names):
            return None
        return tuple(self.positions[name] for name in names)

    def check_alive(self) -> None:
        """Raise SimulationUnresponsive if /joint_states has stopped."""
        if self._last is not None and time.monotonic() - self._last > FEEDBACK_TIMEOUT_S:
            raise SimulationUnresponsive(
                f'no /joint_states for {FEEDBACK_TIMEOUT_S:.0f} s of wall time')


class Arm:
    """Plan and execute arm moves with MoveIt's move_group action."""

    def __init__(self, node: Node, feedback: JointFeedback):
        self._node = node
        self._feedback = feedback
        self._client = ActionClient(node, MoveGroup, '/move_action')

    def wait_for_server(self) -> bool:
        """Wait (wall time) until move_group accepts goals."""
        return self._client.wait_for_server(timeout_sec=SERVER_TIMEOUT_S)

    def joint_positions(self) -> Optional[Tuple[float, ...]]:
        """Return the latest measured arm joint angles, or None before feedback."""
        return self._feedback.get(ARM_JOINTS)

    def _max_error(self, goal: Sequence[float]) -> float:
        current = self.joint_positions()
        return max(abs(a - b) for a, b in zip(current, goal)) if current else float('inf')

    def move_to(self, goal: Sequence[float], timeout_s: float = MOVE_TIMEOUT_S) -> MoveResult:
        """
        Plan a collision-free path to joint angles goal and execute it.

        Returns planned=False if MoveIt found no valid plan; reached=True only
        if execution succeeded and every joint is within
        REACHED_TOLERANCE_RAD of the goal. Times are simulation seconds.
        """
        start = sim_now(self._node)
        request = MoveGroup.Goal(request=plan_request(goal))
        request.planning_options.plan_only = False
        request.planning_options.planning_scene_diff.is_diff = True
        request.planning_options.planning_scene_diff.robot_state.is_diff = True
        sent = self._client.send_goal_async(request)
        if not spin_until(self._node, sent.done, sim_timeout_s=timeout_s):
            return MoveResult(False, False, sim_now(self._node) - start, self._max_error(goal))
        handle = sent.result()
        if not handle.accepted:
            return MoveResult(False, False, sim_now(self._node) - start, self._max_error(goal))
        result = handle.get_result_async()

        def finished() -> bool:
            self._feedback.check_alive()
            return result.done()

        if not spin_until(self._node, finished, sim_timeout_s=timeout_s):
            handle.cancel_goal_async()
            return MoveResult(True, False, sim_now(self._node) - start, self._max_error(goal))
        code = result.result().result.error_code.val
        elapsed = sim_now(self._node) - start
        error = self._max_error(goal)
        if code in PLANNING_FAILURES:
            return MoveResult(False, False, elapsed, error)
        reached = code == MoveItErrorCodes.SUCCESS and error <= REACHED_TOLERANCE_RAD
        return MoveResult(True, reached, elapsed, error)

    def home(self) -> MoveResult:
        """Move to the SRDF home pose."""
        return self.move_to(HOME)
