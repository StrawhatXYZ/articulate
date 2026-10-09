#!/usr/bin/env python3
"""
Command node: sort one piece (or everything) from a typed or spoken instruction.

Send an instruction on /lab/instruction (std_msgs/String); progress and results
appear on /lab/instruction/status. Normally started by launch/demo.launch.py.
"""

from collections import deque
import json
import math
import random
import sys
import time

from lab_simulation import layout, targets
from lab_simulation.arm import Arm, JointFeedback
from lab_simulation.description import robot_description
from lab_simulation.gripper import Gripper
from lab_simulation.instruction import InstructionMatcher
from lab_simulation.ros_util import sim_now, spin_until
from lab_simulation.scene import Scene
from lab_simulation.so101_kinematics import Kinematics
from lab_simulation.tasks import (BIN_RELEASE_RAD, PickPlaceTask, SimulationUnresponsive,
                                  SortTask)
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

HOME_WORDS = ('home', 'go home')
SORT_ALL_WORDS = ('sort everything', 'sort all', 'sort everything out')
TARGET_KINDS = ('bin', 'start', 'near', 'region', 'xy')
SCENE_PERIOD_S = 1.0


def describe_target(target: dict) -> str:
    """Return a short phrase for a plan target."""
    kind = target.get('kind')
    if kind == 'near':
        return f'near {target.get("piece")} side {target.get("side")}'
    if kind == 'region':
        return f'region {target.get("region")}'
    if kind == 'xy':
        return f'xy {target.get("xy")}'
    return str(kind)


def check_step(step) -> str:
    """Return why a plan step is malformed, or an empty string if it is fine."""
    if not isinstance(step, dict):
        return 'step is not an object'
    action = step.get('action')
    if action in ('home', 'sort_all'):
        return ''
    if action != 'move':
        return f'unknown action {action}'
    if step.get('piece') not in layout.PIECES:
        return f'unknown piece {step.get("piece")}'
    target = step.get('to')
    if not isinstance(target, dict) or target.get('kind') not in TARGET_KINDS:
        return 'unknown or missing target'
    kind = target['kind']
    if kind == 'near':
        if target.get('piece') not in layout.PIECES or target['piece'] == step['piece']:
            return f'cannot place near {target.get("piece")}'
        if target.get('side') not in targets.SIDES:
            return f'unknown side {target.get("side")}'
    elif kind == 'region' and target.get('region') not in targets.REGIONS:
        return f'unknown region {target.get("region")}'
    elif kind == 'xy':
        xy = target.get('xy')
        if (not isinstance(xy, (list, tuple)) or len(xy) != 2
                or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in xy)):
            return 'xy must be two numbers'
    return ''


class CommandNode:
    """Listens for instructions and runs them one at a time."""

    def __init__(self):
        self.node = rclpy.create_node(
            'command', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.embedder = self.node.declare_parameter('embedder', 'embeddinggemma').value
        self.perception = self.node.declare_parameter('perception', 'ground_truth').value
        self.speed = self.node.declare_parameter('speed', 1.0).value
        self.log = self.node.get_logger()
        self.status = self.node.create_publisher(String, '/lab/instruction/status', 10)
        self.queue = deque()
        self.busy = False
        self.matcher = InstructionMatcher(self._load_embedder())
        feedback = JointFeedback(self.node)
        self.arm = Arm(self.node, feedback, self.speed)
        self.gripper = Gripper(self.node, feedback)
        self.scene = Scene(self.node, self.perception)
        kinematics = Kinematics(robot_description())
        now = (lambda: sim_now(self.node))
        self.pick = PickPlaceTask(self.arm, self.gripper, self.scene, kinematics, now, 'latched')
        self.sort = SortTask(self.arm, self.gripper, self.scene, kinematics, now, 'latched')
        self.node.create_subscription(String, '/lab/instruction', self._on_instruction, 10)
        self.node.create_subscription(String, '/lab/plan', self._on_plan, 10)
        self.scene_pub = self.node.create_publisher(String, '/lab/scene', 10)
        self.starts = {}
        self.positions = {}
        self.last_scene = 0.0

    def _load_embedder(self):
        if self.embedder != 'embeddinggemma':
            return None
        try:
            from lab_perception.naming import EmbeddingGemmaEmbedder
            return EmbeddingGemmaEmbedder().embed_texts
        except Exception as error:
            self.log.warning(f'EmbeddingGemma unavailable ({error}); using keywords')
            return None

    def say(self, text: str) -> None:
        """Log a status line and publish it."""
        self.log.info(text)
        self.status.publish(String(data=text))

    def _on_instruction(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        if self.busy:
            self.say(f'busy: ignoring "{text}"')
        else:
            self.queue.append(text)

    def _on_plan(self, msg: String) -> None:
        if self.busy or self.queue:
            self.say('busy: ignoring plan')
        else:
            self.queue.append(('plan', msg.data))

    def observe(self):
        """Return the scene observation plus pieces sitting in bins (hidden from vision)."""
        observation = self.scene.observe()
        poses = dict(observation.poses)
        for name in layout.PIECES:
            truth = self.scene.pose(name)
            if name not in poses and truth is not None and any(
                    b.contains(truth.xy) for b in layout.BINS.values()):
                poses[name] = truth
        return observation._replace(poses=poses)

    def refresh_scene(self, observe: bool = True) -> None:
        """Update the known piece positions (when idle) and publish /lab/scene."""
        if observe:
            poses = self.observe().poses
            self.positions = {name: tuple(pose.xy) for name, pose in poses.items()}
            for name, xy in self.positions.items():
                self.starts.setdefault(name, xy)
        pieces = {}
        for name, xy in self.positions.items():
            colour, shape = name.split('_')
            pieces[name] = {
                'xy': [round(xy[0], 4), round(xy[1], 4)],
                'start_xy': [round(v, 4) for v in self.starts[name]],
                'in_bin': layout.BINS[name].contains(xy), 'colour': colour, 'shape': shape}
        bins = {b.name.split('_')[0]: [round(v, 4) for v in b.xy] for b in layout.BINS.values()}
        self.scene_pub.publish(String(data=json.dumps(
            {'busy': self.busy, 'pieces': pieces, 'bins': bins})))
        self.last_scene = time.monotonic()

    def prepare(self) -> bool:
        """Wait for the simulation, let go of everything and go home."""
        try:
            ready = spin_until(
                self.node, lambda: sim_now(self.node) > 0
                and self.arm.joint_positions() is not None and self.gripper.ready()
                and self.scene.has_poses() and self.scene.has_detections(),
                sim_timeout_s=float('inf'), wall_timeout_s=180.0) and self.arm.wait_for_server()
        except SimulationUnresponsive:
            ready = False
        if not ready:
            self.log.error('simulation not running')
            return False
        self.reset_arm()
        self.refresh_scene()
        return True

    def reset_arm(self):
        """Unlatch, open the jaws partly and go home, ignoring the pieces and bins."""
        self.scene.unlatch_all()
        self.gripper.release(BIN_RELEASE_RAD)
        self.scene.update_planning_scene(exclude=list(layout.PIECES), bins=False)
        result = self.arm.home()
        self.scene.update_planning_scene()
        return result

    def check_plan(self, steps: list) -> str:
        """Return why a plan cannot run (e.g. 'step 2/3: ...'), or an empty string."""
        for index, step in enumerate(steps, 1):
            reason = check_step(step)
            if reason:
                return f'step {index}/{len(steps)}: {reason}'
        positions = dict(self.positions)
        for index, step in enumerate(steps, 1):
            if step['action'] == 'sort_all':
                positions.update({name: layout.BINS[name].xy for name in positions})
            elif step['action'] == 'move':
                piece = step['piece']
                if piece not in positions:
                    return f'step {index}/{len(steps)}: {piece} not found'
                xy, note = targets.resolve(step['to'], piece, positions, self.starts)
                if xy is None:
                    return f'step {index}/{len(steps)}: {note}'
                positions[piece] = xy
        return ''

    def run_plan(self, data: str) -> None:
        """Validate a JSON plan as a whole, then run its steps in order."""
        try:
            plan = json.loads(data)
            steps = plan['steps']
            if not isinstance(steps, list) or not steps:
                raise ValueError('steps must be a non-empty list')
        except (ValueError, KeyError, TypeError) as error:
            self.say(f'refused: bad plan ({error})')
            return
        self.say(f'heard: {plan.get("text", "")}')
        self.refresh_scene()
        reason = self.check_plan(steps)
        if reason:
            self.say(f'refused: {reason}')
            return
        total = len(steps)
        self.say(f'plan: {total} steps')
        for index, step in enumerate(steps, 1):
            label = f'step {index}/{total}'
            if step['action'] == 'move':
                self.say(f'{label}: move {step["piece"]} to {describe_target(step["to"])}')
            else:
                self.say(f'{label}: {step["action"]}')
            failure = self.run_step(step)
            self.refresh_scene()
            if failure:
                self.say(f'failed: {label}: {failure}')
                return
            self.say(f'done: {label}')
        self.say('plan done')

    def run_step(self, step: dict) -> str:
        """Run one validated plan step; return a failure description or an empty string."""
        if step['action'] == 'home':
            result = self.reset_arm()
            return '' if result.planned and result.reached else 'could not go home'
        if step['action'] == 'sort_all':
            outcome = self.sort.run(random.Random(0), 0)
            return '' if outcome.success else str(outcome.failure)
        piece = step['piece']
        observation = self.observe()
        if piece not in observation.poses:
            return f'{piece} not found'
        positions = {name: tuple(pose.xy) for name, pose in observation.poses.items()}
        xy, note = targets.resolve(step['to'], piece, positions, self.starts)
        if xy is None:
            return note
        details = {}
        try:
            failure = self.pick._execute(
                layout.PIECES[piece], observation.poses[piece], xy, details,
                aim_jaws=step['to']['kind'] == 'bin', nominal_grip=observation.vision,
                from_bin=layout.BINS[piece].contains(observation.poses[piece].xy))
        finally:
            self.scene.unlatch(piece)
            self.gripper.release(BIN_RELEASE_RAD)
            self.scene.update_planning_scene()
        return f'{failure} (stage {details.get("stage")})' if failure else ''

    def handle(self, text) -> None:
        """Run one instruction (text), or a plan given as ('plan', json)."""
        if isinstance(text, tuple):
            self.run_plan(text[1])
            return
        self.say(f'heard: {text}')
        lowered = text.lower().strip(' .!')
        if lowered in HOME_WORDS:
            result = self.reset_arm()
            self.say('done: home' if result.planned and result.reached else 'failed: home')
            return
        if lowered in SORT_ALL_WORDS:
            self.say('picking everything...')
            outcome = self.sort.run(random.Random(0), 0)
            self.say('done: sorted everything' if outcome.success
                     else f'failed: {outcome.failure}')
            return
        match = self.matcher.match(text)
        if match.piece is None:
            self.say('did not understand')
            return
        self.say(f'target: {match.piece} ({match.method}, score {match.score:.2f})')
        bin_ = layout.BINS[match.piece]
        observation = self.scene.observe()
        if match.piece not in observation.poses:
            self.say(f'failed: {match.piece} not found')
            return
        self.say('picking...')
        details = {}
        try:
            failure = self.pick._execute(
                layout.PIECES[match.piece], observation.poses[match.piece], bin_.xy, details,
                aim_jaws=True, nominal_grip=observation.vision)
        finally:
            self.scene.unlatch(match.piece)
            self.gripper.release(BIN_RELEASE_RAD)
            self.scene.update_planning_scene()
        if failure:
            self.say(f'failed: {failure} (stage {details.get("stage")})')
        else:
            self.say(f'done: {match.piece} in {bin_.name.replace("_", " ")}')
        self.refresh_scene()

    def spin(self) -> None:
        """Serve instructions until shutdown."""
        self.say('ready')
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if not self.queue:
                if time.monotonic() - self.last_scene > SCENE_PERIOD_S:
                    self.refresh_scene(observe=False)
                continue
            self.busy = True
            try:
                self.handle(self.queue.popleft())
            except SimulationUnresponsive as error:
                self.say(f'failed: {error}')
            finally:
                self.busy = False


def main() -> int:
    """Start the node."""
    rclpy.init(args=sys.argv)
    command = CommandNode()
    try:
        if command.prepare():
            command.spin()
    except KeyboardInterrupt:
        pass
    command.node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
