"""
Piece poses, observing them, scene reset, the grasp latch and MoveIt's planning scene.

In vision mode the robot's view of the pieces comes from the perception
node's detections; the simulator's true poses are kept for scoring.
"""

from functools import partial
import math
import subprocess
import time
from typing import Collection, Dict, Optional, Sequence, Set, Tuple, Union

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Empty, String
from vision_msgs.msg import Detection3DArray

from . import layout
from .perception_eval import evaluate
from .ros_util import sim_now, spin_for, spin_until
from .tasks import Observation, PiecePose, SimulationUnresponsive

# One pose topic per piece, bridged from each piece's PosePublisher.
POSE_TOPIC = '/lab/pieces/{name}/pose'
LATCH_TOPIC = '/lab/latch/{name}/{verb}'
SET_POSE_SERVICE = '/world/table_world/set_pose'
LIGHT_SERVICE = '/world/table_world/light_config'
DETECTIONS_TOPIC = '/lab/perception/objects'
REQUEST_TOPIC = '/lab/perception/request'
PERCEPTION_MODES = ('ground_truth', 'vision')
OBSERVE_TIMEOUT_S = 20.0    # simulation seconds to wait for a fresh detection
SETTLE_SPEED = 0.001    # m/s: below this a piece counts as at rest
SETTLE_SAMPLE_S = 0.1
LATCH_ACK_S = 2.0       # simulation seconds to wait for a latch state change
RESET_ATTEMPTS = 5      # teleports per reset while a piece is still moving
RESET_SETTLE_S = 1.0    # wait per attempt for the pieces to come to rest

# Planning scene. The arm's base box sits on the table top, so MoveIt's table
# is 5 mm lower than the real one to keep the base out of collision with it.
FRAME = 'world'
TABLE_SIZE = (1.0, 1.0, 0.05)
TABLE_TOP_FOR_PLANNING = layout.TABLE_TOP - 0.005
HELD_LINK = 'gripper_link'
TOUCH_LINKS = ['gripper_link', 'moving_jaw_so101_v1_link']


def pose_from_quaternion(position, q) -> PiecePose:
    """Return a PiecePose from a position and an orientation quaternion."""
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    # Tilt: angle between the piece's z axis and the world's.
    up_z = 1 - 2 * (q.x * q.x + q.y * q.y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    return PiecePose(position.x, position.y, position.z, yaw, tilt)


def piece_primitive(name: str) -> SolidPrimitive:
    """Return the collision shape of a piece."""
    piece = layout.PIECES[name]
    if piece.shape == 'cylinder':
        return SolidPrimitive(type=SolidPrimitive.CYLINDER,
                              dimensions=[2 * piece.half_height, piece.half_x])
    return SolidPrimitive(type=SolidPrimitive.BOX,
                          dimensions=[2 * piece.half_x, 2 * piece.half_y, 2 * piece.half_height])


Names = Union[None, str, Collection[str]]


def _names(value: Names) -> Collection[str]:
    if value is None:
        return ()
    return (value,) if isinstance(value, str) else value


def bin_object(bin_: layout.Bin) -> CollisionObject:
    """Return a sorting bin's walls as one collision object."""
    obj = CollisionObject(id=bin_.name, operation=CollisionObject.ADD)
    obj.header.frame_id = FRAME
    c, s = math.cos(bin_.bearing), math.sin(bin_.bearing)
    for (x, y, z), size in layout.bin_walls(bin_):
        obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))
        pose = Pose()
        pose.position.x = bin_.xy[0] + c * x - s * y
        pose.position.y = bin_.xy[1] + s * x + c * y
        pose.position.z = layout.TABLE_TOP + z
        pose.orientation.z = math.sin(bin_.bearing / 2)
        pose.orientation.w = math.cos(bin_.bearing / 2)
        obj.primitive_poses.append(pose)
    return obj


def present_pieces(poses: Dict[str, Pose], exclude: Names = None,
                   held: Optional[str] = None) -> Set[str]:
    """Return the pieces a planning_scene diff leaves in MoveIt's world."""
    exclude = _names(exclude)
    return {n for n in poses if n in layout.PIECES and n not in exclude and n != held}


def planning_scene(poses: Dict[str, Pose], exclude: Names = None,
                   held: Optional[str] = None, previously_held: Optional[str] = None,
                   in_world: Collection[str] = (), bins: bool = True,
                   bins_in_world: bool = False) -> PlanningScene:
    """
    Return a planning-scene diff: the table, the pieces, and a held piece.

    exclude leaves pieces out (one the open jaws are around, or all of them
    while the scene is being reset); held attaches a piece to the gripper so
    planning carries it along. Pieces missing from poses (not seen) are left
    out too. in_world lists the pieces MoveIt already holds as world objects;
    any of those not added again is removed, so no ghost of an unseen piece
    stays behind. MoveIt puts a detached object back into the world, so a
    piece being released counts as held by MoveIt.
    bins=False leaves the sorting bins out (removing them if bins_in_world),
    so an arm stuck inside a bin can still plan its way home.
    """
    exclude = _names(exclude)
    in_world = set(in_world)
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    table = CollisionObject(id='table', operation=CollisionObject.ADD)
    table.header.frame_id = FRAME
    table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(TABLE_SIZE))]
    table_pose = Pose()
    table_pose.position.z = TABLE_TOP_FOR_PLANNING - TABLE_SIZE[2] / 2
    table_pose.orientation.w = 1.0
    table.primitive_poses = [table_pose]
    scene.world.collision_objects.append(table)
    for bin_ in layout.BINS.values():
        if bins:
            scene.world.collision_objects.append(bin_object(bin_))
        elif bins_in_world:
            scene.world.collision_objects.append(
                CollisionObject(id=bin_.name, operation=CollisionObject.REMOVE))
    if previously_held is not None and previously_held != held:
        released = AttachedCollisionObject(link_name=HELD_LINK)
        released.object.id = previously_held
        released.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects.append(released)
        in_world.add(previously_held)
    for name in sorted(layout.PIECES):
        obj = CollisionObject(id=name)
        obj.header.frame_id = FRAME
        if name in exclude or name not in poses:
            if name in in_world:
                obj.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(obj)
            continue
        obj.operation = CollisionObject.ADD
        # Without its own colour, RViz draws a held piece in MoveIt's
        # attached-object violet.
        r, g, b = layout.PIECES[name].colour
        scene.object_colors.append(ObjectColor(id=name, color=ColorRGBA(r=r, g=g, b=b, a=1.0)))
        obj.primitives = [piece_primitive(name)]
        obj.primitive_poses = [poses[name]]
        if name == held:
            scene.robot_state.attached_collision_objects.append(AttachedCollisionObject(
                link_name=HELD_LINK, object=obj, touch_links=TOUCH_LINKS))
        else:
            scene.world.collision_objects.append(obj)
    return scene


class Scene:
    """Read piece poses, reset the layout, latch pieces, sync MoveIt's scene."""

    def __init__(self, node: Node, perception: str = 'ground_truth'):
        if perception not in PERCEPTION_MODES:
            raise ValueError(f'perception must be one of {PERCEPTION_MODES}')
        self._node = node
        self.perception = perception
        self._poses: Dict[str, Pose] = {}
        # What planning believes: the true poses, or the last observation's.
        self._planning: Dict[str, Pose] = {}
        self._detections: Optional[Detection3DArray] = None
        if perception == 'vision':
            node.create_subscription(Detection3DArray, DETECTIONS_TOPIC,
                                     self._on_detections, 10)
            self._request = node.create_publisher(Empty, REQUEST_TOPIC, 10)
        for name in layout.PIECES:
            node.create_subscription(PoseStamped, POSE_TOPIC.format(name=name),
                                     partial(self._on_pose, name), qos_profile_sensor_data)
        self._set_pose = node.create_client(SetEntityPose, SET_POSE_SERVICE)
        self._apply_scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._held: Optional[str] = None
        self._in_world: Set[str] = set()   # pieces MoveIt has as world objects
        self._bins_in_world = False
        self._attach = {n: node.create_publisher(
            Empty, LATCH_TOPIC.format(name=n, verb='attach'), 10) for n in layout.PIECES}
        self._detach = {n: node.create_publisher(
            Empty, LATCH_TOPIC.format(name=n, verb='detach'), 10) for n in layout.PIECES}
        # Last state each latch reported since our last command; None until then.
        self._latch: Dict[str, Optional[str]] = {n: None for n in layout.PIECES}
        # Pieces this node has latched and not yet released.
        self._latched: Set[str] = set()
        for name in layout.PIECES:
            node.create_subscription(String, LATCH_TOPIC.format(name=name, verb='state'),
                                     partial(self._on_latch, name), 10)

    def _on_pose(self, name: str, msg: PoseStamped) -> None:
        self._poses[name] = msg.pose

    def _on_detections(self, msg: Detection3DArray) -> None:
        self._detections = msg

    def has_detections(self) -> bool:
        """Return True once perception is listening (always True without vision)."""
        return self.perception != 'vision' or self._request.get_subscription_count() > 0

    def observe(self, timeout_s: float = OBSERVE_TIMEOUT_S) -> Observation:
        """
        Return what the robot believes about the pieces now.

        Ground truth: the simulator's poses. Vision: the detections of the
        first camera frame taken after this call, so a frame from before the
        last move is never used. Each label is kept once (best score);
        unknown objects are ignored. Raises SimulationUnresponsive if no
        fresh detections arrive.
        """
        if self.perception == 'ground_truth':
            self._planning = dict(self._poses)
            return Observation({n: self.pose(n) for n in layout.PIECES if n in self._poses},
                               'ground_truth')
        from lab_perception.messages import detections_from_msg
        requested, wall = sim_now(self._node), time.monotonic()

        def fresh() -> bool:
            stamp = self._detections.header.stamp if self._detections else None
            return stamp is not None and stamp.sec + stamp.nanosec * 1e-9 > requested

        def answered() -> bool:
            # Re-asked while waiting: cheap, and immune to a lost request.
            if not fresh():
                self._request.publish(Empty())
                spin_for(self._node, 0.05)
            return fresh()

        if not spin_until(self._node, answered, sim_timeout_s=timeout_s):
            raise SimulationUnresponsive(f'no detections for {timeout_s:.0f} s of sim time')
        detections = detections_from_msg(self._detections)
        best = {}
        for d in detections:
            if d.label in layout.PIECES and (d.label not in best or d.score > best[d.label].score):
                best[d.label] = d
        poses = {label: PiecePose(*d.centroid, d.yaw, 0.0) for label, d in best.items()}
        self._planning = {}
        for label, d in best.items():
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = d.centroid
            pose.orientation.z, pose.orientation.w = math.sin(d.yaw / 2), math.cos(d.yaw / 2)
            self._planning[label] = pose
        # Score only pieces on the open table: sorted ones sit in a bin, which
        # perception masks out by design.
        truth = {n: (p.position.x, p.position.y) for n, p in self._poses.items()
                 if not any(b.contains((p.position.x, p.position.y))
                            for b in layout.BINS.values())}
        evaluation = evaluate([(d.label, d.centroid[:2]) for d in detections], truth)
        return Observation(poses, 'vision', evaluation, sim_now(self._node) - requested,
                           time.monotonic() - wall)

    def _on_latch(self, name: str, msg: String) -> None:
        self._latch[name] = msg.data

    def has_poses(self) -> bool:
        """Return True once every piece's pose has been received."""
        return all(name in self._poses for name in layout.PIECES)

    def pose(self, name: str) -> Optional[PiecePose]:
        """Return the latest pose of a piece's centre."""
        pose = self._poses.get(name)
        return None if pose is None else pose_from_quaternion(pose.position, pose.orientation)

    def _command_latch(self, names: Sequence[str], state: str,
                       confirm: Collection[str]) -> bool:
        """
        Command the latch on names; wait until those in confirm report state.

        A command is not assumed to have taken effect after a fixed delay: with
        the simulation running faster than real time, a late message would
        engage the latch after the arm has already moved on. Nor is an old
        report trusted: the plugins report only changes, and this node may
        have heard the start-up "attached" but not the release that followed.
        A plugin already in the commanded state reports nothing, so only
        changes this node expects are waited for.
        """
        topics = self._attach if state == 'attached' else self._detach
        for name in names:
            self._latch[name] = None
            topics[name].publish(Empty())
        return spin_until(self._node, lambda: all(self._latch[n] == state for n in confirm),
                          sim_timeout_s=LATCH_ACK_S)

    def latch(self, name: str) -> bool:
        """Lock a gripped piece to the gripper; True once confirmed."""
        ok = self._command_latch([name], 'attached', confirm=[name])
        if ok:
            self._latched.add(name)
        return ok

    def unlatch(self, name: str) -> None:
        """Release the latch on a piece (waiting for it if this node latched it)."""
        self._command_latch([name], 'detached', confirm=[name] if name in self._latched else [])
        self._latched.discard(name)

    def unlatch_all(self) -> None:
        """Release every latch, including any left engaged at start-up."""
        names = list(layout.PIECES)
        self._command_latch(names, 'detached', confirm=[n for n in names if n in self._latched])
        self._latched.clear()

    def update_planning_scene(self, exclude: Names = None, held: Optional[str] = None,
                              bins: bool = True) -> bool:
        """Push the current piece poses to MoveIt's planning scene (see planning_scene)."""
        # MoveIt sees what the robot believes: in vision mode, pieces it has
        # not detected are not obstacles. A held piece is placed where it is
        # believed to be when grasped.
        source = self._planning if self.perception == 'vision' else self._poses
        poses = {name: pose for name, pose in source.items() if name in layout.PIECES}
        request = ApplyPlanningScene.Request(
            scene=planning_scene(poses, exclude, held, self._held, self._in_world, bins,
                                 self._bins_in_world))
        self._held = held
        self._bins_in_world = bins
        self._in_world = present_pieces(poses, exclude, held)
        if not self._apply_scene.wait_for_service(timeout_sec=10.0):
            return False
        future = self._apply_scene.call_async(request)
        spin_until(self._node, future.done, sim_timeout_s=10.0)
        return future.done() and future.result() is not None and future.result().success

    def _place(self, name: str, x: float, y: float, yaw: float = 0.0) -> bool:
        request = SetEntityPose.Request()
        request.entity = Entity(name=name, type=Entity.MODEL)
        request.pose = Pose()
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = layout.PIECES[name].centre_z
        request.pose.orientation.z = math.sin(yaw / 2)
        request.pose.orientation.w = math.cos(yaw / 2)
        if not self._set_pose.wait_for_service(timeout_sec=5.0):
            return False
        future = self._set_pose.call_async(request)
        spin_until(self._node, future.done, sim_timeout_s=10.0)
        return future.done() and future.result() is not None and future.result().success

    def set_light(self, direction: Sequence[float], brightness: float) -> bool:
        """Point the sun and set its brightness (Gazebo light_config service)."""
        dx, dy, dz = direction
        request = (f'name: "sun", type: DIRECTIONAL, direction: {{x: {dx}, y: {dy}, z: {dz}}},'
                   f' diffuse: {{r: {brightness}, g: {brightness}, b: {brightness}, a: 1}},'
                   f' specular: {{r: 0.2, g: 0.2, b: 0.2, a: 1}}, cast_shadows: true')
        command = ['gz', 'service', '-s', LIGHT_SERVICE, '--reqtype', 'gz.msgs.Light',
                   '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', request]
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=15)
        except (OSError, subprocess.SubprocessError):
            return False
        return 'true' in result.stdout

    def reset(self, positions: Dict[str, Tuple[float, float]],
              yaws: Optional[Dict[str, float]] = None) -> bool:
        """
        Teleport every piece to positions (turned to yaws), upright; wait until they rest.

        Teleporting keeps a body's velocity, so a piece knocked into rolling
        arrives still rolling. Rolling does die out through friction, so the
        pieces are teleported back to their targets until they come to rest.
        """
        for _ in range(RESET_ATTEMPTS):
            for name, (x, y) in positions.items():
                if not self._place(name, x, y, (yaws or {}).get(name, 0.0)):
                    return False
            if self.wait_settled(RESET_SETTLE_S):
                return True
        return False

    def wait_settled(self, timeout_s: float = 3.0) -> bool:
        """Wait until every piece moves slower than SETTLE_SPEED."""
        def xyz(pose):
            return (pose.position.x, pose.position.y, pose.position.z)

        waited = 0.0
        previous = {n: xyz(p) for n, p in self._poses.items()}
        while waited < timeout_s:
            spin_for(self._node, SETTLE_SAMPLE_S)
            waited += SETTLE_SAMPLE_S
            current = {n: xyz(p) for n, p in self._poses.items()}
            if all(math.dist(current[n], previous[n]) <= SETTLE_SPEED * SETTLE_SAMPLE_S
                   for n in layout.PIECES if n in current and n in previous):
                return True
            previous = current
        return False
