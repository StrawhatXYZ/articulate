"""Piece poses, scene reset, the grasp latch and MoveIt's planning scene."""

from functools import partial
import math
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

from . import layout
from .ros_util import spin_for, spin_until
from .tasks import PiecePose

# One pose topic per piece, bridged from each piece's PosePublisher.
POSE_TOPIC = '/lab/pieces/{name}/pose'
LATCH_TOPIC = '/lab/latch/{name}/{verb}'
SET_POSE_SERVICE = '/world/table_world/set_pose'
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


def planning_scene(poses: Dict[str, Pose], exclude: Names = None,
                   held: Optional[str] = None, previously_held: Optional[str] = None,
                   in_world: Collection[str] = (), bins: bool = True,
                   bins_in_world: bool = False) -> PlanningScene:
    """
    Return a planning-scene diff: the table, the pieces, and a held piece.

    exclude leaves pieces out (one the open jaws are around, or all of them
    while the scene is being reset); held attaches a piece to the gripper so
    planning carries it along. in_world lists the pieces the scene already
    holds as world objects; only those are removed. MoveIt puts a detached
    object back into the world, so a piece being released counts as present.
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
    for name in sorted(poses):
        obj = CollisionObject(id=name)
        obj.header.frame_id = FRAME
        if name in exclude:
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

    def __init__(self, node: Node):
        self._node = node
        self._poses: Dict[str, Pose] = {}
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
        poses = {name: self._poses.get(name, Pose()) for name in layout.PIECES}
        request = ApplyPlanningScene.Request(
            scene=planning_scene(poses, exclude, held, self._held, self._in_world, bins,
                                 self._bins_in_world))
        self._held = held
        self._bins_in_world = bins
        self._in_world = {n for n in poses if n not in _names(exclude) and n != held}
        if not self._apply_scene.wait_for_service(timeout_sec=10.0):
            return False
        future = self._apply_scene.call_async(request)
        spin_until(self._node, future.done, sim_timeout_s=10.0)
        return future.done() and future.result() is not None and future.result().success

    def _place(self, name: str, x: float, y: float) -> bool:
        request = SetEntityPose.Request()
        request.entity = Entity(name=name, type=Entity.MODEL)
        request.pose = Pose()
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = layout.PIECES[name].centre_z
        request.pose.orientation.w = 1.0
        if not self._set_pose.wait_for_service(timeout_sec=5.0):
            return False
        future = self._set_pose.call_async(request)
        spin_until(self._node, future.done, sim_timeout_s=10.0)
        return future.done() and future.result() is not None and future.result().success

    def reset(self, positions: Dict[str, Tuple[float, float]]) -> bool:
        """
        Teleport every piece to positions, upright, and wait until they rest.

        Teleporting keeps a body's velocity, so a piece knocked into rolling
        arrives still rolling. Rolling does die out through friction, so the
        pieces are teleported back to their targets until they come to rest.
        """
        for _ in range(RESET_ATTEMPTS):
            for name, (x, y) in positions.items():
                if not self._place(name, x, y):
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
