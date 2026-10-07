"""Tests for the planning-scene diffs sent to MoveIt and pose conversion."""

import math

from geometry_msgs.msg import Point, Pose, Quaternion
from lab_simulation import layout
from lab_simulation.scene import HELD_LINK, planning_scene, pose_from_quaternion
from moveit_msgs.msg import CollisionObject
import pytest
from shape_msgs.msg import SolidPrimitive


def poses():
    result = {}
    for i, name in enumerate(sorted(layout.PIECES)):
        pose = Pose()
        pose.position.x, pose.position.y = 0.2, 0.1 * (i - 1)
        pose.position.z = layout.PIECES[name].centre_z
        pose.orientation.w = 1.0
        result[name] = pose
    return result


def world_ids(scene, operation=CollisionObject.ADD):
    return {o.id for o in scene.world.collision_objects if o.operation == operation}


def test_table_and_all_pieces_are_obstacles():
    scene = planning_scene(poses())
    assert scene.is_diff and scene.robot_state.is_diff
    assert world_ids(scene) == {'table', *layout.PIECES}
    table = next(o for o in scene.world.collision_objects if o.id == 'table')
    top = table.primitive_poses[0].position.z + table.primitives[0].dimensions[2] / 2
    # Just below the real table top, so the arm's base box is not in collision.
    assert layout.TABLE_TOP - 0.01 < top < layout.TABLE_TOP


def test_excluded_piece_is_removed_only_if_present():
    scene = planning_scene(poses(), exclude='red_cube', in_world=set(layout.PIECES))
    assert 'red_cube' not in world_ids(scene)
    assert world_ids(scene, CollisionObject.REMOVE) == {'red_cube'}
    fresh = planning_scene(poses(), exclude=list(layout.PIECES), in_world=())
    assert world_ids(fresh) == {'table'}
    assert world_ids(fresh, CollisionObject.REMOVE) == set()


def test_held_piece_is_attached_to_the_gripper():
    scene = planning_scene(poses(), held='green_block')
    [attached] = scene.robot_state.attached_collision_objects
    assert attached.link_name == HELD_LINK
    assert attached.object.id == 'green_block'
    assert 'moving_jaw_so101_v1_link' in attached.touch_links
    assert 'green_block' not in world_ids(scene)


def test_released_piece_is_detached():
    scene = planning_scene(poses(), previously_held='green_block')
    [released] = scene.robot_state.attached_collision_objects
    assert released.object.id == 'green_block'
    assert released.object.operation == CollisionObject.REMOVE
    assert 'green_block' in world_ids(scene)


def test_piece_shapes_match_layout():
    scene = planning_scene(poses())
    shapes = {o.id: o.primitives[0] for o in scene.world.collision_objects}
    cylinder = layout.PIECES['blue_cylinder']
    assert shapes['blue_cylinder'].type == SolidPrimitive.CYLINDER
    assert list(shapes['blue_cylinder'].dimensions) == pytest.approx(
        [2 * cylinder.half_height, cylinder.half_x])
    block = layout.PIECES['green_block']
    assert list(shapes['green_block'].dimensions) == pytest.approx(
        [2 * block.half_x, 2 * block.half_y, 2 * block.half_height])


def test_pose_from_quaternion_yaw_and_tilt():
    half = math.radians(30) / 2
    upright_turned = pose_from_quaternion(Point(x=1.0, y=2.0, z=3.0),
                                          Quaternion(z=math.sin(half), w=math.cos(half)))
    assert upright_turned.xyz == (1.0, 2.0, 3.0)
    assert upright_turned.yaw == pytest.approx(math.radians(30))
    assert upright_turned.tilt_deg == pytest.approx(0.0, abs=1e-6)
    on_its_side = pose_from_quaternion(Point(), Quaternion(x=math.sin(math.pi / 4),
                                                           w=math.cos(math.pi / 4)))
    assert on_its_side.tilt_deg == pytest.approx(90.0)


def test_releasing_and_excluding_a_piece_removes_it_from_the_world():
    """A released piece goes back into MoveIt's world; the open jaws are around it."""
    scene = planning_scene(poses(), exclude='green_block', previously_held='green_block',
                           in_world=())
    [released] = scene.robot_state.attached_collision_objects
    assert released.object.operation == CollisionObject.REMOVE
    assert world_ids(scene, CollisionObject.REMOVE) == {'green_block'}


def test_pieces_keep_their_colour_in_moveit():
    """A held piece is drawn in MoveIt's violet unless it has its own colour."""
    scene = planning_scene(poses(), held='red_cube')
    colours = {c.id: (c.color.r, c.color.g, c.color.b, c.color.a) for c in scene.object_colors}
    for name, piece in layout.PIECES.items():
        assert colours[name] == pytest.approx((*piece.colour, 1.0))
