"""
Consistency checks between the lab robot description and its consumers.

Gazebo, RViz and MoveIt all read urdf/lab_so101.urdf.xacro. These tests tie
it to the ros2_control and MoveIt configuration, the world's pieces, the
bridge configuration, the RViz markers and the joint check script.
"""

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from lab_simulation import gripper, layout
from lab_simulation.arm import HOME
from lab_simulation.object_markers import SHAPES
from lab_simulation.so101_kinematics import ARM_JOINTS
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
WORLD = ET.parse(PKG / 'worlds' / 'table_world.sdf').getroot().find('world')
BRIDGE = yaml.safe_load((PKG / 'config' / 'bridge.yaml').read_text())
CONTROLLERS = yaml.safe_load((PKG / 'config' / 'ros2_controllers.yaml').read_text())
MOVEIT = Path(get_package_share_directory('so101_moveit_config')) / 'config'
WORLD_MODELS = {m.get('name'): m for m in WORLD.findall('model')}
DETACHABLE = 'gz::sim::systems::DetachableJoint'
ACTUATED = ARM_JOINTS + ('gripper',)


@pytest.fixture(scope='module')
def robot(lab_description):
    return ET.fromstring(lab_description)


def floats(text):
    return [float(v) for v in text.split()]


def load_move_arm():
    spec = importlib.util.spec_from_file_location('move_arm', PKG / 'scripts' / 'move_arm.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def bridged(ros_topic):
    return next((e for e in BRIDGE if e.get('ros_topic_name') == ros_topic), None)


def test_robot_is_named_like_the_srdf(robot):
    assert robot.get('name') == ET.parse(MOVEIT / 'so101.srdf').getroot().get('name')


def test_arm_stands_on_the_table_top(robot):
    mount = next(j for j in robot.findall('joint') if j.get('name') == 'so101_mount')
    assert mount.find('parent').get('link') == 'world'
    assert floats(mount.find('origin').get('xyz')) == pytest.approx([0, 0, layout.TABLE_TOP])


def test_world_does_not_contain_the_robot():
    """The robot is spawned from robot_description; a second copy would clash."""
    assert 'so101' not in WORLD_MODELS
    assert WORLD.find('include') is None


def test_physics_step_matches_the_controller_rate():
    step = float(WORLD.find('physics/max_step_size').text)
    rate = CONTROLLERS['controller_manager']['ros__parameters']['update_rate']
    assert step * rate == pytest.approx(1.0)


def ros2_control_joints(robot):
    return {j.get('name'): j for j in robot.find('ros2_control').findall('joint')}


def test_ros2_control_drives_every_joint_by_position(robot):
    joints = ros2_control_joints(robot)
    assert tuple(joints) == ACTUATED
    for joint in joints.values():
        assert [c.get('name') for c in joint.findall('command_interface')] == ['position']
        assert {s.get('name') for s in joint.findall('state_interface')} == {
            'position', 'velocity'}


def test_simulation_starts_at_home_with_the_jaws_open(robot):
    initial = {name: float(j.find("state_interface[@name='position']/param").text)
               for name, j in ros2_control_joints(robot).items()}
    assert tuple(initial[n] for n in ARM_JOINTS) == pytest.approx(HOME)
    assert initial['gripper'] == pytest.approx(gripper.OPEN)


def test_controllers_match_the_robot_and_moveit():
    params = CONTROLLERS['controller_manager']['ros__parameters']
    assert params['arm_controller']['type'] == \
        'joint_trajectory_controller/JointTrajectoryController'
    # Not a gripper action controller: those stop squeezing on a stall.
    assert params['gripper_controller']['type'] == \
        'position_controllers/JointGroupPositionController'
    assert tuple(CONTROLLERS['arm_controller']['ros__parameters']['joints']) == ARM_JOINTS
    assert CONTROLLERS['gripper_controller']['ros__parameters']['joints'] == ['gripper']
    moveit = yaml.safe_load((MOVEIT / 'moveit_controllers.yaml').read_text())
    assert tuple(moveit['moveit_simple_controller_manager']['arm_controller']['joints']) == \
        ARM_JOINTS


def test_position_gain_is_raised_from_the_slow_default():
    gain = CONTROLLERS['gz_ros_control']['ros__parameters']['position_proportional_gain']
    assert gain >= 5.0


def test_gripper_targets_are_within_limits(robot):
    limit = next(j for j in robot.findall('joint') if j.get('name') == 'gripper').find('limit')
    lower, upper = float(limit.get('lower')), float(limit.get('upper'))
    assert lower <= gripper.CLOSED < gripper.OPEN <= upper
    assert gripper.CLOSED == pytest.approx(lower, abs=0.01)


def latch_plugins(robot):
    return [p for g in robot.findall('gazebo') for p in g.findall('plugin')
            if p.get('name') == DETACHABLE]


def test_one_grasp_latch_per_piece_on_the_gripper(robot):
    plugins = latch_plugins(robot)
    assert sorted(p.find('child_model').text for p in plugins) == sorted(layout.PIECES)
    for plugin in plugins:
        assert plugin.find('parent_link').text == 'gripper_link'
        model = WORLD_MODELS[plugin.find('child_model').text]
        assert plugin.find('child_link').text == model.find('link').get('name')


@pytest.mark.parametrize('name', sorted(layout.PIECES))
def test_latch_topics_are_bridged(robot, name):
    plugin = next(p for p in latch_plugins(robot) if p.find('child_model').text == name)
    for tag, direction in (('attach_topic', 'ROS_TO_GZ'), ('detach_topic', 'ROS_TO_GZ'),
                           ('output_topic', 'GZ_TO_ROS')):
        entry = bridged(plugin.find(tag).text)
        assert entry is not None and entry['direction'] == direction


@pytest.mark.parametrize('name', sorted(layout.PIECES))
def test_piece_pose_is_published_and_bridged(name):
    model = WORLD_MODELS[name]
    assert model.find("plugin[@name='gz::sim::systems::PosePublisher']") is not None
    entry = bridged(f'/lab/pieces/{name}/pose')
    assert entry['gz_topic_name'] == f'/model/{name}/pose'


def test_reset_service_is_bridged():
    world = WORLD.get('name')
    assert any(e.get('service_name') == f'/world/{world}/set_pose' for e in BRIDGE)


def test_joint_states_come_from_ros2_control_not_the_bridge():
    assert bridged('/joint_states') is None


@pytest.mark.parametrize('name', sorted(layout.PIECES))
def test_piece_geometry_matches_layout(name):
    piece = layout.PIECES[name]
    link = WORLD_MODELS[name].find('link')
    geometry = link.find('collision/geometry')
    if piece.shape == 'cylinder':
        assert float(geometry.find('cylinder/radius').text) == pytest.approx(piece.half_x)
        assert float(geometry.find('cylinder/length').text) == pytest.approx(
            2 * piece.half_height)
    else:
        assert floats(geometry.find('box/size').text) == pytest.approx(
            [2 * piece.half_x, 2 * piece.half_y, 2 * piece.half_height])
    assert float(link.find('inertial/mass').text) == pytest.approx(piece.mass)
    # Resting on the table at start.
    assert floats(WORLD_MODELS[name].find('pose').text)[2] == pytest.approx(piece.centre_z)


@pytest.mark.parametrize('name', sorted(layout.PIECES))
def test_piece_inertia_matches_geometry(name):
    piece = layout.PIECES[name]
    inertia = WORLD_MODELS[name].find('link/inertial/inertia')
    m = piece.mass
    if piece.shape == 'cylinder':
        r, h = piece.half_x, 2 * piece.half_height
        expected = (m * (3 * r * r + h * h) / 12,) * 2 + (m * r * r / 2,)
    else:
        x, y, z = 2 * piece.half_x, 2 * piece.half_y, 2 * piece.half_height
        expected = (m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12)
    actual = [float(inertia.find(k).text) for k in ('ixx', 'iyy', 'izz')]
    assert actual == pytest.approx(expected, rel=1e-3)


def test_pieces_start_inside_the_workspace():
    starts = {n: floats(WORLD_MODELS[n].find('pose').text)[:2] for n in layout.PIECES}
    for name, xy in starts.items():
        assert layout.in_workspace(xy), name
        assert layout.clear_of_bins(xy), name
        for other, other_xy in starts.items():
            if other != name:
                assert math.dist(xy, other_xy) >= layout.MIN_SEPARATION


@pytest.mark.parametrize('name', sorted(layout.PIECES))
def test_piece_colour_matches_the_world(name):
    diffuse = WORLD_MODELS[name].find('link/visual/material/diffuse').text
    assert floats(diffuse)[:3] == pytest.approx(layout.PIECES[name].colour)


@pytest.mark.parametrize('name', sorted(layout.PIECES))
def test_markers_match_pieces(name):
    piece = layout.PIECES[name]
    _, sx, sy, sz, *_ = SHAPES[name]
    assert (sx, sy, sz) == pytest.approx(
        (2 * piece.half_x, 2 * piece.half_y, 2 * piece.half_height))


def test_demo_respects_urdf_limits(robot):
    demo = load_move_arm()
    assert demo.ARM_JOINTS == ARM_JOINTS
    limits = {j.get('name'): j.find('limit') for j in robot.findall('joint')
              if j.find('limit') is not None}
    for name, (lower, upper) in demo.JOINT_LIMITS.items():
        assert lower == pytest.approx(float(limits[name].get('lower')))
        assert upper == pytest.approx(float(limits[name].get('upper')))
    for pose, _ in demo.DEMO_SEQUENCE:
        for name, value in zip(demo.JOINT_NAMES, pose):
            lower, upper = demo.JOINT_LIMITS[name]
            assert lower <= value <= upper


@pytest.mark.parametrize('piece', sorted(layout.BINS))
def test_bins_in_the_world_match_layout(piece):
    bin_ = layout.BINS[piece]
    model = WORLD_MODELS[bin_.name]
    assert model.find('static').text == 'true'
    x, y, z, _, _, yaw = floats(model.find('pose').text)
    assert (x, y, z, yaw) == pytest.approx((*bin_.xy, layout.TABLE_TOP, bin_.bearing), abs=1e-4)
    walls = [v for c in model.findall('link/collision')
             for v in floats(c.find('pose').text)[:3] + floats(c.find('geometry/box/size').text)]
    expected = [v for centre, size in layout.bin_walls(bin_) for v in (*centre, *size)]
    assert walls == pytest.approx(expected, abs=1e-4)
    diffuse = model.find('link/visual/material/diffuse').text
    assert floats(diffuse)[:3] == pytest.approx(layout.PIECES[piece].colour)
