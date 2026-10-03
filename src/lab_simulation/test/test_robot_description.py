"""
Consistency checks between the robot descriptions and their consumers.

Gazebo simulates worlds/simple_arm.sdf while ROS tooling reads
urdf/simple_arm.urdf. These tests fail as soon as the two drift apart, and
also check the contracts that tie the world, the bridge configuration and the
demo script to the model.
"""

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
URDF = ET.parse(PKG / 'urdf' / 'simple_arm.urdf').getroot()
SDF_MODEL = ET.parse(PKG / 'worlds' / 'simple_arm.sdf').getroot().find('model')
WORLD = ET.parse(PKG / 'worlds' / 'table_world.sdf').getroot().find('world')
BRIDGE = yaml.safe_load((PKG / 'config' / 'bridge.yaml').read_text())

ACTUATED = ('joint_1', 'joint_2', 'joint_3')


def floats(text):
    return [float(v) for v in text.split()]


def sdf_xyz(elem):
    """Return the translation of an SDF element's <pose>, or zeros."""
    pose = elem.find('pose')
    if pose is None or not (pose.text or '').strip():
        return [0.0, 0.0, 0.0]
    return floats(pose.text)[:3]


def urdf_xyz(elem):
    """Return the translation of a URDF element's <origin>, or zeros."""
    origin = elem.find('origin')
    if origin is None:
        return [0.0, 0.0, 0.0]
    return floats(origin.get('xyz', '0 0 0'))


def urdf_joints():
    return {j.get('name'): j for j in URDF.findall('joint')}


def sdf_joints():
    return {j.get('name'): j for j in SDF_MODEL.findall('joint')}


def urdf_links():
    return {link.get('name'): link for link in URDF.findall('link')}


def sdf_links():
    return {link.get('name'): link for link in SDF_MODEL.findall('link')}


def load_move_arm():
    spec = importlib.util.spec_from_file_location(
        'move_arm', PKG / 'scripts' / 'move_arm.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_same_joints_and_links():
    assert set(urdf_joints()) == set(sdf_joints())
    assert set(urdf_links()) - {'world'} == set(sdf_links())


@pytest.mark.parametrize('name', sorted(set(urdf_joints()) - {'world_joint'}))
def test_joint_matches(name):
    u, s = urdf_joints()[name], sdf_joints()[name]
    assert u.get('type') == s.get('type')
    assert u.find('parent').get('link') == s.find('parent').text
    assert u.find('child').get('link') == s.find('child').text
    assert urdf_xyz(u) == pytest.approx(sdf_xyz(s))

    if u.get('type') == 'revolute':
        assert floats(u.find('axis').get('xyz')) == floats(s.find('axis/xyz').text)
        limit = s.find('axis/limit')
        for key in ('lower', 'upper', 'effort', 'velocity'):
            assert float(u.find('limit').get(key)) == pytest.approx(
                float(limit.find(key).text)), key
        dyn = s.find('axis/dynamics')
        for key in ('damping', 'friction'):
            assert float(u.find('dynamics').get(key)) == pytest.approx(
                float(dyn.find(key).text)), key


@pytest.mark.parametrize('name', sorted(set(urdf_links()) - {'world'}))
def test_link_matches(name):
    u, s = urdf_links()[name], sdf_links()[name]

    ui, si = u.find('inertial'), s.find('inertial')
    assert float(ui.find('mass').get('value')) == pytest.approx(float(si.find('mass').text))
    assert urdf_xyz(ui) == pytest.approx(sdf_xyz(si))
    for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'):
        assert float(ui.find('inertia').get(key)) == pytest.approx(
            float(si.find(f'inertia/{key}').text)), key

    for kind in ('visual', 'collision'):
        ue, se = u.find(kind), s.find(kind)
        assert urdf_xyz(ue) == pytest.approx(sdf_xyz(se)), kind
        ug, sg = ue.find('geometry')[0], se.find('geometry')[0]
        assert ug.tag == sg.tag, kind
        if ug.tag == 'box':
            assert floats(ug.get('size')) == pytest.approx(floats(sg.find('size').text))
        elif ug.tag == 'cylinder':
            assert float(ug.get('radius')) == pytest.approx(float(sg.find('radius').text))
            assert float(ug.get('length')) == pytest.approx(float(sg.find('length').text))
        elif ug.tag == 'sphere':
            assert float(ug.get('radius')) == pytest.approx(float(sg.find('radius').text))


@pytest.mark.parametrize('name', sorted(set(urdf_links()) - {'world'}))
def test_inertia_matches_geometry(name):
    """Inertia must be that of the solid collision primitive about its centre."""
    link = urdf_links()[name]
    mass = float(link.find('inertial/mass').get('value'))
    geom = link.find('collision/geometry')[0]
    if geom.tag == 'box':
        x, y, z = floats(geom.get('size'))
        expected = (mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12,
                    mass * (x * x + y * y) / 12)
    elif geom.tag == 'cylinder':
        r, h = float(geom.get('radius')), float(geom.get('length'))
        side = mass * (3 * r * r + h * h) / 12
        expected = (side, side, mass * r * r / 2)
    else:
        r = float(geom.get('radius'))
        expected = (2 * mass * r * r / 5,) * 3
    inertia = link.find('inertial/inertia')
    actual = tuple(float(inertia.get(k)) for k in ('ixx', 'iyy', 'izz'))
    assert actual == pytest.approx(expected, rel=1e-3)


def test_arm_placement_matches_world():
    include = next(i for i in WORLD.findall('include')
                   if i.find('name').text == 'simple_arm')
    assert urdf_xyz(urdf_joints()['world_joint']) == pytest.approx(sdf_xyz(include))
    assert SDF_MODEL.find('pose') is None, 'placement belongs in the world <include>'


def test_controller_topics_are_bridged():
    ros_to_gz = {(e['ros_topic_name'], e['gz_topic_name']) for e in BRIDGE
                 if e['direction'] == 'ROS_TO_GZ'}
    controllers = [p for p in SDF_MODEL.findall('plugin')
                   if p.get('name') == 'gz::sim::systems::JointPositionController']
    assert sorted(p.find('joint_name').text for p in controllers) == list(ACTUATED)
    for plugin in controllers:
        topic = plugin.find('topic').text
        assert topic == f'/simple_arm/{plugin.find("joint_name").text}/cmd_pos'
        assert (topic, topic) in ros_to_gz


def test_controller_effort_clamped_to_limit():
    for plugin in SDF_MODEL.findall('plugin'):
        if plugin.get('name') != 'gz::sim::systems::JointPositionController':
            continue
        joint = sdf_joints()[plugin.find('joint_name').text]
        effort = float(joint.find('axis/limit/effort').text)
        assert float(plugin.find('cmd_max').text) == pytest.approx(effort)
        assert float(plugin.find('cmd_min').text) == pytest.approx(-effort)


def test_joint_states_bridged_from_world():
    entry = next(e for e in BRIDGE if e['ros_topic_name'] == '/joint_states')
    world = WORLD.get('name')
    assert entry['gz_topic_name'] == f'/world/{world}/model/simple_arm/joint_state'


def test_demo_respects_urdf_limits():
    move_arm = load_move_arm()
    assert tuple(move_arm.JOINT_NAMES) == ACTUATED
    for name, (lower, upper) in move_arm.JOINT_LIMITS.items():
        limit = urdf_joints()[name].find('limit')
        assert lower == pytest.approx(float(limit.get('lower')))
        assert upper == pytest.approx(float(limit.get('upper')))
    for pose, hold in move_arm.DEMO_SEQUENCE:
        assert hold > 0
        for name, value in zip(move_arm.JOINT_NAMES, pose):
            lower, upper = move_arm.JOINT_LIMITS[name]
            assert lower <= value <= upper, (name, value)
            assert math.isfinite(value)


def test_clamp():
    clamp = load_move_arm().clamp
    assert clamp(2.0, -1.0, 1.0) == 1.0
    assert clamp(-2.0, -1.0, 1.0) == -1.0
    assert clamp(0.5, -1.0, 1.0) == 0.5
