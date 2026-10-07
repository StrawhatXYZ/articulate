"""Consistency of the MoveIt configuration with the SO-101 description."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

PKG = Path(__file__).resolve().parents[1]
SRDF = ET.parse(PKG / 'config' / 'so101.srdf').getroot()
URDF = ET.fromstring(xacro.process_file(str(
    Path(get_package_share_directory('so101_description')) / 'urdf' / 'so101.urdf.xacro'))
    .toxml())
LINKS = {link.get('name') for link in URDF.findall('link')}
JOINTS = {j.get('name'): j for j in URDF.findall('joint')}
ARM = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


def load(name):
    return yaml.safe_load((PKG / 'config' / name).read_text())


def test_robot_name_matches():
    assert SRDF.get('name') == URDF.get('name')


def test_arm_chain_ends_at_the_grasp_frame():
    chain = SRDF.find("group[@name='arm']/chain")
    assert chain.get('base_link') == 'base_link'
    assert chain.get('tip_link') == 'grasp_link'


def test_gripper_group_is_the_gripper_joint():
    assert [j.get('name') for j in SRDF.findall("group[@name='gripper']/joint")] == ['gripper']


@pytest.mark.parametrize('state', SRDF.findall('group_state'), ids=lambda s: s.get('name'))
def test_named_states_are_within_limits(state):
    for joint in state.findall('joint'):
        limit = JOINTS[joint.get('name')].find('limit')
        assert float(limit.get('lower')) <= float(joint.get('value')) <= float(limit.get('upper'))


def test_home_state_sets_every_arm_joint():
    home = SRDF.find("group_state[@name='home']")
    assert sorted(j.get('name') for j in home.findall('joint')) == sorted(ARM)


def test_disabled_collision_pairs_reference_real_links():
    pairs = SRDF.findall('disable_collisions')
    assert pairs
    for pair in pairs:
        assert pair.get('link1') in LINKS and pair.get('link2') in LINKS
        assert pair.get('reason') in ('Adjacent', 'Never', 'Default')


def test_adjacent_links_are_disabled():
    disabled = {frozenset((p.get('link1'), p.get('link2')))
                for p in SRDF.findall('disable_collisions')}
    colliding = {link.get('name') for link in URDF.findall('link')
                 if link.find('collision') is not None}
    parent = {j.find('child').get('link'): j.find('parent').get('link')
              for j in JOINTS.values()}
    for link in colliding:
        up = parent.get(link)
        while up is not None and up not in colliding:
            up = parent.get(up)
        if up is not None:
            assert frozenset((link, up)) in disabled


def test_end_effector_references_real_links_and_groups():
    ee = SRDF.find('end_effector')
    assert ee.get('parent_link') in LINKS
    groups = {g.get('name') for g in SRDF.findall('group')}
    assert ee.get('group') in groups and ee.get('parent_group') in groups


def test_trajectory_controller_drives_the_arm_joints():
    controllers = load('moveit_controllers.yaml')['moveit_simple_controller_manager']
    assert controllers['controller_names'] == ['arm_controller']
    arm = controllers['arm_controller']
    assert arm['type'] == 'FollowJointTrajectory'
    assert arm['joints'] == ARM


def test_every_joint_has_planning_limits_no_faster_than_the_urdf():
    limits = load('joint_limits.yaml')['joint_limits']
    for name in ARM + ['gripper']:
        assert limits[name]['has_velocity_limits']
        assert limits[name]['max_velocity'] <= float(JOINTS[name].find('limit').get('velocity'))
        assert limits[name]['has_acceleration_limits']


def test_kinematics_plugin_for_the_arm():
    assert load('kinematics.yaml')['arm']['kinematics_solver'] == 'pick_ik/PickIkPlugin'
