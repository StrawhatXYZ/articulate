#!/usr/bin/env python3
"""
Generate urdf/so101_macro.xacro from the upstream SO-101 URDF.

Kinematics, inertials, joint limits and visual meshes are copied from
urdf/upstream/so101_new_calib.urdf. The changes are listed in
meshes/SOURCE.md. Re-run after updating the upstream files:

  python3 tools/generate_xacro.py > urdf/so101_macro.xacro

Needs NumPy only (no ROS).
"""

from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

PKG = Path(__file__).resolve().parents[1]
UPSTREAM = PKG / 'urdf' / 'upstream' / 'so101_new_calib.urdf'

EFFORT_NM = 2.94        # STS3215 stall torque (upstream lists a placeholder 10)
VELOCITY_RAD_S = 4.7    # STS3215 no-load speed (upstream lists a placeholder 10)
DAMPING = 0.6           # upstream MuJoCo motor model
FRICTION = 0.052        # upstream MuJoCo motor model
COLOURS = {'3d_printed': '1.0 0.45 0.0 1.0', 'sts3215': '0.2 0.2 0.22 1.0'}

# gripper_frame_link sits on the fixed jaw's inner face at the fingertip,
# with z pointing out of the fingers. These are in that frame.
FINGER_LENGTH = 0.052   # finger region: frame z above -FINGER_LENGTH
GRASP_POINT = (-0.022, 0.0, -0.004)   # between the open jaws, behind the tip
# Jaw pads: (centre xyz, size xyz, yaw) in each jaw's link frame.
FIXED_PAD = ((-0.0149, -0.0002, -0.0791), (0.010, 0.014, 0.050), 0.0)
MOVING_PAD = ((-0.0041, -0.0557, 0.0190), (0.012, 0.050, 0.014), -0.25)


def floats(text):
    return [float(v) for v in text.split()]


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch),
                              np.cos(yaw), np.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def stl_vertices(path):
    data = path.read_bytes()
    count = struct.unpack('<I', data[80:84])[0]
    records = np.frombuffer(data[84:84 + count * 50], dtype=np.dtype(
        [('normal', '<3f4'), ('v', '<9f4'), ('attr', '<u2')]))
    return records['v'].reshape(-1, 3).astype(float)


def visual_points(link):
    """Return every visual mesh vertex of a link, in the link frame."""
    points = []
    for visual in link.findall('visual'):
        origin = visual.find('origin')
        mesh = Path(visual.find('geometry/mesh').get('filename')).name
        rotation = rpy_matrix(*floats(origin.get('rpy')))
        points.append(stl_vertices(PKG / 'meshes' / mesh) @ rotation.T
                      + floats(origin.get('xyz')))
    return np.vstack(points)


def box(centre, size, yaw=0.0):
    c = ' '.join(f'{v:.4f}' for v in centre)
    s = ' '.join(f'{v:.4f}' for v in size)
    return (f'      <collision>\n        <origin xyz="{c}" rpy="0 0 {yaw:g}"/>\n'
            f'        <geometry><box size="{s}"/></geometry>\n      </collision>')


def bounds_box(points):
    lo, hi = points.min(axis=0), points.max(axis=0)
    return box((lo + hi) / 2, hi - lo)


def collisions(name, link, frame_z_in_gripper):
    if name == 'gripper_link':
        points = visual_points(link)
        # Body only; the fingers get pads. Frame z (out of the fingers) is
        # -(gripper_link z - frame origin z), see gripper_frame_joint.
        frame_z = -(points[:, 2] - frame_z_in_gripper)
        return [bounds_box(points[frame_z < -FINGER_LENGTH]), box(*FIXED_PAD[:2], FIXED_PAD[2])]
    if name == 'moving_jaw_so101_v1_link':
        return [box(*MOVING_PAD[:2], MOVING_PAD[2])]
    return [bounds_box(visual_points(link))]


def origin_tag(element):
    origin = element.find('origin')
    return f'<origin xyz="{origin.get("xyz")}" rpy="{origin.get("rpy")}"/>'


def render_link(link, frame_z_in_gripper):
    name = link.get('name')
    parts = [f'    <link name="{name}">']
    inertial = link.find('inertial')
    inertia = inertial.find('inertia')
    moments = ' '.join(f'{k}="{inertia.get(k)}"'
                       for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'))
    parts.append(
        f'      <inertial>\n        {origin_tag(inertial)}\n'
        f'        <mass value="{inertial.find("mass").get("value")}"/>\n'
        f'        <inertia {moments}/>\n      </inertial>')
    for visual in link.findall('visual'):
        mesh = Path(visual.find('geometry/mesh').get('filename')).name
        material = visual.find('material').get('name')
        parts.append(
            f'      <visual>\n        {origin_tag(visual)}\n'
            f'        <geometry><mesh filename="file://$(find so101_description)/meshes/{mesh}"/>'
            f'</geometry>\n        <material name="{material}"/>\n      </visual>')
    if link.findall('visual'):
        parts.extend(collisions(name, link, frame_z_in_gripper))
    parts.append('    </link>')
    return '\n'.join(parts)


def render_joint(joint):
    origin = joint.find('origin')
    lines = [f'    <joint name="{joint.get("name")}" type="{joint.get("type")}">',
             f'      <origin xyz="{origin.get("xyz")}" rpy="{origin.get("rpy")}"/>',
             f'      <parent link="{joint.find("parent").get("link")}"/>',
             f'      <child link="{joint.find("child").get("link")}"/>']
    if joint.get('type') == 'revolute':
        limit = joint.find('limit')
        lines += [f'      <axis xyz="{joint.find("axis").get("xyz")}"/>',
                  f'      <limit effort="{EFFORT_NM}" velocity="{VELOCITY_RAD_S}"'
                  f' lower="{limit.get("lower")}" upper="{limit.get("upper")}"/>',
                  f'      <dynamics damping="{DAMPING}" friction="{FRICTION}"/>']
    lines.append('    </joint>')
    return '\n'.join(lines)


def main():
    robot = ET.parse(UPSTREAM).getroot()
    frame_joint = next(j for j in robot.findall('joint') if j.get('name') == 'gripper_frame_joint')
    frame_z_in_gripper = floats(frame_joint.find('origin').get('xyz'))[2]
    out = [
        '<?xml version="1.0"?>',
        '<!--',
        '  SO-101 arm macro. GENERATED by tools/generate_xacro.py from',
        '  urdf/upstream/so101_new_calib.urdf; do not edit by hand. The changes',
        '  from upstream are listed in meshes/SOURCE.md.',
        '',
        '  Usage:  <xacro:so101 parent="world"><origin xyz="0 0 0" rpy="0 0 0"/></xacro:so101>',
        '-->',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
        '  <xacro:macro name="so101" params="parent *origin">',
    ]
    for name, rgba in COLOURS.items():
        out.append(f'    <material name="{name}"><color rgba="{rgba}"/></material>')
    out += ['    <joint name="so101_mount" type="fixed">',
            '      <parent link="${parent}"/>',
            '      <child link="base_link"/>',
            '      <xacro:insert_block name="origin"/>',
            '    </joint>']
    for link in robot.findall('link'):
        out.append(render_link(link, frame_z_in_gripper))
    for joint in robot.findall('joint'):
        out.append(render_joint(joint))
    xyz = ' '.join(f'{v:g}' for v in GRASP_POINT)
    out += ['    <!-- Between the open jaws, where task goals put the piece centre. -->',
            '    <link name="grasp_link"/>',
            '    <joint name="grasp_joint" type="fixed">',
            f'      <origin xyz="{xyz}" rpy="0 0 0"/>',
            '      <parent link="gripper_frame_link"/>',
            '      <child link="grasp_link"/>',
            '    </joint>',
            '  </xacro:macro>',
            '</robot>']
    print('\n'.join(out))


if __name__ == '__main__':
    main()
