"""
Launch the table-world simulation.

Starts:
  1. Gazebo with the table world (headless server by default)
  2. ros_gz_bridge for /clock, /joint_states and the joint position commands
  3. robot_state_publisher, which turns /joint_states into TF
  4. RViz 2 with the arm display configuration (optional)
  5. Piece markers for RViz (object_markers)

Usage:
  ros2 launch lab_simulation arm_world.launch.py
  ros2 launch lab_simulation arm_world.launch.py rviz:=false
  ros2 launch lab_simulation arm_world.launch.py gz_gui:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from lab_simulation.layout import PIECES
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lab_simulation')
    worlds_dir = os.path.join(pkg_share, 'worlds')
    world_sdf = os.path.join(worlds_dir, 'table_world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_arm.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'arm.rviz')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    gz_gui = LaunchConfiguration('gz_gui')

    declare_args = [
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz 2 for 3D visual monitoring'),
        DeclareLaunchArgument(
            'gz_gui', default_value='false',
            description='Launch the Gazebo GUI in addition to the physics server'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock instead of wall time'),
    ]

    # Lets the world's model://simple_arm.sdf include resolve, without
    # discarding any resource paths the user already has set.
    resource_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', worlds_dir)

    # -r starts the simulation unpaused; -s runs the server without a GUI.
    # Shutting down the whole launch when Gazebo exits avoids orphaned nodes.
    def gazebo(gz_args, condition):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={
                'gz_args': gz_args,
                'on_exit_shutdown': 'true',
            }.items(),
            condition=condition)

    gazebo_server = gazebo(f'-r -s {world_sdf}', UnlessCondition(gz_gui))
    gazebo_with_gui = gazebo(f'-r {world_sdf}', IfCondition(gz_gui))

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
        output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    object_markers = Node(
        package='lab_simulation',
        executable='object_markers.py',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # DetachableJoint attaches at start-up, so release every piece. Wait until
    # each suction plugin is subscribed (Gazebo load time varies by machine);
    # a message sent earlier would be lost.
    release_suction = [
        ExecuteProcess(
            cmd=['bash', '-c',
                 f'for _ in $(seq 1 60); do gz topic -i -t {topic} | grep -q tcp && break;'
                 f' sleep 1; done; gz topic -t {topic} -m gz.msgs.Empty -p ""'],
            output='log')
        for topic in (f'/simple_arm/suction/{name}/detach' for name in sorted(PIECES))]

    return LaunchDescription([
        *declare_args,
        resource_path,
        gazebo_server,
        gazebo_with_gui,
        bridge,
        robot_state_publisher,
        rviz,
        object_markers,
        *release_suction,
    ])
