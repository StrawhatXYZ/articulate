"""
Launch the table-world simulation with the SO-101.

Starts:
  1. Gazebo with the table world (headless server by default)
  2. robot_state_publisher with the lab robot description, and the SO-101
     spawned into Gazebo from it
  3. ros2_control controllers: joint_state_broadcaster, arm_controller,
     gripper_controller (running inside Gazebo through gz_ros2_control)
  4. ros_gz_bridge for /clock, the piece poses, the grasp latch and resets
  5. MoveIt's move_group, and RViz with the motion planning panel (optional)
  6. Piece markers for RViz (object_markers)

Usage:
  ros2 launch lab_simulation arm_world.launch.py
  ros2 launch lab_simulation arm_world.launch.py rviz:=false
  ros2 launch lab_simulation arm_world.launch.py gz_gui:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from lab_simulation.description import robot_description
from lab_simulation.layout import PIECES
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONTROLLERS = ('joint_state_broadcaster', 'arm_controller', 'gripper_controller')


def generate_launch_description():
    pkg_share = get_package_share_directory('lab_simulation')
    world_sdf = os.path.join(pkg_share, 'worlds', 'table_world.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'lab_so101.urdf.xacro')
    controllers = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'arm.rviz')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    move_group_launch = os.path.join(
        get_package_share_directory('so101_moveit_config'), 'launch', 'move_group.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gz_gui = LaunchConfiguration('gz_gui')

    declare_args = [
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz 2 with the motion planning panel'),
        DeclareLaunchArgument(
            'gz_gui', default_value='false',
            description='Launch the Gazebo GUI in addition to the physics server'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock instead of wall time'),
    ]

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

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description(xacro_file, controllers),
            'use_sim_time': use_sim_time,
        }],
        output='screen')

    # The URDF's fixed world joint places the arm on the table.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'so101'],
        output='screen')

    spawn_controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[*CONTROLLERS, '--controller-manager-timeout', '120'],
            output='screen')]))

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
        output='screen')

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch),
        launch_arguments={
            'robot_description_file': xacro_file,
            'use_sim_time': use_sim_time,
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': rviz_config,
        }.items())

    object_markers = Node(
        package='lab_simulation',
        executable='object_markers.py',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # DetachableJoint attaches at start-up, so release every piece. Wait until
    # each latch plugin has subscribed (the robot is spawned after the world
    # loads, and load time varies by machine); a message sent earlier would be
    # lost. Only the Subscribers section counts: the bridge already publishes
    # on these topics before the robot exists.
    release_latches = [
        ExecuteProcess(
            cmd=['bash', '-c',
                 f'for _ in $(seq 1 120); do gz topic -i -t {topic}'
                 f" | sed -n '/Subscribers/,$p' | grep -q tcp && break; sleep 1; done;"
                 f' gz topic -t {topic} -m gz.msgs.Empty -p ""'],
            output='log')
        for topic in (f'/lab/latch/{name}/detach' for name in sorted(PIECES))]

    return LaunchDescription([
        *declare_args,
        gazebo_server,
        gazebo_with_gui,
        robot_state_publisher,
        spawn,
        spawn_controllers,
        bridge,
        move_group,
        object_markers,
        *release_latches,
    ])
