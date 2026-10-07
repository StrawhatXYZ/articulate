"""
Start MoveIt's move_group for the SO-101, and optionally RViz with it.

Usage (from another launch file, or directly next to a running robot):
  ros2 launch so101_moveit_config move_group.launch.py
  ros2 launch so101_moveit_config move_group.launch.py \
      robot_description_file:=/path/to/robot.urdf.xacro rviz:=true

robot_description_file must describe the same robot that
robot_state_publisher publishes (for the lab: lab_simulation's
lab_so101.urdf.xacro, which places the arm on the table).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def moveit_config(robot_description_file):
    """Return the MoveIt configuration for a given robot description file."""
    return (
        MoveItConfigsBuilder('so101', package_name='so101_moveit_config')
        .robot_description(file_path=robot_description_file)
        .robot_description_semantic(file_path='config/so101.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .planning_scene_monitor(publish_robot_description=False,
                                publish_robot_description_semantic=True)
        .to_moveit_configs())


def launch_setup(context):
    config = moveit_config(LaunchConfiguration('robot_description_file').perform(context))
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context)
                    == 'true'}

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[config.to_dict(), use_sim_time],
        output='screen')

    nodes = [move_group]
    if LaunchConfiguration('rviz').perform(context) == 'true':
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config').perform(context)],
            parameters=[config.robot_description, config.robot_description_semantic,
                        config.robot_description_kinematics, config.planning_pipelines,
                        config.joint_limits, use_sim_time],
            output='screen'))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('so101_moveit_config')
    standalone = os.path.join(
        get_package_share_directory('so101_description'), 'urdf', 'so101.urdf.xacro')
    return LaunchDescription([
        DeclareLaunchArgument('robot_description_file', default_value=standalone,
                              description='URDF or xacro file of the robot'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the simulation clock'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Also start RViz with the motion planning panel'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'config', 'moveit.rviz'),
                              description='RViz configuration file'),
        OpaqueFunction(function=launch_setup),
    ])
