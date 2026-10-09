"""
Interactive demo: simulation, RViz and the instruction-following command node.

Usage:
  ros2 launch lab_simulation demo.launch.py
  ros2 launch lab_simulation demo.launch.py embedder:=keyword perception:=ground_truth
Then send an instruction on /lab/instruction (std_msgs/String) and watch
/lab/instruction/status.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    world_launch = os.path.join(
        get_package_share_directory('lab_simulation'), 'launch', 'arm_world.launch.py')

    declare_args = [
        DeclareLaunchArgument('embedder', default_value='embeddinggemma',
                              description='embeddinggemma (default) or keyword'),
        DeclareLaunchArgument('perception', default_value='vision',
                              description='vision (default) or ground_truth'),
        DeclareLaunchArgument('rviz', default_value='true', description='Show RViz'),
        DeclareLaunchArgument('speed', default_value='1.0',
                              description='Arm speed, 0-1 of the joint limits (benchmark: 0.5)'),
        DeclareLaunchArgument('python', default_value='/opt/lab-ai/bin/python',
                              description='Interpreter for the command node (needs PyTorch'
                                          ' for embeddinggemma; /usr/bin/python3 for keyword)'),
    ]

    simulation = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'perception': PythonExpression(
                ["'true' if '", LaunchConfiguration('perception'), "' == 'vision' else 'false'"]),
            'embedder': LaunchConfiguration('embedder'),
        }.items())])

    command = Node(
        package='lab_simulation',
        executable='command_node.py',
        name='command',
        prefix=LaunchConfiguration('python'),
        parameters=[{'use_sim_time': True,
                     'embedder': LaunchConfiguration('embedder'),
                     'perception': LaunchConfiguration('perception'),
                     'speed': LaunchConfiguration('speed')}],
        output='screen')

    return LaunchDescription([*declare_args, simulation, command])
