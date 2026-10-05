"""
Run the benchmark against a headless simulation, then shut everything down.

Usage:
  ros2 launch lab_simulation benchmark.launch.py
  ros2 launch lab_simulation benchmark.launch.py task:=pick_place trials:=10 seed:=7
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_launch = os.path.join(
        get_package_share_directory('lab_simulation'), 'launch', 'arm_world.launch.py')

    declare_args = [
        DeclareLaunchArgument('task', default_value='all',
                              description='all, reach or pick_place'),
        DeclareLaunchArgument('trials', default_value='100', description='Trials per task'),
        DeclareLaunchArgument('seed', default_value='42', description='Random seed'),
        DeclareLaunchArgument('out', default_value='/workspace/results',
                              description='Directory for run folders'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Show RViz while benchmarking'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items())

    benchmark = Node(
        package='lab_simulation',
        executable='benchmark.py',
        name='benchmark',
        arguments=['run',
                   '--task', LaunchConfiguration('task'),
                   '--trials', LaunchConfiguration('trials'),
                   '--seed', LaunchConfiguration('seed'),
                   '--out', LaunchConfiguration('out')],
        parameters=[{'use_sim_time': True}],
        output='screen')

    stop_when_done = RegisterEventHandler(OnProcessExit(
        target_action=benchmark,
        on_exit=[EmitEvent(event=Shutdown(reason='benchmark finished'))]))

    return LaunchDescription([*declare_args, simulation, benchmark, stop_when_done])
