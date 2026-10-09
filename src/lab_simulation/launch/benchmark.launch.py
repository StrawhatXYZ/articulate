"""
Run the benchmark against a headless simulation, then shut everything down.

Usage:
  ros2 launch lab_simulation benchmark.launch.py
  ros2 launch lab_simulation benchmark.launch.py task:=pick_place trials:=10 seed:=7
  ros2 launch lab_simulation benchmark.launch.py task:=pick_place trials:=5 \
      rviz:=true realtime:=true
  ros2 launch lab_simulation benchmark.launch.py grasp_mode:=friction
  ros2 launch lab_simulation benchmark.launch.py task:=sort perception:=vision
  ros2 launch lab_simulation benchmark.launch.py task:=instruct trials:=50 perception:=vision
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    world_launch = os.path.join(
        get_package_share_directory('lab_simulation'), 'launch', 'arm_world.launch.py')

    declare_args = [
        DeclareLaunchArgument('task', default_value='all',
                              description='all, reach, pick_place, sort or instruct'),
        DeclareLaunchArgument('trials', default_value='100', description='Trials per task'),
        DeclareLaunchArgument('seed', default_value='42', description='Random seed'),
        DeclareLaunchArgument('out', default_value='/workspace/results',
                              description='Directory for run folders'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Show RViz while benchmarking'),
        DeclareLaunchArgument('realtime', default_value='false',
                              description='Run at real time instead of uncapped (for watching)'),
        DeclareLaunchArgument('grasp_mode', default_value='latched',
                              description='latched (default) or friction; see the README'),
        DeclareLaunchArgument('perception', default_value='ground_truth',
                              description='ground_truth (default) or vision: plan from the'
                                          ' overhead camera'),
        DeclareLaunchArgument('ollama_url', default_value='http://host.docker.internal:11434',
                              description='instruct: Ollama server with the planner model'),
        DeclareLaunchArgument('model', default_value='gemma4:e2b',
                              description='instruct: planner model'),
    ]

    # Scoped, so the included launch's own perception argument (true/false)
    # does not overwrite this launch's (ground_truth/vision).
    simulation = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'perception': PythonExpression(
                ["'true' if '", LaunchConfiguration('perception'), "' == 'vision' else 'false'"]),
        }.items())])

    benchmark = Node(
        package='lab_simulation',
        executable='benchmark.py',
        name='benchmark',
        arguments=['run',
                   '--task', LaunchConfiguration('task'),
                   '--trials', LaunchConfiguration('trials'),
                   '--seed', LaunchConfiguration('seed'),
                   '--out', LaunchConfiguration('out'),
                   '--realtime', LaunchConfiguration('realtime'),
                   '--grasp-mode', LaunchConfiguration('grasp_mode'),
                   '--perception', LaunchConfiguration('perception'),
                   '--ollama-url', LaunchConfiguration('ollama_url'),
                   '--model', LaunchConfiguration('model')],
        parameters=[{'use_sim_time': True}],
        output='screen')

    stop_when_done = RegisterEventHandler(OnProcessExit(
        target_action=benchmark,
        on_exit=[EmitEvent(event=Shutdown(reason='benchmark finished'))]))

    return LaunchDescription([*declare_args, simulation, benchmark, stop_when_done])
