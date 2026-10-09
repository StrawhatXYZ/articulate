"""
Talk to the robot: the simulation plus the instruction node (/lab/instruct).

Usage:
  ros2 launch lab_simulation demo.launch.py                      # camera, RViz
  ros2 launch lab_simulation demo.launch.py perception:=ground_truth rviz:=false
Then send instructions with host/voice.py, or:
  ros2 action send_goal --feedback /lab/instruct lab_interfaces/action/Instruct \
      "{text: 'put the red cube in its box'}"
The planner needs Ollama with gemma4:e2b on the host (see the README).
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
        DeclareLaunchArgument('perception', default_value='vision',
                              description='vision (default): plan from the camera; or'
                                          ' ground_truth'),
        DeclareLaunchArgument('rviz', default_value='true', description='Show RViz'),
        DeclareLaunchArgument('ollama_url', default_value='http://host.docker.internal:11434',
                              description='Ollama server with the planner model'),
        DeclareLaunchArgument('model', default_value='gemma4:e2b',
                              description='Planner model'),
        DeclareLaunchArgument('speed', default_value='0.8',
                              description='Arm speed, a fraction of the joint limits (20 pick'
                                          ' and 20 sort trials at 0.8 matched 0.5)'),
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

    instruction = Node(
        package='lab_simulation',
        executable='instruction_node.py',
        parameters=[{'use_sim_time': True,
                     'perception': LaunchConfiguration('perception'),
                     'ollama_url': LaunchConfiguration('ollama_url'),
                     'model': LaunchConfiguration('model'),
                     'speed': PythonExpression([LaunchConfiguration('speed')])}],
        output='screen')

    return LaunchDescription([*declare_args, simulation, instruction])
