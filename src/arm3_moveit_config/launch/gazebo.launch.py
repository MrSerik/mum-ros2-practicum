"""MoveIt 2 управляет манипулятором arm3 в Gazebo Harmonic.

  ros2 launch arm3_moveit_config gazebo.launch.py
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import build_moveit_config, move_group_and_rviz  # noqa: E402


def generate_launch_description():
    moveit_config = build_moveit_config('gazebo')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('arm3_description'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'control_mode': 'position', 'rviz': 'false'}.items())

    # move_group запускаем с задержкой, когда контроллеры уже активны
    return LaunchDescription([
        sim,
        TimerAction(period=5.0, actions=move_group_and_rviz(moveit_config, use_sim_time=True)),
    ])
