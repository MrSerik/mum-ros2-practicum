"""MoveIt 2 + RViz для arm3 на «фиктивном» железе (mock_components), без Gazebo.

  ros2 launch arm3_moveit_config demo.launch.py
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import build_moveit_config, move_group_and_rviz  # noqa: E402


def generate_launch_description():
    moveit_config = build_moveit_config('mock')
    controllers_yaml = os.path.join(get_package_share_directory('arm3_description'),
                                    'config', 'arm3_controllers.yaml')

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen', parameters=[moveit_config.robot_description])

    ros2_control = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[controllers_yaml],
        remappings=[('/controller_manager/robot_description', '/robot_description')])

    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner',
             arguments=['arm_controller', '--param-file', controllers_yaml]),
    ]

    return LaunchDescription(
        [rsp, ros2_control, *spawners, *move_group_and_rviz(moveit_config, use_sim_time=False)])
