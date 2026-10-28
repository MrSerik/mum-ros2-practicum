"""Просмотр модели arm3 в RViz с ползунками углов (без Gazebo).

  ros2 launch arm3_description display.launch.py
"""
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('arm3_description')
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ',
                 PathJoinSubstitution([pkg, 'urdf', 'arm3.urdf.xacro']),
                 ' sim_mode:=mock']),
        value_type=str)

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'arm3.rviz'])]),
    ])
