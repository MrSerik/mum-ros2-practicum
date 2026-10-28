"""Мобильный робот diffbot в учебном полигоне Gazebo Harmonic.

  ros2 launch diffbot_description gazebo.launch.py            # Gazebo + RViz
  ros2 launch diffbot_description gazebo.launch.py rviz:=false
  ros2 launch diffbot_description gazebo.launch.py x:=0.0 y:=1.0 yaw:=1.57

Управление с клавиатуры (в отдельном терминале):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('diffbot_description')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ',
                 PathJoinSubstitution([pkg, 'urdf', 'diffbot.urdf.xacro'])]),
        value_type=str)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r -v 1 ', LaunchConfiguration('world')]}.items())

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'diffbot',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.02', '-Y', LaunchConfiguration('yaw')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': PathJoinSubstitution([pkg, 'config', 'bridge.yaml']),
                     'use_sim_time': True}])

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'diffbot.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('world',
                              default_value=PathJoinSubstitution([pkg, 'worlds', 'lab_world.sdf'])),
        DeclareLaunchArgument('x', default_value='-3.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        gz, spawn, bridge, rsp, rviz,
    ])
