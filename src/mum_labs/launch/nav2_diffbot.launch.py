"""ПР №9. diffbot в Gazebo + Nav2 (+ SLAM Toolbox) + RViz одной командой.

  # построение карты и навигация одновременно (SLAM):
  ros2 launch mum_labs nav2_diffbot.launch.py slam:=True
  # навигация по сохранённой карте (AMCL):
  ros2 launch mum_labs nav2_diffbot.launch.py slam:=False map:=$HOME/maps/lab_map.yaml
  # свои параметры Nav2:
  ros2 launch mum_labs nav2_diffbot.launch.py params_file:=$HOME/mum_ws/my_nav2_params.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_bringup')
    bot_dir = get_package_share_directory('diffbot_description')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bot_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'rviz': 'false'}.items())

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'True',
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'True',
        }.items())

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'use_sim_time': 'True'}.items())

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='True'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(nav2_dir, 'params', 'nav2_params.yaml')),
        sim,
        TimerAction(period=6.0, actions=[nav, rviz]),
    ])
