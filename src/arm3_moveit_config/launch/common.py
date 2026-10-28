"""Общие функции для launch-файлов arm3_moveit_config."""
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


# Алгоритмы OMPL, доступные группе arm (сравниваются в ПР №6).
# Общие настройки конвейера берутся из moveit_configs_utils (совпадают с установленной версией MoveIt).
ARM_PLANNERS = {
    'default_planner_config': 'RRTConnect',
    'planner_configs': ['RRTConnect', 'RRT', 'RRTstar', 'PRM', 'PRMstar', 'EST', 'KPIECE'],
}


def build_moveit_config(sim_mode: str):
    xacro_path = os.path.join(get_package_share_directory('arm3_description'),
                              'urdf', 'arm3.urdf.xacro')
    cfg = (
        MoveItConfigsBuilder('arm3', package_name='arm3_moveit_config')
        .robot_description(file_path=xacro_path, mappings={'sim_mode': sim_mode})
        .robot_description_semantic(file_path='config/arm3.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    cfg.planning_pipelines['ompl']['arm'] = ARM_PLANNERS
    return cfg


def move_group_and_rviz(moveit_config, use_sim_time: bool):
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}])

    rviz_cfg = os.path.join(get_package_share_directory('arm3_moveit_config'),
                            'config', 'moveit.rviz')
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', rviz_cfg],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    {'use_sim_time': use_sim_time}])
    return [move_group, rviz]
