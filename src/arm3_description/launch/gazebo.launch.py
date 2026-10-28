"""Манипулятор arm3 в Gazebo Harmonic + ros2_control.

Примеры:
  ros2 launch arm3_description gazebo.launch.py                       # позиционное управление (JTC)
  ros2 launch arm3_description gazebo.launch.py control_mode:=effort  # моменты, регулятор пишет студент
  ros2 launch arm3_description gazebo.launch.py control_mode:=effort_jtc
  ros2 launch arm3_description gazebo.launch.py rviz:=true
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


CONTROLLER_FILES = {
    'position': 'arm3_controllers.yaml',
    'effort': 'arm3_effort_controllers.yaml',
    'effort_jtc': 'arm3_effort_jtc_controllers.yaml',
}
MAIN_CONTROLLER = {
    'position': 'arm_controller',
    'effort': 'effort_controller',
    'effort_jtc': 'arm_controller',
}


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('control_mode').perform(context)
    if mode not in CONTROLLER_FILES:
        raise RuntimeError(f'control_mode должен быть одним из {list(CONTROLLER_FILES)}')

    pkg = FindPackageShare('arm3_description')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'arm3.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg, 'config', CONTROLLER_FILES[mode]])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file,
                 ' sim_mode:=gazebo', ' control_mode:=', mode]),
        value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r -v 1 ', LaunchConfiguration('world')]}.items())

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'arm3', '-allow_renaming', 'true'])

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'])

    main_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[MAIN_CONTROLLER[mode], '--param-file', controllers_yaml])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'arm3.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return [
        gz, clock_bridge, rsp, spawn,
        # контроллеры запускаются после появления робота в Gazebo
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[main_spawner])),
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('control_mode', default_value='position',
                              description='position | effort | effort_jtc'),
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='Мир Gazebo'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
