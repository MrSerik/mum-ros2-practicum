# Шаг 2. Мобильный робот diffbot: модель, датчики, мир Gazebo

**Результат шага:** пакет `diffbot_description` — робот с дифференциальным приводом, 2D-лидаром и RGB-D камерой в учебном полигоне 8 × 6 м. Робот ездит с клавиатуры, по лидару строится карта.
**Время:** 1,5–2 часа. **Нужно:** выполненный [шаг 1](step1_manipulator.md) (рабочее пространство `~/mum_ws`).

> Адрес для `curl` (если новый терминал): `export REPO_RAW=https://raw.githubusercontent.com/MrSerik/mum-ros2-practicum/main`

## 2.1. Создание пакета

```bash
cd ~/mum_ws/src
ros2 pkg create diffbot_description --build-type ament_cmake --license Apache-2.0
cd diffbot_description
rm -rf include src
mkdir -p urdf config launch rviz worlds
```

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>diffbot_description</name>
  <version>1.0.0</version>
  <description>Учебный мобильный робот с дифференциальным приводом: URDF, лидар, RGB-D камера, мир Gazebo Harmonic</description>
  <maintainer email="ssydykanov@gmail.com">Sydykanov S.</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>teleop_twist_keyboard</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

```cmake
cmake_minimum_required(VERSION 3.8)
project(diffbot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch rviz worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
```

## 2.2. Модель робота

Файл `urdf/diffbot.urdf.xacro`. Что в нём:

- **`base_footprint`** — проекция центра робота на пол, **`base_link`** — центр оси колёс. Эти имена ожидает Nav2.
- **Два ведущих колеса** (`continuous`-сочленения `wheel_left_joint`, `wheel_right_joint`): радиус r = 0,05 м, колея L = 0,30 м. Трение колёс `mu = 1`.
- **Две шаровые опоры** спереди и сзади с нулевым трением — чтобы робот не «тормозил брюхом».
- **Лидар** `lidar_link` (`gpu_lidar`: 360 лучей, 0,12–12 м, 10 Гц) и **RGB-D камера** `camera_link` (640 × 480, обзор 60°). Отдельная оптическая система `camera_optical_link` повёрнута так, как принято для камер: z — вперёд, x — вправо, y — вниз.
- **Плагины Gazebo**: `DiffDrive` — принимает `cmd_vel`, публикует одометрию и TF `odom → base_footprint`; `JointStatePublisher` — углы колёс; `OdometryPublisher` — «истинное» положение робота для сравнения с одометрией.

<details>
<summary>Показать содержимое <code>urdf/diffbot.urdf.xacro</code></summary>

```xml
<?xml version="1.0"?>
<!--
  diffbot — учебный мобильный робот с дифференциальным приводом.
  Два ведущих колеса + две шаровые опоры, 2D-лидар, RGB-D камера.
  Размеры близки к TurtleBot3, поэтому стандартный nav2_params.yaml подходит без изменений.

  Системы координат:  odom -> base_footprint -> base_link -> {wheels, lidar_link, camera_link}
  Кинематика:  v = r(wR + wL)/2,  w = r(wR - wL)/L,  r = WHEEL_R, L = WHEEL_SEP
-->
<robot name="diffbot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:property name="PI" value="3.141592653589793"/>
  <xacro:property name="WHEEL_R" value="0.05"/>     <!-- радиус колеса r, м -->
  <xacro:property name="WHEEL_W" value="0.03"/>     <!-- ширина колеса, м -->
  <xacro:property name="WHEEL_SEP" value="0.30"/>   <!-- колея L, м -->
  <xacro:property name="BODY_R" value="0.17"/>      <!-- радиус корпуса, м -->
  <xacro:property name="BODY_H" value="0.10"/>      <!-- высота корпуса, м -->
  <xacro:property name="CASTER_R" value="0.025"/>
  <xacro:property name="M_BODY" value="2.0"/>
  <xacro:property name="M_WHEEL" value="0.2"/>

  <material name="body"><color rgba="0.20 0.45 0.75 1"/></material>
  <material name="black"><color rgba="0.08 0.08 0.08 1"/></material>
  <material name="grey"><color rgba="0.6 0.6 0.6 1"/></material>

  <!-- ===== base_footprint: проекция центра робота на пол ===== -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${WHEEL_R}" rpy="0 0 0"/>
  </joint>

  <!-- ===== base_link: центр оси колёс ===== -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${BODY_H/2}"/>
      <geometry><cylinder radius="${BODY_R}" length="${BODY_H}"/></geometry>
      <material name="body"/>
    </visual>
    <!-- метка "перед" -->
    <visual>
      <origin xyz="${BODY_R-0.03} 0 ${BODY_H+0.002}"/>
      <geometry><box size="0.04 0.04 0.004"/></geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${BODY_H/2}"/>
      <geometry><cylinder radius="${BODY_R}" length="${BODY_H}"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${BODY_H/2}"/>
      <mass value="${M_BODY}"/>
      <inertia ixx="${M_BODY*(3*BODY_R*BODY_R+BODY_H*BODY_H)/12}" ixy="0" ixz="0"
               iyy="${M_BODY*(3*BODY_R*BODY_R+BODY_H*BODY_H)/12}" iyz="0"
               izz="${M_BODY*BODY_R*BODY_R/2}"/>
    </inertial>
  </link>

  <!-- ===== Ведущие колёса ===== -->
  <xacro:macro name="wheel" params="side y">
    <joint name="wheel_${side}_joint" type="continuous">
      <parent link="base_link"/>
      <child link="wheel_${side}_link"/>
      <origin xyz="0 ${y} 0" rpy="${-PI/2} 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
    <link name="wheel_${side}_link">
      <visual>
        <geometry><cylinder radius="${WHEEL_R}" length="${WHEEL_W}"/></geometry>
        <material name="black"/>
      </visual>
      <collision>
        <geometry><cylinder radius="${WHEEL_R}" length="${WHEEL_W}"/></geometry>
      </collision>
      <inertial>
        <mass value="${M_WHEEL}"/>
        <inertia ixx="${M_WHEEL*(3*WHEEL_R*WHEEL_R+WHEEL_W*WHEEL_W)/12}" ixy="0" ixz="0"
                 iyy="${M_WHEEL*(3*WHEEL_R*WHEEL_R+WHEEL_W*WHEEL_W)/12}" iyz="0"
                 izz="${M_WHEEL*WHEEL_R*WHEEL_R/2}"/>
      </inertial>
    </link>
    <gazebo reference="wheel_${side}_link">
      <mu1>1.0</mu1>
      <mu2>1.0</mu2>
    </gazebo>
  </xacro:macro>

  <xacro:wheel side="left" y="${WHEEL_SEP/2}"/>
  <xacro:wheel side="right" y="${-WHEEL_SEP/2}"/>

  <!-- ===== Шаровые опоры (без трения) ===== -->
  <xacro:macro name="caster" params="name x">
    <joint name="${name}_joint" type="fixed">
      <parent link="base_link"/>
      <child link="${name}_link"/>
      <origin xyz="${x} 0 ${-WHEEL_R+CASTER_R}"/>
    </joint>
    <link name="${name}_link">
      <visual>
        <geometry><sphere radius="${CASTER_R}"/></geometry>
        <material name="grey"/>
      </visual>
      <collision>
        <geometry><sphere radius="${CASTER_R}"/></geometry>
      </collision>
      <inertial>
        <mass value="0.05"/>
        <inertia ixx="1.25e-5" ixy="0" ixz="0" iyy="1.25e-5" iyz="0" izz="1.25e-5"/>
      </inertial>
    </link>
    <gazebo reference="${name}_link">
      <mu1>0.0</mu1>
      <mu2>0.0</mu2>
    </gazebo>
  </xacro:macro>

  <xacro:caster name="caster_front" x="0.12"/>
  <xacro:caster name="caster_back" x="-0.12"/>

  <!-- ===== Лидар ===== -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 ${BODY_H+0.02}"/>
  </joint>
  <link name="lidar_link">
    <visual>
      <geometry><cylinder radius="0.035" length="0.04"/></geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.035" length="0.04"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="4.4e-5" ixy="0" ixz="0" iyy="4.4e-5" iyz="0" izz="6.1e-5"/>
    </inertial>
  </link>

  <!-- ===== RGB-D камера ===== -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${BODY_R+0.01} 0 ${BODY_H-0.02}"/>
  </joint>
  <link name="camera_link">
    <visual>
      <geometry><box size="0.02 0.08 0.03"/></geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry><box size="0.02 0.08 0.03"/></geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="3e-5" ixy="0" ixz="0" iyy="5e-6" iyz="0" izz="3e-5"/>
    </inertial>
  </link>
  <!-- Оптическая СК камеры: z — вперёд по оси объектива, x — вправо, y — вниз -->
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_link"/>
    <origin xyz="0 0 0" rpy="${-PI/2} 0 ${-PI/2}"/>
  </joint>
  <link name="camera_optical_link"/>

  <!-- ======================= Gazebo Harmonic ======================= -->
  <gazebo reference="base_link">
    <visual><material><diffuse>0.20 0.45 0.75 1</diffuse><ambient>0.20 0.45 0.75 1</ambient></material></visual>
  </gazebo>

  <gazebo reference="lidar_link">
    <sensor name="lidar" type="gpu_lidar">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <topic>scan</topic>
      <gz_frame_id>lidar_link</gz_frame_id>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.12</min>
          <max>12.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor name="rgbd_camera" type="rgbd_camera">
      <always_on>true</always_on>
      <update_rate>15</update_rate>
      <topic>camera</topic>
      <gz_frame_id>camera_optical_link</gz_frame_id>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>8.0</far>
        </clip>
      </camera>
    </sensor>
  </gazebo>

  <gazebo>
    <!-- Дифференциальный привод: принимает cmd_vel, публикует одометрию и TF odom->base_footprint -->
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>${WHEEL_SEP}</wheel_separation>
      <wheel_radius>${WHEEL_R}</wheel_radius>
      <max_linear_acceleration>1.0</max_linear_acceleration>
      <min_linear_acceleration>-1.0</min_linear_acceleration>
      <max_angular_acceleration>3.0</max_angular_acceleration>
      <min_angular_acceleration>-3.0</min_angular_acceleration>
      <max_linear_velocity>0.5</max_linear_velocity>
      <min_linear_velocity>-0.5</min_linear_velocity>
      <max_angular_velocity>2.0</max_angular_velocity>
      <min_angular_velocity>-2.0</min_angular_velocity>
      <topic>cmd_vel</topic>
      <odom_topic>odom</odom_topic>
      <tf_topic>tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
      <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>

    <!-- «Истинное» положение робота в мире (для сравнения с одометрией в ПР №8).
         TF этого плагина в ROS не передаётся. -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <odom_topic>ground_truth</odom_topic>
      <tf_topic>ground_truth_tf</tf_topic>
      <dimensions>2</dimensions>
      <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>

    <!-- Публикация углов колёс (для robot_state_publisher и собственной одометрии в ПР №8) -->
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <topic>joint_states</topic>
      <joint_name>wheel_left_joint</joint_name>
      <joint_name>wheel_right_joint</joint_name>
      <update_rate>50</update_rate>
    </plugin>
  </gazebo>

</robot>
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o urdf/diffbot.urdf.xacro $REPO_RAW/src/diffbot_description/urdf/diffbot.urdf.xacro
```

## 2.3. Мост ROS 2 ↔ Gazebo

У Gazebo свои топики и типы сообщений. `ros_gz_bridge` переводит их в ROS 2: каждая запись — имя топика, тип ROS, тип Gazebo и направление передачи.

```yaml
# Мост ROS 2 <-> Gazebo Harmonic (ros_gz_bridge/parameter_bridge)
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/odom"
  gz_topic_name: "/odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/tf"
  gz_topic_name: "/tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

- ros_topic_name: "/joint_states"
  gz_topic_name: "/joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "/scan"
  gz_topic_name: "/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/image"
  gz_topic_name: "/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/depth_image"
  gz_topic_name: "/camera/depth_image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera_info"
  gz_topic_name: "/camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS

- ros_topic_name: "/ground_truth"
  gz_topic_name: "/ground_truth"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
```

## 2.4. Мир Gazebo

`worlds/lab_world.sdf` — комната 8 × 6 м с двумя перегородками, ящиками, колоннами, красным мячом и двумя цветными цилиндрами. В начале файла подключены системы Gazebo: физика, сенсоры (движок рендеринга `ogre2` нужен лидару и камере), команды пользователя.

Файл длинный и не требует ручного набора — скачайте его (из папки пакета):

```bash
curl -sL --create-dirs -o worlds/lab_world.sdf $REPO_RAW/src/diffbot_description/worlds/lab_world.sdf
```

## 2.5. Launch-файлы и RViz

**`launch/gazebo.launch.py`** — мир, появление робота в точке (−3; 0), мост, `robot_state_publisher`, RViz. Начальную позу можно поменять аргументами `x`, `y`, `yaw`.

```python
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
```

**`launch/display.launch.py`** — просмотр модели без симулятора:

```python
"""Просмотр модели diffbot в RViz (без Gazebo).

  ros2 launch diffbot_description display.launch.py
"""
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('diffbot_description')
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ',
                 PathJoinSubstitution([pkg, 'urdf', 'diffbot.urdf.xacro'])]),
        value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        # в режиме просмотра нет одометрии, поэтому фиксируем odom = base_footprint
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'base_footprint']),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'diffbot.rviz'])]),
    ])
```

**`rviz/diffbot.rviz`**:

Файл длинный и не требует ручного набора — скачайте его (из папки пакета):

```bash
curl -sL --create-dirs -o rviz/diffbot.rviz $REPO_RAW/src/diffbot_description/rviz/diffbot.rviz
```

## 2.6. Сборка и проверка

```bash
cd ~/mum_ws && colcon build --symlink-install && source install/setup.bash
```

**Проверка 1 — модель:** `ros2 launch diffbot_description display.launch.py`. Ползунками крутятся колёса.

**Проверка 2 — Gazebo и езда:**

```bash
ros2 launch diffbot_description gazebo.launch.py
```

Во втором терминале:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Клавиши `i` — вперёд, `j`/`l` — поворот, `k` — стоп. В RViz видны точки лидара на стенах, стрелки одометрии и изображение камеры.

**Проверка 3 — датчики и TF:**

```bash
ros2 topic hz /scan                 # ~10 Гц
ros2 topic echo /odom --once        # поза и скорость
ros2 run tf2_tools view_frames      # PDF с деревом odom → base_footprint → base_link → ...
```

**Проверка 4 — карта по лидару (SLAM):** не закрывая Gazebo и teleop,

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

В RViz смените *Fixed Frame* на `map`, нажмите *Add → By topic → /map → Map*. Поездите по полигону — карта будет дорисовываться.

## Итог шага 2

```
mum_ws/src/diffbot_description/
├── CMakeLists.txt  package.xml
├── urdf/diffbot.urdf.xacro
├── config/bridge.yaml
├── worlds/lab_world.sdf
├── launch/display.launch.py  gazebo.launch.py
└── rviz/diffbot.rviz
```

Сверить с эталоном: `git checkout step-2`.

**Контрольные вопросы:** какие формулы связывают скорости колёс со скоростью робота? Какой узел публикует `odom → base_footprint`, а какой `base_footprint → base_link`? Зачем нужен `use_sim_time:=true`? Почему у шаровых опор нулевое трение?

[← Шаг 1](step1_manipulator.md) · [Шаг 3. Узлы практических работ →](step3_lab_nodes.md)
