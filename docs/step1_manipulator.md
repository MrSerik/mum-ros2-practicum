# Шаг 1. Манипулятор arm3: модель, Gazebo, MoveIt 2

**Результат шага:** рабочее пространство `~/mum_ws` с двумя пакетами — `arm3_description` (модель манипулятора с тремя сервоприводами, управление ros2_control, запуск в Gazebo) и `arm3_moveit_config` (планирование движений в MoveIt 2).
**Время:** 2–3 часа. **Нужно:** установленный ROS 2 Jazzy (см. [README](../README.md#подготовка)).

> Перед началом в каждом новом терминале должен быть подключён ROS 2: `source /opt/ros/jazzy/setup.bash` (скрипт установки добавил это в `~/.bashrc`).
> Для команд `curl` ниже один раз задайте адрес репозитория:
> ```bash
> export REPO_RAW=https://raw.githubusercontent.com/MrSerik/mum-ros2-practicum/main
> ```

## 1.1. Рабочее пространство

Рабочее пространство (workspace) — папка, в подпапке `src` которой лежат пакеты. Сборщик `colcon` находит в `src` все пакеты и собирает их в `build/` и `install/`.

```bash
mkdir -p ~/mum_ws/src
cd ~/mum_ws/src
```

## 1.2. Пакет arm3_description

Создаём пакет типа `ament_cmake` — в нём нет компилируемого кода, только файлы модели, настройки и launch-файлы:

```bash
ros2 pkg create arm3_description --build-type ament_cmake --license Apache-2.0
cd arm3_description
rm -rf include src                  # папки для C++-кода нам не нужны
mkdir -p urdf config launch rviz
```

### package.xml — описание пакета и зависимости

Замените сгенерированный файл. Теги `exec_depend` перечисляют пакеты, нужные при запуске: по ним `rosdep` доустанавливает недостающее.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>arm3_description</name>
  <version>1.0.0</version>
  <description>Учебный 3-звенный манипулятор (3 сервопривода): URDF, ros2_control, запуск в Gazebo Harmonic</description>
  <maintainer email="ssydykanov@gmail.com">Sydykanov S.</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>gz_ros2_control</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>ros2_controllers</exec_depend>
  <exec_depend>hardware_interface</exec_depend>  <!-- mock_components/GenericSystem -->

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt — что устанавливать

Пакет ничего не компилирует, он только копирует папки в `install/share/arm3_description`, откуда их находят launch-файлы.

```cmake
cmake_minimum_required(VERSION 3.8)
project(arm3_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
```

## 1.3. Модель манипулятора (URDF + xacro)

Файл `urdf/arm3.urdf.xacro` — главный файл шага. Что в нём:

- **Звенья (`link`)**: подставка `pedestal`, основание `base_link` (система координат 0), поворотная колонна `link1`, плечо `link2`, предплечье `link3`, точка схвата `tool0` (система 3). Каждое звено имеет `visual` (как выглядит), `collision` (форма для столкновений) и `inertial` (масса и тензор инерции, считаются макросами `box_inertia`, `cyl_inertia`).
- **Сочленения (`joint`)**: `joint1` вращается вокруг z, `joint2` и `joint3` — вокруг оси (0, −1, 0), чтобы положительный угол поднимал звено. Так углы совпадают с таблицей Денавита–Хартенберга из лекций (D₁ = 0,10 м, A₂ = A₃ = 0,12 м).
- **Блок `ros2_control`**: какие интерфейсы есть у каждого сервопривода — команда `position` или `effort` (момент) и состояния `position`, `velocity`, `effort`. Аргумент `control_mode` выбирает режим.
- **Плагин `gz_ros2_control`**: связывает Gazebo с контроллерами ROS 2; аргумент `sim_mode:=mock` вместо Gazebo подключает «фиктивное» железо для MoveIt.

<details>
<summary>Показать содержимое <code>urdf/arm3.urdf.xacro</code></summary>

```xml
<?xml version="1.0"?>
<!--
  arm3 — учебный 3-звенный манипулятор (3 сервопривода): поворот основания + плечо + локоть.
  Кинематика соответствует таблице Денавита–Хартенберга (стандартная форма):
      i | theta_i | d_i | a_i | alpha_i
      1 |   q1    | D1  |  0  |  +pi/2
      2 |   q2    |  0  | A2  |   0
      3 |   q3    |  0  | A3  |   0
  Система координат 0 = base_link, система 3 = tool0.
  Аргументы:
      sim_mode     : gazebo | mock        (Gazebo Harmonic или "фиктивное" железо для MoveIt)
      control_mode : position | effort | effort_jtc
-->
<robot name="arm3" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:arg name="sim_mode" default="gazebo"/>
  <xacro:arg name="control_mode" default="position"/>
  <xacro:property name="sim_mode" value="$(arg sim_mode)"/>
  <xacro:property name="control_mode" value="$(arg control_mode)"/>

  <!-- ===== Геометрические параметры (м) — те же, что в mum_labs/arm3_kinematics.py ===== -->
  <xacro:property name="PI" value="3.141592653589793"/>
  <xacro:property name="H_PED" value="0.20"/>   <!-- высота подставки -->
  <xacro:property name="H_BASE" value="0.05"/>  <!-- высота корпуса основания (до оси q1) -->
  <xacro:property name="H_COL" value="0.05"/>   <!-- высота поворотной колонны (до оси q2) -->
  <xacro:property name="A2" value="0.12"/>      <!-- длина плеча -->
  <xacro:property name="A3" value="0.12"/>      <!-- длина предплечья -->
  <!-- D1 = H_BASE + H_COL = 0.10 м -->

  <!-- ===== Массы (кг) ===== -->
  <xacro:property name="M_BASE" value="0.50"/>
  <xacro:property name="M_L1" value="0.10"/>
  <xacro:property name="M_L2" value="0.12"/>
  <xacro:property name="M_L3" value="0.08"/>

  <!-- ===== Ограничения сервоприводов ===== -->
  <xacro:property name="EFFORT" value="1.5"/>   <!-- Н·м -->
  <xacro:property name="VEL" value="3.0"/>      <!-- рад/с -->
  <xacro:property name="DAMP" value="0.05"/>    <!-- вязкое трение в редукторе сервопривода, Н·м·с/рад -->

  <!-- ===== Материалы ===== -->
  <material name="grey"><color rgba="0.35 0.35 0.38 1"/></material>
  <material name="dark"><color rgba="0.12 0.12 0.14 1"/></material>
  <material name="orange"><color rgba="0.95 0.45 0.10 1"/></material>
  <material name="blue"><color rgba="0.15 0.35 0.80 1"/></material>
  <material name="red"><color rgba="0.85 0.10 0.10 1"/></material>

  <!-- ===== Макросы инерции ===== -->
  <xacro:macro name="box_inertia" params="m x y z ox oy oz">
    <inertial>
      <origin xyz="${ox} ${oy} ${oz}" rpy="0 0 0"/>
      <mass value="${m}"/>
      <inertia ixx="${m*(y*y+z*z)/12}" ixy="0" ixz="0"
               iyy="${m*(x*x+z*z)/12}" iyz="0"
               izz="${m*(x*x+y*y)/12}"/>
    </inertial>
  </xacro:macro>
  <xacro:macro name="cyl_inertia" params="m r h oz">
    <inertial>
      <origin xyz="0 0 ${oz}" rpy="0 0 0"/>
      <mass value="${m}"/>
      <inertia ixx="${m*(3*r*r+h*h)/12}" ixy="0" ixz="0"
               iyy="${m*(3*r*r+h*h)/12}" iyz="0"
               izz="${m*r*r/2}"/>
    </inertial>
  </xacro:macro>

  <!-- ===== Звенья ===== -->
  <link name="world"/>

  <joint name="world_to_pedestal" type="fixed">
    <parent link="world"/>
    <child link="pedestal"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="pedestal">
    <visual>
      <origin xyz="0 0 ${H_PED/2}"/>
      <geometry><box size="0.16 0.16 ${H_PED}"/></geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${H_PED/2}"/>
      <geometry><box size="0.16 0.16 ${H_PED}"/></geometry>
    </collision>
    <xacro:box_inertia m="2.0" x="0.16" y="0.16" z="${H_PED}" ox="0" oy="0" oz="${H_PED/2}"/>
  </link>

  <joint name="pedestal_to_base" type="fixed">
    <parent link="pedestal"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${H_PED}" rpy="0 0 0"/>
  </joint>

  <!-- base_link = система координат 0 (ось z0 — ось первого сервопривода) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${H_BASE/2}"/>
      <geometry><cylinder radius="0.06" length="${H_BASE}"/></geometry>
      <material name="dark"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${H_BASE/2}"/>
      <geometry><cylinder radius="0.06" length="${H_BASE}"/></geometry>
    </collision>
    <xacro:cyl_inertia m="${M_BASE}" r="0.06" h="${H_BASE}" oz="${H_BASE/2}"/>
  </link>

  <!-- Сервопривод 1: поворот вокруг z0 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 ${H_BASE}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-PI/2}" upper="${PI/2}" effort="${EFFORT}" velocity="${VEL}"/>
    <dynamics damping="${DAMP}" friction="0.0"/>
  </joint>

  <link name="link1">
    <visual>
      <origin xyz="0 0 ${H_COL/2}"/>
      <geometry><cylinder radius="0.035" length="${H_COL}"/></geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${H_COL/2}"/>
      <geometry><cylinder radius="0.035" length="${H_COL}"/></geometry>
    </collision>
    <xacro:cyl_inertia m="${M_L1}" r="0.035" h="${H_COL}" oz="${H_COL/2}"/>
  </link>

  <!-- Сервопривод 2: плечо. Ось (0,-1,0): положительный угол поднимает звено вверх
       (так ось совпадает с осью z1 таблицы ДХ при alpha1 = +pi/2). -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 ${H_COL}" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-0.3" upper="2.8" effort="${EFFORT}" velocity="${VEL}"/>
    <dynamics damping="${DAMP}" friction="0.0"/>
  </joint>

  <link name="link2">
    <visual>
      <origin xyz="${A2/2} 0 0"/>
      <geometry><box size="${A2} 0.03 0.03"/></geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="${A2/2} 0 0"/>
      <geometry><box size="${A2} 0.03 0.03"/></geometry>
    </collision>
    <xacro:box_inertia m="${M_L2}" x="${A2}" y="0.03" z="0.03" ox="${A2/2}" oy="0" oz="0"/>
  </link>

  <!-- Сервопривод 3: локоть -->
  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="${A2} 0 0" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-2.5" upper="2.5" effort="${EFFORT}" velocity="${VEL}"/>
    <dynamics damping="${DAMP}" friction="0.0"/>
  </joint>

  <link name="link3">
    <visual>
      <origin xyz="${A3/2} 0 0"/>
      <geometry><box size="${A3} 0.025 0.025"/></geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="${A3/2} 0 0"/>
      <geometry><box size="${A3} 0.025 0.025"/></geometry>
    </collision>
    <xacro:box_inertia m="${M_L3}" x="${A3}" y="0.025" z="0.025" ox="${A3/2}" oy="0" oz="0"/>
  </link>

  <!-- Рабочая точка (схват) -->
  <joint name="tool_joint" type="fixed">
    <parent link="link3"/>
    <child link="tool0"/>
    <origin xyz="${A3} 0 0" rpy="0 0 0"/>
  </joint>

  <link name="tool0">
    <visual>
      <geometry><sphere radius="0.012"/></geometry>
      <material name="red"/>
    </visual>
    <inertial>
      <mass value="0.005"/>
      <inertia ixx="1e-7" ixy="0" ixz="0" iyy="1e-7" iyz="0" izz="1e-7"/>
    </inertial>
  </link>

  <!-- ===== ros2_control ===== -->
  <xacro:macro name="arm_joint_if" params="name init">
    <joint name="${name}">
      <xacro:if value="${control_mode == 'position'}">
        <command_interface name="position"/>
      </xacro:if>
      <xacro:unless value="${control_mode == 'position'}">
        <command_interface name="effort">
          <param name="min">${-EFFORT}</param>
          <param name="max">${EFFORT}</param>
        </command_interface>
      </xacro:unless>
      <state_interface name="position">
        <param name="initial_value">${init}</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </xacro:macro>

  <ros2_control name="arm3_system" type="system">
    <hardware>
      <xacro:if value="${sim_mode == 'gazebo'}">
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </xacro:if>
      <xacro:if value="${sim_mode == 'mock'}">
        <plugin>mock_components/GenericSystem</plugin>
      </xacro:if>
    </hardware>
    <xacro:arm_joint_if name="joint1" init="0.0"/>
    <xacro:arm_joint_if name="joint2" init="0.0"/>
    <xacro:arm_joint_if name="joint3" init="0.0"/>
  </ros2_control>

  <xacro:if value="${sim_mode == 'gazebo'}">
    <gazebo>
      <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <xacro:if value="${control_mode == 'position'}">
          <parameters>$(find arm3_description)/config/arm3_controllers.yaml</parameters>
        </xacro:if>
        <xacro:if value="${control_mode == 'effort'}">
          <parameters>$(find arm3_description)/config/arm3_effort_controllers.yaml</parameters>
        </xacro:if>
        <xacro:if value="${control_mode == 'effort_jtc'}">
          <parameters>$(find arm3_description)/config/arm3_effort_jtc_controllers.yaml</parameters>
        </xacro:if>
      </plugin>
    </gazebo>

    <gazebo reference="pedestal"><visual><material><diffuse>0.35 0.35 0.38 1</diffuse><ambient>0.35 0.35 0.38 1</ambient></material></visual></gazebo>
    <gazebo reference="base_link"><visual><material><diffuse>0.12 0.12 0.14 1</diffuse><ambient>0.12 0.12 0.14 1</ambient></material></visual></gazebo>
    <gazebo reference="link1"><visual><material><diffuse>0.95 0.45 0.10 1</diffuse><ambient>0.95 0.45 0.10 1</ambient></material></visual></gazebo>
    <gazebo reference="link2"><visual><material><diffuse>0.15 0.35 0.80 1</diffuse><ambient>0.15 0.35 0.80 1</ambient></material></visual></gazebo>
    <gazebo reference="link3"><visual><material><diffuse>0.95 0.45 0.10 1</diffuse><ambient>0.95 0.45 0.10 1</ambient></material></visual></gazebo>
    <gazebo reference="tool0"><visual><material><diffuse>0.85 0.10 0.10 1</diffuse><ambient>0.85 0.10 0.10 1</ambient></material></visual></gazebo>
  </xacro:if>

</robot>
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o urdf/arm3.urdf.xacro $REPO_RAW/src/arm3_description/urdf/arm3.urdf.xacro
```

Проверьте, что xacro разворачивается без ошибок:

```bash
xacro urdf/arm3.urdf.xacro sim_mode:=mock > /tmp/arm3.urdf && check_urdf /tmp/arm3.urdf
```

`check_urdf` должен вывести дерево `world → pedestal → base_link → link1 → link2 → link3 → tool0`.

## 1.4. Контроллеры ros2_control

Три файла — три режима управления, режим выбирается при запуске.

**`config/arm3_controllers.yaml`** — позиционный режим: `JointTrajectoryController` принимает траекторию и сам интерполирует между точками.

```yaml
# Режим position: JointTrajectoryController управляет позициями сервоприводов.
controller_manager:
  ros__parameters:
    update_rate: 200  # Гц

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
```

**`config/arm3_effort_controllers.yaml`** — режим моментов: регулятор пишет студент (практическая работа № 7).

```yaml
# Режим effort: прямое задание моментов. Регулятор пишет студент (mum_labs/pid_node).
# Топик команд: /effort_controller/commands  (std_msgs/msg/Float64MultiArray, [u1, u2, u3] в Н·м)
controller_manager:
  ros__parameters:
    update_rate: 200  # Гц (частота /joint_states и цикла регулятора на Python)

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    effort_controller:
      type: effort_controllers/JointGroupEffortController

effort_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
```

**`config/arm3_effort_jtc_controllers.yaml`** — режим моментов со встроенным ПИД-регулятором контроллера.

```yaml
# Режим effort_jtc: JointTrajectoryController с выходом по моменту.
# Внутри контроллера работает ПИД-регулятор (gains), коэффициенты — предмет настройки в ПР №7.
controller_manager:
  ros__parameters:
    update_rate: 500  # Гц

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    gains:
      joint1: {p: 1.0, i: 0.0, d: 0.03, i_clamp: 0.5}
      joint2: {p: 6.0, i: 0.0, d: 0.25, i_clamp: 0.5}
      joint3: {p: 3.0, i: 0.0, d: 0.06, i_clamp: 0.5}
```

## 1.5. Launch-файлы

**`launch/display.launch.py`** — модель в RViz с ползунками углов, без симулятора:

```python
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
```

**`launch/gazebo.launch.py`** — запуск Gazebo, появление робота, мост времени `/clock`, запуск контроллеров по очереди (сначала `joint_state_broadcaster`, затем основной контроллер выбранного режима):

<details>
<summary>Показать содержимое <code>launch/gazebo.launch.py</code></summary>

```python
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
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o launch/gazebo.launch.py $REPO_RAW/src/arm3_description/launch/gazebo.launch.py
```

**`rviz/arm3.rviz`** — настройки окна RViz (сетка, модель робота, системы координат TF):

Файл длинный и не требует ручного набора — скачайте его (из папки пакета):

```bash
curl -sL --create-dirs -o rviz/arm3.rviz $REPO_RAW/src/arm3_description/rviz/arm3.rviz
```

## 1.6. Сборка и проверка arm3_description

```bash
cd ~/mum_ws
colcon build --symlink-install
source install/setup.bash
echo "source ~/mum_ws/install/setup.bash" >> ~/.bashrc     # один раз
```

`--symlink-install` делает в `install/` ссылки на ваши файлы: после правки URDF, YAML или Python пересобирать не нужно, достаточно перезапустить launch.

**Проверка 1 — модель в RViz:**

```bash
ros2 launch arm3_description display.launch.py
```

Двигайте ползунки joint1…joint3 — звенья должны поворачиваться, плечо и локоть при положительных углах — подниматься.

**Проверка 2 — Gazebo и контроллеры:**

```bash
ros2 launch arm3_description gazebo.launch.py rviz:=true
```

Во втором терминале:

```bash
ros2 control list_controllers
```

Должно быть `joint_state_broadcaster ... active` и `arm_controller ... active`. Отправьте траекторию из одной точки:

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3], points: [{positions: [0.5, 0.8, -1.0], time_from_start: {sec: 2}}]}"
```

Манипулятор в Gazebo должен за 2 с повернуться и согнуться.

**Проверка 3 — режим моментов:** `ros2 launch arm3_description gazebo.launch.py control_mode:=effort`. Рука падает под действием силы тяжести — регулятора ещё нет, так и должно быть.

## 1.7. Пакет arm3_moveit_config

MoveIt 2 нужны: модель робота (берём из `arm3_description`), семантическое описание SRDF, решатель обратной задачи, ограничения скоростей и ускорений и сведения о контроллере.

```bash
cd ~/mum_ws/src
ros2 pkg create arm3_moveit_config --build-type ament_cmake --license Apache-2.0
cd arm3_moveit_config
rm -rf include src
mkdir -p config launch
```

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>arm3_moveit_config</name>
  <version>1.0.0</version>
  <description>Конфигурация MoveIt 2 для учебного манипулятора arm3</description>
  <maintainer email="ssydykanov@gmail.com">Sydykanov S.</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>arm3_description</exec_depend>
  <exec_depend>moveit_ros_move_group</exec_depend>
  <exec_depend>moveit_kinematics</exec_depend>
  <exec_depend>moveit_planners_ompl</exec_depend>
  <exec_depend>moveit_ros_visualization</exec_depend>
  <exec_depend>moveit_simple_controller_manager</exec_depend>
  <exec_depend>moveit_configs_utils</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

```cmake
cmake_minimum_required(VERSION 3.8)
project(arm3_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
```

**`config/arm3.srdf`** — группа `arm` (цепь `base_link → tool0`), именованные позы `home`, `ready`, `up`, `left`, и пары соседних звеньев, для которых не проверяются столкновения. Этот же файл можно получить в мастере `ros2 launch moveit_setup_assistant setup_assistant.launch.py`.

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!-- SRDF для arm3. Можно сгенерировать заново в MoveIt Setup Assistant (ПР №6, п. 6). -->
<robot name="arm3">
  <!-- Планируемая группа: кинематическая цепь от основания до схвата -->
  <group name="arm">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>

  <!-- Именованные конфигурации -->
  <group_state name="home" group="arm">
    <joint name="joint1" value="0"/>
    <joint name="joint2" value="0"/>
    <joint name="joint3" value="0"/>
  </group_state>
  <group_state name="ready" group="arm">
    <joint name="joint1" value="0"/>
    <joint name="joint2" value="0.9"/>
    <joint name="joint3" value="-1.5"/>
  </group_state>
  <group_state name="up" group="arm">
    <joint name="joint1" value="0"/>
    <joint name="joint2" value="1.5708"/>
    <joint name="joint3" value="0"/>
  </group_state>
  <group_state name="left" group="arm">
    <joint name="joint1" value="1.2"/>
    <joint name="joint2" value="0.6"/>
    <joint name="joint3" value="-1.0"/>
  </group_state>

  <!-- Пары звеньев, для которых не проверяются столкновения -->
  <disable_collisions link1="pedestal" link2="base_link" reason="Adjacent"/>
  <disable_collisions link1="pedestal" link2="link1" reason="Never"/>
  <disable_collisions link1="base_link" link2="link1" reason="Adjacent"/>
  <disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
  <disable_collisions link1="link2" link2="link3" reason="Adjacent"/>
  <disable_collisions link1="link3" link2="tool0" reason="Adjacent"/>
</robot>
```

**`config/kinematics.yaml`** — решатель KDL. У arm3 три степени подвижности, поэтому решаем задачу только по положению схвата (`position_only_ik: true`).

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  # У манипулятора 3 степени подвижности: задаём только положение схвата, ориентацию игнорируем
  position_only_ik: true
```

**`config/joint_limits.yaml`** — ограничения для построения траектории по времени.

```yaml
# Ограничения для планировщика (переопределяют/дополняют URDF).
# Ускорения обязательны для параметризации траектории по времени.
default_velocity_scaling_factor: 0.5
default_acceleration_scaling_factor: 0.5

joint_limits:
  joint1:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: true
    max_acceleration: 4.0
  joint2:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: true
    max_acceleration: 4.0
  joint3:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: true
    max_acceleration: 4.0
```

**`config/moveit_controllers.yaml`** — каким контроллером MoveIt исполняет траекторию (тот же `arm_controller`).

```yaml
trajectory_execution:
  allowed_execution_duration_scaling: 1.5
  allowed_goal_duration_margin: 1.0
  allowed_start_tolerance: 0.05
  execution_duration_monitoring: true

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller

  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - joint1
      - joint2
      - joint3
```

**`config/moveit.rviz`** — окно RViz с панелью MotionPlanning:

Файл длинный и не требует ручного набора — скачайте его (из папки пакета):

```bash
curl -sL --create-dirs -o config/moveit.rviz $REPO_RAW/src/arm3_moveit_config/config/moveit.rviz
```

**`launch/common.py`** — общая часть: собирает все настройки через `MoveItConfigsBuilder` и запускает `move_group` и RViz. Здесь же список алгоритмов OMPL, доступных группе `arm`.

```python
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
```

**`launch/demo.launch.py`** — MoveIt без симулятора (фиктивное железо `mock_components`):

```python
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
```

**`launch/gazebo.launch.py`** — MoveIt управляет манипулятором в Gazebo:

```python
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
```

## 1.8. Сборка и проверка MoveIt

```bash
cd ~/mum_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch arm3_moveit_config demo.launch.py
```

В RViz на панели **MotionPlanning → Planning** выберите *Goal State: ready* и нажмите **Plan & Execute**. Затем перетащите шар-маркер у схвата и снова **Plan & Execute**.

После этого проверьте вариант с Gazebo: `ros2 launch arm3_moveit_config gazebo.launch.py`.

## Итог шага 1

```
mum_ws/src/
├── arm3_description/
│   ├── CMakeLists.txt  package.xml
│   ├── urdf/arm3.urdf.xacro
│   ├── config/arm3_controllers.yaml  arm3_effort_controllers.yaml  arm3_effort_jtc_controllers.yaml
│   ├── launch/display.launch.py  gazebo.launch.py
│   └── rviz/arm3.rviz
└── arm3_moveit_config/
    ├── CMakeLists.txt  package.xml
    ├── config/arm3.srdf  kinematics.yaml  joint_limits.yaml  moveit_controllers.yaml  moveit.rviz
    └── launch/common.py  demo.launch.py  gazebo.launch.py
```

Сверить свой результат с эталоном: в клоне репозитория `git checkout step-1`.

**Контрольные вопросы:** чем `visual` отличается от `collision`? Зачем `world` и неподвижное сочленение `world_to_pedestal`? Что делает `joint_state_broadcaster`? Почему в `gazebo.launch.py` контроллеры запускаются только после появления робота?

[← README](../README.md) · [Шаг 2. Мобильный робот →](step2_mobile_robot.md)
