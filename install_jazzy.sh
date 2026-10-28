#!/usr/bin/env bash
# Установка ROS 2 Jazzy + Gazebo Harmonic + Nav2 + MoveIt 2 для практикума (Ubuntu 24.04)
set -e

# 0. apt должен быть в исправном состоянии, иначе установка оборвётся на первой же команде
if [ -n "$(dpkg --audit 2>/dev/null)" ]; then
  echo "ОШИБКА: в системе есть недонастроенные пакеты (dpkg --audit):"
  dpkg --audit
  echo "Сначала исправьте их (см. раздел «Типичные неполадки» методички), затем запустите скрипт снова."
  exit 1
fi
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
# полный apt upgrade не делаем: он тянет новое ядро и драйвер NVIDIA, а для ROS не нужен
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-moveit ros-jazzy-moveit-resources-panda-moveit-config \
  ros-jazzy-cv-bridge python3-opencv python3-matplotlib python3-numpy liburdfdom-tools
grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
# Если скрипт лежит в корне рабочего пространства с пакетами (готовый клон репозитория) — собираем его.
# Если скрипт скачан отдельно (путь «соберу сам по шагам») — только устанавливаем ROS 2.
WS="$(cd "$(dirname "$0")" && pwd)"
if [ -d "$WS/src" ] && [ -n "$(find "$WS/src" -name package.xml -print -quit)" ]; then
  cd "$WS"
  rosdep install --from-paths src -y --ignore-src || true
  colcon build --symlink-install
  grep -q "$WS/install/setup.bash" ~/.bashrc || echo "source $WS/install/setup.bash" >> ~/.bashrc
  echo "Готово. Откройте новый терминал и проверьте: ros2 launch arm3_description display.launch.py"
else
  echo "ROS 2 Jazzy установлен. Откройте новый терминал и переходите к шагу 1: docs/step1_manipulator.md"
fi
