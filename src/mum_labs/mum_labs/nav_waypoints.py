"""ПР №9. Навигация Nav2 через Python API (nav2_simple_commander).

Режимы:
  path_stats — запросить у планировщика путь из текущей позы в каждую точку и вывести длину пути,
               число узлов и время планирования (сравнение Dijkstra / A* / Smac);
  waypoints  — объехать точки по очереди (FollowWaypoints) и замерить время и длину пройденного пути.

Точки задаются в СК map плоским списком [x1, y1, yaw1, x2, y2, yaw2, ...].
  ros2 run mum_labs nav_waypoints --ros-args -p mode:=path_stats -p use_sim_time:=true
  ros2 run mum_labs nav_waypoints --ros-args -p mode:=waypoints -p use_sim_time:=true
При работе с готовой картой (AMCL) добавьте -p localizer:=amcl -p initial_pose:="[0.0, 0.0, 0.0]".
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

# Точки по умолчанию — в СК map, начало которой совпадает со стартом робота (мир: x=-3, y=0)
DEFAULT_POINTS = [2.0, -1.5, 0.0,     # у ящика в юго-западной части
                  3.5, 2.0, 1.57,     # за перегородкой 1 (север)
                  5.5, 0.5, 0.0,      # восточная комната
                  0.0, 0.0, 3.14]     # возврат на старт


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def path_length(path):
    pts = np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))) if len(pts) > 1 else 0.0


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.declare_parameter('mode', 'waypoints')
    nav.declare_parameter('points', DEFAULT_POINTS)
    nav.declare_parameter('localizer', 'slam')       # slam | amcl
    nav.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
    nav.declare_parameter('planner_id', '')
    mode = nav.get_parameter('mode').value
    pts = np.array(nav.get_parameter('points').value, dtype=float).reshape(-1, 3)

    if nav.get_parameter('localizer').value == 'amcl':
        ip = nav.get_parameter('initial_pose').value
        nav.setInitialPose(make_pose(nav, *ip))
        nav.waitUntilNav2Active()
    else:
        # со SLAM локализатор не lifecycle-узел amcl — ждём только bt_navigator
        nav.waitUntilNav2Active(localizer='robot_localization')

    # пройденный путь по одометрии
    travelled = {'d': 0.0, 'prev': None}

    def on_odom(msg):
        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        if travelled['prev'] is not None:
            travelled['d'] += float(np.linalg.norm(p - travelled['prev']))
        travelled['prev'] = p
    nav.create_subscription(Odometry, '/odom', on_odom, 20)

    poses = [make_pose(nav, *row) for row in pts]

    if mode == 'path_stats':
        planner = nav.get_parameter('planner_id').value
        nav.info(f'Планировщик: {planner or "по умолчанию (GridBased)"}')
        for i, goal in enumerate(poses):
            t0 = time.time()
            path = nav.getPath(PoseStamped(), goal, planner_id=planner, use_start=False)
            dt = (time.time() - t0) * 1000
            if path is None:
                nav.warn(f'Точка {i + 1}: путь не найден')
                continue
            nav.info(f'Точка {i + 1} {pts[i, :2]}: длина пути {path_length(path):.2f} м, '
                     f'узлов {len(path.poses)}, время запроса {dt:.0f} мс')
    else:
        t0 = time.time()
        nav.followWaypoints(poses)
        while not nav.isTaskComplete():   # isTaskComplete() сам обрабатывает входящие сообщения
            fb = nav.getFeedback()
            if fb is not None:
                nav.get_logger().info(f'Едем к точке {fb.current_waypoint + 1}/{len(poses)}',
                                      throttle_duration_sec=5.0)
        res = nav.getResult()
        status = {TaskResult.SUCCEEDED: 'успешно', TaskResult.CANCELED: 'отменено',
                  TaskResult.FAILED: 'ошибка'}.get(res, str(res))
        nav.info(f'Маршрут завершён: {status}; время {time.time() - t0:.1f} с; '
                 f'пройдено по одометрии {travelled["d"]:.2f} м')

    nav.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
