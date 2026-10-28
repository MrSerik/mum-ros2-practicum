"""ПР №3. Обратная задача кинематики: перемещение схвата в заданную точку.

Цель задаётся:
  1) топиком /ik_target (geometry_msgs/Point, координаты в base_link):
       ros2 topic pub --once /ik_target geometry_msgs/msg/Point "{x: 0.15, y: 0.05, z: 0.12}"
  2) инструментом RViz «Publish Point» (топик /clicked_point) — щёлкните по серой плоскости;
     высота точки берётся из параметра click_z.

Параметры:  elbow (up|down), move_time (с), click_z (м, в base_link)
  ros2 run mum_labs ik_node --ros-args -p elbow:=down
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from mum_labs import arm3_kinematics as kin
from mum_labs.common import color, joint_vector, make_trajectory, sphere_marker


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.declare_parameter('elbow', 'up')
        self.declare_parameter('move_time', 2.0)
        self.declare_parameter('click_z', 0.10)
        self.q = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/lab_markers', 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(Point, '/ik_target', self.on_target, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.on_click, 10)
        self.create_timer(1.0, self.publish_plane)
        self.get_logger().info('Готов. Задайте цель в /ik_target или щёлкните Publish Point в RViz.')

    def on_js(self, msg):
        q = joint_vector(msg)
        if q is not None:
            self.q = q

    def publish_plane(self):
        """Полупрозрачная плоскость-мишень для инструмента Publish Point."""
        z = self.get_parameter('click_z').value
        m = Marker()
        m.header.frame_id = 'base_link'
        m.ns, m.id, m.type, m.action = 'ik_plane', 0, Marker.CYLINDER, Marker.ADD
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        R = kin.A2 + kin.A3
        m.scale.x = m.scale.y = 2 * R
        m.scale.z = 0.002
        m.color = color(0.6, 0.6, 0.6, 0.25)
        self.marker_pub.publish(MarkerArray(markers=[m]))

    def on_click(self, msg):
        try:
            tr = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Нет TF {msg.header.frame_id}->base_link: {e}')
            return
        t = tr.transform.translation
        # опорные кадры world и base_link отличаются только сдвигом (поворотов нет)
        p = np.array([msg.point.x + t.x, msg.point.y + t.y, self.get_parameter('click_z').value])
        self.solve_and_move(p)

    def on_target(self, msg):
        self.solve_and_move(np.array([msg.x, msg.y, msg.z]))

    def solve_and_move(self, p):
        elbow = self.get_parameter('elbow').value
        q = kin.ik(p, elbow)
        ok = q is not None and kin.within_limits(q)
        self.marker_pub.publish(MarkerArray(markers=[
            sphere_marker('base_link', 1, p, (0, 1, 0, 0.9) if ok else (1, 0, 0, 0.9), d=0.03, ns='ik_target')]))
        if q is None:
            self.get_logger().warn(f'Точка {np.round(p, 3)} вне рабочей зоны (|c3| > 1).')
            return
        if not kin.within_limits(q):
            self.get_logger().warn(f'Решение q = {np.round(q, 3)} выходит за пределы сочленений '
                                   f'[{np.round(kin.Q_MIN, 2)}, {np.round(kin.Q_MAX, 2)}]. '
                                   f'Попробуйте elbow:={"down" if elbow == "up" else "up"}.')
            return
        check = kin.fk(q)[:3, 3]
        self.get_logger().info(
            f'p* = {np.round(p, 4)} -> q = {np.round(q, 4)} рад = {np.round(np.degrees(q), 1)}°; '
            f'проверка FK(q) = {np.round(check, 4)}')
        T = self.get_parameter('move_time').value
        self.traj_pub.publish(make_trajectory([T], [q], V=[np.zeros(3)]))


def main():
    rclpy.init()
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
