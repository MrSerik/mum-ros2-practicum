"""ПР №2 и №4. Прямая задача кинематики и матрица Якоби в реальном времени.

Узел читает углы сервоприводов из /joint_states, вычисляет T_0^3 по таблице ДХ,
сравнивает положение схвата с TF (base_link -> tool0), которое строит robot_state_publisher по URDF,
и рисует системы координат ДХ в RViz (топик /lab_markers).
Дополнительно выводит матрицу Якоби Jv, det(Jv) и меру манипулируемости.

  ros2 run mum_labs fk_node
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from mum_labs import arm3_kinematics as kin
from mum_labs.common import arrow_marker, joint_vector, sphere_marker, text_marker


class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')
        self.declare_parameter('print_period', 1.0)
        self.declare_parameter('axis_length', 0.05)
        self.declare_parameter('singularity_threshold', 1e-4)
        self.q = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.pub = self.create_publisher(MarkerArray, '/lab_markers', 10)
        period = self.get_parameter('print_period').value
        self.create_timer(period, self.on_timer)
        self.get_logger().info('Ожидаю /joint_states ...')

    def on_joint_states(self, msg):
        q = joint_vector(msg)
        if q is not None:
            self.q = q

    def on_timer(self):
        if self.q is None:
            return
        q = self.q
        frames = kin.fk_all(q)
        T = frames[-1]
        p_dh = T[:3, 3]

        # ---- положение по TF (URDF) ----
        p_tf = None
        try:
            tr = self.tf_buffer.lookup_transform('base_link', 'tool0', Time())
            t = tr.transform.translation
            p_tf = np.array([t.x, t.y, t.z])
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'TF base_link->tool0 недоступно: {e}', throttle_duration_sec=5.0)

        Jv = kin.jacobian(q)[:3, :]
        det = float(np.linalg.det(Jv))
        w = kin.manipulability(q)

        lines = [
            f'q = [{q[0]:+.3f}, {q[1]:+.3f}, {q[2]:+.3f}] рад  '
            f'({np.degrees(q[0]):+.1f}°, {np.degrees(q[1]):+.1f}°, {np.degrees(q[2]):+.1f}°)',
            'T_0^3 (ДХ) =\n' + np.array2string(T, precision=4, suppress_small=True),
            f'p (ДХ) = [{p_dh[0]:.4f}, {p_dh[1]:.4f}, {p_dh[2]:.4f}] м',
        ]
        if p_tf is not None:
            err = np.linalg.norm(p_dh - p_tf) * 1000.0
            lines.append(f'p (TF) = [{p_tf[0]:.4f}, {p_tf[1]:.4f}, {p_tf[2]:.4f}] м   '
                         f'|ошибка| = {err:.3f} мм')
        lines.append('Jv =\n' + np.array2string(Jv, precision=4, suppress_small=True))
        lines.append(f'det(Jv) = {det:+.6f}   манипулируемость w = {w:.6f}')
        if abs(det) < self.get_parameter('singularity_threshold').value:
            lines.append('!!! Конфигурация близка к СИНГУЛЯРНОЙ (det Jv ≈ 0)')
        self.get_logger().info('\n' + '\n'.join(lines))

        # ---- маркеры: системы координат ДХ 0..3 ----
        L = self.get_parameter('axis_length').value
        ma = MarkerArray()
        mid = 0
        for i, F in enumerate(frames):
            o = F[:3, 3]
            for k, rgba in enumerate([(1, 0, 0, 1), (0, 1, 0, 1), (0, 0.4, 1, 1)]):
                ma.markers.append(arrow_marker('base_link', mid, o, o + L * F[:3, k], rgba, ns='dh_frames'))
                mid += 1
            ma.markers.append(text_marker('base_link', mid, o + np.array([0, 0, 0.02]), f'o{i}', ns='dh_frames'))
            mid += 1
        ma.markers.append(sphere_marker('base_link', mid, p_dh, (1, 1, 0, 0.8), d=0.025, ns='dh_frames'))
        self.pub.publish(ma)


def main():
    rclpy.init()
    node = FKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
