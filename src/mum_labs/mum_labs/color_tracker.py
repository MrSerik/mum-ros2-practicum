"""ПР №10. Техническое зрение (гл. 4): цветовая бинаризация, координаты объекта, дальность
по карте глубины и визуальное управление мобильным роботом.

Алгоритм (как в п. 4.2 пособия):
  1) перевод RGB -> HSV, бинаризация по порогам цвета (маска);
  2) морфологическая фильтрация, поиск наибольшей связной области;
  3) центр масс области по моментам: u = M10/M00, v = M01/M00;
  4) дальность Z — медиана карты глубины в окрестности (u, v) (аналог времяпролётной камеры, п. 4.3);
  5) координаты в оптической СК камеры:  X = (u - cx) Z / fx,  Y = (v - cy) Z / fy;
  6) перевод точки в base_link через TF и публикация /target_point;
  7) (follow:=true) П-регулятор: ω = -k_w (u - cx)/cx,  v = k_v (Z - Z*).

  ros2 run mum_labs color_tracker --ros-args -p color:=red -p follow:=true
Маска публикуется в /camera/mask (смотрите в RViz или rqt_image_view).
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

# Пороги HSV (OpenCV: H 0..179, S 0..255, V 0..255). Красный «переходит» через 0, поэтому 2 диапазона.
COLORS = {
    'red': [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    'green': [((40, 80, 50), (85, 255, 255))],
    'blue': [((100, 120, 50), (130, 255, 255))],
}


class ColorTracker(Node):
    def __init__(self):
        super().__init__('color_tracker')
        d = self.declare_parameter
        d('color', 'red')
        d('min_area', 150)
        d('follow', False)
        d('stop_distance', 0.6)
        d('k_v', 0.5)
        d('k_w', 1.2)
        d('search_w', 0.4)      # рад/с — вращение на месте, если объект не виден
        self.bridge = CvBridge()
        self.K = None
        self.depth = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, '/camera/image', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth_image', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.mask_pub = self.create_publisher(Image, '/camera/mask', 10)
        self.pt_pub = self.create_publisher(PointStamped, '/target_point', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Ожидаю изображение /camera/image ...')

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)

    def on_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)

    def on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        ranges = COLORS.get(self.get_parameter('color').value, COLORS['red'])
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
        target = None
        if n > 1:
            i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            area = stats[i, cv2.CC_STAT_AREA]
            if area >= self.get_parameter('min_area').value:
                M = cv2.moments((labels == i).astype(np.uint8))
                u, v = M['m10'] / M['m00'], M['m01'] / M['m00']
                target = (u, v, area)

        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        h, w = mask.shape
        cmd = Twist()
        if target is None:
            if self.get_parameter('follow').value:
                cmd.angular.z = float(self.get_parameter('search_w').value)
        else:
            u, v, area = target
            cv2.circle(vis, (int(u), int(v)), 8, (0, 0, 255), 2)
            Z = self.depth_at(u, v)
            info = f'u={u:.0f} v={v:.0f} S={area}px'
            if Z is not None and self.K is not None:
                fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
                X, Y = (u - cx) * Z / fx, (v - cy) * Z / fy
                info += f'  Z={Z:.2f} м'
                self.publish_point(msg, X, Y, Z)
            self.get_logger().info(info, throttle_duration_sec=1.0)
            if self.get_parameter('follow').value:
                cx = self.K[0, 2] if self.K is not None else w / 2
                cmd.angular.z = float(-self.get_parameter('k_w').value * (u - cx) / cx)
                if Z is not None:
                    e = Z - self.get_parameter('stop_distance').value
                    cmd.linear.x = float(np.clip(self.get_parameter('k_v').value * e, -0.1, 0.3))
        if self.get_parameter('follow').value:
            self.cmd_pub.publish(cmd)
        out = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        out.header = msg.header
        self.mask_pub.publish(out)

    def depth_at(self, u, v, r=4):
        if self.depth is None:
            return None
        h, w = self.depth.shape[:2]
        u0, v0 = int(np.clip(u, r, w - r - 1)), int(np.clip(v, r, h - r - 1))
        patch = self.depth[v0 - r:v0 + r + 1, u0 - r:u0 + r + 1]
        patch = patch[np.isfinite(patch) & (patch > 0.05)]
        return float(np.median(patch)) if patch.size else None

    def publish_point(self, img_msg, X, Y, Z):
        ps = PointStamped()
        ps.header = img_msg.header           # frame_id = camera_optical_link
        ps.point.x, ps.point.y, ps.point.z = float(X), float(Y), float(Z)
        try:
            tr = self.tf_buffer.lookup_transform('base_link', ps.header.frame_id, Time())
            q = tr.transform.rotation
            t = tr.transform.translation
            R = quat_to_rot(q.x, q.y, q.z, q.w)
            p = R @ np.array([X, Y, Z]) + np.array([t.x, t.y, t.z])
            out = PointStamped()
            out.header.stamp = img_msg.header.stamp
            out.header.frame_id = 'base_link'
            out.point.x, out.point.y, out.point.z = (float(c) for c in p)
            self.pt_pub.publish(out)
            self.get_logger().info(f'Объект в base_link: x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.2f} м',
                                   throttle_duration_sec=1.0)
        except Exception:  # noqa: BLE001
            self.pt_pub.publish(ps)


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = ColorTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
