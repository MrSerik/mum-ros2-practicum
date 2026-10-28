"""ПР №8 (часть 1). Кинематика дифференциального привода и одометрия по энкодерам колёс.

Узел интегрирует собственную одометрию по углам колёс (/joint_states):
    ds = r (dφR + dφL) / 2,   dθ = r (dφR - dφL) / L
    x += ds cos(θ + dθ/2),   y += ds sin(θ + dθ/2),   θ += dθ
и сравнивает её с «истинным» положением робота из Gazebo (/ground_truth).
Можно задать ошибку калибровки радиуса колеса (r_error, %) и колеи (L_error, %),
чтобы увидеть накопление ошибки одометрии.
Опционально выполняет движение по шаблону (pattern: none | square | circle) по разомкнутой схеме.

  ros2 run mum_labs diff_odometry_node --ros-args -p pattern:=square -p r_error:=2.0
Пути выводятся в RViz: /my_odom_path (зелёный) и /gt_path (красный).
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mum_labs.common import stamp_name, yaw_from_quat

WHEELS = ['wheel_left_joint', 'wheel_right_joint']


class DiffOdometry(Node):
    def __init__(self):
        super().__init__('diff_odometry_node')
        d = self.declare_parameter
        d('wheel_radius', 0.05)
        d('wheel_separation', 0.30)
        d('r_error', 0.0)        # % ошибки радиуса в модели одометрии
        d('L_error', 0.0)        # % ошибки колеи
        d('pattern', 'none')     # none | square | circle
        d('side', 1.5)           # м, сторона квадрата
        d('v', 0.2)              # м/с
        d('w', 0.5)              # рад/с
        d('laps', 1)

        self.x = self.y = self.th = 0.0
        self.phi_prev = None
        self.gt0 = None
        self.gt = None
        self.my_path, self.gt_path = Path(), Path()
        self.my_path.header.frame_id = self.gt_path.header.frame_id = 'odom'
        self.hist = []

        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.create_subscription(Odometry, '/ground_truth', self.on_gt, 50)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.my_path_pub = self.create_publisher(Path, '/my_odom_path', 10)
        self.gt_path_pub = self.create_publisher(Path, '/gt_path', 10)
        self.my_odom_pub = self.create_publisher(Odometry, '/my_odom', 10)
        self.create_timer(0.2, self.publish_paths)

        self.plan = self.make_plan()
        self.plan_idx = 0
        self.plan_t0 = None
        self.create_timer(0.05, self.drive)

    def gp(self, n):
        return self.get_parameter(n).value

    # ---------- одометрия ----------
    def on_js(self, msg):
        try:
            phi = np.array([msg.position[msg.name.index(n)] for n in WHEELS])
        except ValueError:
            return
        if self.phi_prev is None:
            self.phi_prev = phi
            return
        dphi = phi - self.phi_prev
        self.phi_prev = phi
        r = self.gp('wheel_radius') * (1 + self.gp('r_error') / 100.0)
        L = self.gp('wheel_separation') * (1 + self.gp('L_error') / 100.0)
        ds = r * (dphi[1] + dphi[0]) / 2.0
        dth = r * (dphi[1] - dphi[0]) / L
        self.x += ds * np.cos(self.th + dth / 2)
        self.y += ds * np.sin(self.th + dth / 2)
        self.th += dth

        od = Odometry()
        od.header.stamp = msg.header.stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_footprint'
        od.pose.pose.position.x, od.pose.pose.position.y = self.x, self.y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = np.sin(self.th / 2), np.cos(self.th / 2)
        self.my_odom_pub.publish(od)

    def on_gt(self, msg):
        p = msg.pose.pose
        x, y, th = p.position.x, p.position.y, yaw_from_quat(p.orientation)
        if self.gt0 is None:
            self.gt0 = (x, y, th)
        x0, y0, th0 = self.gt0
        # истинная поза в системе координат начального положения робота (= odom)
        dx, dy = x - x0, y - y0
        self.gt = (np.cos(th0) * dx + np.sin(th0) * dy, -np.sin(th0) * dx + np.cos(th0) * dy, th - th0)

    def publish_paths(self):
        now = self.get_clock().now().to_msg()
        for path, pose, pub in ((self.my_path, (self.x, self.y), self.my_path_pub),
                                (self.gt_path, self.gt[:2] if self.gt else None, self.gt_path_pub)):
            if pose is None:
                continue
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.header.stamp = now
            ps.pose.position.x, ps.pose.position.y = float(pose[0]), float(pose[1])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
            path.header.stamp = now
            pub.publish(path)
        if self.gt:
            self.hist.append((self.x, self.y, self.th, *self.gt))

    # ---------- движение по шаблону (разомкнутое управление) ----------
    def make_plan(self):
        v, w, a = self.gp('v'), self.gp('w'), self.gp('side')
        pat, laps = self.gp('pattern'), int(self.gp('laps'))
        plan = []
        if pat == 'square':
            for _ in range(4 * laps):
                plan.append((v, 0.0, a / v))
                plan.append((0.0, w, (np.pi / 2) / w))
        elif pat == 'circle':
            plan.append((v, w, laps * 2 * np.pi / w))
        if plan:
            plan.append((0.0, 0.0, 1.0))
        return plan

    def drive(self):
        if not self.plan or self.plan_idx >= len(self.plan) or self.gt is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.plan_t0 is None:
            self.plan_t0 = t
        v, w, dur = self.plan[self.plan_idx]
        if t - self.plan_t0 >= dur:
            self.plan_idx += 1
            self.plan_t0 = t
            if self.plan_idx >= len(self.plan):
                self.cmd_pub.publish(Twist())
                self.finish()
            return
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)

    def finish(self):
        gx, gy, gth = self.gt
        err = np.hypot(self.x - gx, self.y - gy)
        self.get_logger().info(
            f'Шаблон выполнен.\n  одометрия: x={self.x:+.3f} y={self.y:+.3f} θ={np.degrees(self.th):+.1f}°'
            f'\n  истина   : x={gx:+.3f} y={gy:+.3f} θ={np.degrees(gth):+.1f}°'
            f'\n  ошибка положения {err * 100:.1f} см')
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        H = np.array(self.hist)
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.plot(H[:, 0], H[:, 1], label='одометрия (энкодеры)')
        ax.plot(H[:, 3], H[:, 4], '--', label='истина (Gazebo)')
        ax.set_aspect('equal'); ax.grid(True); ax.legend()
        ax.set_xlabel('x, м'); ax.set_ylabel('y, м')
        ax.set_title(f'pattern={self.gp("pattern")}, r_error={self.gp("r_error")} %, '
                     f'L_error={self.gp("L_error")} %')
        path = stamp_name('odometry', 'png')
        fig.savefig(path, dpi=120)
        self.get_logger().info(f'График: {path}')


def main():
    rclpy.init()
    node = DiffOdometry()
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
