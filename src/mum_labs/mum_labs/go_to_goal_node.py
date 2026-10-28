"""ПР №8 (часть 2). Замкнутое управление мобильным роботом: выход в заданную позу.

Ошибка в полярных координатах (ρ — расстояние до цели, α — угол на цель относительно курса,
β — угол между направлением на цель и требуемой ориентацией):
    ρ = sqrt(dx² + dy²),   α = atan2(dy, dx) - θ,   β = θ* - θ - α
Закон управления (устойчив при kρ > 0, kβ < 0, kα - kρ > 0):
    v = kρ ρ,   ω = kα α + kβ β
Обратная связь — одометрия /odom. Цель задаётся параметрами или инструментом RViz «2D Goal Pose»
(/goal_pose, фиксированная СК odom).

  ros2 run mum_labs go_to_goal_node --ros-args -p goal:="[1.5, 1.0, 1.57]"
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from mum_labs.common import stamp_name, wrap_angle, yaw_from_quat


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal_node')
        d = self.declare_parameter
        d('goal', [1.5, 1.0, 1.57])
        d('k_rho', 0.5)
        d('k_alpha', 1.5)
        d('k_beta', -0.4)
        d('v_max', 0.3)
        d('w_max', 1.5)
        d('tol_pos', 0.03)
        d('tol_yaw', 0.05)
        self.pose = None
        self.goal = np.array(self.get_parameter('goal').value, dtype=float)
        self.active = True
        self.hist = []
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.control)
        self.get_logger().info(f'Цель: x={self.goal[0]}, y={self.goal[1]}, θ={self.goal[2]} рад')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = np.array([p.position.x, p.position.y, yaw_from_quat(p.orientation)])

    def on_goal(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y, yaw_from_quat(msg.pose.orientation)])
        self.active = True
        self.hist = []
        self.get_logger().info(f'Новая цель: {np.round(self.goal, 3)}')

    def control(self):
        if self.pose is None or not self.active:
            return
        x, y, th = self.pose
        gx, gy, gth = self.goal
        dx, dy = gx - x, gy - y
        rho = np.hypot(dx, dy)
        alpha = wrap_angle(np.arctan2(dy, dx) - th)
        beta = wrap_angle(gth - th - alpha)
        kr, ka, kb = (self.get_parameter(n).value for n in ('k_rho', 'k_alpha', 'k_beta'))

        backwards = abs(alpha) > np.pi / 2       # цель позади — едем задним ходом
        if backwards:
            alpha = wrap_angle(alpha + np.pi)
            beta = wrap_angle(beta + np.pi)
        v = kr * rho * (-1.0 if backwards else 1.0)
        w = ka * alpha + kb * beta

        if rho < self.get_parameter('tol_pos').value:
            v = 0.0
            e_yaw = wrap_angle(gth - th)
            w = 1.5 * e_yaw
            if abs(e_yaw) < self.get_parameter('tol_yaw').value:
                self.pub.publish(Twist())
                self.active = False
                self.get_logger().info(f'Цель достигнута: поза {np.round(self.pose, 3)}')
                self.save()
                return

        cmd = Twist()
        cmd.linear.x = float(np.clip(v, -self.get_parameter('v_max').value, self.get_parameter('v_max').value))
        cmd.angular.z = float(np.clip(w, -self.get_parameter('w_max').value, self.get_parameter('w_max').value))
        self.pub.publish(cmd)
        self.hist.append((x, y, th, rho, alpha, beta, cmd.linear.x, cmd.angular.z))

    def save(self):
        if len(self.hist) < 2:
            return
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        H = np.array(self.hist)
        fig, ax = plt.subplots(1, 2, figsize=(12, 5))
        ax[0].plot(H[:, 0], H[:, 1])
        ax[0].plot(*self.goal[:2], 'r*', ms=14)
        ax[0].set_aspect('equal'); ax[0].grid(True); ax[0].set_xlabel('x, м'); ax[0].set_ylabel('y, м')
        t = np.arange(len(H)) * 0.05
        for k, lab in ((3, 'ρ, м'), (4, 'α, рад'), (5, 'β, рад')):
            ax[1].plot(t, H[:, k], label=lab)
        ax[1].grid(True); ax[1].legend(); ax[1].set_xlabel('t, с')
        kr, ka, kb = (self.get_parameter(n).value for n in ('k_rho', 'k_alpha', 'k_beta'))
        fig.suptitle(f'kρ={kr}, kα={ka}, kβ={kb}')
        path = stamp_name('go_to_goal', 'png')
        fig.savefig(path, dpi=120)
        self.get_logger().info(f'График: {path}')


def main():
    rclpy.init()
    node = GoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
