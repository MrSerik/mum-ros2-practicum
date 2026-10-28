"""ПР №4. Обратная задача кинематики скорости: прямолинейное движение схвата.

Схват перемещается по отрезку от текущего положения p0 до точки target (в base_link).
Закон по пути — полином 5-й степени s(t), 0 <= s <= 1. Скорости сочленений находятся из
    q' = Jv(q)^(-1) * (p'_d + K (p_d - p(q)))        (замкнутая схема, CLIK)
и интегрируются методом Эйлера. Траектория q(t) отправляется в arm_controller.
Вблизи сингулярности (|det Jv| < eps) расчёт прерывается.

  ros2 run mum_labs cartesian_line_node --ros-args -p target:="[0.18, -0.10, 0.08]" -p T:=4.0
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import MarkerArray

from mum_labs import arm3_kinematics as kin
from mum_labs import trajectory_planning as tp
from mum_labs.common import joint_vector, line_marker, make_trajectory, stamp_name


class CartesianLine(Node):
    def __init__(self):
        super().__init__('cartesian_line_node')
        self.declare_parameter('target', [0.18, -0.10, 0.08])
        self.declare_parameter('T', 4.0)
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('K', 5.0)
        self.declare_parameter('det_eps', 2e-4)
        # перед прямолинейным участком манипулятор выводится в q_start (q = 0 — сингулярная конфигурация!)
        self.declare_parameter('q_start', [0.0, 0.9, -1.5])
        self.declare_parameter('approach_time', 2.0)
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.mpub = self.create_publisher(MarkerArray, '/lab_markers', 10)
        self.q = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_timer(0.5, self.run_once)
        self.done = False

    def on_js(self, msg):
        q = joint_vector(msg)
        if q is not None:
            self.q = q

    def run_once(self):
        if self.done or self.q is None or self.pub.get_subscription_count() == 0:
            return
        self.done = True
        q = np.array(self.get_parameter('q_start').value, dtype=float)
        Ta = self.get_parameter('approach_time').value
        ta, Qa, _, _ = tp.sample_quintic(self.q, q, Ta, 0.02)
        p0 = kin.fk(q)[:3, 3]
        p1 = np.array(self.get_parameter('target').value, dtype=float)
        T = self.get_parameter('T').value
        dt = self.get_parameter('dt').value
        K = self.get_parameter('K').value
        eps = self.get_parameter('det_eps').value
        cs = tp.quintic_coeffs(0.0, 1.0, T)

        ts, Qs, P, Pd, W, D = [], [], [], [], [], []
        stopped = False
        for t in np.arange(0.0, T + 1e-9, dt):
            s, sd, _ = tp.poly_eval(cs, t)
            p_des = p0 + s * (p1 - p0)
            v_des = sd * (p1 - p0)
            Jv = kin.jacobian(q)[:3, :]
            det = np.linalg.det(Jv)
            ts.append(t); Qs.append(q.copy()); P.append(kin.fk(q)[:3, 3]); Pd.append(p_des)
            W.append(kin.manipulability(q)); D.append(det)
            if abs(det) < eps:
                self.get_logger().error(f't = {t:.2f} с: |det Jv| = {abs(det):.2e} < {eps} — '
                                        'СИНГУЛЯРНОСТЬ, расчёт остановлен.')
                stopped = True
                break
            qd = np.linalg.solve(Jv, v_des + K * (p_des - P[-1]))
            q = q + qd * dt
            if not kin.within_limits(q):
                self.get_logger().error(f't = {t:.2f} с: выход за пределы сочленений q = {np.round(q, 3)}')
                stopped = True
                break

        ts, Qs, P, Pd = map(np.array, (ts, Qs, P, Pd))
        dev = np.linalg.norm(P - Pd, axis=1).max() * 1000
        self.get_logger().info(f'Отрезок {np.round(p0, 3)} -> {np.round(p1, 3)}, точек {len(ts)}, '
                               f'макс. отклонение от прямой {dev:.3f} мм, '
                               f'w_min = {min(W):.2e}, w_max = {max(W):.2e}')
        if len(ts) > 2:
            # подход в q_start (пространство сочленений) + прямолинейный участок
            t_all = np.concatenate([ta, Ta + ts[1:]])
            q_all = np.vstack([Qa, Qs[1:]])
            self.pub.publish(make_trajectory(t_all, q_all))
        self.mpub.publish(MarkerArray(markers=[
            line_marker('base_link', 0, [p0, p1], (1, 1, 0, 1), ns='cart_line'),
            line_marker('base_link', 1, P, (0, 1, 1, 1), ns='cart_line', width=0.006)]))
        self.plot(ts, Qs, P, W, D, stopped)

    def plot(self, ts, Qs, P, W, D, stopped):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        n = len(ts)
        fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        for j in range(3):
            ax[0].plot(ts, Qs[:, j], label=f'q{j + 1}')
        for j, c in enumerate('xyz'):
            ax[1].plot(ts, P[:, j], label=c)
        ax[2].plot(ts, W[:n], label='w = |det Jv|')
        ax[0].set_ylabel('q, рад'); ax[1].set_ylabel('p, м'); ax[2].set_ylabel('w')
        ax[2].set_xlabel('t, с')
        for a in ax:
            a.grid(True); a.legend(fontsize=8)
        ax[0].set_title('Прямолинейное движение схвата' + (' (остановлено: сингулярность)' if stopped else ''))
        fig.tight_layout()
        path = stamp_name('cartesian_line', 'png')
        fig.savefig(path, dpi=120)
        self.get_logger().info(f'График: {path}')


def main():
    rclpy.init()
    node = CartesianLine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
