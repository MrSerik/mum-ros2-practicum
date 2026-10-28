"""ПР №7. Регуляторы сочленений манипулятора (гл. 3.1–3.2): П, ПД, ПИД, компенсация гравитации,
насыщение управления и антивиндап-коррекция.

Запуск Gazebo в режиме моментов:
  ros2 launch arm3_description gazebo.launch.py control_mode:=effort
Регулятор:
  ros2 run mum_labs pid_node --ros-args -p kp:="[0.5, 1.0, 0.4]" -p kd:="[0.03, 0.08, 0.02]"
Звенья лёгкие (моменты инерции ~1e-4…3e-3 кг·м²), поэтому коэффициенты малы. Если появились
колебания — уменьшайте kd и kp: цикл регулятора на Python имеет запаздывание 1–2 такта (5–10 мс).

Закон управления для каждого звена i (e = q* - q):
  u_i = kp_i e_i + ki_i ∫ e_i dt - kd_i q'_i  [+ g_i(q), если gravity_comp]
  u_i := sat(u_i, ±u_max)
  антивиндап (back-calculation): d/dt ∫e = e + k_aw (u_sat - u) / ki
Каждые `duration` секунд записывается переходный процесс; графики и показатели качества
сохраняются в ~/mum_results. Новую уставку можно подать «на лету»:
  ros2 topic pub --once /q_ref std_msgs/msg/Float64MultiArray "{data: [0.5, 1.0, -0.8]}"
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from mum_labs import arm3_kinematics as kin
from mum_labs.common import joint_vector, stamp_name


def step_metrics(t, y, y0, yref):
    """Перерегулирование (%), время регулирования (±5 %), установившаяся ошибка."""
    amp = yref - y0
    if abs(amp) < 1e-6:
        return float('nan'), float('nan'), float(yref - y[-1])
    yn = (y - y0) / amp
    overshoot = max(0.0, (yn.max() - 1.0) * 100.0)
    outside = np.where(np.abs(yn - 1.0) > 0.05)[0]
    ts = t[outside[-1]] if len(outside) else 0.0
    if len(outside) and outside[-1] == len(t) - 1:
        ts = float('inf')
    return overshoot, ts, float(yref - y[-1])


class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')
        d = self.declare_parameter
        d('q_ref', [0.5, 1.0, -0.8])
        d('kp', [0.5, 1.0, 0.4])
        d('ki', [0.0, 0.0, 0.0])
        d('kd', [0.03, 0.08, 0.02])
        d('gravity_comp', False)
        d('u_max', 1.5)
        d('anti_windup', False)
        d('k_aw', 1.0)
        d('duration', 6.0)
        self.pub = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 100)
        self.create_subscription(Float64MultiArray, '/q_ref', self.on_ref, 10)
        self.integ = np.zeros(3)
        self.t_prev = None
        self.reset_log()
        self.get_logger().info('ПИД-регулятор запущен. Параметры: ' + self.describe())

    def p(self, name):
        return np.array(self.get_parameter(name).value, dtype=float)

    def describe(self):
        return (f'kp={self.p("kp")}, ki={self.p("ki")}, kd={self.p("kd")}, '
                f'g-комп.={self.get_parameter("gravity_comp").value}, u_max={self.get_parameter("u_max").value}, '
                f'антивиндап={self.get_parameter("anti_windup").value}')

    def reset_log(self):
        self.log = []
        self.t_start = None
        self.q_start = None

    def on_ref(self, msg):
        if len(msg.data) == 3:
            self.set_parameters([Parameter('q_ref', value=list(msg.data))])
            self.integ[:] = 0.0
            self.reset_log()
            self.get_logger().info(f'Новая уставка q* = {list(msg.data)}')

    def on_js(self, msg):
        q = joint_vector(msg)
        dq = joint_vector(msg, field='velocity')
        if q is None or dq is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_prev is None or t <= self.t_prev:
            self.t_prev = t
            return
        dt = min(t - self.t_prev, 0.05)
        self.t_prev = t

        q_ref = self.p('q_ref')
        kp, ki, kd = self.p('kp'), self.p('ki'), self.p('kd')
        u_max = float(self.get_parameter('u_max').value)

        e = q_ref - q
        u = kp * e + ki * self.integ - kd * dq
        if self.get_parameter('gravity_comp').value:
            u = u + kin.gravity_torques(q)
        u_sat = np.clip(u, -u_max, u_max)

        # интегратор
        if self.get_parameter('anti_windup').value:
            k_aw = float(self.get_parameter('k_aw').value)
            corr = np.where(ki > 1e-9, k_aw * (u_sat - u) / np.maximum(ki, 1e-9), 0.0)
            self.integ += (e + corr) * dt
        else:
            self.integ += e * dt

        self.pub.publish(Float64MultiArray(data=[float(x) for x in u_sat]))

        # запись переходного процесса
        if self.t_start is None:
            self.t_start, self.q_start = t, q.copy()
        tt = t - self.t_start
        if tt <= self.get_parameter('duration').value:
            self.log.append([tt, *q, *u_sat, *q_ref])
        elif self.log:
            self.save()
            self.log = []

    def save(self):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        L = np.array(self.log)
        t = L[:, 0]
        fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        report = []
        for j in range(3):
            ax[0].plot(t, L[:, 1 + j], label=f'q{j + 1}')
            ax[0].plot(t, L[:, 7 + j], 'k:', lw=0.8)
            ax[1].plot(t, L[:, 4 + j], label=f'u{j + 1}')
            ov, ts, ess = step_metrics(t, L[:, 1 + j], self.q_start[j], L[-1, 7 + j])
            report.append(f'звено {j + 1}: σ = {ov:.1f} %, t_рег(5%) = {ts:.2f} с, e_уст = {ess:+.4f} рад')
        ax[0].set_ylabel('q, рад'); ax[1].set_ylabel('u, Н·м'); ax[1].set_xlabel('t, с')
        ax[0].set_title(self.describe(), fontsize=8)
        for a in ax:
            a.grid(True); a.legend(fontsize=8)
        fig.tight_layout()
        path = stamp_name('pid', 'png')
        fig.savefig(path, dpi=120)
        np.savetxt(path.replace('.png', '.csv'), L, delimiter=',',
                   header='t,q1,q2,q3,u1,u2,u3,q1_ref,q2_ref,q3_ref', comments='')
        self.get_logger().info('Показатели качества:\n  ' + '\n  '.join(report) + f'\nГрафик: {path}')


def main():
    rclpy.init()
    node = PIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
