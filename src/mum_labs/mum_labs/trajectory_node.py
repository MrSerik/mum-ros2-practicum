"""ПР №5. Планирование траектории полиномами (гл. 2.3) и её отработка в Gazebo.

Режимы (параметр mode):
  quintic — односегментная траектория 5-й степени q_start -> q_goal за время T;
  434     — трёхсегментная 4-3-4: q_start -> q_lift -> q_set -> q_goal за t1, t2, t3.
Узел: 1) выводит манипулятор в q_start, 2) отправляет рассчитанную траекторию в
/arm_controller/joint_trajectory, 3) записывает /joint_states и сохраняет графики и CSV в ~/mum_results.

  ros2 run mum_labs trajectory_node --ros-args -p mode:=quintic -p T:=3.0
  ros2 run mum_labs trajectory_node --ros-args -p mode:=434 -p t1:=1.0 -p t2:=2.0 -p t3:=1.0
"""
import csv

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from mum_labs import trajectory_planning as tp
from mum_labs.common import ARM_JOINTS, joint_vector, make_trajectory, stamp_name


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        p = self.declare_parameter
        p('mode', 'quintic')
        p('q_start', [0.0, 0.2, 0.0])
        p('q_lift', [0.0, 0.8, -0.6])
        p('q_set', [1.2, 0.8, -0.6])
        p('q_goal', [1.2, 0.2, 0.0])
        p('T', 3.0)
        p('t1', 1.0)
        p('t2', 2.0)
        p('t3', 1.0)
        p('dt', 0.02)
        p('approach_time', 2.0)
        p('record_extra', 1.0)

        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.q = None
        self.sim_t = 0.0
        self.log = []          # (t, q1, q2, q3) фактические
        self.t0 = None
        self.state = 'wait'
        self.plan = None
        self.create_timer(0.1, self.step)

    def gp(self, name):
        return self.get_parameter(name).value

    def on_js(self, msg):
        q = joint_vector(msg)
        if q is None:
            return
        self.q = q
        # время симуляции берём из штампа сообщения (контроллер работает по /clock)
        self.sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.state == 'run':
            self.log.append((self.sim_t - self.t0, *q))

    def build_plan(self):
        dt = self.gp('dt')
        if self.gp('mode') == '434':
            t, Q, V, A = tp.sample_434(self.gp('q_start'), self.gp('q_lift'), self.gp('q_set'),
                                       self.gp('q_goal'), self.gp('t1'), self.gp('t2'), self.gp('t3'), dt)
        else:
            t, Q, V, A = tp.sample_quintic(self.gp('q_start'), self.gp('q_goal'), self.gp('T'), dt)
        return t, Q, V, A

    def step(self):
        if self.state == 'wait':
            if self.q is None:
                self.get_logger().info('Ожидаю /joint_states ...', throttle_duration_sec=3.0)
                return
            if self.pub.get_subscription_count() == 0:
                self.get_logger().info('Ожидаю контроллер arm_controller ...', throttle_duration_sec=3.0)
                return
            qs = np.array(self.gp('q_start'))
            Ta = self.gp('approach_time')
            self.get_logger().info(f'Вывод в начальную конфигурацию {qs} за {Ta} с')
            self.pub.publish(make_trajectory([Ta], [qs], V=[np.zeros(3)]))
            self.t_wait = self.sim_t + Ta + 0.5
            self.state = 'approach'
        elif self.state == 'approach':
            if self.sim_t < self.t_wait:
                return
            self.plan = self.build_plan()
            t, Q, V, A = self.plan
            self.get_logger().info(f'Отправляю траекторию: режим {self.gp("mode")}, {len(t)} точек, '
                                   f'длительность {t[-1]:.2f} с; max|q\'| = {np.abs(V).max():.3f} рад/с, '
                                   f'max|q\'\'| = {np.abs(A).max():.3f} рад/с²')
            self.t0 = self.sim_t
            self.pub.publish(make_trajectory(t, Q, V, A))
            self.state = 'run'
        elif self.state == 'run':
            if self.sim_t - self.t0 > self.plan[0][-1] + self.gp('record_extra'):
                self.state = 'done'
                self.save()
                raise SystemExit

    def save(self):
        t, Q, V, A = self.plan
        log = np.array(self.log)
        mode = self.gp('mode')
        csv_path = stamp_name(f'trajectory_{mode}', 'csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t'] + [f'{j}_des' for j in ARM_JOINTS] + [f'{j}_vel_des' for j in ARM_JOINTS]
                       + [f'{j}_acc_des' for j in ARM_JOINTS])
            for i in range(len(t)):
                w.writerow([t[i], *Q[i], *V[i], *A[i]])
        act_path = csv_path.replace('.csv', '_actual.csv')
        with open(act_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t'] + ARM_JOINTS)
            w.writerows(log.tolist())

        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        for j in range(3):
            ax[0].plot(t, Q[:, j], label=f'q{j + 1} задание')
            if len(log):
                ax[0].plot(log[:, 0], log[:, j + 1], '--', label=f'q{j + 1} факт')
            ax[1].plot(t, V[:, j], label=f"q{j + 1}'")
            ax[2].plot(t, A[:, j], label=f"q{j + 1}''")
        ax[0].set_ylabel('q, рад')
        ax[1].set_ylabel("q', рад/с")
        ax[2].set_ylabel("q'', рад/с²")
        ax[2].set_xlabel('t, с')
        for a in ax:
            a.grid(True)
            a.legend(fontsize=8, ncol=3)
        ax[0].set_title(f'Траектория {mode}')
        png = csv_path.replace('.csv', '.png')
        fig.tight_layout()
        fig.savefig(png, dpi=120)
        if len(log):
            # ошибка слежения в конце
            err = np.abs(np.array([np.interp(t, log[:, 0], log[:, j + 1]) for j in range(3)]).T - Q)
            self.get_logger().info(f'Макс. ошибка слежения по звеньям: {np.round(err.max(axis=0), 4)} рад')
        self.get_logger().info(f'Сохранено: {png}\n          {csv_path}\n          {act_path}')


def main():
    rclpy.init()
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
