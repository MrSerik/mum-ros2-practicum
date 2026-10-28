"""ПР №6. Планирование движений в MoveIt 2 (гл. 2.1–2.2): препятствия и сравнение планировщиков.

Режимы (параметр mode):
  obstacles — добавить в сцену препятствия (стенка между начальной и целевой конфигурацией);
  clear     — удалить препятствия;
  benchmark — для каждого планировщика из списка planners выполнить N запросов планирования
              (только план, без исполнения) и вывести: долю успехов, среднее время,
              среднюю длину пути в пространстве конфигураций;
  move      — спланировать и исполнить движение в q_goal выбранным планировщиком.

  ros2 run mum_labs moveit_lab --ros-args -p mode:=obstacles
  ros2 run mum_labs moveit_lab --ros-args -p mode:=benchmark -p runs:=10
  ros2 run mum_labs moveit_lab --ros-args -p mode:=move -p planner:=PRM
(при работе с Gazebo добавьте -p use_sim_time:=true)
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, MotionPlanRequest,
                             PlanningScene, RobotState)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from mum_labs.common import ARM_JOINTS, joint_vector

ERROR_TEXT = {1: 'SUCCESS', 99999: 'FAILURE', -1: 'PLANNING_FAILED', -2: 'INVALID_MOTION_PLAN',
              -4: 'CONTROL_FAILED', -6: 'TIMED_OUT', -10: 'START_STATE_IN_COLLISION',
              -12: 'GOAL_IN_COLLISION', -31: 'NO_IK_SOLUTION'}


class MoveItLab(Node):
    def __init__(self):
        super().__init__('moveit_lab')
        d = self.declare_parameter
        d('mode', 'benchmark')
        d('planners', ['RRTConnect', 'RRT', 'PRM', 'RRTstar'])
        d('planner', 'RRTConnect')
        d('runs', 10)
        d('planning_time', 2.0)
        d('q_start', [-1.2, 0.6, -1.0])
        d('q_goal', [1.2, 0.6, -1.0])
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.client = ActionClient(self, MoveGroup, 'move_action')
        self.q = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)

    def on_js(self, msg):
        q = joint_vector(msg)
        if q is not None:
            self.q = q

    # ---------------- сцена ----------------
    def box(self, name, size, xyz):
        co = CollisionObject()
        co.header.frame_id = 'base_link'
        co.id = name
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(c) for c in xyz)
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        return co

    def set_obstacles(self, add=True):
        ps = PlanningScene(is_diff=True)
        # тонкая стенка в плоскости xz прямо на пути поворота основания + «потолок»
        objs = [self.box('wall', (0.16, 0.02, 0.20), (0.18, 0.0, 0.10)),
                self.box('roof', (0.30, 0.30, 0.02), (0.05, 0.0, 0.33))]
        for o in objs:
            if not add:
                o.operation = CollisionObject.REMOVE
                o.primitives, o.primitive_poses = [], []
            ps.world.collision_objects.append(o)
        for _ in range(3):  # несколько раз — на случай позднего подключения подписчика
            self.scene_pub.publish(ps)
            rclpy.spin_once(self, timeout_sec=0.3)
        self.get_logger().info('Препятствия ' + ('добавлены' if add else 'удалены'))

    # ---------------- планирование ----------------
    def request(self, planner, q_start, q_goal, plan_only=True):
        req = MotionPlanRequest()
        req.group_name = 'arm'
        req.pipeline_id = 'ompl'
        req.planner_id = planner
        req.num_planning_attempts = 1
        req.allowed_planning_time = float(self.get_parameter('planning_time').value)
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        if q_start is not None:
            req.start_state = RobotState(joint_state=JointState(name=ARM_JOINTS,
                                                                position=[float(x) for x in q_start]))
        else:
            req.start_state.is_diff = True
        c = Constraints()
        for n, v in zip(ARM_JOINTS, q_goal):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v),
                                                       tolerance_above=0.01, tolerance_below=0.01,
                                                       weight=1.0))
        req.goal_constraints.append(c)
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            return None
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        return res_fut.result().result

    @staticmethod
    def path_length(result):
        pts = np.array([p.positions for p in result.planned_trajectory.joint_trajectory.points])
        return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))) if len(pts) > 1 else 0.0

    def benchmark(self):
        qs = self.get_parameter('q_start').value
        qg = self.get_parameter('q_goal').value
        runs = int(self.get_parameter('runs').value)
        rows = []
        for planner in self.get_parameter('planners').value:
            ok, times, lengths = 0, [], []
            for _ in range(runs):
                r = self.request(planner, qs, qg, plan_only=True)
                if r is not None and r.error_code.val == 1:
                    ok += 1
                    times.append(r.planning_time)
                    lengths.append(self.path_length(r))
            rows.append((planner, ok, runs, np.mean(times) if times else float('nan'),
                         np.mean(lengths) if lengths else float('nan'),
                         np.std(lengths) if lengths else float('nan')))
        straight = float(np.linalg.norm(np.array(qg) - np.array(qs)))
        text = [f'Старт {qs} -> цель {qg}; прямой отрезок в C-пространстве = {straight:.3f} рад',
                f'{"планировщик":<12}{"успех":>8}{"t_ср, с":>10}{"L_ср, рад":>12}{"σ_L":>8}']
        for p, ok, n, t, L, sL in rows:
            text.append(f'{p:<12}{ok:>4}/{n:<3}{t:>10.3f}{L:>12.3f}{sL:>8.3f}')
        self.get_logger().info('\n' + '\n'.join(text))

    def move(self):
        qg = self.get_parameter('q_goal').value
        planner = self.get_parameter('planner').value
        r = self.request(planner, None, qg, plan_only=False)
        if r is None:
            self.get_logger().error('Запрос отклонён move_group')
            return
        code = r.error_code.val
        self.get_logger().info(f'{planner}: {ERROR_TEXT.get(code, code)}, время планирования '
                               f'{r.planning_time:.3f} с, длина пути {self.path_length(r):.3f} рад')


def main():
    rclpy.init()
    node = MoveItLab()
    mode = node.get_parameter('mode').value
    if mode in ('benchmark', 'move'):
        node.get_logger().info('Ожидаю move_group ...')
        node.client.wait_for_server()
    if mode == 'obstacles':
        node.set_obstacles(True)
    elif mode == 'clear':
        node.set_obstacles(False)
    elif mode == 'benchmark':
        node.benchmark()
    elif mode == 'move':
        node.move()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
