"""Вспомогательные функции, общие для узлов mum_labs."""
import os
import time

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

ARM_JOINTS = ['joint1', 'joint2', 'joint3']


def results_dir():
    """Каталог для графиков и CSV: ~/mum_results (создаётся автоматически)."""
    path = os.path.expanduser('~/mum_results')
    os.makedirs(path, exist_ok=True)
    return path


def stamp_name(prefix, ext):
    return os.path.join(results_dir(), f'{prefix}_{time.strftime("%Y%m%d_%H%M%S")}.{ext}')


def to_duration(t):
    sec = int(t)
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))


def joint_vector(msg, names=ARM_JOINTS, field='position'):
    """Вектор значений из sensor_msgs/JointState в порядке names (или None)."""
    data = getattr(msg, field)
    if len(data) == 0:
        return None
    try:
        return np.array([data[msg.name.index(n)] for n in names])
    except ValueError:
        return None


def make_trajectory(times, Q, V=None, A=None, names=ARM_JOINTS):
    """trajectory_msgs/JointTrajectory из массивов (N x n). Пустой stamp = «начать сейчас»."""
    traj = JointTrajectory()
    traj.joint_names = list(names)
    for i, t in enumerate(times):
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in Q[i]]
        if V is not None:
            pt.velocities = [float(x) for x in V[i]]
        if A is not None:
            pt.accelerations = [float(x) for x in A[i]]
        pt.time_from_start = to_duration(float(t))
        traj.points.append(pt)
    return traj


def color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def point(p):
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


def sphere_marker(frame, mid, p, rgba, d=0.02, ns='lab'):
    m = Marker()
    m.header.frame_id = frame
    m.ns, m.id, m.type, m.action = ns, mid, Marker.SPHERE, Marker.ADD
    m.pose.position = point(p)
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = d
    m.color = color(*rgba)
    return m


def arrow_marker(frame, mid, p0, p1, rgba, ns='lab', shaft=0.005):
    m = Marker()
    m.header.frame_id = frame
    m.ns, m.id, m.type, m.action = ns, mid, Marker.ARROW, Marker.ADD
    m.points = [point(p0), point(p1)]
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = shaft, 2 * shaft, 3 * shaft
    m.color = color(*rgba)
    return m


def line_marker(frame, mid, pts, rgba, ns='lab', width=0.004):
    m = Marker()
    m.header.frame_id = frame
    m.ns, m.id, m.type, m.action = ns, mid, Marker.LINE_STRIP, Marker.ADD
    m.points = [point(p) for p in pts]
    m.pose.orientation.w = 1.0
    m.scale.x = width
    m.color = color(*rgba)
    return m


def text_marker(frame, mid, p, text, ns='lab', h=0.025):
    m = Marker()
    m.header.frame_id = frame
    m.ns, m.id, m.type, m.action = ns, mid, Marker.TEXT_VIEW_FACING, Marker.ADD
    m.pose.position = point(p)
    m.pose.orientation.w = 1.0
    m.scale.z = h
    m.color = color(1, 1, 1)
    m.text = text
    return m


def yaw_from_quat(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def wrap_angle(a):
    return float((a + np.pi) % (2 * np.pi) - np.pi)
