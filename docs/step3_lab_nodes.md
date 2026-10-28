# Шаг 3. Пакет mum_labs: узлы практических работ

**Результат шага:** Python-пакет `mum_labs` с расчётными модулями (кинематика, траектории) и узлами для всех десяти практических работ, а также запуск Nav2 для мобильного робота одной командой.
**Время:** 2–3 часа. **Нужно:** выполненные [шаг 1](step1_manipulator.md) и [шаг 2](step2_mobile_robot.md).

> Адрес для `curl` (если новый терминал): `export REPO_RAW=https://raw.githubusercontent.com/MrSerik/mum-ros2-practicum/main`

| Файл | Что делает | Практическая работа |
| --- | --- | --- |
| `arm3_kinematics.py` | ДХ, прямая и обратная задачи, Якобиан, g(q) — без ROS | 2, 3, 4, 7 |
| `trajectory_planning.py` | Полиномы 5-й степени и 4-3-4 — без ROS | 4, 5 |
| `common.py` | Общие функции: траектории, маркеры RViz, углы | все |
| `fk_node.py` | Прямая задача в реальном времени, сравнение с TF, Якобиан | 2, 4 |
| `ik_node.py` | Обратная задача, перемещение схвата в точку | 3 |
| `cartesian_line_node.py` | Прямолинейное движение через обратную задачу скорости | 4 |
| `trajectory_node.py` | Отработка траекторий 5-й степени и 4-3-4, графики | 5 |
| `moveit_lab.py` | Препятствия в MoveIt и сравнение планировщиков | 6 |
| `pid_node.py` | П/ПД/ПИД-регулятор, компенсация гравитации, антивиндап | 7 |
| `diff_odometry_node.py` | Одометрия по колёсам, сравнение с истиной | 8 |
| `go_to_goal_node.py` | Выход мобильного робота в позу | 8 |
| `nav_waypoints.py` | Nav2 через Python: путевые точки, статистика путей | 9 |
| `color_tracker.py` | Цветовая бинаризация, глубина, следование за объектом | 10 |

## 3.1. Создание пакета

Пакет типа `ament_python`: код на Python, каждый узел — функция `main()`, которую `setup.py` превращает в команду `ros2 run mum_labs <узел>`.

```bash
cd ~/mum_ws/src
ros2 pkg create mum_labs --build-type ament_python --license Apache-2.0
cd mum_labs
mkdir -p launch
```

Генератор уже создал `mum_labs/__init__.py`, `resource/mum_labs`, `setup.cfg` и тесты в `test/` — их оставляем.

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mum_labs</name>
  <version>1.0.0</version>
  <description>Узлы практических работ курса «Механика управляемых машин» (ROS 2 Jazzy, Nav2, MoveIt 2)</description>
  <maintainer email="ssydykanov@gmail.com">Sydykanov S.</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>trajectory_msgs</exec_depend>
  <exec_depend>visualization_msgs</exec_depend>
  <exec_depend>moveit_msgs</exec_depend>
  <exec_depend>shape_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>nav2_simple_commander</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>diffbot_description</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-matplotlib</exec_depend>
  <exec_depend>python3-opencv</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### setup.py — список узлов

Раздел `entry_points` связывает имя команды с функцией `main` в модуле; `data_files` устанавливает launch-файлы.

```python
from glob import glob

from setuptools import setup

package_name = 'mum_labs'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sydykanov S.',
    maintainer_email='ssydykanov@gmail.com',
    description='Узлы практических работ курса «Механика управляемых машин» (ROS 2 Jazzy)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fk_node = mum_labs.fk_node:main',
            'ik_node = mum_labs.ik_node:main',
            'cartesian_line_node = mum_labs.cartesian_line_node:main',
            'trajectory_node = mum_labs.trajectory_node:main',
            'pid_node = mum_labs.pid_node:main',
            'moveit_lab = mum_labs.moveit_lab:main',
            'diff_odometry_node = mum_labs.diff_odometry_node:main',
            'go_to_goal_node = mum_labs.go_to_goal_node:main',
            'nav_waypoints = mum_labs.nav_waypoints:main',
            'color_tracker = mum_labs.color_tracker:main',
        ],
    },
)
```

Проверьте, что `setup.cfg` совпадает:

```ini
[develop]
script_dir=$base/lib/mum_labs
[install]
install_scripts=$base/lib/mum_labs
```

## 3.2. Расчётные модули (без ROS)

Эти два файла — прямое воплощение формул из лекций. Их стоит набрать и разобрать построчно: функции `dh_matrix`, `fk`, `ik`, `jacobian`, `gravity_torques`, `quintic_coeffs`, `plan_434`.

**`mum_labs/arm3_kinematics.py`**

```python
"""Кинематика и статика учебного манипулятора arm3 (3 вращательных сочленения).

Обозначения и алгоритмы соответствуют пособию:
Борисов О.И., Громов В.С., Пыркин А.А. «Методы управления робототехническими приложениями», 2016.
  гл. 1.2 — прямая задача кинематики (параметры Денавита–Хартенберга, однородные преобразования)
  гл. 1.3 — обратная задача кинематики (геометрический подход)
  гл. 1.4 — матрица Якоби, сингулярности, обратная задача кинематики скорости
  гл. 3.2 — вектор гравитационных моментов g(q) для компенсации силы тяжести

Модуль не зависит от ROS — его можно запускать и проверять отдельно:
    python3 -m mum_labs.arm3_kinematics
"""
import numpy as np

# ---------------- Параметры (должны совпадать с arm3.urdf.xacro) ----------------
D1 = 0.10   # м, смещение по z0 до оси плеча (H_BASE + H_COL)
A2 = 0.12   # м, длина плеча
A3 = 0.12   # м, длина предплечья
BASE_Z_IN_WORLD = 0.20  # м, высота подставки (base_link относительно world)

M2, M3, M_TOOL = 0.12, 0.08, 0.005   # кг, массы звеньев 2, 3 и схвата
G = 9.81

Q_MIN = np.array([-np.pi / 2, -0.3, -2.5])
Q_MAX = np.array([np.pi / 2, 2.8, 2.5])

# Таблица ДХ: (theta_offset, d, a, alpha) для каждого звена; theta_i = q_i + offset
DH_TABLE = [
    (0.0, D1, 0.0, np.pi / 2),
    (0.0, 0.0, A2, 0.0),
    (0.0, 0.0, A3, 0.0),
]


def dh_matrix(theta, d, a, alpha):
    """Матрица однородного преобразования A_i = Rot_z(θ) Trans_z(d) Trans_x(a) Rot_x(α)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def fk_all(q):
    """Список матриц T_0^0, T_0^1, T_0^2, T_0^3 (относительно base_link)."""
    T = np.eye(4)
    frames = [T.copy()]
    for qi, (off, d, a, alpha) in zip(q, DH_TABLE):
        T = T @ dh_matrix(qi + off, d, a, alpha)
        frames.append(T.copy())
    return frames


def fk(q):
    """Прямая задача: матрица T_0^3 (положение и ориентация схвата tool0 в base_link)."""
    return fk_all(q)[-1]


def fk_position_closed_form(q):
    """Положение схвата в явном виде (удобно для проверки)."""
    q1, q2, q3 = q
    r = A2 * np.cos(q2) + A3 * np.cos(q2 + q3)
    return np.array([np.cos(q1) * r, np.sin(q1) * r, D1 + A2 * np.sin(q2) + A3 * np.sin(q2 + q3)])


def ik(p, elbow='up'):
    """Обратная задача по положению p = (x, y, z) в base_link.

    Возвращает np.array([q1, q2, q3]) или None, если точка вне рабочей зоны.
    elbow='up'   — «локоть вверх» (q3 < 0),
    elbow='down' — «локоть вниз»  (q3 > 0).
    """
    x, y, z = p
    q1 = np.arctan2(y, x)
    r = np.hypot(x, y)          # проекция на горизонтальную плоскость
    s = z - D1                  # высота над осью плеча
    c3 = (r * r + s * s - A2 * A2 - A3 * A3) / (2 * A2 * A3)   # теорема косинусов
    if abs(c3) > 1.0:
        return None
    s3 = np.sqrt(1.0 - c3 * c3)
    if elbow == 'up':
        s3 = -s3
    q3 = np.arctan2(s3, c3)
    q2 = np.arctan2(s, r) - np.arctan2(A3 * s3, A2 + A3 * c3)
    return np.array([q1, q2, q3])


def within_limits(q, margin=0.0):
    q = np.asarray(q)
    return bool(np.all(q >= Q_MIN + margin) and np.all(q <= Q_MAX - margin))


def jacobian(q):
    """Геометрическая матрица Якоби 6x3: [Jv; Jw], столбцы J_i = [z_{i-1} x (o_n - o_{i-1}); z_{i-1}]."""
    frames = fk_all(q)
    o_n = frames[-1][:3, 3]
    J = np.zeros((6, 3))
    for i in range(3):
        z = frames[i][:3, 2]
        o = frames[i][:3, 3]
        J[:3, i] = np.cross(z, o_n - o)
        J[3:, i] = z
    return J


def jacobian_linear_closed_form(q):
    """Линейная часть Jv в явном виде (производные fk_position_closed_form)."""
    q1, q2, q3 = q
    r = A2 * np.cos(q2) + A3 * np.cos(q2 + q3)
    dr2 = -A2 * np.sin(q2) - A3 * np.sin(q2 + q3)
    dr3 = -A3 * np.sin(q2 + q3)
    return np.array([
        [-np.sin(q1) * r, np.cos(q1) * dr2, np.cos(q1) * dr3],
        [np.cos(q1) * r, np.sin(q1) * dr2, np.sin(q1) * dr3],
        [0.0, A2 * np.cos(q2) + A3 * np.cos(q2 + q3), A3 * np.cos(q2 + q3)],
    ])


def manipulability(q):
    """Мера манипулируемости Йошикавы w = sqrt(det(Jv Jv^T)) = |det Jv| для 3x3."""
    Jv = jacobian(q)[:3, :]
    return float(abs(np.linalg.det(Jv)))


def det_jv_closed_form(q):
    """det Jv = A2*A3*sin(q3)*(A2 cos q2 + A3 cos(q2+q3)) — нули дают сингулярности."""
    _, q2, q3 = q
    return A2 * A3 * np.sin(q3) * (A2 * np.cos(q2) + A3 * np.cos(q2 + q3))


def gravity_torques(q):
    """Вектор g(q) моментов от силы тяжести (центры масс звеньев — в середине звеньев)."""
    _, q2, q3 = q
    c2, c23 = np.cos(q2), np.cos(q2 + q3)
    g3 = G * (M3 * A3 / 2 * c23 + M_TOOL * A3 * c23)
    g2 = G * (M2 * A2 / 2 * c2 + M3 * (A2 * c2 + A3 / 2 * c23) + M_TOOL * (A2 * c2 + A3 * c23))
    return np.array([0.0, g2, g3])


# ---------------- Самопроверка ----------------
if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    rng = np.random.default_rng(0)
    err_fk = err_ik = err_j = 0.0
    for _ in range(2000):
        q = rng.uniform(Q_MIN, Q_MAX)
        p = fk(q)[:3, 3]
        err_fk = max(err_fk, np.abs(p - fk_position_closed_form(q)).max())
        err_j = max(err_j, np.abs(jacobian(q)[:3] - jacobian_linear_closed_form(q)).max())
        for elbow in ('up', 'down'):
            qs = ik(p, elbow)
            if qs is not None:
                err_ik = max(err_ik, np.abs(fk(qs)[:3, 3] - p).max())
    print('max |FK(ДХ) - FK(явная)|  =', err_fk)
    print('max |FK(IK(p)) - p|       =', err_ik)
    print('max |J(геом) - J(явная)|  =', err_j)
    print('T_0^3 при q = 0:\n', fk([0, 0, 0]))
```

**`mum_labs/trajectory_planning.py`**

```python
"""Планирование траекторий в пространстве обобщённых координат (гл. 2.3 пособия).

  quintic_coeffs  — односегментная траектория, полином 5-й степени (2.18)–(2.20)
  plan_434        — трёхсегментная траектория 4-3-4: подъём, перенос, опускание
Все функции работают для одной обобщённой координаты; для манипулятора вызываются по каждому звену.
Модуль не зависит от ROS:  python3 -m mum_labs.trajectory_planning
"""
import numpy as np


def poly_rows(t, n):
    """Строки [q, q', q''] полинома степени n-1 (n коэффициентов a0..a_{n-1}) в момент t."""
    p = np.array([t ** k for k in range(n)], dtype=float)
    v = np.array([k * t ** (k - 1) if k >= 1 else 0.0 for k in range(n)], dtype=float)
    a = np.array([k * (k - 1) * t ** (k - 2) if k >= 2 else 0.0 for k in range(n)], dtype=float)
    return p, v, a


def poly_eval(c, t):
    """Значение, скорость, ускорение полинома с коэффициентами c = [a0, a1, ...]."""
    p, v, a = poly_rows(t, len(c))
    return p @ c, v @ c, a @ c


def quintic_coeffs(q0, q1, T, v0=0.0, v1=0.0, a0=0.0, a1=0.0):
    """Коэффициенты a0..a5 из матричного уравнения M a = b (6 граничных условий)."""
    r0 = poly_rows(0.0, 6)
    rT = poly_rows(T, 6)
    M = np.vstack([r0[0], r0[1], r0[2], rT[0], rT[1], rT[2]])
    b = np.array([q0, v0, a0, q1, v1, a1], dtype=float)
    return np.linalg.solve(M, b)


def plan_434(q0, q1, q2, qf, t1, t2, t3):
    """Траектория 4-3-4 через две промежуточные точки q1 (lift-off) и q2 (set-down).

    Сегмент a: полином 4-й степени на [0, t1], b: 3-й степени на [0, t2], c: 4-й степени на [0, t3]
    (локальное время каждого сегмента). Нулевые скорости и ускорения в начале и конце,
    непрерывность положения, скорости и ускорения в узлах. 14 неизвестных, 14 уравнений.
    Возвращает (ca, cb, cc) — коэффициенты сегментов.
    """
    na, nb, nc = 5, 4, 5
    N = na + nb + nc
    rows, rhs = [], []

    def put(seg_rows):
        row = np.zeros(N)
        for offset, vec in seg_rows:
            row[offset:offset + len(vec)] += vec
        rows.append(row)

    A, B, C = 0, na, na + nb
    pa0, va0, aa0 = poly_rows(0.0, na)
    pa1, va1, aa1 = poly_rows(t1, na)
    pb0, vb0, ab0 = poly_rows(0.0, nb)
    pb1, vb1, ab1 = poly_rows(t2, nb)
    pc0, vc0, ac0 = poly_rows(0.0, nc)
    pc1, vc1, ac1 = poly_rows(t3, nc)

    # начало: q, q', q''
    put([(A, pa0)]); rhs.append(q0)
    put([(A, va0)]); rhs.append(0.0)
    put([(A, aa0)]); rhs.append(0.0)
    # точка подъёма q1: конец a, начало b, непрерывность скорости и ускорения
    put([(A, pa1)]); rhs.append(q1)
    put([(B, pb0)]); rhs.append(q1)
    put([(A, va1), (B, -vb0)]); rhs.append(0.0)
    put([(A, aa1), (B, -ab0)]); rhs.append(0.0)
    # точка опускания q2: конец b, начало c
    put([(B, pb1)]); rhs.append(q2)
    put([(C, pc0)]); rhs.append(q2)
    put([(B, vb1), (C, -vc0)]); rhs.append(0.0)
    put([(B, ab1), (C, -ac0)]); rhs.append(0.0)
    # конец: q, q', q''
    put([(C, pc1)]); rhs.append(qf)
    put([(C, vc1)]); rhs.append(0.0)
    put([(C, ac1)]); rhs.append(0.0)

    x = np.linalg.solve(np.array(rows), np.array(rhs, dtype=float))
    return x[A:B], x[B:C], x[C:]


def sample_quintic(q0, qf, T, dt):
    """Дискретизация многомерной траектории 5-го порядка: t, Q, V, Acc (массивы N x n)."""
    q0, qf = np.asarray(q0, float), np.asarray(qf, float)
    coeffs = [quintic_coeffs(a, b, T) for a, b in zip(q0, qf)]
    t = np.arange(0.0, T + 1e-9, dt)
    if t[-1] < T - 1e-9:
        t = np.append(t, T)
    Q, V, Acc = (np.zeros((len(t), len(q0))) for _ in range(3))
    for j, c in enumerate(coeffs):
        for i, ti in enumerate(t):
            Q[i, j], V[i, j], Acc[i, j] = poly_eval(c, ti)
    return t, Q, V, Acc


def sample_434(q0, q1, q2, qf, t1, t2, t3, dt):
    """Дискретизация многомерной траектории 4-3-4."""
    q0, q1, q2, qf = (np.asarray(v, float) for v in (q0, q1, q2, qf))
    segs = [plan_434(*(v[j] for v in (q0, q1, q2, qf)), t1, t2, t3) for j in range(len(q0))]
    T = t1 + t2 + t3
    t = np.arange(0.0, T + 1e-9, dt)
    if t[-1] < T - 1e-9:
        t = np.append(t, T)
    Q, V, Acc = (np.zeros((len(t), len(q0))) for _ in range(3))
    for i, ti in enumerate(t):
        if ti <= t1:
            k, tau = 0, ti
        elif ti <= t1 + t2:
            k, tau = 1, ti - t1
        else:
            k, tau = 2, ti - t1 - t2
        for j in range(len(q0)):
            Q[i, j], V[i, j], Acc[i, j] = poly_eval(segs[j][k], tau)
    return t, Q, V, Acc


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    c = quintic_coeffs(0.0, 1.0, 2.0)
    print('quintic a0..a5 =', c)
    for tt in (0.0, 1.0, 2.0):
        print(f'  t={tt}: q, v, a =', np.round(poly_eval(c, tt), 6))
    ca, cb, cc = plan_434(0.0, 0.2, 0.8, 1.0, 1.0, 2.0, 1.0)
    print('4-3-4: a =', ca, '\n       b =', cb, '\n       c =', cc)
    print('  узел 1:', poly_eval(ca, 1.0), poly_eval(cb, 0.0))
    print('  узел 2:', poly_eval(cb, 2.0), poly_eval(cc, 0.0))
    print('  конец :', poly_eval(cc, 1.0))
```

Проверка сразу, до сборки:

```bash
cd ~/mum_ws/src/mum_labs
python3 -m mum_labs.arm3_kinematics        # ошибки порядка 1e-16
python3 -m mum_labs.trajectory_planning    # a3 = 1,25; a4 = -0,9375; a5 = 0,1875
```

## 3.3. Общие функции

**`mum_labs/common.py`**

<details>
<summary>Показать содержимое <code>mum_labs/common.py</code></summary>

```python
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
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o mum_labs/common.py $REPO_RAW/src/mum_labs/mum_labs/common.py
```

## 3.4. Узлы манипулятора

**`mum_labs/fk_node.py`** — подписывается на `/joint_states`, считает T₀³ по ДХ, сравнивает с TF, рисует системы ДХ в RViz.

<details>
<summary>Показать содержимое <code>mum_labs/fk_node.py</code></summary>

```python
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
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o mum_labs/fk_node.py $REPO_RAW/src/mum_labs/mum_labs/fk_node.py
```

**`mum_labs/ik_node.py`** — цель из топика `/ik_target` или щелчка Publish Point в RViz.

<details>
<summary>Показать содержимое <code>mum_labs/ik_node.py</code></summary>

```python
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
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o mum_labs/ik_node.py $REPO_RAW/src/mum_labs/mum_labs/ik_node.py
```

**`mum_labs/cartesian_line_node.py`**

<details>
<summary>Показать содержимое <code>mum_labs/cartesian_line_node.py</code></summary>

```python
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
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o mum_labs/cartesian_line_node.py $REPO_RAW/src/mum_labs/mum_labs/cartesian_line_node.py
```

**`mum_labs/trajectory_node.py`**

<details>
<summary>Показать содержимое <code>mum_labs/trajectory_node.py</code></summary>

```python
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
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o mum_labs/trajectory_node.py $REPO_RAW/src/mum_labs/mum_labs/trajectory_node.py
```

**`mum_labs/moveit_lab.py`**

<details>
<summary>Показать содержимое <code>mum_labs/moveit_lab.py</code></summary>

```python
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
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o mum_labs/moveit_lab.py $REPO_RAW/src/mum_labs/mum_labs/moveit_lab.py
```

**`mum_labs/pid_node.py`**

<details>
<summary>Показать содержимое <code>mum_labs/pid_node.py</code></summary>

```python
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
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o mum_labs/pid_node.py $REPO_RAW/src/mum_labs/mum_labs/pid_node.py
```

## 3.5. Узлы мобильного робота

**`mum_labs/diff_odometry_node.py`**

<details>
<summary>Показать содержимое <code>mum_labs/diff_odometry_node.py</code></summary>

```python
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
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o mum_labs/diff_odometry_node.py $REPO_RAW/src/mum_labs/mum_labs/diff_odometry_node.py
```

**`mum_labs/go_to_goal_node.py`**

<details>
<summary>Показать содержимое <code>mum_labs/go_to_goal_node.py</code></summary>

```python
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
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o mum_labs/go_to_goal_node.py $REPO_RAW/src/mum_labs/mum_labs/go_to_goal_node.py
```

**`mum_labs/nav_waypoints.py`**

<details>
<summary>Показать содержимое <code>mum_labs/nav_waypoints.py</code></summary>

```python
"""ПР №9. Навигация Nav2 через Python API (nav2_simple_commander).

Режимы:
  path_stats — запросить у планировщика путь из текущей позы в каждую точку и вывести длину пути,
               число узлов и время планирования (сравнение Dijkstra / A* / Smac);
  waypoints  — объехать точки по очереди (FollowWaypoints) и замерить время и длину пройденного пути.

Точки задаются в СК map плоским списком [x1, y1, yaw1, x2, y2, yaw2, ...].
  ros2 run mum_labs nav_waypoints --ros-args -p mode:=path_stats -p use_sim_time:=true
  ros2 run mum_labs nav_waypoints --ros-args -p mode:=waypoints -p use_sim_time:=true
При работе с готовой картой (AMCL) добавьте -p localizer:=amcl -p initial_pose:="[0.0, 0.0, 0.0]".
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

# Точки по умолчанию — в СК map, начало которой совпадает со стартом робота (мир: x=-3, y=0)
DEFAULT_POINTS = [2.0, -1.5, 0.0,     # у ящика в юго-западной части
                  3.5, 2.0, 1.57,     # за перегородкой 1 (север)
                  5.5, 0.5, 0.0,      # восточная комната
                  0.0, 0.0, 3.14]     # возврат на старт


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def path_length(path):
    pts = np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))) if len(pts) > 1 else 0.0


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.declare_parameter('mode', 'waypoints')
    nav.declare_parameter('points', DEFAULT_POINTS)
    nav.declare_parameter('localizer', 'slam')       # slam | amcl
    nav.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
    nav.declare_parameter('planner_id', '')
    mode = nav.get_parameter('mode').value
    pts = np.array(nav.get_parameter('points').value, dtype=float).reshape(-1, 3)

    if nav.get_parameter('localizer').value == 'amcl':
        ip = nav.get_parameter('initial_pose').value
        nav.setInitialPose(make_pose(nav, *ip))
        nav.waitUntilNav2Active()
    else:
        # со SLAM локализатор не lifecycle-узел amcl — ждём только bt_navigator
        nav.waitUntilNav2Active(localizer='robot_localization')

    # пройденный путь по одометрии
    travelled = {'d': 0.0, 'prev': None}

    def on_odom(msg):
        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        if travelled['prev'] is not None:
            travelled['d'] += float(np.linalg.norm(p - travelled['prev']))
        travelled['prev'] = p
    nav.create_subscription(Odometry, '/odom', on_odom, 20)

    poses = [make_pose(nav, *row) for row in pts]

    if mode == 'path_stats':
        planner = nav.get_parameter('planner_id').value
        nav.info(f'Планировщик: {planner or "по умолчанию (GridBased)"}')
        for i, goal in enumerate(poses):
            t0 = time.time()
            path = nav.getPath(PoseStamped(), goal, planner_id=planner, use_start=False)
            dt = (time.time() - t0) * 1000
            if path is None:
                nav.warn(f'Точка {i + 1}: путь не найден')
                continue
            nav.info(f'Точка {i + 1} {pts[i, :2]}: длина пути {path_length(path):.2f} м, '
                     f'узлов {len(path.poses)}, время запроса {dt:.0f} мс')
    else:
        t0 = time.time()
        nav.followWaypoints(poses)
        while not nav.isTaskComplete():   # isTaskComplete() сам обрабатывает входящие сообщения
            fb = nav.getFeedback()
            if fb is not None:
                nav.get_logger().info(f'Едем к точке {fb.current_waypoint + 1}/{len(poses)}',
                                      throttle_duration_sec=5.0)
        res = nav.getResult()
        status = {TaskResult.SUCCEEDED: 'успешно', TaskResult.CANCELED: 'отменено',
                  TaskResult.FAILED: 'ошибка'}.get(res, str(res))
        nav.info(f'Маршрут завершён: {status}; время {time.time() - t0:.1f} с; '
                 f'пройдено по одометрии {travelled["d"]:.2f} м')

    nav.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o mum_labs/nav_waypoints.py $REPO_RAW/src/mum_labs/mum_labs/nav_waypoints.py
```

**`mum_labs/color_tracker.py`**

<details>
<summary>Показать содержимое <code>mum_labs/color_tracker.py</code></summary>

```python
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
```

</details>

Или скачайте готовый файл (из папки пакета):

```bash
curl -sL --create-dirs -o mum_labs/color_tracker.py $REPO_RAW/src/mum_labs/mum_labs/color_tracker.py
```

## 3.6. Запуск Nav2 для diffbot

**`launch/nav2_diffbot.launch.py`** — Gazebo с роботом, затем через 6 с Nav2 (`bringup_launch.py` из `nav2_bringup`) и RViz. Отдельные параметры Nav2 не нужны: размеры и имена топиков diffbot совпадают со стандартным `nav2_params.yaml`.

```python
"""ПР №9. diffbot в Gazebo + Nav2 (+ SLAM Toolbox) + RViz одной командой.

  # построение карты и навигация одновременно (SLAM):
  ros2 launch mum_labs nav2_diffbot.launch.py slam:=True
  # навигация по сохранённой карте (AMCL):
  ros2 launch mum_labs nav2_diffbot.launch.py slam:=False map:=$HOME/maps/lab_map.yaml
  # свои параметры Nav2:
  ros2 launch mum_labs nav2_diffbot.launch.py params_file:=$HOME/mum_ws/my_nav2_params.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_bringup')
    bot_dir = get_package_share_directory('diffbot_description')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bot_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'rviz': 'false'}.items())

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'True',
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'True',
        }.items())

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'use_sim_time': 'True'}.items())

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='True'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(nav2_dir, 'params', 'nav2_params.yaml')),
        sim,
        TimerAction(period=6.0, actions=[nav, rviz]),
    ])
```

## 3.7. Сборка и проверка

```bash
cd ~/mum_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install && source install/setup.bash
ros2 pkg executables mum_labs     # должно вывести 10 узлов
```

**Проверка 1 — прямая задача:** в одном терминале `ros2 launch arm3_description display.launch.py`, во втором `ros2 run mum_labs fk_node`. При движении ползунков ошибка «ДХ – TF» должна быть меньше 0,001 мм.

**Проверка 2 — траектория в Gazebo:**

```bash
ros2 launch arm3_description gazebo.launch.py
ros2 run mum_labs trajectory_node --ros-args -p mode:=434
```

В папке `~/mum_results` появятся PNG-график и два CSV.

**Проверка 3 — регулятор:** `ros2 launch arm3_description gazebo.launch.py control_mode:=effort` и `ros2 run mum_labs pid_node --ros-args -p gravity_comp:=true` — рука поднимается и держит позу.

**Проверка 4 — Nav2:** `ros2 launch mum_labs nav2_diffbot.launch.py slam:=True`. В RViz кнопкой **Nav2 Goal** укажите точку — робот сам построит путь и поедет.

## Итог шага 3

```
mum_ws/src/mum_labs/
├── package.xml  setup.py  setup.cfg  resource/mum_labs  test/
├── launch/nav2_diffbot.launch.py
└── mum_labs/
    ├── __init__.py  common.py  arm3_kinematics.py  trajectory_planning.py
    ├── fk_node.py  ik_node.py  cartesian_line_node.py  trajectory_node.py  moveit_lab.py  pid_node.py
    └── diff_odometry_node.py  go_to_goal_node.py  nav_waypoints.py  color_tracker.py
```

Сверить с эталоном: `git checkout step-3` (это же состояние ветки `main`).

Рабочее пространство готово — дальше выполняются практические работы по методическим указаниям.

[← Шаг 2](step2_mobile_robot.md) · [README](../README.md)
