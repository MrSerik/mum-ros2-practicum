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
