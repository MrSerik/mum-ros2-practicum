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
