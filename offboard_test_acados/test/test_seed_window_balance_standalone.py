"""窗口竖直力平衡种子单测(2026-09-14):机动(竖直加速度+倾角)下仍恢复真值质量,
而瞬时 T/g 偏差很大。无需 ROS 运行时。"""
import math
import os
import sys
import numpy as np
os.environ['MHE_SEED_WINDOW_BALANCE'] = '1'
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from offboard_test_acados import mhe_node as mn  # noqa: E402
P = mn.mhe_p


class _S:
    _window_balance_mass = mn.MHENode._window_balance_mass


def _fill(m, rng, noise=0.0):
    N, dt, g = P.N, P.dt, P.g
    s = _S(); s.y_buf = []; s.u_buf = []
    vz = 0.0
    for i in range(N + 1):
        t = i * dt
        az = 3.0 * math.sin(2.0 * t)                   # 竖直加速度 ±3 m/s²
        th = 0.35 * math.sin(1.3 * t)                  # 倾角 ±20°
        y = np.zeros(13); y[5] = vz
        y[6] = math.cos(th / 2); y[7] = math.sin(th / 2)
        s.y_buf.append(y)
        if i < N:
            # 区间平均加速度对应的推力,保证 Σ 伸缩求和精确
            t2 = t + dt
            az_avg = (-1.5 * math.cos(2 * t2) + 1.5 * math.cos(2 * t)) / dt
            T = m * (g + az_avg) / math.cos(th) * (1 + noise * rng.standard_normal())
            s.u_buf.append(np.array([T, 0, 0, 0]))
            vz += az_avg * dt
    return s


def test_recovers_mass_under_maneuver():
    rng = np.random.default_rng(0)
    for m in (2.064, 2.364):
        s = _fill(m, rng)
        est = s._window_balance_mass()
        assert abs(est - m) / m < 0.01, (m, est)
        T_last = s.u_buf[-1][0]
        print(f'm={m}: window={est:.4f} (err {100*(est-m)/m:+.2f}%), '
              f'instant T/g={T_last / P.g:.3f} (err {100*(T_last/P.g-m)/m:+.1f}%)')


def test_noise_and_degenerate():
    rng = np.random.default_rng(1)
    s = _fill(2.364, rng, noise=0.05)
    est = s._window_balance_mass()
    assert abs(est - 2.364) / 2.364 < 0.03, est
    s.u_buf = s.u_buf[:-1]
    assert s._window_balance_mass() is None


if __name__ == '__main__':
    test_recovers_mass_under_maneuver()
    test_noise_and_degenerate()
    print('[PASS] seed window balance')
