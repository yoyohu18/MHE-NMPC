#!/usr/bin/env python3
# 阶段 A 验证(跟 test_acados_ocp_standalone.py 同一个套路):不需要 ROS2/PX4/Gazebo,
# 只测 MHE 求解器本身对不对——用纯 numpy 生成一段"真值"轨迹(已知质量阶跃,模拟
# 配送场景里的抓/放包裹),加测量噪声,喂给 MHE 滑动窗口,检查质量估计能不能在
# 阶跃前后都收敛到真值附近。运行方式跟 test_acados_ocp_standalone.py 一样:
#   export ACADOS_SOURCE_DIR=/home/clear/acados
#   export LD_LIBRARY_PATH=/home/clear/acados/lib:$LD_LIBRARY_PATH
#   python3 test_mhe_standalone.py   (或者用 pytest)

import os

import numpy as np

os.environ.setdefault('ACADOS_SOURCE_DIR', '/home/clear/acados')
os.environ['LD_LIBRARY_PATH'] = (
    '/home/clear/acados/lib:' + os.environ.get('LD_LIBRARY_PATH', ''))

from offboard_test_acados.mhe_params import p as mhe_p  # noqa: E402
from offboard_test_acados.mhe_solver_builder import ensure_mhe_ocp_solver  # noqa: E402

np.random.seed(42)


# =========================================================
#  纯 numpy 版四旋翼动力学,只用来生成"真值"轨迹(独立于 acados/CasADi,
#  避免测试本身的正确性依赖于被测代码里的同一份符号表达式)
# =========================================================
def _rotmat_from_quat(q):
    qw, qx, qy, qz = q
    return np.array([
        [qw**2+qx**2-qy**2-qz**2, 2*(qx*qy-qw*qz),         2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz),         qw**2-qx**2+qy**2-qz**2, 2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy),         2*(qy*qz+qw*qx),         qw**2-qx**2-qy**2+qz**2],
    ])


def _quad_dynamics_np(x13, u4, m):
    vel = x13[3:6]
    q = x13[6:10]
    om = x13[10:13]
    T_, tau = u4[0], u4[1:4]
    R = _rotmat_from_quat(q)
    g_vec = np.array([0.0, 0.0, mhe_p.g])
    vel_dot = (1.0/m) * (R @ np.array([0.0, 0.0, T_]) - mhe_p.kd * vel) - g_vec
    qw, qx, qy, qz = q
    Xi = np.array([[-qx, -qy, -qz], [qw, -qz, qy], [qz, qw, -qx], [-qy, qx, qw]])
    quat_dot = 0.5 * Xi @ om
    J = np.array([mhe_p.Jxx, mhe_p.Jyy, mhe_p.Jzz])
    om_dot = (tau - np.cross(om, J*om)) / J
    return np.concatenate([vel, vel_dot, quat_dot, om_dot])


def _rk4_step_np(x13, u4, m, dt):
    k1 = _quad_dynamics_np(x13, u4, m)
    k2 = _quad_dynamics_np(x13 + dt/2*k1, u4, m)
    k3 = _quad_dynamics_np(x13 + dt/2*k2, u4, m)
    k4 = _quad_dynamics_np(x13 + dt*k3, u4, m)
    x_next = x13 + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
    x_next[6:10] /= np.linalg.norm(x_next[6:10])
    return x_next


def _control_at(t, m_true_fn):
    # 故意让推力偏离"刚好抵消重力"、再叠加小幅度的滚转/俯仰力矩,制造平动+
    # 转动的混合激励——质量主要通过 vel_dot 里的 (1/m)*T 项才能被看见,如果
    # T 恒等于 m*g(纯悬停)净加速度长期是 0,质量会变得不可观测。
    T = m_true_fn(t) * mhe_p.g + 3.0 * np.sin(2*np.pi*0.2*t)
    tau_x = 0.02 * np.sin(2*np.pi*0.1*t)
    tau_y = 0.02 * np.cos(2*np.pi*0.1*t)
    tau_z = 0.0
    return np.array([T, tau_x, tau_y, tau_z])


def _simulate_truth(duration, dt, m_true_fn):
    n_steps = int(round(duration/dt))
    x13 = np.array([0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    X = np.zeros((n_steps+1, 13))
    U = np.zeros((n_steps, 4))
    X[0] = x13
    t = 0.0
    for k in range(n_steps):
        u = _control_at(t, m_true_fn)
        U[k] = u
        x13 = _rk4_step_np(x13, u, m_true_fn(t), dt)
        X[k+1] = x13
        t += dt
    return X, U


def _add_measurement_noise(X):
    std = np.array([mhe_p.std_pos]*3 + [mhe_p.std_vel]*3 +
                    [mhe_p.std_quat]*4 + [mhe_p.std_omega]*3)
    Y = X + std * np.random.standard_normal(X.shape)
    Y[:, 6:10] /= np.linalg.norm(Y[:, 6:10], axis=1, keepdims=True)
    return Y


def _run_mhe(Y, U, m_init_guess):
    """滑动窗口在线 MHE:每来一帧新数据,窗口往前滑一步,到达代价的先验均值
    用上一次窗口里 x[1] 的估计值更新(shift-forward,跟 NMPC 的 warm-start
    shift 是同一套思路)。返回每个能出估计值的时刻对应的质量估计序列。"""
    solver = ensure_mhe_ocp_solver()
    N, nx, nw = mhe_p.N, mhe_p.nx, mhe_p.nw

    n_total = Y.shape[0]
    m_est_seq = np.full(n_total, np.nan)

    extra0 = np.zeros(mhe_p.ns)
    x0_bar = np.concatenate([Y[0], [m_init_guess], extra0])
    x_guess = [np.concatenate([Y[min(i, n_total-1)], [m_init_guess], extra0])
               for i in range(N+1)]

    for k in range(N, n_total):
        y_win = Y[k-N:k+1]       # N+1 个测量, 索引 0..N
        u_win = U[k-N:k]         # N 个已知输入, 索引 0..N-1

        yref_0 = np.concatenate([y_win[0], np.zeros(nw), x0_bar])
        solver.set(0, 'yref', yref_0)
        # model.p = [已知输入(4); 已知几何(3)](2026-07-07 加的 n_geom;
        # 这两个测试当时漏改,一直报 'trying to set 4 parameters ... has 7'
        # 直接退出——2026-08-24 修)。本测试是无载荷/居中场景,几何恒零:
        # legacy 档 [dJ,cx,cy]=0,coupled 档 r_p=0,两档都退化回空机 J。
        _GEOM0 = np.zeros(mhe_p.n_geom)
        solver.set(0, 'p', np.concatenate([u_win[0], _GEOM0]))
        solver.set(0, 'x', x_guess[0])

        for j in range(1, N):
            yref = np.concatenate([y_win[j], np.zeros(nw)])
            solver.set(j, 'yref', yref)
            solver.set(j, 'p', np.concatenate([u_win[j], _GEOM0]))
            solver.set(j, 'x', x_guess[j])

        solver.set(N, 'x', x_guess[N])

        status = solver.solve()
        if status != 0:
            # 偶尔不收敛就跳过这一帧,沿用上一次的估计/先验,不让脏解污染后续窗口
            m_est_seq[k] = m_est_seq[k-1] if k > N else m_init_guess
            continue

        x_sol = [solver.get(i, 'x') for i in range(N+1)]
        m_est_seq[k] = x_sol[N][nx]

        # 滑动窗口 shift:下一窗口的先验取这一次窗口里 x[1] 的估计值
        x0_bar = x_sol[1].copy()
        x_guess = [x_sol[min(i+1, N)].copy() for i in range(N+1)]

    return m_est_seq


def _compute_mhe_mass_estimate():
    dt = mhe_p.dt
    duration = 14.0
    t_jump = 6.0
    m_before = mhe_p.m_nominal
    m_after = mhe_p.m_nominal + 1.5  # 模拟抓起约 1.5kg 的包裹

    m_true_fn = lambda t: m_before if t < t_jump else m_after  # noqa: E731

    X, U = _simulate_truth(duration, dt, m_true_fn)
    Y = _add_measurement_noise(X)

    # 故意从一个错误的质量先验开始(既不等于 m_before 也不等于 m_after),
    # 用来验证 MHE 真的是从数据里学出来的,不是先验"凑巧蒙对"。
    m_est_seq = _run_mhe(Y, U, m_init_guess=1.0)
    t = np.arange(len(m_est_seq)) * dt
    return t, m_est_seq, m_true_fn, m_before, m_after, t_jump, duration


def _assert_mass_estimate(t, m_est_seq, m_before, m_after, t_jump, duration):
    pre_jump_mask = (t >= 3.0) & (t < t_jump - 0.5)
    post_jump_mask = (t >= t_jump + 4.0) & (t < duration - 0.2)

    m_est_pre = np.nanmean(m_est_seq[pre_jump_mask])
    m_est_post = np.nanmean(m_est_seq[post_jump_mask])

    print(f'pre-jump  估计质量={m_est_pre:.3f}kg (真值={m_before:.3f}kg)')
    print(f'post-jump 估计质量={m_est_post:.3f}kg (真值={m_after:.3f}kg)')

    assert abs(m_est_pre - m_before) < 0.15, (
        f'阶跃前质量估计偏差过大: 估计={m_est_pre:.3f}kg, 真值={m_before:.3f}kg')
    assert abs(m_est_post - m_after) < 0.15, (
        f'阶跃后质量估计偏差过大: 估计={m_est_post:.3f}kg, 真值={m_after:.3f}kg')


def test_mhe_tracks_mass_step():
    t, m_est_seq, _m_true_fn, m_before, m_after, t_jump, duration = (
        _compute_mhe_mass_estimate())
    _assert_mass_estimate(t, m_est_seq, m_before, m_after, t_jump, duration)


if __name__ == '__main__':
    t, m_est_seq, m_true_fn, m_before, m_after, t_jump, duration = (
        _compute_mhe_mass_estimate())
    _assert_mass_estimate(t, m_est_seq, m_before, m_after, t_jump, duration)
    print('[PASS] test_mhe_tracks_mass_step')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        m_true_seq = np.array([m_true_fn(ti) for ti in t])
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, m_true_seq, 'r--', label='true mass')
        ax.plot(t, m_est_seq, 'b-', label='MHE estimate')
        ax.set_xlabel('t [s]')
        ax.set_ylabel('mass [kg]')
        ax.set_title('MHE mass estimation under a step change (simulated pickup)')
        ax.legend()
        ax.grid(True)
        out_path = os.path.join(
            os.path.dirname(__file__), '..', '..', '..',
            'nmpc_test_results', 'mhe_mass_estimate_standalone.png')
        out_path = os.path.abspath(out_path)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        fig.savefig(out_path, dpi=120)
        print(f'saved plot to {out_path}')
    except ImportError:
        pass
