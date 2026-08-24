#!/usr/bin/env python3
# M0 验证(离线,不需要 ROS2/PX4/Gazebo):事件触发权重调度(mhe_event_weights.py)
# 对比固定权重,质量阶跃后的收敛速度必须显著更快、且阶跃前/稳态精度不劣化。
# 合成真值/噪声/滑窗机制全部复用 test_mhe_standalone.py,唯一差别是 solve 前
# 多一次 scheduler.apply()。运行方式同 test_mhe_standalone.py:
#   export ACADOS_SOURCE_DIR=/home/clear/acados
#   export LD_LIBRARY_PATH=/home/clear/acados/lib:$LD_LIBRARY_PATH
#   python3 test_mhe_event_trigger_standalone.py   (或者用 pytest)

import os

import numpy as np

os.environ.setdefault('ACADOS_SOURCE_DIR', '/home/clear/acados')
os.environ['LD_LIBRARY_PATH'] = (
    '/home/clear/acados/lib:' + os.environ.get('LD_LIBRARY_PATH', ''))

from offboard_test_acados.mhe_params import p as mhe_p  # noqa: E402
from offboard_test_acados.mhe_solver_builder import ensure_mhe_ocp_solver  # noqa: E402
from offboard_test_acados.mhe_event_weights import EventWeightScheduler  # noqa: E402

from test_mhe_standalone import (  # noqa: E402
    _simulate_truth, _add_measurement_noise)

np.random.seed(42)


def _run_mhe(Y, U, m_init_guess, scheduler=None, event_frame=None, lam=1.0):
    """跟 test_mhe_standalone._run_mhe 相同的滑窗在线 MHE,唯一差别:
    scheduler 非 None 时,solve 前调 scheduler.apply()(事件触发降权)。
    每次调用都显式把全部 stage 恢复名义权重起步——两次运行共享同一个
    solver 实例,上一轮留下的降权状态不能串到下一轮。"""
    solver = ensure_mhe_ocp_solver()
    N, nx, nw = mhe_p.N, mhe_p.nx, mhe_p.nw

    # 显式复位名义权重(消除跨运行串扰)
    _reset = EventWeightScheduler()
    if lam < 1.0:
        # 遗忘因子臂:逐 stage 按年龄打折 λ^(N-1-j),到达代价块不打折。
        # **不用任何事件信号**——这正是要对比的点。
        from scipy.linalg import block_diag as _bd
        solver.cost_set(0, 'W', _bd(lam**(N-1)*mhe_p.R,
                                    lam**(N-1)*mhe_p.Q, mhe_p.Q0))
        for j in range(1, N):
            solver.cost_set(j, 'W', (lam**(N-1-j)) * _reset.W_nom)
    else:
        solver.cost_set(0, 'W', _reset.W0_nom)
        for j in range(1, N):
            solver.cost_set(j, 'W', _reset.W_nom)

    if scheduler is not None and event_frame is not None:
        scheduler.notify_event(event_frame)

    n_total = Y.shape[0]
    m_est_seq = np.full(n_total, np.nan)

    x0_bar = np.concatenate([Y[0], [m_init_guess]])
    x_guess = [np.concatenate([Y[min(i, n_total-1)], [m_init_guess]])
               for i in range(N+1)]

    for k in range(N, n_total):
        y_win = Y[k-N:k+1]
        u_win = U[k-N:k]

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

        if scheduler is not None:
            scheduler.apply(solver, k)

        status = solver.solve()
        if status != 0:
            m_est_seq[k] = m_est_seq[k-1] if k > N else m_init_guess
            continue

        x_sol = [solver.get(i, 'x') for i in range(N+1)]
        m_est_seq[k] = x_sol[N][nx]

        x0_bar = x_sol[1].copy()
        x_guess = [x_sol[min(i+1, N)].copy() for i in range(N+1)]

    return m_est_seq


def _settle_time(t, m_est_seq, t_jump, m_after, band=0.10):
    """阶跃后收敛时间:最后一次落在 ±band 之外的时刻到 t_jump 的间隔
    (进带后必须待住,不许再出来——比"首次进带"更严格,不给瞬冲蒙混)。"""
    post = (t >= t_jump) & np.isfinite(m_est_seq)
    outside = post & (np.abs(m_est_seq - m_after) > band)
    if not np.any(outside):
        return 0.0
    return float(t[outside][-1] + mhe_p.dt - t_jump)


def _compare():
    dt = mhe_p.dt
    duration = 14.0
    t_jump = 6.0
    m_before = mhe_p.m_nominal
    m_after = mhe_p.m_nominal + 1.5

    m_true_fn = lambda t: m_before if t < t_jump else m_after  # noqa: E731

    X, U = _simulate_truth(duration, dt, m_true_fn)
    Y = _add_measurement_noise(X)

    # 事件帧序号:_simulate_truth 里第 k 步用 m_true_fn(k*dt) 积分出 X[k+1],
    # k*dt>=t_jump 的第一步是 k=60 → 第一个体现新质量物理的测量是 X[61]
    event_frame = int(round(t_jump / dt)) + 1

    m_fixed = _run_mhe(Y, U, m_init_guess=1.0)
    m_event = _run_mhe(Y, U, m_init_guess=1.0,
                       scheduler=EventWeightScheduler(), event_frame=event_frame)
    # 遗忘因子臂(2026-08-24):**零外部信号**,靠逐 stage 年龄打折跟上突变。
    lams = [float(x) for x in
            os.environ.get('LAMBDAS', '0.95,0.9,0.8,0.7').split(',')]
    m_lams = [(l, _run_mhe(Y, U, m_init_guess=1.0, lam=l)) for l in lams]

    t = np.arange(len(m_fixed)) * dt
    return (t, m_fixed, m_event, m_before, m_after, t_jump, duration, m_lams)


def test_event_trigger_speeds_up_convergence():
    t, m_fixed, m_event, m_before, m_after, t_jump, duration, m_lams = _compare()

    ts_fixed = _settle_time(t, m_fixed, t_jump, m_after)
    ts_event = _settle_time(t, m_event, t_jump, m_after)
    print(f'固定权重  收敛时间 = {ts_fixed:.2f}s')
    print(f'事件触发  收敛时间 = {ts_event:.2f}s')
    for _l, _m in m_lams:
        _tl = _settle_time(t, _m, t_jump, m_after)
        _sel = t > t_jump + 3.0
        _err = float(np.nanmean(np.abs(_m[_sel] - m_after)))
        print(f'遗忘 λ={_l:<5} 收敛时间 = {_tl:.2f}s  稳态误差 = {_err:.4f}kg  (零外部信号)')

    # 阶跃前精度不许劣化(事件触发在 t_jump 前根本不该动权重)
    pre = (t >= 3.0) & (t < t_jump - 0.5)
    err_pre_fixed = np.nanmean(np.abs(m_fixed[pre] - m_before))
    err_pre_event = np.nanmean(np.abs(m_event[pre] - m_before))
    print(f'阶跃前平均误差: 固定={err_pre_fixed:.4f}kg, 事件={err_pre_event:.4f}kg')
    assert abs(err_pre_fixed - err_pre_event) < 1e-6, \
        '事件触发版在阶跃前动了权重(不该发生)'

    # 稳态精度不许劣化(事件滑出窗口后应恢复名义权重)
    post = (t >= t_jump + 4.0) & (t < duration - 0.2)
    err_post_fixed = np.nanmean(np.abs(m_fixed[post] - m_after))
    err_post_event = np.nanmean(np.abs(m_event[post] - m_after))
    print(f'稳态平均误差:   固定={err_post_fixed:.4f}kg, 事件={err_post_event:.4f}kg')
    assert err_post_event < err_post_fixed + 0.05, '事件触发版稳态精度劣化'

    # 核心指标:收敛显著加速(至少减半)
    assert ts_event <= 0.5 * ts_fixed, (
        f'事件触发未显著加速收敛: {ts_event:.2f}s vs 固定 {ts_fixed:.2f}s')


if __name__ == '__main__':
    t, m_fixed, m_event, m_before, m_after, t_jump, duration, m_lams = _compare()
    ts_fixed = _settle_time(t, m_fixed, t_jump, m_after)
    ts_event = _settle_time(t, m_event, t_jump, m_after)
    print(f'固定权重  收敛时间 = {ts_fixed:.2f}s')
    print(f'事件触发  收敛时间 = {ts_event:.2f}s')
    for _l, _m in m_lams:
        _tl = _settle_time(t, _m, t_jump, m_after)
        _sel = t > t_jump + 3.0
        _err = float(np.nanmean(np.abs(_m[_sel] - m_after)))
        print(f'遗忘 λ={_l:<5} 收敛时间 = {_tl:.2f}s  稳态误差 = {_err:.4f}kg  (零外部信号)')

    test_event_trigger_speeds_up_convergence()
    print('[PASS] test_event_trigger_speeds_up_convergence')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        m_true_fn = lambda ti: m_before if ti < t_jump else m_after  # noqa: E731
        m_true_seq = np.array([m_true_fn(ti) for ti in t])
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, m_true_seq, 'r--', label='true mass')
        ax.plot(t, m_fixed, 'b-', alpha=0.8,
                label=f'fixed weights (settle {ts_fixed:.2f}s)')
        ax.plot(t, m_event, 'g-', alpha=0.8,
                label=f'event-triggered (settle {ts_event:.2f}s)')
        ax.set_xlabel('t [s]')
        ax.set_ylabel('mass [kg]')
        ax.set_title('MHE mass step: fixed vs event-triggered de-weighting (M0)')
        ax.legend()
        ax.grid(True)
        out_path = os.path.join(
            os.path.dirname(__file__), '..', '..', '..',
            'nmpc_test_results', 'mhe_event_trigger_standalone.png')
        out_path = os.path.abspath(out_path)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        fig.savefig(out_path, dpi=120)
        print(f'saved plot to {out_path}')
    except ImportError:
        pass
