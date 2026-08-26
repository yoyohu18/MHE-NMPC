#!/usr/bin/env python3
# 冷启动("从头估计")验证,2026-08-25。分两层:
#
#  A. 纯逻辑层(不需要 acados/ROS):pre-offboard 的起飞门控、收敛判定、发布闸、
#     以及 no_mass_prior 档下种子兜底**绝不回退 m_nominal** 这条。
#  B. 可观测性层(需要 acados):把 Q0 质量维降到 1e-6、lbx 下界从 0.95·m_B 放到
#     0.2kg 之后,从**箱约束中点**(2.6kg,不含任何 m_B 信息)起步,MHE 还能不能
#     只靠窗口内的测量把质量估回真值 —— 这才是"不给初始值、从头估计"能不能
#     成立的核心问题。对照组是历史默认档(有先验)。
#
# 跑法:
#   export ACADOS_SOURCE_DIR=/home/clear/acados
#   export LD_LIBRARY_PATH=/home/clear/acados/lib:$LD_LIBRARY_PATH
#   MHE_NO_MASS_PRIOR=1 python3 test_mhe_cold_start_standalone.py
# 注意 B 层必须在 MHE_NO_MASS_PRIOR=1 下跑(lbx 变了要重新生成 solver,首轮慢
# 约 30s);A 层两档都能跑。

import math
import os

import numpy as np

os.environ.setdefault('ACADOS_SOURCE_DIR', '/home/clear/acados')
os.environ['LD_LIBRARY_PATH'] = (
    '/home/clear/acados/lib:' + os.environ.get('LD_LIBRARY_PATH', ''))

from offboard_test_acados.mhe_params import p as mhe_p  # noqa: E402


# ============================ A. 纯逻辑层 ============================
# 用一个最小 stub 复现 mhe_node 里那三个方法依赖的属性,避免为了测几十行
# 判定逻辑去起 rclpy。方法本体直接从类上借用(未绑定函数 + 显式传 self),
# 所以测的是**真实实现**,不是复制品。
from offboard_test_acados import mhe_node as mn  # noqa: E402


class _Stub:
    def __init__(self, **kw):
        self.pre_offboard_estimate = True
        self.pre_offboard_min_z = 0.5
        self.pre_off_conv_std = 0.02
        self.pre_off_conv_frames = 5
        self.pre_off_timeout_frames = 100
        self._u_from_nmpc = False
        self._pre_off_converged = False
        self._pre_off_m_hist = []
        self._pre_off_first_frame = None
        self.thrust_phys = 20.0
        self.x_meas = np.zeros(13)
        self.x_meas[2] = 3.0
        self.m_est = 2.0
        self.frames = 0
        self.__dict__.update(kw)
        self.logged = []

    def get_logger(self):
        stub = self

        class _L:
            def info(self, m):
                stub.logged.append(('info', m))

            def warn(self, m):
                stub.logged.append(('warn', m))
        return _L()


_gate = mn.MHENode._pre_offboard_gate_ok
_conv = mn.MHENode._update_pre_offboard_convergence
_pub = mn.MHENode._mass_publish_allowed


def test_gate_rejects_on_ground():
    """地面段必须拒绝:z 低于门限 / 推力低于 seed_thrust_min。
    这是全仓反复踩过的地面支持力坑 —— pre-offboard 段真的会在地上。"""
    assert _gate(_Stub()) is True
    assert _gate(_Stub(x_meas=np.array([0., 0., 0.1] + [0.]*10))) is False
    assert _gate(_Stub(thrust_phys=0.3)) is False
    assert _gate(_Stub(thrust_phys=None)) is False
    assert _gate(_Stub(x_meas=None)) is False
    print('[A1] 起飞门控 OK')


def test_convergence_latches():
    """m_est 稳定住之后判收敛,且收敛后锁存(不因后续抖动翻回未收敛)。"""
    s = _Stub()
    for i in range(4):
        s.frames = i
        s.m_est = 2.0 + 0.0001 * i
        _conv(s)
        assert not s._pre_off_converged, f'第 {i} 帧不该收敛(样本不够)'
    s.frames = 4
    _conv(s)
    assert s._pre_off_converged, '5 帧稳定值应判收敛'
    # 锁存:之后剧烈抖动也不翻回
    s.m_est = 4.0
    _conv(s)
    assert s._pre_off_converged
    print('[A2] 收敛判定 + 锁存 OK')


def test_convergence_rejects_drifting():
    """还在漂的时候绝不能判收敛(否则会把未收敛值灌给 NMPC)。"""
    s = _Stub()
    for i in range(10):
        s.frames = i
        s.m_est = 2.0 + 0.1 * i      # 持续漂,std 远超 0.02
        _conv(s)
    assert not s._pre_off_converged
    print('[A3] 漂移期不误判收敛 OK')


def test_publish_gate():
    """未收敛不发布;收敛/接管后放行;开关关闭时恒放行(历史行为不变)。"""
    s = _Stub()
    s._pre_off_first_frame = 0
    assert _pub(s) is False, '未收敛不该发布'
    s._pre_off_converged = True
    assert _pub(s) is True
    s2 = _Stub(_u_from_nmpc=True)
    assert _pub(s2) is True, 'NMPC 接管后恒放行'
    s3 = _Stub(pre_offboard_estimate=False)
    assert _pub(s3) is True, '开关关闭 = 历史行为'
    print('[A4] 发布闸 OK')


def test_publish_gate_timeout():
    """超时兜底:等太久还不收敛要放行 + WARN,不能悄悄退化成 use_mhe=False。"""
    s = _Stub()
    s._pre_off_first_frame = 0
    s.frames = 101
    assert _pub(s) is True
    assert any(lv == 'warn' for lv, _ in s.logged), '超时必须 WARN'
    print('[A5] 超时兜底 OK')


def test_seed_fallback_never_m_nominal_when_cold():
    """no_mass_prior 档下拿不到推力时,兜底必须是箱约束中点而**不是**
    m_nominal —— 后者正是要去掉的那个先验。"""
    seed = mn.MHENode._seed_mass_from_thrust
    s = _Stub(thrust_phys=None)
    s.m_est = 2.0
    m, from_thrust = seed(s, 'unit test')
    assert not from_thrust
    if mhe_p.no_mass_prior:
        mid = 0.5 * (mhe_p.m_min + mhe_p.m_max)
        assert abs(m - mid) < 1e-9, f'冷启动兜底应为中点 {mid},实得 {m}'
        assert abs(m - mhe_p.m_nominal) > 0.1, '兜底绝不能是 m_nominal'
        print(f'[A6] 冷启动兜底 = 箱约束中点 {m:.3f} kg (非 m_nominal) OK')
    else:
        assert abs(m - mhe_p.m_nominal) < 1e-9
        print(f'[A6] 默认档兜底 = m_nominal {m:.3f} kg OK')


# ======================= B. 可观测性层(需要 acados)=======================
def test_cold_start_converges_from_midpoint():
    """核心问题:卸掉 Q0 软锚 + lbx 硬下界之后,从箱约束中点起步,悬停段的
    MHE 能不能只靠测量把质量估回真值?

    合成一段恒定质量的悬停/小机动数据(复用 test_mhe_standalone 的真值仿真),
    对照两个起点:①箱约束中点(冷启动,不含 m_B 信息);②真值附近。若两者收敛
    到同一个值,说明**起点不影响最终解** —— 这正是"零先验"的操作性定义。
    """
    if not mhe_p.no_mass_prior:
        print('[B] 跳过:需要 MHE_NO_MASS_PRIOR=1 (当前是默认档)')
        return
    from test_mhe_standalone import _simulate_truth, _add_measurement_noise
    from offboard_test_acados.mhe_solver_builder import ensure_mhe_ocp_solver

    np.random.seed(7)
    m_true = 2.0643
    X, U = _simulate_truth(12.0, mhe_p.dt, lambda t: m_true)
    Y = _add_measurement_noise(X)

    def _run(m_start):
        solver = ensure_mhe_ocp_solver()
        N, nx, nw = mhe_p.N, mhe_p.nx, mhe_p.nw
        nx_aug = mhe_p.nx_aug
        x0_bar = np.concatenate([Y[0], [m_start], np.zeros(mhe_p.ns)])
        x_guess = [x0_bar.copy() for _ in range(N + 1)]
        traj = []
        for k in range(N, len(U)):
            yw = Y[k - N:k + 1]
            uw = U[k - N:k]
            for i in range(N):
                yref = np.concatenate([yw[i], np.zeros(nw)])
                if i == 0:
                    yref = np.concatenate([yref, x0_bar])
                solver.set(i, 'yref', yref)
                solver.set(i, 'p', np.concatenate(
                    [uw[i], np.zeros(mhe_p.n_geom)]))
                solver.set(i, 'x', x_guess[i])
            solver.set(N, 'x', x_guess[N])
            solver.set(N, 'p', np.concatenate(
                [uw[-1], np.zeros(mhe_p.n_geom)]))
            solver.solve()
            xs = [solver.get(i, 'x') for i in range(N + 1)]
            x0_bar = xs[1].copy()
            x_guess = [xs[min(i + 1, N)].copy() for i in range(N + 1)]
            traj.append(float(xs[N][nx]))
        return np.array(traj)

    mid = 0.5 * (mhe_p.m_min + mhe_p.m_max)
    t_cold = _run(mid)
    t_warm = _run(m_true)

    err_cold = abs(t_cold[-1] - m_true) / m_true * 100
    err_warm = abs(t_warm[-1] - m_true) / m_true * 100
    gap = abs(t_cold[-1] - t_warm[-1])
    print(f'[B] 起点 {mid:.3f}kg (冷) -> 末值 {t_cold[-1]:.4f} (err {err_cold:+.2f}%)')
    print(f'[B] 起点 {m_true:.3f}kg (暖) -> 末值 {t_warm[-1]:.4f} (err {err_warm:+.2f}%)')
    print(f'[B] 两个起点的末值差 = {gap:.4f} kg')

    assert err_cold < 5.0, f'冷启动末值误差 {err_cold:.2f}% 过大'
    assert gap < 0.05, f'起点显著影响最终解({gap:.4f}kg)=> 不是真零先验'
    # 收敛用时:第一次进入真值 ±2% 带并保持
    band = np.abs(t_cold - m_true) / m_true < 0.02
    idx = next((i for i in range(len(band)) if band[i:].all()), None)
    if idx is not None:
        print(f'[B] 冷启动入带(±2%)用时 {idx * mhe_p.dt:.1f}s')
    print('[B] 冷启动可观测性 OK')


if __name__ == '__main__':
    print(f'=== no_mass_prior={mhe_p.no_mass_prior}, '
          f'm_min={mhe_p.m_min:.3f}, Q0_m={mhe_p.Q0[mhe_p.nx, mhe_p.nx]:.1e} ===')
    test_gate_rejects_on_ground()
    test_convergence_latches()
    test_convergence_rejects_drifting()
    test_publish_gate()
    test_publish_gate_timeout()
    test_seed_fallback_never_m_nominal_when_cold()
    test_cold_start_converges_from_midpoint()
    print('\nall passed')
