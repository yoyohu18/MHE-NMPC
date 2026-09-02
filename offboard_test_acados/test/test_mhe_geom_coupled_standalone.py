#!/usr/bin/env python3
"""MHE 几何-质量耦合档的独立验证(不需要 ROS2/PX4/Gazebo)。

回答的问题:把 J 和复合质心写成被估质量的函数(mhe_params.geom_coupled=True)之后,
**偏心吊挂**场景下的质量估计精度/收敛速度到底改善多少,以及是否引入新的失效。

真值 plant 用纯 numpy 写、且用**完整 3x3 惯量**(含平行轴定理的非对角项),所以
两档 MHE 面对的都是"比自己模型更全"的真实物理:
  legacy  档的模型:对角 J、dJ/c 是外部常数(∂(J,c)/∂m ≡ 0)
  coupled 档的模型:完整 3x3 J(m)、c(m)
时间线: 0~5s 空机悬停 -> attach 0.3kg@r_p=[0,0.05,-0.47] -> 15s drop -> 20s 结束。

跑法(两档各跑一次,solver codegen 目录不同、不会互相覆盖):
  export ACADOS_SOURCE_DIR=/home/clear/acados
  MHE_GEOM_COUPLED=0 python3 test_mhe_geom_coupled_standalone.py
  MHE_GEOM_COUPLED=1 python3 test_mhe_geom_coupled_standalone.py
"""
import os
import sys

import numpy as np
from scipy.linalg import block_diag

os.environ.setdefault('ACADOS_SOURCE_DIR', '/home/clear/acados')
os.environ['LD_LIBRARY_PATH'] = (
    '/home/clear/acados/lib:' + os.environ.get('LD_LIBRARY_PATH', ''))

from offboard_test_acados.mhe_params import p as mhe_p          # noqa: E402
from offboard_test_acados.mhe_solver_builder import (            # noqa: E402
    ensure_mhe_ocp_solver)

np.random.seed(int(os.environ.get('SEED', '7')))

M_B = mhe_p.m_B
G = mhe_p.g
R_P = np.array([0.0, 0.05, -0.47])     # 载荷相对机体原点的偏移(rz<0)
# 载荷质量。0.15kg 是 self 档触界现象最重的工况(2026-09-02),故可配。
M_P = float(os.environ.get('PAYLOAD_KG', '0.3'))
T_ATTACH, T_DROP, T_END = 5.0, 15.0, 20.0
DT = mhe_p.dt
# legacy 档的几何来源:'ideal'=操作先验给真值;'ratchet'=从 m_est 反推+棘轮+地板
LEGACY_GEOM = os.environ.get('LEGACY_GEOM', 'ideal')
GEOM_FLOOR = float(os.environ.get('GEOM_FLOOR', '0.15'))
# 喂给**估计器**的 r_p 相对真值的缩放(真值 plant 始终用 R_P)。用来量化
# "coupled 档把 r_xy 的先验误差直接转成质量偏差"这个已知风险有多大。
RP_XY_SCALE = float(os.environ.get('RP_XY_SCALE', '1.0'))
RP_Z_SCALE = float(os.environ.get('RP_Z_SCALE', '1.0'))
# 'event'(默认)=drop 时把模型几何清零;'self'=模型不被告知 drop,
# 载荷特征必须靠估计自己归零(耦合档靠 m_P⁺,moment 档靠 s)。
# 'sgate'=仍不被告知 drop,但用 **|s| 自己的衰减** 当判据去清模型杆臂:
#   s 有独立观测(τ/T 通路)、drop 后 2s 内衰减 ~90%,不像 m 那样身兼二职。
#   杆臂一清,幽灵几何不复存在 → A=μr_z² 可以继续随 m 走,转动通路的质量信息
#   全部保留(那是 coupled 档 94% 的质量信息来源)。
RELEASE = os.environ.get('RELEASE_MODE', 'event')
S_REL_THRESH = float(os.environ.get('S_REL_THRESH', '0.003'))   # kg·m
S_REL_PERSIST = int(os.environ.get('S_REL_PERSIST', '10'))      # 帧


# ---------------- 真值 plant(完整 3x3 惯量,含非对角项) ----------------
def J_true(m_t, r_p):
    J = np.diag([mhe_p.Jxx, mhe_p.Jyy, mhe_p.Jzz])
    m_p = m_t - M_B
    if m_p <= 0.0:
        return J
    mu = M_B * m_p / m_t
    return J + mu * (float(r_p @ r_p) * np.eye(3) - np.outer(r_p, r_p))


def c_true(m_t, r_p):
    m_p = m_t - M_B
    return np.zeros(2) if m_p <= 0.0 else (m_p / m_t) * r_p[0:2]


def rotmat(q):
    qw, qx, qy, qz = q
    return np.array([
        [qw**2+qx**2-qy**2-qz**2, 2*(qx*qy-qw*qz),         2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz),         qw**2-qx**2+qy**2-qz**2, 2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy),         2*(qy*qz+qw*qx),         qw**2-qx**2-qy**2+qz**2],
    ])


def plant_ode(x, u, m_t, r_p):
    vel, q, om = x[3:6], x[6:10], x[10:13]
    T, tau = u[0], u[1:4]
    R = rotmat(q)
    vel_dot = (1.0/m_t) * (R @ np.array([0.0, 0.0, T]) - mhe_p.kd*vel) \
        - np.array([0.0, 0.0, G])
    qw, qx, qy, qz = q
    Xi = np.array([[-qx, -qy, -qz], [qw, -qz, qy], [qz, qw, -qx], [-qy, qx, qw]])
    q_dot = 0.5 * Xi @ om
    J = J_true(m_t, r_p)
    c = c_true(m_t, r_p)
    tau_com = np.array([-c[1]*T, c[0]*T, 0.0])
    om_dot = np.linalg.solve(J, tau + tau_com - np.cross(om, J @ om))
    return np.concatenate([vel, vel_dot, q_dot, om_dot])


def plant_step(x, u, m_t, r_p, dt):
    k1 = plant_ode(x, u, m_t, r_p)
    k2 = plant_ode(x + dt/2*k1, u, m_t, r_p)
    k3 = plant_ode(x + dt/2*k2, u, m_t, r_p)
    k4 = plant_ode(x + dt*k3, u, m_t, r_p)
    xn = x + dt/6*(k1 + 2*k2 + 2*k3 + k4)
    xn[6:10] /= np.linalg.norm(xn[6:10])
    return xn


# ---------------- 一个简单的稳定控制器(只为把飞机保持在悬停附近) --------
def controller(x, m_cmd, z_ref=3.0):
    """PD:高度 -> T,姿态(把 q 拉回单位四元数)+ 角速度阻尼 -> tau。
    m_cmd 用**真值质量**(这里在测估计器,不测控制器)。"""
    z, vz = x[2], x[5]
    T = m_cmd * (G + 2.0*(z_ref - z) - 2.4*vz)
    T = float(np.clip(T, 1.0, 2.0*3.5*G))
    qv, om = x[7:10], x[10:13]
    tau = -18.0*np.array([qv[0], qv[1], qv[2]]) - 1.2*om
    tau = np.clip(tau, [-0.5, -0.5, -0.2], [0.5, 0.5, 0.2])
    return np.concatenate([[T], tau])


def _aug(y13, m_seed):
    """13 维量测 -> MHE 增广状态初值(14 或 16 维);s 一律从 0 起。"""
    return np.concatenate([y13, [m_seed], np.zeros(mhe_p.ns)])


# ---------------- 跑一轮 ----------------
def run(verbose=True):
    solver = ensure_mhe_ocp_solver()
    N, nx, nw = mhe_p.N, mhe_p.nx, mhe_p.nw
    SIG = np.array([mhe_p.std_pos]*3 + [mhe_p.std_vel]*3 +
                   [mhe_p.std_quat]*4 + [mhe_p.std_omega]*3)

    x = np.zeros(13); x[2] = 3.0; x[6] = 1.0
    applied = [False]             # 遗忘因子权重只设一次
    ratchet = [0.0]               # legacy ratchet 路径的状态(只增不减)
    r_p_last = np.zeros(3)        # 'self' 释放模式下模型只认它(drop 不清)
    s_low = [0]                   # 'sgate' 的连续低于阈值帧数
    y_buf, u_buf = [], []
    x0_bar = x_guess = None
    m_est = M_B
    s_est = np.zeros(mhe_p.ns)
    fails = 0
    rec = []                      # (t, m_true, m_est, attached)

    n_steps = int(round(T_END/DT))
    for k in range(n_steps):
        t = k*DT
        attached = T_ATTACH <= t < T_DROP
        if attached:
            r_p_last = R_P
        if RELEASE == 'sgate' and mhe_p.ns and np.any(r_p_last):
            # 判据只看 s(估计器自己的量),不看 attached —— 与 'self' 一样零外部信号
            if float(np.linalg.norm(s_est)) < S_REL_THRESH:
                s_low[0] += 1
                if s_low[0] >= S_REL_PERSIST:
                    r_p_last = np.zeros(3)
                    s_low[0] = 0
            else:
                s_low[0] = 0
        if not attached:
            ratchet[0] = 0.0      # drop 释放几何(与 mhe_node.mass_event_cb 一致)
        m_t = M_B + (M_P if attached else 0.0)
        r_p = R_P if attached else np.zeros(3)

        u = controller(x, m_t)
        # 已知输入:真实推力/力矩(对应真机用电机转速反算的 T_phys/tau_phys)
        y = x + SIG*np.random.randn(13)
        y[6:10] /= np.linalg.norm(y[6:10])

        y_buf.append(y); u_buf.append(u.copy())
        if len(y_buf) > N+1: y_buf.pop(0)
        if len(u_buf) > N:   u_buf.pop(0)

        if len(y_buf) == N+1:
            if x0_bar is None:
                x0_bar = _aug(y_buf[0], M_B)
                x_guess = [_aug(y_buf[min(i, N)], M_B) for i in range(N+1)]
            # geom 槛位:两档语义不同(见 mhe_params.n_geom)
            if mhe_p.geom_coupled:
                r_model = r_p_last if RELEASE in ('self', 'sgate') else r_p
                geom = np.asarray(r_model, dtype=float) * np.array(
                    [RP_XY_SCALE, RP_XY_SCALE, RP_Z_SCALE])
                if mhe_p.estimate_moment and mhe_p.moment_a_mode == 'frozen':
                    # geom[0] 在 moment 档空闲,借来传上一窗口的 m̂(见 mhe_model)
                    geom = geom.copy()
                    geom[0] = m_est
            else:
                # legacy 有两条几何来源(见 mhe_node._payload_geometry):
                #   LEGACY_GEOM=ideal  操作先验=真值(grip_geom_mp_prior 完全解耦路径)
                #   LEGACY_GEOM=ratchet 从 m_est 反推+棘轮+地板(prior 未给时的默认)
                if attached:
                    if LEGACY_GEOM == 'ratchet':
                        mp_inst = max(m_est - M_B, 0.0)
                        ratchet[0] = min(max(ratchet[0], mp_inst), mhe_p.m_max-M_B)
                        m_p_hat = max(ratchet[0], GEOM_FLOOR)
                    else:
                        m_p_hat = M_P
                    m_tt = M_B + m_p_hat
                    mu = M_B*m_p_hat/m_tt
                    dJ = mu*(r_p[2]**2 + 0.5*(r_p[0]**2 + r_p[1]**2))
                    c = (m_p_hat/m_tt)*r_p[0:2]
                    geom = np.array([dJ, c[0], c[1]])
                else:
                    geom = np.zeros(3)

            solver.set(0, 'yref', np.concatenate([y_buf[0], np.zeros(nw), x0_bar]))
            solver.set(0, 'p', np.concatenate([u_buf[0], geom]))
            solver.set(0, 'x', x_guess[0])
            for j in range(1, N):
                solver.set(j, 'yref', np.concatenate([y_buf[j], np.zeros(nw)]))
                solver.set(j, 'p', np.concatenate([u_buf[j], geom]))
                solver.set(j, 'x', x_guess[j])
            solver.set(N, 'x', x_guess[N])
            if mhe_p.forgetting_lambda < 1.0 and not applied[0]:
                lam = mhe_p.forgetting_lambda
                wn = block_diag(mhe_p.R, mhe_p.Q)
                solver.cost_set(0, 'W', block_diag(
                    lam**(N-1)*mhe_p.R, lam**(N-1)*mhe_p.Q, mhe_p.Q0))
                for jj in range(1, N):
                    solver.cost_set(jj, 'W', (lam**(N-1-jj))*wn)
                applied[0] = True
            st = solver.solve()
            if st != 0:
                fails += 1
            else:
                xs = [solver.get(i, 'x') for i in range(N+1)]
                m_est = float(xs[N][nx])
                if mhe_p.ns:
                    s_est = np.array(xs[N][nx+1:nx+1+mhe_p.ns], dtype=float)
                x0_bar = xs[1].copy()
                x_guess = [xs[min(i+1, N)].copy() for i in range(N+1)]
            rec.append((t, m_t, m_est, attached,
                        float(s_est[0]) if mhe_p.ns else 0.0,
                        float(s_est[1]) if mhe_p.ns else 0.0))

        x = plant_step(x, u, m_t, r_p, DT)
        if not np.all(np.isfinite(x)):
            print('plant diverged'); break

    return np.array([(r[0], r[1], r[2], r[4], r[5]) for r in rec]), fails


def band_time(rec, t_evt, t_end, m_target, tol=0.02):
    """在 [t_evt, t_end) 段内,m_est 稳定进入 ±tol 相对带的时刻(相对 t_evt,s)。
    "稳定"= 之后直到 t_end 不再出带(所以量的是收敛,不是第一次擦到带边)。"""
    seg = rec[(rec[:, 0] >= t_evt) & (rec[:, 0] < t_end)]
    ok = np.abs(seg[:, 2] - m_target)/m_target <= tol
    if not ok.any():
        return float('nan')
    # 最后一次离开带之后的第一帧 = 稳定入带时刻
    idx = np.where(~ok)[0]
    first = 0 if idx.size == 0 else idx[-1]+1
    if first >= len(seg):
        return float('nan')
    return float(seg[first, 0] - t_evt)


if __name__ == '__main__':
    rec, fails = run()
    lab = (('moment' if mhe_p.estimate_moment else
            f'coupled-rxy{RP_XY_SCALE:g}-rz{RP_Z_SCALE:g}')
           if mhe_p.geom_coupled else f'legacy-{LEGACY_GEOM}') + f'-{RELEASE}'
    if mhe_p.forgetting_lambda < 1.0:
        lab += f'-lam{mhe_p.forgetting_lambda:g}'
    ld = rec[(rec[:, 0] > T_ATTACH+2.5) & (rec[:, 0] < T_DROP)]
    ul = rec[rec[:, 0] > T_DROP+2.5]
    pre = rec[rec[:, 0] < T_ATTACH]
    def rmse(a): return float(np.sqrt(np.mean((a[:, 2]-a[:, 1])**2))) if len(a) else float('nan')
    def bias(a): return float(np.mean(a[:, 2]-a[:, 1])) if len(a) else float('nan')
    att_seg = rec[(rec[:, 0] >= T_ATTACH) & (rec[:, 0] < T_DROP)]
    over = (float(att_seg[:, 2].max() - (M_B+M_P)) if len(att_seg) else float('nan'))
    print(f'=== geom mode: {lab}  (solve fails={fails}) ===')
    print(f'  空机段 RMSE={rmse(pre):.4f}kg  bias={bias(pre):+.4f}kg')
    print(f'  带载稳态 RMSE={rmse(ld):.4f}kg  bias={bias(ld):+.4f}kg '
          f'({100*bias(ld)/(M_B+M_P):+.2f}%)')
    print(f'  drop后稳态 RMSE={rmse(ul):.4f}kg  bias={bias(ul):+.4f}kg '
          f'({100*bias(ul)/M_B:+.2f}%)')
    print(f'  attach 入带(±2%)={band_time(rec, T_ATTACH, T_DROP, M_B+M_P):.2f}s   '
          f'drop 入带(±2%)={band_time(rec, T_DROP, T_END, M_B):.2f}s')
    print(f'  attach 过冲={over:+.4f}kg')
    # 触界占空比:m_est 贴在 lbx 硬下界 m_min 上的采样比例。self 档下这是
    # "几何靠压低质量来熄灭"的直接读数 —— 贴界样本是约束截断值,不是估计值。
    def pin(a):
        return (100.0 * float(np.mean(a[:, 2] <= mhe_p.m_min + 1e-4))
                if len(a) else float('nan'))
    print(f'  触界占空比(m<=m_min={mhe_p.m_min:.3f}): 带载={pin(ld):.1f}%  '
          f'drop后={pin(ul):.1f}%  全程={pin(rec):.1f}%')
    if mhe_p.ns:
        s_true = M_P * R_P[1]
        s_ld = rec[(rec[:, 0] > T_ATTACH+2.5) & (rec[:, 0] < T_DROP), 3:5]
        s_ul = rec[rec[:, 0] > T_DROP+2.5, 3:5]
        print(f'  s_y: 带载均值={np.mean(s_ld[:, 1]):+.4f} (真值 {s_true:+.4f}, '
              f'{100*(np.mean(s_ld[:, 1])-s_true)/s_true:+.1f}%)  '
              f'drop后|s|均值={np.mean(np.abs(s_ul)):.5f} kg·m')
    out = f'/tmp/claude-1000/mhe_geom_{lab}.csv'
    np.savetxt(out, rec, delimiter=',', header='t,m_true,m_est,s_x,s_y', comments='')
    print(f'  逐帧数据 -> {out}')
