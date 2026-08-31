#!/usr/bin/env python3
"""online 三参数(m / J / c_xy)收敛图。

数据源(全部来自既有日志,不需要新埋点):
  grip_mhe_*.log
    [truth]     m_hat/m_true, Jxx_hat/Jxx_true  —— 2s 一行
    [c_xy_est]  cx/cy 在线估计 + truth c=       —— 2s 一行
    [transition]/[post-event]  m_est 10Hz       —— 事件窗口内
    [geom] model_geom_on=True  —— attach 时刻,用作 t=0
  grip_nmpc_*.log
    [dJ_online] 模型侧 dJ 的在线轨迹(仅 dj_track_mest=true 时有)—— 5s 一行

用法:
  python3 plot_online_convergence.py <stamp> [<stamp> ...] -o out.png
  stamp 形如 20260828_190105
"""
import argparse
import glob
import os
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RUNDIR = '/home/clear/ros2_ws_HJH/nmpc_test_results'
T_RE = r'\[INFO\] \[(\d+\.\d+)\]'

RE_TRUTH = re.compile(
    T_RE + r'.*\[truth\] m_hat=([\d.]+) m_true=([\d.]+) err=([-+][\d.]+)% \| '
    r'c_hat=\[([-+][\d.]+),([-+][\d.]+)\] c_true=\[([-+][\d.]+),([-+][\d.]+)\] \| '
    r'Jxx_hat=([\d.]+) Jxx_true=([\d.]+)')
RE_CXY = re.compile(
    T_RE + r'.*\[c_xy_est\] cx=([-+][\d.]+) cy=([-+][\d.]+) m'
    r'(?: \| truth c=\[([-+][\d.]+),([-+][\d.]+)\])?')
RE_HF = re.compile(T_RE + r'.*\[(?:transition|post-event)\] m_est=([\d.]+)')
RE_ATTACH = re.compile(T_RE + r'.*\[geom\] model_geom_on=True')
RE_DROP = re.compile(T_RE + r'.*\| DROP')
RE_DJ = re.compile(
    T_RE + r'.*\[dJ_online\] m_est=([\d.]+) m_p=([-\d.]+) ratchet=([\d.]+) '
    r'floor=([\d.]+) -> m_p_model=([\d.]+)\((\w+)\) dJ=([\d.]+)')


def parse(stamp):
    mhe = os.path.join(RUNDIR, f'grip_mhe_{stamp}.log')
    nmpc = os.path.join(RUNDIR, f'grip_nmpc_{stamp}.log')
    d = {'stamp': stamp, 'truth': [], 'cxy': [], 'hf': [], 'dj': [],
         't0': None, 't_drop': None}
    with open(mhe, errors='ignore') as f:
        for ln in f:
            m = RE_ATTACH.search(ln)
            if m and d['t0'] is None:
                d['t0'] = float(m.group(1))
                continue
            m = RE_TRUTH.search(ln)
            if m:
                d['truth'].append([float(x) for x in m.groups()])
                continue
            m = RE_CXY.search(ln)
            if m:
                g = m.groups()
                d['cxy'].append([float(g[0]), float(g[1]), float(g[2]),
                                 float(g[3]) if g[3] else np.nan,
                                 float(g[4]) if g[4] else np.nan])
                continue
            m = RE_HF.search(ln)
            if m:
                d['hf'].append([float(m.group(1)), float(m.group(2))])
    if os.path.exists(nmpc):
        with open(nmpc, errors='ignore') as f:
            for ln in f:
                m = RE_DROP.search(ln)
                if m and d['t_drop'] is None:
                    d['t_drop'] = float(m.group(1))
                    continue
                m = RE_DJ.search(ln)
                if m:
                    g = m.groups()
                    d['dj'].append([float(g[0]), float(g[1]), float(g[2]),
                                    float(g[3]), float(g[4]), float(g[5]),
                                    g[6], float(g[7])])
    if d['t_drop'] is not None and d['cxy']:
        for row in d['cxy']:
            if row[0] > d['t_drop'] and np.isnan(row[3]):
                row[3] = row[4] = 0.0
    # 载荷离机时模型 dJ 被显式清零(LOST 走 payload_lost_cb、计划 DROP 走
    # _grip_drop_phase),但 [dJ_online] 在 grip_dropped 分支提前 return,**日志
    # 里不会有那个 0** —— 不补的话曲线停在最后一个带载值,看着像"没归零"。
    if d['dj'] and d['t_drop'] is not None:
        d['dj'] = [r for r in d['dj'] if r[0] <= d['t_drop']]
        last = list(d['dj'][-1]) if d['dj'] else [d['t_drop'], 0, 0, 0, 0, 0,
                                                  'drop', 0.0]
        last[0] = d['t_drop']; last[7] = 0.0; last[6] = 'dropped'
        d['dj'].append(tuple(last))
    for k in ('truth', 'cxy', 'hf'):
        d[k] = np.array(d[k]) if d[k] else np.zeros((0, 9))
    if d['t0'] is None and len(d['truth']):
        d['t0'] = d['truth'][0][0]
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('stamps', nargs='+')
    ap.add_argument('-o', '--out', default='/tmp/online_convergence.png')
    ap.add_argument('--title', default='')
    ap.add_argument('--tmax', type=float, default=None)
    ap.add_argument('--ref', default='')
    ap.add_argument('--arm-d', type=float, default=0.47)
    ap.add_argument('--mB', type=float, default=2.0643)
    args = ap.parse_args()

    runs = [parse(s) for s in args.stamps]
    runs = [r for r in runs if len(r['truth'])]
    if not runs:
        raise SystemExit('no data parsed')

    def dJ_nmpc(m_p):
        # 与 acados_nmpc_node._dJ_from_mp 同式:dJ = mu*d^2, mu = m_B*m_p/m_t。
        # NMPC 用标称力臂 grip_arm_d,与 MHE 那条用实际 attach r_p 的口径不同,
        # 两者不能直接比 —— NMPC 的误差必须对它自己口径的真值算。
        m_t = args.mB + m_p
        return (args.mB * m_p / m_t) * args.arm_d ** 2 if m_t > 0 else 0.0

    ref = next((r for r in runs if r['stamp'] == args.ref), runs[0])
    JB = 0.0142  # 空机 Jxx(x500),NMPC 侧模型惯量 = JB + dJ,与 MHE 的 Jxx 可比

    fig, axes = plt.subplots(3, 2, figsize=(13.0, 9.8), sharex=True)
    cmap = plt.get_cmap('tab10')

    # ---------- 左列:代表轮的绝对量(est vs 该轮自己的 attach 真值) ----------
    t0 = ref['t0']
    tr = ref['truth']
    t = tr[:, 0] - t0
    ax = axes[0, 0]
    if len(ref['hf']):
        ax.plot(ref['hf'][:, 0] - t0, ref['hf'][:, 1], color='C0', alpha=0.35,
                lw=0.9, label='10 Hz')
    ax.plot(t, tr[:, 1], 'C0-', lw=1.6, label=r'$\hat{m}$ (MHE online)')
    ax.plot(t, tr[:, 2], 'k--', lw=1.8, label='truth')
    ax.set_ylabel('mass  [kg]')
    ax.set_title(f"(a) mass — run {ref['stamp'][-6:]}", fontsize=10, loc='left')

    ax = axes[1, 0]
    ax.plot(t, tr[:, 8], 'C1-', lw=1.6, label=r'$J_{xx}$ from $\hat{m}$ (MHE model)')
    ax.plot(t, tr[:, 9], 'k--', lw=1.8, label='truth')
    if ref['dj']:
        dj = np.array([[row[0] - t0, JB + row[7]] for row in ref['dj']])
        ax.plot(dj[:, 0], dj[:, 1], 'C3:', lw=1.8, drawstyle='steps-post',
                label=r'$J_B+\Delta J$ (NMPC model, nominal arm)')
        mp_t = np.interp(dj[:, 0], t, tr[:, 2]) - args.mB
        ax.plot(dj[:, 0], JB + np.array([dJ_nmpc(max(v, 0.0)) for v in mp_t]),
                color='0.35', ls='-.', lw=1.3,
                label='truth (NMPC arm convention)')
    ax.set_ylabel(r'$J_{xx}$  [kg m$^2$]')
    ax.set_title(f"(b) inertia — run {ref['stamp'][-6:]}", fontsize=10,
                 loc='left')

    ax = axes[2, 0]
    cx = ref['cxy']
    if len(cx):
        tc = cx[:, 0] - t0
        ax.plot(tc, cx[:, 2] * 1e3, 'C2-', lw=1.6, label=r'$\hat{c}_y$ online')
        ax.plot(tc, cx[:, 4] * 1e3, 'k--', lw=1.8, label=r'truth $c_y$')
        ax.plot(tc, cx[:, 1] * 1e3, 'C4-', lw=1.0, alpha=0.7,
                label=r'$\hat{c}_x$ online')
        ax.plot(tc, cx[:, 3] * 1e3, color='0.45', ls=':', lw=1.4,
                label=r'truth $c_x$')
    ax.set_ylabel('CoM offset  [mm]')
    ax.set_title(f"(c) eccentricity — run {ref['stamp'][-6:]}", fontsize=10,
                 loc='left')
    ax.set_xlabel('time since attach  [s]')

    # ---------- 右列:全部轮的相对误差(每轮对自己的真值) ----------
    for i, r in enumerate(runs):
        col = cmap(i % 10)
        tr, t0 = r['truth'], r['t0']
        t = tr[:, 0] - t0
        sel = t > -1e-6
        lab = r['stamp'][-6:]
        axes[0, 1].plot(t[sel], 100 * (tr[sel, 1] - tr[sel, 2]) / tr[sel, 2],
                        color=col, lw=1.1, label=lab)
        axes[1, 1].plot(t[sel], 100 * (tr[sel, 8] - tr[sel, 9]) / tr[sel, 9],
                        color=col, lw=1.1)
        if r['dj']:
            dj = np.array([[row[0] - t0, row[7]] for row in r['dj']])
            mp_t = np.interp(dj[:, 0], t, tr[:, 2]) - args.mB
            djt = np.array([dJ_nmpc(max(v, 0.0)) for v in mp_t])
            ok = (dj[:, 0] > 0) & (djt > 1e-6)
            axes[1, 1].plot(dj[ok, 0], 100 * (dj[ok, 1] - djt[ok]) / djt[ok],
                            color=col, ls=':', lw=1.8, drawstyle='steps-post')
        cx = r['cxy']
        if len(cx):
            tc = cx[:, 0] - t0
            s2 = (tc > -1e-6) & ~np.isnan(cx[:, 4])
            axes[2, 1].plot(tc[s2], 1e3 * (cx[s2, 2] - cx[s2, 4]),
                            color=col, lw=1.1)

    for k, (ax, ttl) in enumerate(zip(
            axes[:, 1],
            [f'(d) mass error, n={len(runs)}',
             '(e) inertia error, n=%d  (dotted = NMPC dJ, own arm convention)' % len(runs),
             f'(f) $c_y$ error (absolute), n={len(runs)}'])):
        ax.axhline(0, color='k', lw=1.4, ls='--')
        if k < 2:
            ax.axhspan(-5, 5, color='0.6', alpha=0.18)
            ax.set_ylabel('relative error  [%]')
        else:
            ax.axhspan(-1, 1, color='0.6', alpha=0.18)
            ax.set_ylabel('absolute error  [mm]')
        ax.set_title(ttl, fontsize=10, loc='left')
    axes[2, 1].set_xlabel('time since attach  [s]')
    axes[0, 1].legend(fontsize=7, ncol=3, loc='lower right')

    for ax in axes.ravel():
        ax.grid(alpha=0.3)
        ax.axvline(0, color='r', lw=1.0, alpha=0.5)
        for r in runs:
            if r['t_drop'] is not None:
                ax.axvline(r['t_drop'] - r['t0'], color='m', lw=1.2, ls='-',
                           alpha=0.6)
    if ref['t_drop'] is not None:
        axes[0, 0].text(ref['t_drop'] - ref['t0'], 0.02, ' drop', color='m',
                        fontsize=8, transform=axes[0, 0].get_xaxis_transform())
    for ax in axes[:, 0]:
        ax.legend(fontsize=7, loc='best')
    axes[0, 0].text(0.5, 0.02, 'attach', color='r', fontsize=8,
                    transform=axes[0, 0].get_xaxis_transform())
    if args.tmax:
        axes[2, 0].set_xlim(-8, args.tmax)
    if args.title:
        fig.suptitle(args.title, fontsize=12)
        fig.tight_layout(rect=(0, 0, 1, 0.97))
    else:
        fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print('saved', args.out)

    # 数值汇总:稳态偏差 + 收敛时间(自该时刻起全部样本落在 ±band 内)
    def t_enter(t, err, band):
        ok = np.abs(err) <= band
        idx = None
        for i in range(len(ok)):
            if ok[i:].all():
                idx = i
                break
        return float(t[idx]) if idx is not None else float('nan')

    print('\n              m                    Jxx(MHE)          c_y(online)')
    print('run      bias%  t5%   t10%     bias%   t20%        err[mm]  bias%  t5%')
    for r in runs:
        tr, t0 = r['truth'], r['t0']
        t = tr[:, 0] - t0
        td = (r['t_drop'] - t0) if r['t_drop'] is not None else np.inf
        sel = (t > 0) & (t < td)      # 带载段(drop 后真值换成空机,不能混算)
        em = 100 * (tr[sel, 1] - tr[sel, 2]) / tr[sel, 2]
        ej = 100 * (tr[sel, 8] - tr[sel, 9]) / tr[sel, 9]
        ss = t[sel] > 20
        line = (f"{r['stamp'][-6:]}  {em[ss].mean():+6.2f} "
                f"{t_enter(t[sel], em, 5):5.1f} {t_enter(t[sel], em, 10):5.1f}   "
                f"{ej[ss].mean():+7.2f} {t_enter(t[sel], ej, 20):6.1f}")
        cx = r['cxy']
        if len(cx):
            tc = cx[:, 0] - t0
            s2 = (tc > 0) & ~np.isnan(cx[:, 4])
            if r['t_drop'] is not None:
                s2 &= tc < (r['t_drop'] - t0)   # 汇总只统计带载段
            ec = 100 * (cx[s2, 2] - cx[s2, 4]) / np.where(
                np.abs(cx[s2, 4]) > 1e-4, cx[s2, 4], np.nan)
            ss2 = tc[s2] > 20
            line += (f"      {1e3*np.mean(cx[s2][ss2, 2]-cx[s2][ss2, 4]):+7.3f} "
                     f"{ec[ss2].mean():+6.2f} {t_enter(tc[s2], ec, 5):5.1f}")
        print(line)
        if td < np.inf:
            post = t > td + 5.0
            if post.sum() >= 3:
                ep = 100 * (tr[post, 1] - tr[post, 2]) / tr[post, 2]
                print(f"         post-drop (t>{td+5:.0f}s, n={post.sum()}): "
                      f"m bias={ep.mean():+.2f}%  min m_hat={tr[post,1].min():.3f} "
                      f"(m_min floor=1.961)  Jxx err="
                      f"{100*np.mean((tr[post,8]-tr[post,9])/tr[post,9]):+.2f}%")
                cxp = r['cxy']
                if len(cxp):
                    tcp = cxp[:, 0] - t0
                    sp = tcp > td + 5.0
                    if sp.any():
                        print("         post-drop c_xy_est (truth=0): "
                              f"cx={1e3*cxp[sp,1].mean():+.2f}mm "
                              f"cy={1e3*cxp[sp,2].mean():+.2f}mm  "
                              "<- MHE 侧未归零(NMPC 侧 grip_dropped 已置零)")
        if r['dj']:
            dj = np.array([[row[0] - t0, row[7]] for row in r['dj']])
            mp_t = np.interp(dj[:, 0], t, tr[:, 2]) - args.mB
            djt = np.array([dJ_nmpc(max(v, 0.0)) for v in mp_t])
            ok = (dj[:, 0] > 20) & (djt > 1e-6) & (dj[:, 0] < td)
            if ok.any():
                # ⚠️ 用最后一个**带载**样本,不能用 r['dj'][-1] —— 那是 drop 时
                # 补进去的归零点(见 parse 里的注释)。
                _i = np.where(ok)[0][-1]
                print(f"         NMPC dJ (own arm convention): "
                      f"bias={100*np.mean((dj[ok,1]-djt[ok])/djt[ok]):+.2f}%"
                      f"  last loaded dJ={dj[_i,1]:.4f} vs truth {djt[_i]:.4f}")


if __name__ == '__main__':
    main()
