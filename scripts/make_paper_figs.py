#!/usr/bin/env python3
"""论文图 3/4/5(+6)生成脚本 —— 从现有实验日志画正式版。

图3: 事件触发 vs 固定权重 m_est 收敛时间线(无信号消融批 n=6,两幅度)
图4: c_xy 在线估计收敛 vs 真值 + 偏心扫格 est-vs-truth
图5: B.5 抓取→figure8→投放 全流程时间轴多联图
图6: 分轴速度/位置(需要 plot_logger 落盘的 npz,见 --npz)

数据源(nmpc_test_results/):
  图3  mhe_node_<stamp>.log,批次清单 nosignal_ablation_20260713_182612.txt
  图4  grip_mhe_20260714_013435.log + cxy_ecc_sweep_20260714_143710.txt
  图5  gviz_nmpc_20260715_204725.log + gviz_mhe_20260715_204725.log
  图6  acados_plot_data_*.npz(plot_logger_acados 2026-07-16 起落盘)

输出: nmpc_test_results/paper_figs/fig{3,4,5,6}.{png,pdf}
用法: python3 make_paper_figs.py [--npz PATH]
"""
import argparse
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RES = os.path.expanduser('~/ros2_ws_HJH/nmpc_test_results')
OUT = os.path.join(RES, 'paper_figs')

# dataviz 参考色板(已验证),固定槽位顺序:蓝/绿/品红;文本/网格用中性墨
C1, C2, C3 = '#2a78d6', '#008300', '#e87ba4'
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e6e5e2'

plt.rcParams.update({
    'font.size': 8.5, 'axes.labelsize': 8.5, 'axes.titlesize': 9,
    'legend.fontsize': 7.5, 'xtick.labelsize': 7.5, 'ytick.labelsize': 7.5,
    'axes.edgecolor': INK2, 'axes.linewidth': 0.7,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
    'axes.spines.top': False, 'axes.spines.right': False,
    'text.color': INK, 'axes.labelcolor': INK,
    'xtick.color': INK2, 'ytick.color': INK2,
    'legend.frameon': False, 'figure.dpi': 110,
})

WALL = r'\[(\d+)\.(\d+)\]'


def _wall(m, i=1):
    return float(m.group(i)) + float(m.group(i + 1)) * 1e-9


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(OUT, f'{name}.{ext}'), dpi=300,
                    bbox_inches='tight')
    plt.close(fig)
    print(f'saved {OUT}/{name}.png/.pdf')


# =====================================================================
# 图3 事件触发 vs 固定权重:m_est 时间线(t=0 为物理生效时刻)
# =====================================================================
EV_LINE = re.compile(WALL + r'.*\[(?:transition|post-event)\] '
                     r'm_est=([\d.]+) kg \(T_phys=([\d.]+)N\)')
THRESH = 1.5


def parse_mhe_window(path):
    """返回 (t_rel, m):t 相对物理生效时刻(T_phys 偏基线>1.5N 首帧)。"""
    t, m, T = [], [], []
    for line in open(path):
        ml = EV_LINE.search(line)
        if ml:
            t.append(_wall(ml)); m.append(float(ml.group(3)))
            T.append(float(ml.group(4)))
    t, m, T = np.array(t), np.array(m), np.array(T)
    if len(t) < 10:
        raise RuntimeError(f'{path}: 事件段数据不足')
    base = np.mean(T[:3])
    dev = np.abs(T - base) > THRESH
    if not np.any(dev):
        raise RuntimeError(f'{path}: T_phys 未偏离基线')
    return t - t[dev][0], m


def fig3():
    manifest = os.path.join(RES, 'nosignal_ablation_20260713_182612.txt')
    runs = {}   # (cfg, delta) -> [stamps]
    for line in open(manifest):
        if line.startswith('#'):
            continue
        cfg, delta, rep, mhe_stamp = line.split()[:4]
        if line.split()[-1] != 'ok':
            continue
        runs.setdefault((cfg, float(delta)), []).append(mhe_stamp)

    cfgs = [('fixed', C1, 'fixed weights'),
            ('signal', C2, 'event-triggered (signal)'),
            ('nosignal', C3, 'event-triggered (self-detected)')]
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.6), sharey=False)
    for ax, delta in zip(axes, (-0.3, -0.8)):
        m_true = 2.064 + delta
        ax.axhspan(m_true - 0.08, m_true + 0.08, color=GRID, alpha=0.6, lw=0)
        ax.axhline(m_true, color=INK2, lw=0.8, ls='--')
        for cfg, color, label in cfgs:
            first = True
            for stamp in runs.get((cfg, delta), []):
                path = os.path.join(RES, f'mhe_node_{stamp}.log')
                if not os.path.exists(path):
                    continue
                try:
                    t, m = parse_mhe_window(path)
                except RuntimeError as e:
                    print(f'  skip: {e}', file=sys.stderr)
                    continue
                sel = (t >= -0.6) & (t <= 4.5)
                ax.plot(t[sel], m[sel], color=color, lw=1.1, alpha=0.55,
                        label=label if first else None)
                first = False
        ax.set_xlabel('time since physical mass change [s]')
        ax.set_xlim(-0.6, 4.5)
        ax.set_title(f'$\\Delta m={delta}$ kg  (n=6 each)', fontsize=8.5)
    axes[0].set_ylabel('MHE mass estimate [kg]')
    axes[0].legend(loc='upper right', handlelength=1.4)
    axes[0].annotate('truth $\\pm$0.08 kg', xy=(0.02, 0.06),
                     xycoords='axes fraction', color=INK2, fontsize=7)
    save(fig, 'fig3_event_trigger_mest')


# =====================================================================
# 图4 c_xy 在线估计:收敛时间线 + 偏心扫格 est vs truth
# =====================================================================
CXY = re.compile(WALL + r'.*\[c_xy_est\] cx=([+-][\d.]+) cy=([+-][\d.]+) m'
                 r'(?: \| truth c=\[([+-][\d.]+),([+-][\d.]+)\]'
                 r'(?: \(m_p=([\d.]+)\))?)?')
ATTACH_EV = re.compile(WALL + r'.*mass event \[attach')
SWEEP_ROW = re.compile(r'^([\d.]+)\s+ok.*cx=([+-][\d.]+) cy=([+-][\d.]+) m \| '
                       r'truth c=\[([+-][\d.]+),([+-][\d.]+)\]'
                       r'(?:\s+\(m_p=([\d.]+)\))?')

# ⚠️2026-08-21:日志里那个 `truth c=` **只有几何因子是真值**。质量因子在
# mhe_node 没收到 grip_true_payload_mass(默认 0.0,且**故意不开**——它会拿真值
# 替换 m_p_hat 去喂载荷几何,等于把真值灌进模型)时,退回 m_est 反推的 m_p,
# 于是"真值"跟着 m_est 抖(实测 m_p=0.259~0.298,真值是 0.300)。
# 修法:c_true=(m_p/m_t)·d 里 d 是常量,反算 d 再用**已知真值** m_p=0.3 重算,
# 等价于给每个样本乘一个系数。收敛后差 1.8%、暂态段差约 14%。
M_DRY = 2.0643          # 空机质量 [kg](与 mhe_params.m_nominal 一致)
M_P_TRUE = 0.3          # SITL 载荷真值 [kg]


def _true_c(c_logged, m_p):
    """把日志里 m_p 估计值算出的 c 换算成真值 m_p 对应的 c。"""
    if not m_p or m_p <= 0.0:
        return c_logged
    k = (M_P_TRUE / (M_DRY + M_P_TRUE)) / (m_p / (M_DRY + m_p))
    return c_logged * k


def fig4():
    # (a) 0.3kg 单轮收敛时间线
    path = os.path.join(RES, 'grip_mhe_20260714_013435.log')
    t, cx, cy, tcx, tcy = [], [], [], [], []
    t_attach = None
    for line in open(path):
        ma = ATTACH_EV.search(line)
        if ma and t_attach is None:
            t_attach = _wall(ma)
        mc = CXY.search(line)
        if mc:
            t.append(_wall(mc)); cx.append(float(mc.group(3)))
            cy.append(float(mc.group(4)))
            if mc.group(5):
                _mp = float(mc.group(7)) if mc.group(7) else 0.0
                tcx.append(_true_c(float(mc.group(5)), _mp))
                tcy.append(_true_c(float(mc.group(6)), _mp))
    t = np.array(t) - (t_attach if t_attach else t[0])
    cx, cy = np.array(cx), np.array(cy)
    truth_cx = np.median(tcx) if tcx else None
    truth_cy = np.median(tcy) if tcy else None

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(7.0, 2.6),
                                  gridspec_kw={'width_ratios': [1.5, 1],
                                               'wspace': 0.35})
    sel = (t >= -5) & (t <= 60)
    ax.plot(t[sel], 1e2 * cy[sel], color=C1, lw=1.6, label='$\\hat c_y$ (online)')
    ax.plot(t[sel], 1e2 * cx[sel], color=C2, lw=1.6, label='$\\hat c_x$ (online)')
    if truth_cy is not None:
        ax.axhline(1e2 * truth_cy, color=C1, lw=1.0, ls='--', alpha=0.7,
                   label='$c_y$ truth')
        ax.axhline(1e2 * truth_cx, color=C2, lw=1.0, ls='--', alpha=0.7,
                   label='$c_x$ truth')
    ax.axvline(0, color=INK2, lw=0.8, ls=':')
    ax.annotate('attach', xy=(0.5, ax.get_ylim()[1]), xytext=(2, -8),
                textcoords='offset points', color=INK2, fontsize=7)
    ax.set_xlabel('time since attach [s]')
    ax.set_ylabel('CoM offset [cm]')
    ax.set_title('(a) online CoM estimate, 0.3 kg payload', fontsize=8.5)
    ax.legend(loc='center right', handlelength=1.6)

    # (b) 偏心扫格 est vs truth(主分量 cy)
    sweep = os.path.join(RES, 'cxy_ecc_sweep_20260714_143710.txt')
    ecc, est, tru = [], [], []
    for line in open(sweep):
        ms = SWEEP_ROW.match(line)
        if ms:
            ecc.append(float(ms.group(1)))
            est.append(float(ms.group(3)))
            tru.append(_true_c(float(ms.group(5)),
                               float(ms.group(6)) if ms.group(6) else 0.0))
    est, tru = 1e2 * np.array(est), 1e2 * np.array(tru)
    lim = [min(est.min(), tru.min()) - 0.3, 0.3]
    ax2.plot(lim, lim, color=INK2, lw=0.8, ls='--')
    ax2.scatter(tru, est, s=42, color=C1, zorder=3)
    ann_off = {0.05: (6, -3), 0.08: (5, 6), 0.10: (6, -3), 0.12: (5, -12)}
    for e, x, y in zip(ecc, tru, est):
        ax2.annotate(f'ecc {e:.2f}', xy=(x, y), xytext=ann_off.get(e, (4, 4)),
                     textcoords='offset points', fontsize=7, color=INK2)
    ax2.set_xlabel('truth $c_y$ [cm]')
    ax2.set_ylabel('estimated $\\hat c_y$ [cm]')
    ax2.set_title('(b) eccentricity sweep, 0.3 kg', fontsize=8.5)
    ax2.set_xlim(lim); ax2.set_ylim(lim)
    ax2.set_aspect('equal', adjustable='box')
    save(fig, 'fig4_cxy_online')


# =====================================================================
# 图5 B.5 全流程时间轴(抓取→提起→figure8→相位触发投放)
# =====================================================================
NMPC_PERIODIC = re.compile(WALL + r'.*t=([\d.]+)s \| pos_err=([\d.]+)m \| '
                           r'T=([\d.]+)N')
NMPC_WIN = re.compile(WALL + r'.*\[attach-window\] t=([\d.]+)s '
                      r'pos_err=([\d.]+)m T=([\d.]+)N z=([\d.-]+) '
                      r'm_est=([\d.]+)')
PHASE = re.compile(WALL + r'.*t=([\d.]+)s \| (ATTACH|LIFT|DYNAMIC|DROP)')
MASS_LN = re.compile(WALL + r'.*MHE mass estimate: ([\d.]+) kg '
                     r'\(T_phys=([\d.]+)N\)')
T_END = 115.0
# ⚠️ 论文里**唯一**不是 M0@1.5N 跑出来的数据源(2026-08-03 逐批核实):这一轮用的是
# θ* α版 [-4.80,-1.21,0.49,-0.96,0.99]、阈值 2.9N,录于 07-15、早于 CEM 结论撤回。
# 论文 §VI-B 已主动声明这个例外(定性演示、不承载定量比较,且 θ* 与 M0 已证统计
# 不可区分)。**演示视频(中英双版)基于同一 stamp**(make_overlay.py 默认值就取这里),
# 所以图与视频同源——要换成 M0 必须两者一起重做,只换这里会让论文图与补充视频的
# 曲线对不上。详见 paper/REPRODUCE.md 的"各节实际使用的权重调度"表。
STAMP5 = '20260715_204725'
PAYLOAD5 = 0.3


def fig5():
    nmpc = os.path.join(RES, f'gviz_nmpc_{STAMP5}.log')
    mhe = os.path.join(RES, f'gviz_mhe_{STAMP5}.log')

    tp, ep, Tp = [], [], []          # 周期行
    tw, ew, zw = [], [], []          # attach-window 逐帧
    phases, offsets = {}, []
    for line in open(nmpc):
        mw = NMPC_WIN.search(line)
        if mw:
            tw.append(float(mw.group(3))); ew.append(float(mw.group(4)))
            zw.append(float(mw.group(6)))
            offsets.append(_wall(mw) - float(mw.group(3)))
            continue
        mp_ = NMPC_PERIODIC.search(line)
        if mp_:
            tp.append(float(mp_.group(3))); ep.append(float(mp_.group(4)))
            Tp.append(float(mp_.group(5)))
            offsets.append(_wall(mp_) - float(mp_.group(3)))
        mf = PHASE.search(line)
        if mf and mf.group(4) not in phases:
            phases[mf.group(4)] = float(mf.group(3))
    off = float(np.median(offsets))   # wall → 任务时间 t 的对齐偏移

    tm, mm, tT, TT = [], [], [], []
    tc, cyv, cy_tr = [], [], []
    for line in open(mhe):
        ml = MASS_LN.search(line)
        if ml:
            tm.append(_wall(ml) - off); mm.append(float(ml.group(3)))
            tT.append(_wall(ml) - off); TT.append(float(ml.group(4)))
            continue
        me = EV_LINE.search(line)
        if me:
            tm.append(_wall(me) - off); mm.append(float(me.group(3)))
            tT.append(_wall(me) - off); TT.append(float(me.group(4)))
            continue
        mc = CXY.search(line)
        if mc:
            tc.append(_wall(mc) - off); cyv.append(float(mc.group(4)))
            cy_tr.append(float(mc.group(6)) if mc.group(6) else np.nan)

    def _clip(t, *ys):
        t = np.array(t); keep = (t >= 0) & (t <= T_END)
        return (t[keep],) + tuple(np.array(y)[keep] for y in ys)

    tp, ep, Tp = _clip(tp, ep, Tp)
    tw, ew, zw = _clip(tw, ew, zw)
    tm, mm = _clip(tm, mm)
    tT, TT = _clip(tT, TT)
    tc, cyv, cy_tr = _clip(tc, cyv, cy_tr)

    # 周期行与逐帧窗口合并成单调时间线
    t_err = np.concatenate([tp, tw]); i = np.argsort(t_err)
    t_err, v_err = t_err[i], np.concatenate([ep, ew])[i]
    im = np.argsort(tm); tm, mm = tm[im], np.array(mm)[im]
    iT = np.argsort(tT); tT, TT = tT[iT], np.array(TT)[iT]

    fig, axes = plt.subplots(4, 1, figsize=(7.0, 6.4), sharex=True)
    labels = {'ATTACH': 'attach', 'LIFT': 'lift',
              'DYNAMIC': 'figure-8 start', 'DROP': 'drop'}
    for ax in axes:
        for k, tv in phases.items():
            ax.axvline(tv, color=INK2, lw=0.7, ls=':', alpha=0.8)
    stagger = {'ATTACH': -1, 'LIFT': -10, 'DYNAMIC': -1, 'DROP': -1}
    for k, tv in phases.items():
        axes[0].annotate(labels[k], xy=(tv, 1.0), xycoords=('data', 'axes fraction'),
                         xytext=(3, stagger[k]), textcoords='offset points',
                         fontsize=7, color=INK2, rotation=0, va='top')

    axes[0].plot(t_err, v_err, color=C1, lw=1.4)
    axes[0].set_ylabel('position error [m]')

    axes[1].plot(tT, TT, color=C1, lw=1.4)
    axes[1].set_ylabel('$T_{phys}$ [N]')

    m_truth_t = [0, phases.get('ATTACH', 7), phases.get('ATTACH', 7),
                 phases.get('DROP', 86), phases.get('DROP', 86), T_END]
    m_truth_v = [2.064, 2.064, 2.064 + PAYLOAD5, 2.064 + PAYLOAD5,
                 2.064, 2.064]
    axes[2].plot(m_truth_t, m_truth_v, color=INK2, lw=1.0, ls='--',
                 label='truth')
    axes[2].plot(tm, mm, color=C1, lw=1.4, label='MHE estimate')
    axes[2].set_ylabel('mass [kg]')
    axes[2].legend(loc='center right', handlelength=1.6)

    # c_xy 只在挂载期间有定义(drop 后 NMPC 端几何归零,发布端冻结最后
    # EMA 属 cosmetic 残留,不画以免误导)
    keep_c = np.array(tc) <= phases.get('DROP', T_END)
    tc, cyv, cy_tr = np.array(tc)[keep_c], np.array(cyv)[keep_c], \
        np.array(cy_tr)[keep_c]
    axes[3].plot(tc, 1e2 * np.array(cy_tr), color=INK2, lw=1.0, ls='--',
                 label='truth')
    axes[3].plot(tc, 1e2 * np.array(cyv), color=C1, lw=1.4,
                 label='online estimate')
    axes[3].set_ylabel('CoM offset $c_y$ [cm]')
    axes[3].set_xlabel('mission time [s]')
    axes[3].legend(loc='center right', handlelength=1.6)
    axes[3].set_xlim(0, T_END)
    save(fig, 'fig5_full_mission')


# =====================================================================
# 图6 分轴速度/位置(plot_logger 落盘 npz)
# =====================================================================
LOGGER_START = re.compile(WALL + r'.*plot_logger_acados started')


def fig6(npz_path, nmpc_log=None, logger_log=None):
    d = np.load(npz_path)
    vel, act = d['vel_txyz'], d['actual_txyz']
    if len(vel) == 0:
        raise RuntimeError('npz 里没有速度数据')

    # 时间对齐:npz 的 t 基准是 logger 启动时刻(wall);nmpc 日志给出
    # wall→任务时间 t 的偏移。两者都有才能标任务阶段。
    t0_wall, off, phases = None, None, {}
    if logger_log and os.path.exists(logger_log):
        for line in open(logger_log):
            ms = LOGGER_START.search(line)
            if ms:
                t0_wall = _wall(ms)
                break
    if nmpc_log and os.path.exists(nmpc_log):
        offs = []
        for line in open(nmpc_log):
            mp_ = NMPC_PERIODIC.search(line)
            if mp_:
                offs.append(_wall(mp_) - float(mp_.group(3)))
            mf = PHASE.search(line)
            if mf and mf.group(4) not in phases:
                phases[mf.group(4)] = float(mf.group(3))
        if offs:
            off = float(np.median(offs))

    if t0_wall is not None and off is not None:
        shift = t0_wall - off      # npz 相对时间 → 任务时间
        xlabel = 'mission time [s]'
    else:
        shift = 0.0
        xlabel = 'time since logger start [s]'
    tv = vel[:, 0] + shift
    ta = act[:, 0] + shift
    keep_v = (tv >= 0) & (tv <= T_END)
    keep_a = (ta >= 0) & (ta <= T_END)

    fig, axes = plt.subplots(2, 1, figsize=(7.0, 4.2), sharex=True)
    labels = {'ATTACH': 'attach', 'LIFT': 'lift',
              'DYNAMIC': 'figure-8 start', 'DROP': 'drop'}
    stagger = {'ATTACH': -1, 'LIFT': -10, 'DYNAMIC': -1, 'DROP': -1}
    for ax in axes:
        for k, tp_ in phases.items():
            ax.axvline(tp_, color=INK2, lw=0.7, ls=':', alpha=0.8)
    for k, tp_ in phases.items():
        axes[0].annotate(labels[k], xy=(tp_, 1.0),
                         xycoords=('data', 'axes fraction'),
                         xytext=(3, stagger[k]), textcoords='offset points',
                         fontsize=7, color=INK2, va='top')

    for i, (c, n) in enumerate(zip((C1, C2, C3), ('$v_x$', '$v_y$', '$v_z$'))):
        axes[0].plot(tv[keep_v], vel[keep_v, i + 1], color=c, lw=1.1, label=n)
    axes[0].set_ylabel('velocity (ENU) [m/s]')
    axes[0].legend(loc='upper right', ncol=3, handlelength=1.4)
    for i, (c, n) in enumerate(zip((C1, C2, C3), ('$x$', '$y$', '$z$'))):
        axes[1].plot(ta[keep_a], act[keep_a, i + 1], color=c, lw=1.1, label=n)
    axes[1].set_ylabel('position (ENU) [m]')
    axes[1].set_xlabel(xlabel)
    axes[1].legend(loc='center right', ncol=3, handlelength=1.4)
    axes[1].set_xlim(0, T_END)
    save(fig, 'fig6_axis_velocity')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--npz', help='plot_logger 落盘的 npz(画图6)')
    ap.add_argument('--nmpc-log', help='同一轮的 grip_nmpc 日志(图6任务阶段标注)')
    ap.add_argument('--logger-log', help='同一轮的 plot_logger 日志(图6时间对齐)')
    ap.add_argument('--only', choices=['3', '4', '5', '6'])
    args = ap.parse_args()
    if args.only in (None, '3'):
        fig3()
    if args.only in (None, '4'):
        fig4()
    if args.only in (None, '5'):
        fig5()
    if args.npz and args.only in (None, '6'):
        fig6(args.npz, args.nmpc_log, args.logger_log)
