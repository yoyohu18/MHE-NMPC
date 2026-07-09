#!/usr/bin/env python3
"""解析 M0 SITL 对比:从两份 mhe_node 日志提取事件段 m_est 时间线,
以同一判据(T_phys 偏离事件前基线 >1.5N 的第一帧)定义"物理生效时刻",
计算各自的收敛时间(最后一次出带 → 物理生效的间隔)。"""
import os
import re
import sys

import numpy as np

# 环境变量覆盖:多幅度实验时 M_TRUE=2.064+delta、BAND 建议 0.15*|delta|
M_TRUE = float(os.environ.get('M_TRUE', 1.564))
BAND = float(os.environ.get('BAND', 0.08))
THRESH = 1.5

LINE = re.compile(
    r'\[(\d+)\.(\d+)\].*\[(transition|post-event)\] '
    r'm_est=([\d.]+) kg \(T_phys=([\d.]+)N\)')
EVENT = re.compile(r'\[(\d+)\.(\d+)\].*mass event \[')


def parse(path):
    t, m, T = [], [], []
    t_event = None
    for line in open(path):
        me = EVENT.search(line)
        if me and t_event is None:
            t_event = float(me.group(1)) + float(me.group(2)) * 1e-9
        ml = LINE.search(line)
        if ml:
            t.append(float(ml.group(1)) + float(ml.group(2)) * 1e-9)
            m.append(float(ml.group(4)))
            T.append(float(ml.group(5)))
    t, m, T = np.array(t), np.array(m), np.array(T)
    if t_event is None or len(t) == 0:
        raise RuntimeError(f'{path}: 没找到事件段数据')
    # 物理生效时刻:事件前基线取事件后前 3 帧均值(此时物理必未变,gz CLI 延迟>0.3s)
    base = np.mean(T[:3])
    dev = np.abs(T - base) > THRESH
    if not np.any(dev):
        raise RuntimeError(f'{path}: T_phys 从未偏离基线,drop 可能没生效')
    t_phys = t[dev][0]
    # 收敛:物理生效后最后一次出带
    after = t >= t_phys
    outside = after & (np.abs(m - M_TRUE) > BAND)
    t_settle = (t[outside][-1] - t_phys + 0.1) if np.any(outside) else 0.0
    # 稳态误差:进带后的尾段均值
    tail = t >= (t_phys + t_settle + 0.3)
    m_ss = float(np.mean(m[tail])) if np.any(tail) else float('nan')
    return dict(t_event=t_event, t_phys=t_phys, lag=t_phys - t_event,
                t_settle=t_settle, m_ss=m_ss,
                m_min=float(np.min(m)), n=len(t),
                t=t - t_phys, m=m, T=T)


if __name__ == '__main__':
    # 用法: parse_m0_logs.py 标签1=日志1 标签2=日志2 [标签3=日志3 ...]
    # (兼容旧的两参数无标签形式)
    runs = []
    for i, a in enumerate(sys.argv[1:]):
        if '=' in a:
            lab, path = a.split('=', 1)
        else:
            lab = ['event-triggered', 'fixed'][i] if i < 2 else f'run{i}'
            path = a
        runs.append((lab, parse(path)))

    for name, r in runs:
        inband = (r['t'] >= 0) & (np.abs(r['m'] - M_TRUE) <= BAND)
        t_enter = r['t'][inband][0] if np.any(inband) else float('nan')
        print(f'--- {name} ---')
        print(f'  事件→物理生效延迟: {r["lag"]:.2f}s')
        print(f'  首次入带: {t_enter:.2f}s  驻留收敛: {r["t_settle"]:.2f}s '
              f'(±{BAND}kg)')
        print(f'  过冲最低点: {r["m_min"]:.3f}kg  稳态: {r["m_ss"]:.3f}kg '
              f'(真值 {M_TRUE})')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:purple']
        fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        for (lab, r), c in zip(runs, colors):
            lab = f'{lab} (settle {r["t_settle"]:.2f}s)'
            axes[0].plot(r['t'], r['m'], '-', color=c, label=lab)
            axes[1].plot(r['t'], r['T'], '-', color=c, label=lab)
        axes[0].axhline(M_TRUE, color='r', ls='--', lw=1,
                        label=f'true mass {M_TRUE}')
        axes[0].axhspan(M_TRUE - BAND, M_TRUE + BAND, color='r', alpha=0.08)
        axes[0].set_ylabel('m_est [kg]'); axes[0].legend(); axes[0].grid(True)
        axes[1].set_ylabel('T_phys [N]'); axes[1].set_xlabel(
            't since physical mass change [s]')
        axes[1].legend(); axes[1].grid(True)
        axes[0].set_title('SITL drop: MHE mass convergence comparison')
        out = '/home/clear/ros2_ws_HJH/nmpc_test_results/mhe_event_trigger_sitl_compare.png'
        fig.savefig(out, dpi=120, bbox_inches='tight')
        print(f'saved plot: {out}')
    except ImportError:
        pass
