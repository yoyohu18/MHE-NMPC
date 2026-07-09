#!/usr/bin/env python3
"""解析 nmpc 日志的 [drop-window]/[attach-window] 逐帧记录(10Hz),对齐物理
生效时刻后对比控制层暂态:pos_err 峰值、z 偏移峰值、恢复时间。配 mhe 日志判据
同源(T 偏离基线 >THRESH)。两种 tag 字段格式完全一致(wrench=drop,
gripper=attach),同一份正则解析。用法: parse_dropwindow_logs.py 标签1=nmpc日志1 标签2=... """
import re
import sys

import numpy as np

LINE = re.compile(
    r'\[(?:drop|attach)-window\] t=([\d.]+)s pos_err=([\d.]+)m '
    r'T=([\d.]+)N z=(-?[\d.]+) m_est=([\d.]+)')
THRESH = 1.5
RECOVER = 0.05  # 恢复判据:pos_err 回到 5cm 以内并保持


def parse(path):
    rows = []
    for line in open(path):
        m = LINE.search(line)
        if m:
            rows.append([float(g) for g in m.groups()])
    if not rows:
        raise RuntimeError(f'{path}: 没有 [drop-window] 记录')
    a = np.array(rows)
    t, pe, T, z, mest = a.T
    base = np.mean(T[:3])
    dev = np.abs(T - base) > THRESH
    if not np.any(dev):
        raise RuntimeError(f'{path}: T 未偏离基线,drop 没生效?')
    t_phys = t[dev][0]
    after = t >= t_phys
    pe_peak = float(np.max(pe[after]))
    z_exc = float(np.max(np.abs(z[after] - z[0])))
    # 恢复时间:物理生效后最后一次 pos_err > RECOVER
    bad = after & (pe > RECOVER)
    t_rec = float(t[bad][-1] + 0.1 - t_phys) if np.any(bad) else 0.0
    return dict(t=t - t_phys, pe=pe, z=z - z[0], mest=mest,
                pe_peak=pe_peak, z_exc=z_exc, t_rec=t_rec)


if __name__ == '__main__':
    runs = []
    for a in sys.argv[1:]:
        lab, path = a.split('=', 1)
        runs.append((lab, parse(path)))
    for lab, r in runs:
        print(f'--- {lab} ---')
        print(f'  pos_err 峰值: {r["pe_peak"]:.3f}m   z 偏移峰值: {r["z_exc"]:.3f}m'
              f'   恢复(<{RECOVER}m): {r["t_rec"]:.1f}s')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        colors = ['tab:blue', 'tab:orange', 'tab:green']
        fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        for (lab, r), c in zip(runs, colors):
            axes[0].plot(r['t'], r['pe'], color=c,
                         label=f'{lab} (peak {r["pe_peak"]:.3f}m)')
            axes[1].plot(r['t'], r['mest'], color=c, label=lab)
        axes[0].set_ylabel('pos_err [m]'); axes[0].legend(); axes[0].grid(True)
        axes[1].set_ylabel('m_est [kg]'); axes[1].set_xlabel(
            't since physical mass change [s]')
        axes[1].legend(); axes[1].grid(True)
        axes[0].set_title('SITL drop transient: control-level comparison (10Hz)')
        out = ('/home/clear/ros2_ws_HJH/nmpc_test_results/'
               'mhe_dropwindow_control_compare.png')
        fig.savefig(out, dpi=120, bbox_inches='tight')
        print(f'saved plot: {out}')
    except ImportError:
        pass
