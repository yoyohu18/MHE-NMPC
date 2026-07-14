#!/usr/bin/env python3
"""B.3 Phase2 online-vs-truth A/B 聚合(2026-07-14)。读 run_geom_ab.sh 的 manifest,
按 mode(truth/online)分组,用 parse_dropwindow_logs 解析每轮 nmpc [attach-window]
逐帧段的控制层暂态:pos_err 峰值、恢复时间。输出均值±标准差,证在线估计版≈真值版。

用法:  python3 aggregate_geom_ab.py [manifest]
不给则取 nmpc_test_results/ 下最新 geom_ab_*.txt。"""
import glob
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, '/home/clear/ros2_ws_HJH/src/scripts/masschanger')
import parse_dropwindow_logs as ctl  # noqa: E402

D = '/home/clear/ros2_ws_HJH/nmpc_test_results/'
MODE_ORDER = ['truth', 'online']
MODE_LABEL = {'truth': '真值几何(attach_offset)',
              'online': '在线 c_xy 估计(强闭环)'}


def fmt(vals, prec=3):
    v = np.array(vals, dtype=float)
    v = v[np.isfinite(v)]
    if len(v) == 0:
        return 'n/a'
    return f'{np.mean(v):.{prec}f}±{np.std(v):.{prec}f} (n={len(v)})'


def main():
    if len(sys.argv) > 1:
        manifest = sys.argv[1]
    else:
        cands = sorted(glob.glob(D + 'geom_ab_*.txt'), key=os.path.getmtime)
        cands = [c for c in cands if 'launch' not in os.path.basename(c)]
        if not cands:
            print('没找到 geom_ab_*.txt,先跑 run_geom_ab.sh'); return
        manifest = cands[-1]
    print(f'manifest: {manifest}')

    groups = defaultdict(list)  # mode -> [nmpc_stamp,...],只收 ok
    for line in open(manifest):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        mode, rep, nstamp, status = parts[:4]
        groups[mode].append((nstamp, status))

    for mode in MODE_ORDER:
        runs = groups.get(mode, [])
        pk, rec = [], []
        bad = []
        for nstamp, status in runs:
            if status != 'ok':
                bad.append(f'{nstamp}:{status}')
                continue
            try:
                c = ctl.parse(D + f'grip_nmpc_{nstamp}.log')
                pk.append(c['pe_peak'])
                rec.append(c['t_rec'])
            except (RuntimeError, FileNotFoundError):
                bad.append(f'{nstamp}:parse_fail')
        print(f'\n===== {MODE_LABEL.get(mode, mode)} =====')
        if bad:
            print(f'  剔除: {bad}')
        print(f'  pos_err 峰 [m]   : {fmt(pk)}')
        print(f'  恢复<5cm [s]     : {fmt(rec, 2)}')
        n_div = sum(1 for _, s in runs if s.startswith('DIVERGED'))
        print(f'  发散轮           : {n_div}/{len(runs)}')


if __name__ == '__main__':
    main()
