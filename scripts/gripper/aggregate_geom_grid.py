#!/usr/bin/env python3
"""B.3 主实验网格聚合(2026-07-14)。读 run_geom_grid.sh 的 manifest,按
(mass,ecc,mode) 分组,用 parse_dropwindow_logs 解析每轮 nmpc [attach-window] 逐帧段
的控制层暂态 pos_err 峰/恢复,输出均值±标准差,truth vs online 逐格并列——证在线
估计版≈真值版、消 attach 真值依赖不掉性能。

用法:  python3 aggregate_geom_grid.py [manifest]
不给则取 nmpc_test_results/ 下最新 geom_grid_*.txt(排除 launch)。"""
import glob
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, '/home/clear/ros2_ws_HJH/src/scripts/masschanger')
import parse_dropwindow_logs as ctl  # noqa: E402

D = '/home/clear/ros2_ws_HJH/nmpc_test_results/'
MODE_ORDER = ['truth', 'online']
MODE_LABEL = {'truth': '真值几何', 'online': '在线 c_xy 估计'}


def fmt(vals, prec=3):
    v = np.array(vals, dtype=float)
    v = v[np.isfinite(v)]
    if len(v) == 0:
        return 'n/a'
    return f'{np.mean(v):.{prec}f}±{np.std(v):.{prec}f}(n={len(v)})'


def main():
    if len(sys.argv) > 1:
        manifest = sys.argv[1]
    else:
        cands = sorted(glob.glob(D + 'geom_grid_*.txt'), key=os.path.getmtime)
        cands = [c for c in cands if 'launch' not in os.path.basename(c)]
        if not cands:
            print('没找到 geom_grid_*.txt,先跑 run_geom_grid.sh'); return
        manifest = cands[-1]
    print(f'manifest: {manifest}')

    # (mass,ecc,mode) -> [(nstamp,status)]
    cells = defaultdict(list)
    for line in open(manifest):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) < 6:
            continue
        mass, ecc, mode, rep, nstamp, status = parts[:6]
        cells[(mass, ecc, mode)].append((nstamp, status))

    keys = sorted({(m, e) for (m, e, _) in cells})
    for mass, ecc in keys:
        print(f'\n===== 质量 {mass}kg × 偏心 {ecc}m =====')
        for mode in MODE_ORDER:
            runs = cells.get((mass, ecc, mode), [])
            pk, rec = [], []
            ndiv = 0; bad = []
            for nstamp, status in runs:
                if status.startswith('DIVERGED'):
                    ndiv += 1
                if status != 'ok':
                    bad.append(f'{nstamp}:{status}'); continue
                try:
                    c = ctl.parse(D + f'grip_nmpc_{nstamp}.log')
                    pk.append(c['pe_peak']); rec.append(c['t_rec'])
                except (RuntimeError, FileNotFoundError):
                    bad.append(f'{nstamp}:parse_fail')
            print(f'  --- {MODE_LABEL.get(mode, mode)} ---'
                  + (f'  剔除{bad}' if bad else ''))
            print(f'      pos_err峰[m]: {fmt(pk)}   恢复<5cm[s]: {fmt(rec,2)}'
                  f'   发散: {ndiv}/{len(runs)}')


if __name__ == '__main__':
    main()
