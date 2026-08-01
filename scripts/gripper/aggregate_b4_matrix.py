#!/usr/bin/env python3
"""B.4 学习+强闭环合流矩阵聚合(2026-07-15)。读 run_b4_matrix.sh 的 manifest,按
(method,mass,ecc) 分组,估计层(mhe 日志:入带/驻留/过冲)+ 控制层(nmpc
[attach-window]:pos_err 峰/恢复)用 per-mass M_TRUE/BAND/THRESH 口径解析,M0 vs θ* 逐格并列——问"θ* 在 online 无真值配置下对 M0
的收益是否保持"。

用法:  python3 aggregate_b4_matrix.py [manifest]  (不给取最新 b4_matrix_*.txt)"""
import glob
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, '/home/clear/ros2_ws_HJH/src/scripts/masschanger')
import parse_m0_logs as est          # noqa: E402
import parse_dropwindow_logs as ctl  # noqa: E402

D = '/home/clear/ros2_ws_HJH/nmpc_test_results/'
M_EMPTY = 2.064
METHOD_ORDER = ['M0', 'thetastar']
METHOD_LABEL = {'M0': 'M0 规则', 'thetastar': 'θ* 学习'}


def fmt(vals, prec=3):
    v = np.array(vals, dtype=float)
    v = v[np.isfinite(v)]
    if len(v) == 0:
        return 'n/a'
    return f'{np.mean(v):.{prec}f}±{np.std(v):.{prec}f}(n={len(v)})'


def parse_run(nstamp, mass):
    """per-mass M_TRUE/BAND/THRESH 口径(原 grip_cem_optimize 同款,该脚本已于
    2026-07-31 随 CEM 一并删除,口径定义保留在此)。返回指标 dict 或 None。"""
    est.M_TRUE = M_EMPTY + mass
    est.BAND = max(0.15 * mass, 0.03)
    est.THRESH = min(1.5, max(0.7 * 9.81 * mass, 0.2))
    ctl.THRESH = est.THRESH
    try:
        r = est.parse(D + f'grip_mhe_{nstamp}.log')
        c = ctl.parse(D + f'grip_nmpc_{nstamp}.log')
    except (RuntimeError, FileNotFoundError, Exception):
        return None
    inband = (r['t'] >= 0) & (np.abs(r['m'] - est.M_TRUE) <= est.BAND)
    if not np.any(inband):
        return None
    k_in = int(np.argmax(inband))
    overshoot = float(np.max(np.abs(r['m'][k_in:] - est.M_TRUE)))
    return dict(enter=float(r['t'][inband][0]), settle=float(r['t_settle']),
                overshoot=overshoot, pe_peak=float(c['pe_peak']),
                t_rec=float(c['t_rec']))


def main():
    if len(sys.argv) > 1:
        manifest = sys.argv[1]
    else:
        cands = sorted(glob.glob(D + 'b4_matrix_*.txt'), key=os.path.getmtime)
        if not cands:
            print('没找到 b4_matrix_*.txt,先跑 run_b4_matrix.sh'); return
        manifest = cands[-1]
    print(f'manifest: {manifest}')

    cells = defaultdict(list)
    for line in open(manifest):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) < 6:
            continue
        method, mass, ecc, rep, nstamp, status = parts[:6]
        cells[(mass, ecc, method)].append((nstamp, status))

    keys = sorted({(m, e) for (m, e, _) in cells})
    for mass, ecc in keys:
        print(f'\n===== 质量 {mass}kg × 偏心 {ecc}m (online 无真值几何) =====')
        for method in METHOD_ORDER:
            runs = cells.get((mass, ecc, method), [])
            en, st, ov, pk, rc = [], [], [], [], []
            ndiv = 0; bad = []
            for nstamp, status in runs:
                if status.startswith('DIVERGED'):
                    ndiv += 1
                if status != 'ok':
                    bad.append(f'{nstamp}:{status}'); continue
                m = parse_run(nstamp, float(mass))
                if m is None:
                    bad.append(f'{nstamp}:parse_fail'); continue
                en.append(m['enter']); st.append(m['settle'])
                ov.append(m['overshoot']); pk.append(m['pe_peak'])
                rc.append(m['t_rec'])
            print(f'  --- {METHOD_LABEL.get(method, method)} ---'
                  + (f'  剔除{bad}' if bad else ''))
            print(f'    [估计] 入带[s]:{fmt(en,2)}  驻留[s]:{fmt(st,2)}  '
                  f'过冲[kg]:{fmt(ov)}')
            print(f'    [控制] pos_err峰[m]:{fmt(pk)}  恢复[s]:{fmt(rc,2)}  '
                  f'发散:{ndiv}/{len(runs)}')


if __name__ == '__main__':
    main()
