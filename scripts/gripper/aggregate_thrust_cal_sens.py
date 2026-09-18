#!/usr/bin/env python3
"""聚合 run_thrust_cal_sens.sh(推力系数 ±5% 敏感性,判定规则见该脚本头部)。

用法: aggregate_thrust_cal_sens.py <batch_dir>
"""
import csv
import glob
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

R = Path('/home/clear/ros2_ws_HJH/nmpc_test_results')
TS = re.compile(r'\[(?:INFO|WARN|ERROR)\] \[(\d+\.\d+)\]')
M_B = 2.0643          # 空机质量 [kg]
M_TRUE = M_B + 0.15   # 带载真值
RECOG_KG = 0.08       # ① 识别阈值(载荷真值的一半)
GHOST_KG = 0.05       # ② 幽灵载荷阈值
LB = 1.965


def first(path, pat):
    if not path.exists():
        return None
    for line in open(path, errors='ignore'):
        if pat in line:
            m = TS.search(line)
            if m:
                return float(m.group(1))
    return None


def evt_wall(path, name):
    if not path.exists():
        return None
    for line in open(path, errors='ignore'):
        m = re.search(rf'\[EVT\] name={name} wall=([\d.]+)', line)
        if m:
            return float(m.group(1))
    return None


def metrics(row):
    o = {k: row[k] for k in ('idx', 'rep', 'arm', 'k', 'stamp', 'status')}
    o['peak'] = float(row['peak_pos_err'])
    st = row['stamp']
    if st in ('-', ''):
        o['valid'] = False
        return o
    nm, px = R / f'grip_nmpc_{st}.log', R / f'grip_proximity_{st}.log'
    mh = R / f'grip_mhe_{st}.log'
    t_att = evt_wall(px, 'attach')
    o['valid'] = t_att is not None              # 物理吸附口径
    t_sep = evt_wall(px, 'sep')
    t_cmd = evt_wall(nm, 'cmd')
    t_acc = evt_wall(nm, 'accept')
    if t_acc is not None and t_cmd is not None and t_acc < t_cmd - 0.5:
        t_acc = None
        for line in open(nm, errors='ignore'):
            m = re.search(r'\[EVT\] name=accept wall=([\d.]+)', line)
            if m and float(m.group(1)) >= t_cmd - 0.5:
                t_acc = float(m.group(1)); break
    t_lift, t_dyn = first(nm, 'LIFT: raising'), first(nm, 'DYNAMIC: switch')
    o['unresolved'] = first(nm, 'UNRESOLVED') is not None
    o['premature'] = bool(t_acc and (t_sep is None or t_acc < t_sep - 0.05))
    o['detect_delay'] = (t_acc - t_sep) if (t_acc and t_sep) else np.nan
    o['handoff'] = first(nm, 'bootstrap handoff') is not None
    mtxt = mh.read_text(errors='ignore') if mh.exists() else ''
    o['solve_failed'] = mtxt.count('solve failed')
    for key in ('m_peak_load', 'm_load_med', 'm_ghost', 'LB_lift'):
        o[key] = np.nan
    o['recognized'] = o['ghost'] = False
    try:
        itn = pd.read_csv(glob.glob(f"{row['resid_dir']}/resid_internal_*.csv")[0])
        odo = pd.read_csv(glob.glob(f"{row['resid_dir']}/resid_odom_*.csv")[0])
    except (IndexError, FileNotFoundError):
        return o
    wall = (itn.t_recv_ns + np.median(odo.t_header_ns - odo.t_recv_ns)) / 1e9

    def seg(a, b):
        return itn.m_est[(wall >= a) & (wall < b)]

    # ① 识别:带载段(离地+4s 到 投放指令)m̂ 峰值 − 空机
    if t_lift and t_cmd:
        x = seg(t_lift + 4, t_cmd)
        if len(x):
            o['m_peak_load'] = x.max()
            o['m_load_med'] = x.median()
            o['recognized'] = bool(x.max() - M_B >= RECOG_KG)
    # ② 幽灵:确认卸载后 5~15 s 的 m̂ 中位 − 空机(没确认就用指令 +17~27 s)
    t0 = (t_acc + 5) if t_acc else ((t_cmd + 17) if t_cmd else None)
    if t0:
        x = seg(t0, t0 + 10)
        if len(x):
            o['m_ghost'] = x.median() - M_B
            o['ghost'] = bool(o['m_ghost'] >= GHOST_KG)
    # ④ 吊起段贴下界占比
    if t_lift and t_dyn:
        x = seg(t_lift + 4, t_dyn)
        if len(x):
            o['LB_lift'] = (x < LB).mean()
    return o


def main():
    b = Path(sys.argv[1])
    df = pd.DataFrame([metrics(r) for r in csv.DictReader(open(b / 'manifest.csv'))])
    df.to_csv(b / 'summary_runs.csv', index=False)
    cols = ['idx', 'arm', 'k', 'stamp', 'status', 'valid', 'recognized', 'm_peak_load',
            'm_load_med', 'ghost', 'm_ghost', 'unresolved', 'premature', 'detect_delay',
            'handoff', 'LB_lift', 'solve_failed', 'peak']
    with pd.option_context('display.width', 250, 'display.max_columns', 30,
                           'display.float_format', '{:.3f}'.format):
        print(df[[c for c in cols if c in df]].to_string(index=False))
    print()
    print(f"{'臂':6s} {'k':>5s} {'有效':>4s} {'①识别':>6s} {'带载m̂中位':>10s} "
          f"{'②幽灵':>6s} {'幽灵kg中位':>11s} {'UNRESOLVED':>10s} {'提前清除':>8s} "
          f"{'发散':>4s} {'贴下界%':>8s}")
    for arm, g in df.groupby('arm', sort=True):
        v = g[g.valid.astype(bool)]
        n = len(v)
        print(f"{arm:6s} {v.k.iloc[0] if n else '-':>5} {n:>4d} "
              f"{int(v.recognized.sum()):>4d}/{n:<2d} {v.m_load_med.median():>10.3f} "
              f"{int(v.ghost.sum()):>4d}/{n:<2d} {v.m_ghost.median():>11.3f} "
              f"{int(v.unresolved.sum()):>10d} {int(v.premature.sum()):>8d} "
              f"{int((g.status != 'ok').sum()):>4d} {100 * v.LB_lift.median():>8.1f}")
    print(f"\n判定阈值:① 峰值 m̂−{M_B:.3f} ≥ {RECOG_KG} kg 记识别;"
          f"② 卸载后 m̂−{M_B:.3f} ≥ {GHOST_KG} kg 记幽灵;带载真值 {M_TRUE:.3f} kg")


if __name__ == '__main__':
    main()
