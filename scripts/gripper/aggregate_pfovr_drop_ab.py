#!/usr/bin/env python3
"""聚合 run_pfovr_drop_ab.sh(投放段配对 A/B,判定规则见该脚本头部)。

用法: aggregate_pfovr_drop_ab.py <batch_dir>
"""
import csv, re, sys, glob
from pathlib import Path
import numpy as np
import pandas as pd

R = Path('/home/clear/ros2_ws_HJH/nmpc_test_results')
TS = re.compile(r'\[(?:INFO|WARN|ERROR)\] \[(\d+\.\d+)\]')
M_B, LB = 2.0643, 1.965


def first(p, pat):
    if not p.exists():
        return None
    for line in open(p, errors='ignore'):
        if pat in line:
            m = TS.search(line)
            if m:
                return float(m.group(1))
    return None


def evt(p, name):
    if not p.exists():
        return None
    for line in open(p, errors='ignore'):
        m = re.search(rf'\[EVT\] name={name} wall=([\d.]+)', line)
        if m:
            return float(m.group(1))
    return None


def metrics(row):
    o = {k: row[k] for k in ('idx', 'rep', 'arm', 'stamp', 'status')}
    o['peak'] = float(row['peak_pos_err'])
    st = row['stamp']
    o['valid'] = False
    if st in ('-', ''):
        return o
    nm, px, mh = (R / f'grip_nmpc_{st}.log', R / f'grip_proximity_{st}.log',
                  R / f'grip_mhe_{st}.log')
    t_att, t_sep = evt(px, 'attach'), evt(px, 'sep')
    t_cmd, t_acc = evt(nm, 'cmd'), evt(nm, 'accept')
    o['valid'] = t_att is not None
    if t_acc is not None and t_cmd is not None and t_acc < t_cmd - 0.5:
        t_acc = None
        for line in open(nm, errors='ignore'):
            m = re.search(r'\[EVT\] name=accept wall=([\d.]+)', line)
            if m and float(m.group(1)) >= t_cmd - 0.5:
                t_acc = float(m.group(1)); break
    o['premature'] = bool(t_acc and (t_sep is None or t_acc < t_sep - 0.05))
    o['confirmed'] = t_acc is not None
    o['detect_delay'] = (t_acc - t_sep) if (t_acc and t_sep) else np.nan
    o['unresolved'] = first(nm, 'UNRESOLVED') is not None
    o['handoff'] = first(nm, 'bootstrap handoff') is not None
    mtxt = mh.read_text(errors='ignore') if mh.exists() else ''
    o['solve_failed'] = mtxt.count('solve failed')
    o['ovr_reanchor'] = mtxt.count('moment overrange')
    t_lift, t_dyn = first(nm, 'LIFT: raising'), first(nm, 'DYNAMIC: switch')
    for k in ('LB_lift', 'm_load_med', 'm_after'):
        o[k] = np.nan
    try:
        itn = pd.read_csv(glob.glob(f"{row['resid_dir']}/resid_internal_*.csv")[0])
        odo = pd.read_csv(glob.glob(f"{row['resid_dir']}/resid_odom_*.csv")[0])
    except (IndexError, FileNotFoundError):
        return o
    wall = (itn.t_recv_ns + np.median(odo.t_header_ns - odo.t_recv_ns)) / 1e9
    seg = lambda a, b: itn.m_est[(wall >= a) & (wall < b)]
    if t_lift and t_dyn:
        x = seg(t_lift + 4, t_dyn)
        if len(x):
            o['LB_lift'] = (x < LB).mean()
    if t_dyn and t_cmd:
        x = seg(t_dyn, t_cmd)
        if len(x):
            o['m_load_med'] = x.median()
    t0 = (t_acc + 5) if t_acc else ((t_cmd + 17) if t_cmd else None)
    if t0:
        x = seg(t0, t0 + 10)
        if len(x):
            o['m_after'] = x.median() - M_B
    pe = [(float(TS.search(l).group(1)), float(re.search(r'pos_err=([\d.]+)m', l).group(1)))
          for l in open(nm, errors='ignore')
          if '| pos_err=' in l and '| T=' in l and TS.search(l)]
    if pe and t_cmd:
        post = [v for t, v in pe if t >= t_cmd]
        o['peak_post'] = max(post) if post else np.nan
    return o


def main():
    b = Path(sys.argv[1])
    df = pd.DataFrame([metrics(r) for r in csv.DictReader(open(b / 'manifest.csv'))])
    df.to_csv(b / 'summary_runs.csv', index=False)
    cols = ['idx', 'rep', 'arm', 'stamp', 'status', 'valid', 'handoff', 'LB_lift', 'm_load_med',
            'confirmed', 'premature', 'unresolved', 'detect_delay', 'm_after', 'peak_post',
            'solve_failed', 'ovr_reanchor', 'peak']
    with pd.option_context('display.width', 250, 'display.max_columns', 30,
                           'display.float_format', '{:.3f}'.format):
        print(df[[c for c in cols if c in df]].to_string(index=False))
    print()
    for arm, g in df.groupby('arm'):
        v = g[g.valid.astype(bool)]
        n = len(v)
        print(f"[臂 {arm}] 轮数 {len(g)} 有效 {n} | 发散 {int((g.status != 'ok').sum())} | "
              f"提前清除 {int(v.premature.sum())} | 确认 {int(v.confirmed.sum())}/{n} | "
              f"UNRESOLVED {int(v.unresolved.sum())} | 检出延迟中位 {v.detect_delay.median():.2f}s | "
              f"投放后残留中位 {v.m_after.median():+.3f}kg | 吊起段贴下界中位 {100*v.LB_lift.median():.1f}% | "
              f"不交接 {int((~v.handoff.astype(bool)).sum())}")
    # 配对(同 rep 两臂都有效)
    piv = df[df.valid.astype(bool)].set_index(['rep', 'arm'])[['LB_lift', 'detect_delay', 'm_after']].unstack('arm')
    piv = piv.dropna(subset=[('LB_lift', 'A'), ('LB_lift', 'B')])
    if len(piv):
        d = (piv[('LB_lift', 'A')] - piv[('LB_lift', 'B')]).astype(float)
        print(f"\n配对 {len(d)} 对:吊起段贴下界 A−B 均值 {100*d.mean():+.1f}pp (A {100*piv[('LB_lift','A')].mean():.1f}% / B {100*piv[('LB_lift','B')].mean():.1f}%)")
        print("⚠️ 本批终点均为描述性,事件率低、无统计功效(见脚本头部功效声明)")


if __name__ == '__main__':
    main()
