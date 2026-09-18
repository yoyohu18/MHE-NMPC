#!/usr/bin/env python3
"""汇总 run_safety_fullround.sh(描述性安全冒烟)。

每轮:事件时刻(指令 cmd / 意外脱落 loss)、真实分离 sep、控制器接受空载 accept、
UNRESOLVED、事件前/后 pos_err 峰值、吊起段 m 贴下界占比、bootstrap 是否交接。
安全判读:
  提前清除 = accept 早于 sep(或 sep 从未发生却 accept)——d4/jam 的关键失败
  漏检     = sep 发生但观测窗内没有 accept
  发散     = pos_err 峰值 > 2 m
用法: aggregate_safety_fullround.py <batch_dir>
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
    o = dict(row)
    st = row['stamp']
    if st in ('-', ''):
        return o
    nm, px = R / f'grip_nmpc_{st}.log', R / f'grip_proximity_{st}.log'
    t_att = evt_wall(px, 'attach')
    t_sep = evt_wall(px, 'sep')
    t_cmd = evt_wall(nm, 'cmd')
    t_loss = first(nm, 'UNCOMMANDED LOSS')
    t_ev = t_cmd if t_cmd else t_loss
    t_acc = evt_wall(nm, 'accept')
    # 起飞前的空机 accept 也会打 accept 行 → 只认事件之后的
    if t_acc is not None and t_ev is not None and t_acc < t_ev - 0.5:
        t_acc = None
        for line in open(nm, errors='ignore'):
            m = re.search(r'\[EVT\] name=accept wall=([\d.]+)', line)
            if m and float(m.group(1)) >= t_ev - 0.5:
                t_acc = float(m.group(1)); break
    o['attached'] = t_att is not None
    o['sep_after_ev'] = (t_sep - t_ev) if (t_sep and t_ev) else np.nan
    o['accept_after_ev'] = (t_acc - t_ev) if (t_acc and t_ev) else np.nan
    o['detect_delay'] = (t_acc - t_sep) if (t_acc and t_sep) else np.nan
    o['premature'] = bool(t_acc and (t_sep is None or t_acc < t_sep - 0.05))
    o['missed'] = bool(t_sep and not t_acc)
    o['unresolved'] = first(nm, 'UNRESOLVED') is not None
    o['handoff'] = first(nm, 'bootstrap handoff') is not None
    pe = []
    for line in open(nm, errors='ignore'):
        if '| pos_err=' in line and '| T=' in line:
            m, w = re.search(r'pos_err=([\d.]+)m', line), TS.search(line)
            if m and w:
                pe.append((float(w.group(1)), float(m.group(1))))
    pe = np.array(pe) if pe else np.zeros((0, 2))
    if t_ev and len(pe):
        pre, post = pe[pe[:, 0] < t_ev, 1], pe[pe[:, 0] >= t_ev, 1]
        o['peak_pre'] = pre.max() if len(pre) else np.nan
        o['peak_post'] = post.max() if len(post) else np.nan
    t_lift, t_dyn = first(nm, 'LIFT: raising'), first(nm, 'DYNAMIC: switch')
    o['LB_lift'] = np.nan
    try:
        itn = pd.read_csv(glob.glob(f"{row['resid_dir']}/resid_internal_*.csv")[0])
        odo = pd.read_csv(glob.glob(f"{row['resid_dir']}/resid_odom_*.csv")[0])
        wall = (itn.t_recv_ns + np.median(odo.t_header_ns - odo.t_recv_ns)) / 1e9
        if t_lift and t_dyn:
            x = itn.m_est[(wall >= t_lift + 4) & (wall < t_dyn)]
            o['LB_lift'] = (x < LB).mean() if len(x) else np.nan
            o['m_lift'] = x.median() if len(x) else np.nan
    except (IndexError, FileNotFoundError):
        pass
    return o


def main():
    b = Path(sys.argv[1])
    df = pd.DataFrame([metrics(r) for r in csv.DictReader(open(b / 'manifest.csv'))])
    df.to_csv(b / 'summary_runs.csv', index=False)
    cols = ['idx', 'cond', 'stamp', 'status', 'attached', 'handoff', 'LB_lift', 'm_lift',
            'sep_after_ev', 'accept_after_ev', 'detect_delay', 'premature', 'missed',
            'unresolved', 'peak_pre', 'peak_post']
    with pd.option_context('display.width', 250, 'display.max_columns', 30,
                           'display.float_format', '{:.2f}'.format):
        print(df[[c for c in cols if c in df]].to_string(index=False))
    print()
    for c, g in df.groupby('cond', sort=False):
        print(f"[{c}] n={len(g)} 发散={int((g.status != 'ok').sum())} 提前清除={int(g.premature.sum())} "
              f"漏检={int(g.missed.sum())} UNRESOLVED={int(g.unresolved.sum())} "
              f"不交接={int((~g.handoff.astype(bool)).sum())} "
              f"检出延迟中位={g.detect_delay.median():.2f}s 事后峰值中位={g.peak_post.median():.2f}m")


if __name__ == '__main__':
    main()
