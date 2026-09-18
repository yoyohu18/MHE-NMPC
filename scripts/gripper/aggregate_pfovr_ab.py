#!/usr/bin/env python3
"""聚合 run_pfovr_ab.sh 批次(判定规则见该脚本头部,跑前钉死)。

用法: aggregate_pfovr_ab.py <batch_dir> [--csv out.csv]
"""
import argparse
import csv
import glob
import re
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats

R = Path('/home/clear/ros2_ws_HJH/nmpc_test_results')
TS = re.compile(r'\[(?:INFO|WARN|ERROR)\] \[(\d+\.\d+)\]')
LB = 1.965
M_TRUE = 2.0643 + 0.15


def first(path, pat):
    try:
        for line in open(path, errors='ignore'):
            if pat in line:
                m = TS.search(line)
                if m:
                    return float(m.group(1))
    except FileNotFoundError:
        return None
    return None


def run_metrics(row):
    out = dict(row)
    stamp = row['stamp']
    out['valid'] = False
    if stamp in ('-', ''):
        return out
    nm, mh = R / f'grip_nmpc_{stamp}.log', R / f'grip_mhe_{stamp}.log'
    t_att = first(R / f'grip_proximity_{stamp}.log', '-> ATTACH')
    out['valid'] = t_att is not None           # 物理吸附口径
    out['diverged'] = row['status'] in ('DIVERGED', 'TRACEBACK', 'NODYN')
    t_lift, t_dyn = first(nm, 'LIFT: raising'), first(nm, 'DYNAMIC: switch')
    t_ho = first(nm, 'bootstrap handoff')
    out['no_handoff'] = t_ho is None
    out['handoff_s'] = (t_ho - t_att) if (t_ho and t_att) else np.nan
    txt = nm.read_text(errors='ignore') if nm.exists() else ''
    oms = [float(x) for x in re.findall(r'om_scale=([\d.]+)\(', txt)]
    out['om_late'] = float(np.median(oms[-4:])) if oms else np.nan
    mtxt = mh.read_text(errors='ignore') if mh.exists() else ''
    out['solve_failed'] = mtxt.count('solve failed')
    out['ovr_reanchor'] = mtxt.count('moment overrange')
    out['fail_reanchor'] = len(re.findall(r'consecutive failures', mtxt))
    for k in ('LB_lift', 'm_lift', 'LB_post', 'LB_fig8'):
        out[k] = np.nan
    out['stuck'] = False
    rd = row['resid_dir']
    try:
        itn = pd.read_csv(glob.glob(f'{rd}/resid_internal_*.csv')[0])
        odo = pd.read_csv(glob.glob(f'{rd}/resid_odom_*.csv')[0])
    except (IndexError, FileNotFoundError):
        return out
    off = np.median(odo.t_header_ns - odo.t_recv_ns)
    wall = (itn.t_recv_ns + off) / 1e9
    t_end = wall.max()

    def seg(a, b):
        x = itn.m_est[(wall >= a) & (wall < b)]
        return ((x < LB).mean(), x.median()) if len(x) else (np.nan, np.nan)

    if t_lift and t_dyn:
        out['LB_lift'], out['m_lift'] = seg(t_lift + 4, t_dyn)
    if t_lift:
        out['LB_post'] = seg(t_lift + 4, t_end)[0]
        out['stuck'] = bool(out['LB_post'] >= 0.5)
    if t_dyn:
        out['LB_fig8'] = seg(t_dyn, t_end)[0]
    return out


def mcnemar_exact(a_only, b_only):
    n = a_only + b_only
    return 1.0 if n == 0 else min(1.0, 2 * stats.binom.cdf(min(a_only, b_only), n, 0.5))


def boot_ci(d, n=10000, seed=0):
    rng = np.random.default_rng(seed)
    d = np.asarray(d)
    if len(d) == 0:
        return (np.nan, np.nan)
    means = rng.choice(d, (n, len(d))).mean(axis=1)
    return tuple(np.percentile(means, [2.5, 97.5]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('batch_dir')
    ap.add_argument('--csv')
    a = ap.parse_args()
    rows = list(csv.DictReader(open(Path(a.batch_dir) / 'manifest.csv')))
    df = pd.DataFrame([run_metrics(r) for r in rows])
    if a.csv:
        df.to_csv(a.csv, index=False)

    print(f'轮数 {len(df)}  有效(物理吸附) {int(df.valid.sum())}')
    for arm in ('A', 'B'):
        s = df[df.arm == arm]
        print(f'  臂{arm}: 发散 {int(s.diverged.sum())}/{len(s)}  无效 {int((~s.valid).sum())}')
    divA, divB = int(df[df.arm == 'A'].diverged.sum()), int(df[df.arm == 'B'].diverged.sum())
    print(f'① 否决: B 发散 {divB} vs A {divA} -> {"★否决(不得翻默认)" if divB > divA else "未触发"}')

    # unstack 而非 pivot_table:后者默认丢掉全 NaN 列(如 stuck 轮没有交接时刻)
    cols = ['valid', 'LB_lift', 'stuck', 'no_handoff', 'm_lift', 'handoff_s',
            'om_late', 'solve_failed', 'LB_fig8', 'ovr_reanchor']
    for c in cols:
        if c not in df:
            df[c] = np.nan
    piv = df.set_index(['wp', 'rep', 'arm'])[cols].unstack('arm')
    ok = (piv[('valid', 'A')].fillna(False).astype(bool)
          & piv[('valid', 'B')].fillna(False).astype(bool))
    piv = piv[ok]

    def report(sub, name):
        d = (sub[('LB_lift', 'A')] - sub[('LB_lift', 'B')]).astype(float).dropna()
        if len(d) == 0:
            print(f'[{name}] 无有效配对'); return
        p = stats.wilcoxon(d, zero_method='zsplit').pvalue if np.any(d != 0) else 1.0
        lo, hi = boot_ci(d)
        sa, sb = sub[('stuck', 'A')].astype(bool), sub[('stuck', 'B')].astype(bool)
        ha, hb = sub[('no_handoff', 'A')].astype(bool), sub[('no_handoff', 'B')].astype(bool)
        print(f'[{name}] 有效配对 {len(d)}')
        print(f'  ② LB_lift 均值 A {sub[("LB_lift","A")].mean()*100:.1f}% B {sub[("LB_lift","B")].mean()*100:.1f}%'
              f'  配对差 A−B {d.mean()*100:+.1f}pp  95%CI [{lo*100:+.1f},{hi*100:+.1f}]  Wilcoxon p={p:.3g}')
        print(f'  ③ stuck A {int(sa.sum())} B {int(sb.sum())}  McNemar p={mcnemar_exact(int((sa&~sb).sum()), int((sb&~sa).sum())):.3g}'
              f' | no_handoff A {int(ha.sum())} B {int(hb.sum())}  McNemar p={mcnemar_exact(int((ha&~hb).sum()), int((hb&~ha).sum())):.3g}  (只描述)')
        for k, lab, sc in (('m_lift', '吊起段 m', 1), ('handoff_s', '交接 s', 1), ('om_late', 'om_late', 1),
                           ('solve_failed', 'solve failed', 1), ('LB_fig8', '8字段LB%', 100)):
            print(f'  ④ {lab:12s} A 中位 {sub[(k,"A")].astype(float).median()*sc:.3f}'
                  f'  B 中位 {sub[(k,"B")].astype(float).median()*sc:.3f}')
        print(f'  ④ m 偏差(对 {M_TRUE:.4f}) A {(sub[("m_lift","A")].astype(float)-M_TRUE).median():+.4f}'
              f'  B {(sub[("m_lift","B")].astype(float)-M_TRUE).median():+.4f};'
              f'  B 越界重锚触发总数 {int(sub[("ovr_reanchor","B")].astype(float).sum())}')

    for wp in ('w2', 'w4'):
        if wp in piv.index.get_level_values(0):
            report(piv.loc[wp], wp)
    report(piv, '合并(主结论)')


if __name__ == '__main__':
    main()
