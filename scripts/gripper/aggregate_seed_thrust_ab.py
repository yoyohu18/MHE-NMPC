#!/usr/bin/env python3
"""重锚推力种子 A/B 聚合(预注册: ~/ros2_ws_HJH/重锚推力种子_预注册_20260914.md)。

主指标(figure-8 段, DYNAMIC: switch -> DROP command, 真值 M_LOADED):
  P1 每次重锚种子误差 |m_seed - M_LOADED|/M_LOADED, 按飞行取中位后两臂 Mann-Whitney
  P2 连续重锚串长: 相邻两次重锚之间恰好 5 条 solve failed(无成功解)即同一串
  P3 drop 前 "[s-collapse] 估计不健康 ... age=" 最大值
次要(无伤害): 有效性分类、DROP complete、释放后 20s 内坠机(pos_err>2m)、
  drop 前 20s NMPC 模型质量中位误差、冷启动首帧种子。
用法: python3 aggregate_seed_thrust_ab.py [manifest] [log_dir]
"""
import csv
import os
import re
import sys
import numpy as np
from scipy.stats import mannwhitneyu

RES = '/home/clear/ros2_ws_HJH/nmpc_test_results'
MAN = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RES, 'seedthrust_ab_manifest.csv')
LOG = sys.argv[2] if len(sys.argv) > 2 else RES
M_BARE = 2.0643
M_LOADED = M_BARE + 0.30
TS = re.compile(r'\[(\d+\.\d+)\] \[(\w+)\]: (.*)')
REANCHOR = re.compile(r're-anchored arrival prior.*?m seeded from (.+?): ([\d.]+) -> ([\d.]+) kg')
AW = re.compile(r'\[attach-window\] t=[\d.-]+s pos_err=([\d.]+)m T=[\d.-]+N z=[\d.-]+ m_est=([\d.]+)')


def events(path, node):
    out = []
    for line in open(path, errors='ignore'):
        m = TS.search(line)
        if m and m.group(2) == node:
            out.append((float(m.group(1)), m.group(3)))
    return out


def first(ev, pat):
    for t, msg in ev:
        if pat in msg:
            return t
    return None


def analyze(stamp):
    nm = events(os.path.join(LOG, f'grip_nmpc_{stamp}.log'), 'acados_nmpc_node')
    mh = events(os.path.join(LOG, f'grip_mhe_{stamp}.log'), 'mhe_node')
    t_f8, t_drop = first(nm, 'DYNAMIC: switch'), first(nm, 'DROP command issued')
    done = any('DROP complete' in m for _, m in nm)
    seed0 = next((re.search(r'arrival prior seeded: m=([\d.]+) kg \((.*?)\)', m)
                  for _, m in mh if 'arrival prior seeded' in m), None)
    r = dict(t_f8=t_f8, t_drop=t_drop, done=done,
             seed0=float(seed0.group(1)) if seed0 else np.nan,
             seed0_src=seed0.group(2) if seed0 else '?')
    # P1/P2
    seeds, streaks, cur, fails_since = [], [], 0, None
    srcs = set()
    for t, msg in mh:
        if 'MHE solve failed' in msg:
            fails_since = (fails_since or 0) + 1
            continue
        m = REANCHOR.search(msg)
        if not m:
            continue
        in_f8 = t_f8 is not None and t_drop is not None and t_f8 < t < t_drop
        if in_f8:
            srcs.add(m.group(1))
            seeds.append(abs(float(m.group(3)) - M_LOADED) / M_LOADED)
            if cur and fails_since == 5:
                cur += 1
            else:
                if cur:
                    streaks.append(cur)
                cur = 1
        fails_since = 0
    if cur:
        streaks.append(cur)
    r.update(seed_err=seeds, streaks=streaks, seed_src=srcs)
    # P3
    ages = [float(a) for t, msg in mh if t_drop and t < t_drop
            for a in re.findall(r'估计不健康.*?age=([\d.]+)s', msg)]
    r['max_age'] = max(ages) if ages else 0.0
    # 次要
    aw = [(t, AW.search(msg)) for t, msg in nm if '[attach-window]' in msg]
    aw = [(t, float(m.group(1)), float(m.group(2))) for t, m in aw if m]
    if t_drop:
        pre = [x[2] for x in aw if t_drop - 20 < x[0] < t_drop]
        post = [x[1] for x in aw if t_drop < x[0] < t_drop + 20]
        r['m_err_pre'] = (abs(np.median(pre) - M_LOADED) / M_LOADED) if pre else np.nan
        r['post_crash'] = bool(post) and max(post) > 2.0
    else:
        r['m_err_pre'], r['post_crash'] = np.nan, False
    return r


def main():
    rows = list(csv.DictReader(open(MAN)))
    by = {'S0': [], 'S1': []}
    print(f'manifest: {MAN}\n')
    print(f"{'arm':3} {'stamp':16} {'validity':28} {'seed0':>6} {'f8重锚':>6} {'种子误差中位':>10} "
          f"{'串长':>10} {'max_age':>7} {'drop前m误差':>10} {'完成':>4} {'释放后坠机':>6}")
    for row in rows:
        v = row['validity'].strip('"')
        r = analyze(row['stamp']) if row['stamp'] != 'NA' else None
        if r is None:
            continue
        r.update(arm=row['arm'], v=v, pair=row['pair'])
        by[row['arm']].append(r)
        se = f"{np.median(r['seed_err'])*100:.1f}%" if r['seed_err'] else '—'
        print(f"{row['arm']:3} {row['stamp']:16} {v[:28]:28} {r['seed0']:6.3f} {len(r['seed_err']):6d} "
              f"{se:>10} {str(r['streaks'])[:10]:>10} {r['max_age']:7.1f} "
              f"{(r['m_err_pre']*100 if r['m_err_pre']==r['m_err_pre'] else float('nan')):9.2f}% "
              f"{int(r['done']):4d} {int(r['post_crash']):6d}")
    print()
    for arm in ('S0', 'S1'):
        allr = by[arm]
        val = [r for r in allr if r['v'] == 'valid']
        crash = sum('pre-drop-crash' in r['v'] for r in allr)
        afail = sum('attach-fail' in r['v'] for r in allr)
        ev = [e for r in val for e in r['seed_err']]
        st = [s for r in val for s in r['streaks']]
        srcs = set().union(*[r['seed_src'] for r in val]) if val else set()
        print(f'[{arm}] 尝试 {len(allr)}, 有效 {len(val)}, drop前坠机 {crash}, attach失败 {afail}')
        print(f'   首帧种子来源 {sorted(set(r["seed0_src"] for r in allr))}, '
              f'种子中位 {np.nanmedian([r["seed0"] for r in allr]):.3f}')
        print(f'   P1 fig8 重锚事件 {len(ev)} (来自 {sum(1 for r in val if r["seed_err"])} 轮), 种子来源 {sorted(srcs)}, '
              f'事件误差中位 {np.median(ev)*100 if ev else float("nan"):.1f}% 最大 {max(ev)*100 if ev else float("nan"):.1f}%')
        print(f'   P2 串长 {sorted(st)}  (>1 的串 {sum(s>1 for s in st)})')
        print(f'   P3 drop前最大不健康 age: 中位 {np.median([r["max_age"] for r in val]) if val else float("nan"):.1f}s '
              f'最大 {max([r["max_age"] for r in val]) if val else float("nan"):.1f}s')
        print(f'   次要: 完成 {sum(r["done"] for r in val)}/{len(val)}, 释放后坠机 {sum(r["post_crash"] for r in val)}, '
              f'drop前模型质量中位误差 {np.nanmedian([r["m_err_pre"] for r in val])*100 if val else float("nan"):.2f}%')
    f0 = [np.median(r['seed_err']) for r in by['S0'] if r['v'] == 'valid' and r['seed_err']]
    f1 = [np.median(r['seed_err']) for r in by['S1'] if r['v'] == 'valid' and r['seed_err']]
    e0 = sum(len(r['seed_err']) for r in by['S0'] if r['v'] == 'valid')
    e1 = sum(len(r['seed_err']) for r in by['S1'] if r['v'] == 'valid')
    print()
    if e0 < 3 and e1 < 3:
        print('P1 判定: 两臂 fig8 重锚事件均 <3 → 事件不足,不可判(预注册规则)')
    elif f0 and f1:
        p = mannwhitneyu(f0, f1, alternative='greater').pvalue
        print(f'P1 飞行级种子误差中位 S0 {np.median(f0)*100:.1f}% (n={len(f0)}) vs S1 {np.median(f1)*100:.1f}% '
              f'(n={len(f1)}), MWU 单侧 p={p:.3f}')
    else:
        print(f'P1: 事件数 S0 {e0} / S1 {e1},有一臂无事件,飞行级比较不可做')


if __name__ == '__main__':
    main()
