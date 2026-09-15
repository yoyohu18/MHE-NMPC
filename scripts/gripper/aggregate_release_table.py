#!/usr/bin/env python3
"""论文配对 external-event 消融、表 II(tab:complete)与 B 全样本统计,一条命令重算。

数据源(默认读仓库内冻结数据 paper/data/,由 SHA256SUMS 校验):
  mainline_ab3_manifest.csv   W3,09-10 晚(483db88)
  mainline_ab3b_manifest.csv  W4/W5,09-11 凌晨(483db88)
  mainline_ab3c_manifest.csv  W3 补充批次,09-11 23:12(运行时三文件 sha256 与前两批一致)
  以及 manifest 中每个 stamp 的 grip_nmpc_<stamp>.log / grip_mhe_<stamp>.log

口径:
  配对消融    同一 manifest/wid/pair 下 A、B 各至少一条 validity=valid，取各自最后一次
              有效重飞；主线共有 23 对。
  B 全样本    三份 manifest 中 arm=B 且 validity=valid 的全部 31 次飞行。
  确认延迟    grip_nmpc 中 "DROP command issued" -> "DROP complete" 的时间差。
  路径        若 "PAYLOAD STATE UNRESOLVED" 早于 DROP complete 则为 brake-to-hover,否则 in-maneuver。
  IQR         p25 取 numpy method='nearest',p75 取 method='higher'。这是 21 对表格当初的
              实际口径(任何单一 numpy 方法都不能同时复现 8 个格子),为保持可比沿用;
              ⚠️ 属非标准混合口径,改用标准四分位会改变部分格子的数字。
  释放路径    grip_mhe 中第一条 "载荷释放(<why>)":slow/fastB = moment 判据;mass-based = 质量域;
              无此行 = MHE 从未判释放(确认只靠 no-payload confidence)。
  状态机      [payload-state] 恰为 EMPTY->LOADED、LOADED->EMPTY 两次;LOADED 来源含 "attach" 即
              残差自触发声明带载。
  ovr         [s-collapse] WARN 行 "本轮累计 N 帧" 在 DROP command 之前的最大值(采样 %10,为下界)。
              注意不是 [s-collapse] 普通行的 ovr= 字段(那一字段只在 moment 基准就绪后打印)。

用法: python3 aggregate_release_table.py [data_dir]
"""
import csv
import os
import re
import sys

import numpy as np
from scipy.stats import beta, fisher_exact

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(
    os.path.join(HERE, '..', '..', 'paper', 'data'))
MANIFESTS = ['mainline_ab3_manifest.csv', 'mainline_ab3b_manifest.csv',
             'mainline_ab3c_manifest.csv']
# 覆盖数据源(2026-09-14):RELEASE_MANIFESTS="a.csv:W5 b.csv" —— 空格分隔,
# ":W5[,W4]" 只取该 manifest 里这些工况。用于 mainline_ab3d(W3/W4 在
# MHE_RESID_CONFIRM=false 下重跑)替换旧 W3/W4、保留 ab3b 的 W5。不设 = 历史口径。
MANIFEST_SPECS = [(m.split(':')[0], tuple(m.split(':')[1].split(',')) if ':' in m else None)
                  for m in os.environ.get('RELEASE_MANIFESTS', '').split()] \
    or [(m, None) for m in MANIFESTS]
# 有效性口径(2026-09-15):RELEASE_VALIDITY=physical 时,manifest 里判为
# attach-fail 的飞行若 grip_proximity 日志记录了物理 attach("attach offset"),
# 改判为有效。原因:check_run_valid.py 用 m_est 判"是否挂上",估计器从头锁死
# (m_est 全程贴空机值)会被误分为 attach 失败并剔出分母 —— 有效性判的是实验
# 是否搭起来,不能依赖被测估计器。不设 = manifest 原口径。
PHYSICAL_VALIDITY = os.environ.get('RELEASE_VALIDITY', '') == 'physical'


def physically_attached(stamp):
    path = os.path.join(DATA, f'grip_proximity_{stamp}.log')
    if not os.path.isfile(path):
        return None
    return 'attach offset' in open(path, errors='ignore').read()
WIDS = ('W3', 'W4', 'W5')
TS = re.compile(r'^\[[A-Z]+\] \[(\d+\.\d+)\]')


def first_ts(text, pattern):
    m = re.search(r'\[(\d+\.\d+)\] \[\w+\]: [^\n]*' + pattern, text)
    return float(m.group(1)) if m else None


def analyze(stamp):
    nmpc = open(os.path.join(DATA, f'grip_nmpc_{stamp}.log'), errors='ignore').read()
    t_cmd = first_ts(nmpc, 'DROP command issued')
    t_done = first_ts(nmpc, 'DROP complete')
    t_unres = first_ts(nmpc, 'PAYLOAD STATE UNRESOLVED')
    if t_cmd is None or t_done is None:
        return None
    release, states, ovr = None, [], 0
    for line in open(os.path.join(DATA, f'grip_mhe_{stamp}.log'), errors='ignore'):
        m = TS.match(line)
        t = float(m.group(1)) if m else None
        if release is None and '载荷释放' in line:
            why = re.search(r'载荷释放\((.*?)\)', line)
            release = why.group(1) if why else line.strip()
        if '[payload-state]' in line:
            states.append(line.split('[payload-state]', 1)[1].strip())
        if '[s-collapse]' in line and '本轮累计' in line and t is not None and t < t_cmd:
            k = re.search(r'本轮累计 (\d+) 帧', line)
            if k:
                ovr = max(ovr, int(k.group(1)))
    if release is None:
        path = 'none'
    elif re.search(r'slow|fastB', release):
        path = 'moment'
    elif 'mass' in release:
        path = 'mass'
    else:
        path = 'other'
    loaded = [s for s in states if s.startswith('EMPTY->LOADED')]
    return {
        'delay': t_done - t_cmd,
        'route': 'brake' if (t_unres is not None and t_unres < t_done) else 'man',
        'path': path,
        'two_transitions': (len(states) == 2 and states[0].startswith('EMPTY->LOADED')
                            and states[1].startswith('LOADED->EMPTY')),
        'self_trigger_load': bool(loaded) and '(attach' in loaded[0],
        'ovr': ovr,
    }


def has_completion(stamp):
    text = open(os.path.join(DATA, f'grip_nmpc_{stamp}.log'), errors='ignore').read()
    return 'DROP complete' in text or 'DROP: released' in text


def iqr_cell(values):
    if not values:
        return '---'
    v = sorted(values)
    return (f'{np.median(v):.2f} s [{np.percentile(v, 25, method="nearest"):.2f}, '
            f'{np.percentile(v, 75, method="higher"):.2f}] (n={len(v)})')


def fisher_line(label, flights):
    zero = [f for f in flights if f['ovr'] == 0]
    pos = [f for f in flights if f['ovr'] > 0]
    zm = sum(f['route'] == 'man' for f in zero)
    pm = sum(f['route'] == 'man' for f in pos)
    p = fisher_exact([[zm, len(zero) - zm], [pm, len(pos) - pm]])[1] if zero and pos else float('nan')
    print(f'  ovr {label:<12} ovr=0 in-maneuver {zm}/{len(zero)}  vs  ovr>0 {pm}/{len(pos)}  '
          f'(Fisher two-sided p={p:.4f})')


def main():
    cohort, flights, all_rows, paired, incomplete = [], [], [], [], []
    for name, only in MANIFEST_SPECS:
        path = os.path.join(DATA, name)
        if not os.path.isfile(path):
            print(f'missing manifest: {path}')
            return 1
        rows = [r for r in csv.DictReader(open(path)) if only is None or r['wid'] in only]
        if PHYSICAL_VALIDITY:
            for r in rows:
                if 'attach-fail' not in r['validity']:
                    continue
                att = physically_attached(r['stamp'])
                if att is None:
                    print(f'  [validity] no proximity log for {r["stamp"]}; kept as {r["validity"]}')
                elif att:
                    print(f'  [validity] {name} {r["wid"]} {r["arm"]} {r["stamp"]}: '
                          f'attach-fail -> valid (physical attach logged)')
                    r['validity'] = 'valid'
        all_rows += rows
        groups = {}
        for row in rows:
            groups.setdefault((row['wid'], row['pair']), {}).setdefault(row['arm'], []).append(row)
        for (wid, pair), arms in groups.items():
            av = [r for r in arms.get('A', []) if r['validity'] == 'valid']
            bv = [r for r in arms.get('B', []) if r['validity'] == 'valid']
            if av and bv:
                paired.append((wid, pair, av[-1], bv[-1]))
        b_rows = [r for r in rows if r['arm'] == 'B']
        flights += b_rows
        for b in b_rows:
            if b['validity'] != 'valid':
                continue
            a = analyze(b['stamp'])
            if a is None:
                # 2026-09-14:mainline_ab3d 出现首个"有效但未完成"的 B 飞行
                # (20260914_185509,MHE drop 前 48s 不健康、释放后坠机)。按预注册
                # 它是验收失败,必须计入分母,不能再当数据错误退出。
                print(f'eligible B flight WITHOUT completion (counted as failure): {b["stamp"]}')
                incomplete.append(dict(wid=b['wid'], stamp=b['stamp']))
                continue
            a.update(wid=b['wid'], stamp=b['stamp'])
            cohort.append(a)
    n = len(cohort)
    n_all = n + len(incomplete)
    print(f'data: {DATA}\neligible eventless flights: {n_all} (completed {n}, incomplete {len(incomplete)})\n')
    print('Table I (paired external-event baseline)')
    for wid in WIDS + ('pooled',):
        sel = [p for p in paired if wid == 'pooled' or p[0] == wid]
        ac = sum(has_completion(p[2]['stamp']) for p in sel)
        bc = sum(has_completion(p[3]['stamp']) for p in sel)
        print(f'  {wid:<6} pairs={len(sel):<2}  arm A {ac}/{len(sel)}  arm B {bc}/{len(sel)}')
    print()
    print('Table II (all eligible eventless flights)')
    for g in WIDS + ('pooled',):
        sel = [a for a in cohort if g == 'pooled' or a['wid'] == g]
        inc = sum(1 for x in incomplete if g == 'pooled' or x['wid'] == g)
        man = [a['delay'] for a in sel if a['route'] == 'man']
        brk = [a['delay'] for a in sel if a['route'] == 'brake']
        print(f'  {g:<6} n={len(sel)+inc:<2} compl={len(sel)}/{len(sel)+inc}  brake share={len(brk)/len(sel):.0%}  '
              f'in-maneuver {iqr_cell(man)}  |  brake-to-hover {iqr_cell(brk)}')
    k = n
    lo = beta.ppf(0.025, k, n_all - k + 1)
    hi_s = '100%' if k == n_all else f'{beta.ppf(0.975, k + 1, n_all - k)*100:.1f}%'
    print(f'  pooled completion {k}/{n_all}, Clopper-Pearson two-sided 95% [{lo*100:.1f}%, {hi_s}]; '
          f'max delay {max(a["delay"] for a in cohort):.2f} s\n')
    paths = {k: [a for a in cohort if a['path'] == k] for k in ('moment', 'mass', 'none', 'other')}
    per = lambda k: ', '.join(f'{w} {sum(a["wid"] == w for a in paths[k])}' for w in WIDS)
    print('Release paths')
    for k in ('moment', 'mass', 'none', 'other'):
        print(f'  {k:<7} {len(paths[k])}/{n}  ({per(k)})')
    print(f'\nPresence machine: exactly two transitions {sum(a["two_transitions"] for a in cohort)}/{n}; '
          f'load declared by residual self-trigger {sum(a["self_trigger_load"] for a in cohort)}/{n}\n')
    fisher_line('all speeds', cohort)
    fisher_line('4 m/s', [a for a in cohort if a['wid'] in ('W3', 'W4')])
    print('\nEventless flights and exclusions')
    crash = sum('pre-drop-crash' in r['validity'] for r in flights)
    fail = sum('attach-fail' in r['validity'] for r in flights)
    print(f'  {len(flights)} flights, pre-release crashes {crash}, payload-evidence failures {fail}')
    print(f'  total {len(flights)} flights, voided {sum(r["validity"] != "valid" for r in flights)}')
    print('\nAll paired-campaign attempts')
    for arm in ('A', 'B'):
        sel = [r for r in all_rows if r['arm'] == arm]
        valid = sum(r['validity'] == 'valid' for r in sel)
        crash = sum('pre-drop-crash' in r['validity'] for r in sel)
        fail = sum('attach-fail' in r['validity'] for r in sel)
        print(f'  arm {arm}: attempts {len(sel)}, eligible {valid}, '
              f'pre-release crashes {crash}, payload-evidence failures {fail}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
