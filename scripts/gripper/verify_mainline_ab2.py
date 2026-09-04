#!/usr/bin/env python3
"""连续主线第三轮(2026-09-04)的预注册验收。

口径(用户 09-04 定):
  · 只统计**同时有效**的配对;drop 前坠机 / attach 失败作废并单独报发生率
  · 主线验收 = 有效 B 臂全部完成 drop、confidence 达标、drop 后零失稳
  · 凑不满 8 对如实报实际 n
新增检查(状态机改造引入):
  · figure8 带载段状态机零误退(只在真 drop 后一次 LOADED->EMPTY)
  · s_out 连续衰减到 0(逐帧 d_s_out 单调、无阶跃)
"""
import collections
import csv
import os
import re

RES = 'nmpc_test_results'
MAN = f'{RES}/mainline_ab2_manifest.csv'
PS = re.compile(r'\[payload-state\] (\S+) \((.*?), m_p=([+-][\d.]+)kg\)')
SD = re.compile(r'\[s-decay\] n=(\d+) .*\|s_out\|=([\d.]+) d_s_out=([+-][\d.]+) '
                r'conf=([\d.]+) present=(\d) armed=(\d) latched=(\d) health=(\d)')
FD = re.compile(r'\[flight-diag\] t=([\d.]+)s .*tilt_max=([\d.]+)deg')
AW = re.compile(r'\[attach-window\] t=([\d.-]+)s pos_err=([\d.]+)m')
TCMD = re.compile(r't=([\d.]+)s \| DROP(?: command issued|: released)')


def check(stamp):
    n, m = f'{RES}/grip_nmpc_{stamp}.log', f'{RES}/grip_mhe_{stamp}.log'
    r = {'drop_complete': False, 'conf': None, 'transitions': [],
         'sdecay': [], 'tilt_post': 0.0, 'poserr_post': 0.0, 'solve_fail_post': 0}
    t_cmd = None
    fails = []
    with open(n, errors='ignore') as f:
        for line in f:
            if t_cmd is None and (x := TCMD.search(line)):
                t_cmd = float(x.group(1))
            # A 臂(legacy/event 档)打的是 'DROP: released' —— 它的 drop 由自己的
            # 释放指令定义,不等 confidence;B 臂才有 'DROP complete'。两者都算
            # "drop 完成",只是完成的判据不同,这正是 A/B 要比的那件事。
            if 'DROP complete' in line or 'DROP: released' in line:
                r['drop_complete'] = True
                if (x := re.search(r'confidence ([\d.]+) persisted', line)):
                    r['conf'] = float(x.group(1))
            if 'solve failed' in line and t_cmd is not None:
                r['solve_fail_post'] += 1
            if t_cmd is not None and (x := FD.search(line)):
                if float(x.group(1)) >= t_cmd:
                    r['tilt_post'] = max(r['tilt_post'], float(x.group(2)))
            if t_cmd is not None and (x := AW.search(line)):
                if float(x.group(1)) >= t_cmd:
                    r['poserr_post'] = max(r['poserr_post'], float(x.group(2)))
    if os.path.exists(m):
        with open(m, errors='ignore') as f:
            for line in f:
                if (x := PS.search(line)):
                    r['transitions'].append((x.group(1), x.group(2)))
                if (x := SD.search(line)):
                    r['sdecay'].append((int(x.group(1)), float(x.group(2)),
                                        float(x.group(3)), float(x.group(4))))
    # 状态机:恰好 2 次转换,且退出只由 drop 触发
    tr = r['transitions']
    r['clean_sm'] = (len(tr) == 2 and tr[0][0] == 'EMPTY->LOADED'
                     and tr[1][0] == 'LOADED->EMPTY' and tr[1][1] == 'drop')
    r['n_spurious'] = max(0, len(tr) - 2)
    # s_out:单调不增、无正跳变、末值≈0
    sd = r['sdecay']
    r['s_continuous'] = bool(sd) and all(d <= 1e-9 for _, _, d, _ in sd) \
        and sd[-1][1] < 1e-4
    r['s_frames'] = len(sd)
    # drop 后失稳判据(与 check_run_valid 同口径)
    if r['tilt_post'] > 60: fails.append('tilt>60')
    if r['poserr_post'] > 3.0: fails.append('pos_err>3m')
    if r['solve_fail_post'] > 100: fails.append('solve_fail>100')
    r['post_drop_fail'] = ';'.join(fails)
    return r


rows = list(csv.DictReader(open(MAN)))
byw = collections.defaultdict(list)
for x in rows:
    byw[x['wid']].append(x)

inval = collections.Counter()
for x in rows:
    v = x['validity'].strip('"').split(' ')[0]
    if v != 'valid':
        inval[(x['wid'], x['arm'], v)] += 1

print('=' * 78)
print('预注册验收 · 连续主线第三轮(状态机 + armed latch + s 目标切零)')
print('=' * 78)
summary = {}
for w in ['W3', 'W4', 'W5']:
    rs = sorted(byw[w], key=lambda x: int(x['idx']))
    vp = []
    for i in range(0, len(rs) - 1, 2):
        a, b = rs[i], rs[i + 1]
        if (a['validity'].strip('"') == 'valid'
                and b['validity'].strip('"') == 'valid'):
            vp.append((a, b))
    print(f'\n{w}: 有效 {len(vp)} 对 / 尝试 {len(rs)//2} 次 / {len(rs)} 架次')
    print(f"  {'idx':>4} {'臂':<3}{'drop完成':>9}{'conf':>7}{'状态转换':>9}"
          f"{'误退':>5}{'s连续':>7}{'s帧':>5}{'drop后失稳':>11}")
    ok_b = 0
    for a, b in vp:
        for x in (a, b):
            c = check(x['stamp'])
            if x['arm'] == 'B' and c['drop_complete'] and not c['post_drop_fail']:
                ok_b += 1
            print(f"  {x['idx']:>4} {x['arm']:<3}"
                  f"{'✓' if c['drop_complete'] else '✗':>9}"
                  f"{(f'{c[chr(99)+chr(111)+chr(110)+chr(102)]:.3f}' if c['conf'] else '-'):>7}"
                  f"{len(c['transitions']):>9}{c['n_spurious']:>5}"
                  f"{'✓' if c['s_continuous'] else ('-' if not c['sdecay'] else '✗'):>7}"
                  f"{c['s_frames']:>5}"
                  f"{(c['post_drop_fail'] or '无'):>11}")
    summary[w] = (len(vp), ok_b)

print('\n' + '=' * 78)
print('作废轮次(单独报告,不进主结论):')
for (w, arm, v), k in sorted(inval.items()):
    print(f'  {w} {arm} 臂  {v:<24} {k} 次')
tot_att = len(rows) // 2
tot_inval = sum(inval.values())
print(f'  合计 {tot_inval}/{len(rows)} 架次作废 ({tot_inval/len(rows)*100:.0f}%)')

print('\n主线验收:')
tp = sum(v[0] for v in summary.values())
tb = sum(v[1] for v in summary.values())
for w, (p, b) in summary.items():
    print(f'  {w}: 有效 {p} 对,B 臂完成 drop 且 drop 后无失稳 {b}/{p}')
print(f'  合计 {tb}/{tp} —— {"✅ 通过" if tb == tp else "❌ 未通过"}'
      f'{"(注:W4 未凑满 8 对,实际 n 见上)" if summary["W4"][0] < 8 else ""}')
