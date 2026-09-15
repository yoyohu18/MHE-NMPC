#!/usr/bin/env python3
"""正文 §Limitations "Operating limit" 与 "Estimator failures" 两段的数字,一条命令重算。

数据源: paper/data/ 冻结的 mainline_ab3d(W3/W4,MHE_RESID_CONFIRM=false)与 mainline_ab3b(W5)。
输出:
  1) 执行器占用: figure-8 段(切入后 10 s 起、DROP 指令前)的 [flight-diag] 5 s 窗口里,
     推力占用中位、各通道出现饱和帧的窗口比例、body-rate scale 中位(按工况, B 臂有效飞行)。
  2) 悬停推力占最大推力(Tmax=31.35 N)。
  3) 未识别载荷(physically attached、DROP 指令前 20 s 模型质量中位 < 2.10 kg)的飞行:重锚次数、
     首次重锚相对 LIFT 的时刻。
  4) 释放后坠机(20260914_185509): drop 前最长不健康 age、释放后全部执行器饱和的时刻。
  5) W4 完成飞行: DROP 指令后模型质量首次 < 2.15 kg 的用时。
用法: python3 audit_limit_and_estimator_failures.py [data_dir]
"""
import csv
import os
import re
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
D = sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(os.path.join(HERE, '..', '..', 'paper', 'data'))
M_B, G, TMAX = 2.0643, 9.81, 31.35
FD = re.compile(r'\[(\d+\.\d+)\] \[acados_nmpc_node\]: \[flight-diag\] t=[\d.]+s \| om_scale=([\d.]+).*?'
                r'u_frac=\[T([\d.]+) r([\d.]+) p([\d.]+) y([\d.]+)\] \| u_sat=\[T([\d.]+) r([\d.]+) p([\d.]+) y([\d.]+)\]')
AW = re.compile(r'\[(\d+\.\d+)\] \[acados_nmpc_node\]: \[attach-window\] t=[\d.-]+s pos_err=[\d.]+m '
                r'T=[\d.-]+N z=[\d.-]+ m_est=([\d.]+)')


def rd(kind, st):
    return open(os.path.join(D, f'grip_{kind}_{st}.log'), errors='ignore').read()


def ts(txt, pat):
    m = re.search(r'\[(\d+\.\d+)\] \[acados_nmpc_node\]: [^\n]*' + pat, txt)
    return float(m.group(1)) if m else None


def attached(st):
    p = os.path.join(D, f'grip_proximity_{st}.log')
    return os.path.exists(p) and 'attach offset' in open(p, errors='ignore').read()


def cohort():
    for man, wids in (('mainline_ab3d_manifest.csv', ('W3', 'W4')), ('mainline_ab3b_manifest.csv', ('W5',))):
        for r in csv.DictReader(open(os.path.join(D, man))):
            if r['arm'] != 'B' or r['wid'] not in wids:
                continue
            v = r['validity'].strip('"')
            if v == 'valid' or ('attach-fail' in v and attached(r['stamp'])):
                yield r['wid'], r['stamp']


def main():
    lab = {'W3': '0.15 kg 4 m/s', 'W4': '0.30 kg 4 m/s', 'W5': '0.15 kg 2 m/s'}
    rows = list(cohort())
    print('== 1) figure-8 执行器占用(B 臂有效飞行) ==')
    for w in ('W5', 'W3', 'W4'):
        acc = []
        for wid, st in rows:
            if wid != w:
                continue
            n = rd('nmpc', st)
            tf, td = ts(n, r'\| DYNAMIC: switch'), ts(n, r'\| DROP command issued')
            if tf and td:
                acc += [tuple(map(float, x)) for x in FD.findall(n) if tf + 10 < float(x[0]) < td]
        a = np.array(acc)
        print(f'  {w} {lab[w]}: 窗口 {len(a)} | T 占用中位 {np.median(a[:, 2]):.2f} | 出现饱和的窗口比例 '
              f'T {np.mean(a[:, 6] > 0):.0%} roll {np.mean(a[:, 7] > 0):.0%} pitch {np.mean(a[:, 8] > 0):.0%} '
              f'yaw {np.mean(a[:, 9] > 0):.0%} | body-rate scale 中位 {np.median(a[:, 1]):.2f}')
    print('== 2) 悬停推力 / Tmax ==')
    for mp in (0.15, 0.30):
        print(f'  m_P={mp}: {(M_B + mp) * G:.2f} N = {(M_B + mp) * G / TMAX:.0%}')
    print('== 3) 未识别载荷(物理挂上、drop 前 20 s 模型质量中位 < 2.10 kg) ==')
    for wid, st in rows:
        n = rd('nmpc', st)
        tc = ts(n, 'DROP command issued')
        if not tc:
            continue
        pre = [float(m) for t, m in AW.findall(n) if tc - 20 < float(t) < tc]
        if not pre or np.median(pre) >= 2.10:
            continue
        m = rd('mhe', st)
        ra = [float(x) for x in re.findall(r'\[(\d+\.\d+)\] \[mhe_node\]: re-anchored', m)]
        tl = ts(n, r'\| LIFT')
        print(f'  {wid} {st}: 模型质量中位 {np.median(pre):.3f} | 重锚 {len(ra)} 次, 首次相对 LIFT '
              f'{(ra[0] - tl) if ra and tl else float("nan"):+.1f} s | 完成 {int("DROP complete" in n)}')
    print('== 4) 释放后坠机 20260914_185509 ==')
    n, m = rd('nmpc', '20260914_185509'), rd('mhe', '20260914_185509')
    tc = ts(n, 'DROP command issued')
    ages = [float(t) for tt, t in re.findall(r'\[(\d+\.\d+)\] \[mhe_node\]: [^\n]*估计不健康[^\n]*age=([\d.]+)s', m)
            if float(tt) < tc]
    sat = [float(x[0]) - tc for x in FD.findall(n)
           if float(x[0]) > tc and all(float(x[k]) >= 1.0 for k in (2, 3, 4, 5))]
    ra = len(re.findall(r'\[mhe_node\]: re-anchored', m))
    print(f'  drop 前最长不健康 age {max(ages):.1f} s | 重锚 {ra} 次 | 释放后首个四通道全饱和诊断窗 +{sat[0]:.2f} s')
    print('== 5) W4 完成飞行: DROP 指令后模型质量 < 2.15 kg 的用时 ==')
    lat = []
    for wid, st in rows:
        if wid != 'W4':
            continue
        n = rd('nmpc', st)
        if 'DROP complete' not in n:
            continue
        tc = ts(n, 'DROP command issued')
        hit = [float(t) - tc for t, v in AW.findall(n) if float(t) > tc and float(v) < 2.15]
        if hit:
            lat.append(hit[0])
    print(f'  n={len(lat)} 中位 {np.median(lat):.2f} s 最大 {max(lat):.2f} s')


if __name__ == '__main__':
    main()
