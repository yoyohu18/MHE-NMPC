#!/usr/bin/env python3
"""MHE 窗口长度 x 速度 析因扫描的聚合(2026-08-18),配 run_mhe_window_scan.sh。

判别三个竞争假设(见驱动脚本文件头):把每格的 m_est 相对偏差摆进 (速度 x N) 表,
看哪一种切法的组内变异最小:
  - 沿**等占比对角线**最小 → 窗口占轨迹周期比是决定量(m 是窗口内唯一自由度,
    窗口跨越的轨迹弧长越长,模型失配被 m 吸收得越多)
  - 沿**同列**(同 N)最小 → 纯 N 效应,含"测量项/到达代价权重比随 N 变"的混淆
  - 沿**同行**(同速度)最小 → 与窗口无关,指向 kd 线性阻力或 odom 31Hz 姿态滞后

⚠️ 只统计**带载 figure8 稳态段**(t >= t_dyn + ramp + settle),drop 全程关闭。
⚠️ 触界(m_est <= m_min+0.5g)的格子单独标注:撞下界的估计是被截断值,不能当偏差用
   (08-18 踩过——1.961 恰好 = 0.95*m_b 又 ≈ cos(18.1°)*m_b,数值巧合极易误判)。

用法: python3 aggregate_mhe_window.py <manifest.txt>
"""
import re
import sys
from pathlib import Path

import numpy as np

RUNDIR = Path('/home/clear/ros2_ws_HJH/nmpc_test_results')
M_EMPTY, M_PAYLOAD = 2.0643, 0.3
M_TRUE = M_EMPTY + M_PAYLOAD          # 2.364 kg,带载真值
M_MIN = 1.961                          # MHE 下界地板 = 0.95*m_b

AW = re.compile(r'\[attach-window\] t=([\d.]+)s pos_err=([\d.]+)m T=([\d.]+)N '
                r'z=(-?[\d.]+) m_est=([\d.]+)')
DYN = re.compile(r't=([\d.]+)s \| DYNAMIC: switch to figure8')
# 防呆:本实验 drop 应全程关闭,但万一某轮误开,drop 后是**空载**数据,混进带载
# 统计会把偏差算错(且 drop 后常撞下界)。检测到就截断到 drop 之前并告警。
DROP = re.compile(r't=([\d.]+)s \| DROP: released')


def one_cell(stamp, ramp, settle):
    log = RUNDIR / f'grip_nmpc_{stamp}.log'
    if not log.exists():
        return None
    txt = log.read_text(errors='ignore')
    d = DYN.search(txt)
    r = np.array([[float(g) for g in m.groups()] for m in AW.finditer(txt)])
    if d is None or not len(r):
        return None
    t_dyn = float(d.group(1))
    t, pe, T, z, me = r.T
    sel = t >= (t_dyn + ramp + settle)
    dr = DROP.search(txt)
    dropped = dr is not None
    if dropped:                      # 只保留 drop 之前的带载段
        sel &= t < float(dr.group(1))
    if sel.sum() < 20:
        return None
    m = me[sel]
    q1, q3 = np.percentile(m, [25, 75])
    return dict(dropped=dropped,
                n=int(sel.sum()), med=float(np.median(m)), iqr=float(q3 - q1),
                mmin=float(m.min()),
                bias_pct=float((np.median(m) / M_TRUE - 1) * 100),
                pos_err=float(np.median(pe[sel])),
                clipped=bool(m.min() <= M_MIN + 0.0005),
                failed=txt.count('solve failed'))


def spread(vals):
    """组内极差,用来比较三种切法哪个把数据解释得最一致。"""
    vals = [v for v in vals if v is not None]
    return (max(vals) - min(vals)) if len(vals) >= 2 else float('nan')


def main(man):
    rows = []
    for line in Path(man).read_text().splitlines():
        if line.startswith('#') or not line.strip():
            continue
        f = line.split()
        if len(f) < 8 or f[7] != 'ok':
            print(f'  跳过(status={f[7] if len(f)>7 else "?"}): {line}')
            continue
        v, w, N, ramp, settle, laps, stamp = (float(f[0]), float(f[1]), int(f[2]),
                                              float(f[3]), float(f[4]), int(f[5]), f[6])
        c = one_cell(stamp, ramp, settle)
        if c is None:
            print(f'  跳过(日志无有效稳态段): v={v} N={N} {stamp}')
            continue
        period = 2 * np.pi / w
        c.update(v=v, N=N, w=w, period=period, ratio=100 * N * 0.1 / period,
                 stamp=stamp)
        rows.append(c)
    if not rows:
        print('没有可用数据'); return

    print(f'\n真值 {M_TRUE:.3f}kg (空机 {M_EMPTY} + 载荷 {M_PAYLOAD});'
          f' 下界 {M_MIN}\n')
    print(f'{"v":>4} {"N":>4} {"窗口":>6} {"周期":>7} {"占比":>7} '
          f'{"m_est中位":>10} {"偏差%":>8} {"IQR":>7} {"最小":>7} '
          f'{"pos_err":>8} {"n":>5} {"fail":>5}')
    for r in sorted(rows, key=lambda x: (x['v'], x['N'])):
        flag = ' ⚠️触界' if r['clipped'] else ''
        if r.get('dropped'):
            flag += ' ⚠️该轮开了drop(已截断到drop前)'
        print(f'{r["v"]:>4.0f} {r["N"]:>4d} {r["N"]*0.1:>5.1f}s '
              f'{r["period"]:>6.1f}s {r["ratio"]:>6.1f}% '
              f'{r["med"]:>10.3f} {r["bias_pct"]:>+7.2f}% {r["iqr"]:>7.3f} '
              f'{r["mmin"]:>7.3f} {r["pos_err"]:>8.3f} {r["n"]:>5d} '
              f'{r["failed"]:>5d}{flag}')

    ok = [r for r in rows if not r['clipped']]
    if len(ok) < len(rows):
        print(f'\n⚠️ {len(rows)-len(ok)} 格触界,已排除出假设判别'
              f'(截断值不能当偏差用)')
    if len(ok) < 4:
        print('\n可用格数 < 4,不做假设判别'); return

    def bias(v, N):
        for r in ok:
            if r['v'] == v and r['N'] == N:
                return r['bias_pct']
        return None

    vs = sorted({r['v'] for r in ok}); Ns = sorted({r['N'] for r in ok})
    same_v = [spread([bias(v, N) for N in Ns]) for v in vs]          # 同行
    same_N = [spread([bias(v, N) for v in vs]) for N in Ns]          # 同列
    # 等占比对角线:v 翻倍则周期减半,同占比要求 N 也减半
    diags = []
    for N in Ns:
        if N * 2 in Ns and len(vs) >= 2:
            diags.append(spread([bias(vs[1], N), bias(vs[0], N * 2)]))

    print('\n=== 假设判别(组内极差,越小=该切法解释得越一致) ===')
    print(f'  同行(同速度,扫 N)   极差: '
          f'{"  ".join(f"{s:.2f}" for s in same_v)}   -> 支持"与窗口无关"(kd/姿态滞后)')
    print(f'  同列(同 N,扫速度)   极差: '
          f'{"  ".join(f"{s:.2f}" for s in same_N)}   -> 支持"纯 N 效应"')
    if diags:
        print(f'  等占比对角线        极差: '
              f'{"  ".join(f"{s:.2f}" for s in diags)}   -> 支持"窗口占周期比"')
    else:
        print('  等占比对角线: 数据不足(需要 N 与 2N 同时在列表里,且两档速度成 2 倍)')
    print('\n⚠️ n=1/格是探边界,不是定案。判别结论要按 SITL 纪律复到 n>=8 才能写论文。')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    main(sys.argv[1])
