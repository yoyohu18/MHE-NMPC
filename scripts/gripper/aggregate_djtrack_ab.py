#!/usr/bin/env python3
"""dJ 误差方向 A/B 的聚合(2026-08-31),配 run_djtrack_ab_4ms.sh。

**这是双侧差异检验,不是非劣性检验。** 要回答的是"两种 dJ 给法能不能区分开",
所以 CI 含 0 只能说"在本批分辨率下未检出差异",**不能**说"等价"。

判定(与驱动脚本文件头一致,跑前钉死):
  ① 前置硬条件(一票否决):全部 ok、无发散(peak<2m)、无 TRACEBACK。
  ② 主判据:LIFT 段(attach → attach+20s)pos_err **峰值**的配对差,双侧 95% CI。
     功效:n=8 配对可检出 0.029m(3.7%);批内 SD 取自 geomsrc online 臂 n=7 = 0.0202。
  ③ 次要(只描述不判定):figure-8 段 pos_err 中位数/p90、ω 跟踪误差、solve failed、
     m_est bias、on 臂 dJ 终值散布。

用法: python3 aggregate_djtrack_ab.py <manifest.txt>
"""
import re
import sys
from pathlib import Path

import numpy as np

RUNDIR = Path('/home/clear/ros2_ws_HJH/nmpc_test_results')
LIFT_WIN = 20.0          # LIFT 暂态窗口 [s],与预注册一致
DYN_SKIP = 12.0          # figure-8 跳过 ramp 的秒数
SD_PRIOR = 0.0202        # geomsrc online 臂 n=7 的批内 SD,仅用于功效声明

RE_PE = re.compile(r't=([\d.]+)s \| pos_err=([\d.]+)')
RE_PW = re.compile(r'\[attach-window\] t=([\d.]+)s pos_err=([\d.]+)')
RE_ATT = re.compile(r't=([\d.]+)s \| ATTACH')
RE_DYN = re.compile(r't=([\d.]+)s \| DYNAMIC')
RE_WERR = re.compile(r'\[wcmd\].*err=\[([-+][\d.]+),([-+][\d.]+)\]')
RE_DJ = re.compile(r'\[dJ_online\].*dJ=([\d.]+)')
RE_TRUTH = re.compile(r'\[truth\] m_hat=([\d.]+) m_true=([\d.]+)')


def per_run(stamp):
    """一轮的全部指标。拿不到就返回 None(调用方按前置条件处理)。"""
    n = RUNDIR / f'grip_nmpc_{stamp}.log'
    m = RUNDIR / f'grip_mhe_{stamp}.log'
    if not n.exists():
        return None
    t_att = t_dyn = None
    pts, werr, dj = [], [], []
    for ln in n.open(errors='ignore'):
        if t_att is None and (r := RE_ATT.search(ln)):
            t_att = float(r.group(1))
        if t_dyn is None and (r := RE_DYN.search(ln)):
            t_dyn = float(r.group(1))
        for R in (RE_PE, RE_PW):
            if r := R.search(ln):
                pts.append((float(r.group(1)), float(r.group(2))))
                break
        if r := RE_WERR.search(ln):
            werr.append(abs(float(r.group(1))) + abs(float(r.group(2))))
        if r := RE_DJ.search(ln):
            dj.append(float(r.group(1)))
    if t_att is None or not pts:
        return None
    a = np.array(sorted(pts))
    lift = a[(a[:, 0] > t_att) & (a[:, 0] < t_att + LIFT_WIN)]
    if not len(lift):
        return None
    out = {'lift_peak': float(lift[:, 1].max()),
           'solve_failed': sum(1 for _ in n.open(errors='ignore')
                               if 'solve failed' in _),
           'dj_final': dj[-1] if dj else float('nan'),
           'w_err_med': float(np.median(werr)) if werr else float('nan')}
    if t_dyn is not None:
        fig = a[a[:, 0] > t_dyn + DYN_SKIP]
        if len(fig):
            out['fig_med'] = float(np.median(fig[:, 1]))
            out['fig_p90'] = float(np.percentile(fig[:, 1], 90))
    if m.exists():
        b = [(float(r.group(1)), float(r.group(2)))
             for ln in m.open(errors='ignore') if (r := RE_TRUTH.search(ln))]
        if b:
            b = np.array(b)
            out['m_bias'] = float(100 * np.mean((b[:, 0] - b[:, 1]) / b[:, 1]))
    return out


def main(man):
    rows = []
    for ln in Path(man).read_text().splitlines():
        if ln.startswith('#') or not ln.strip():
            continue
        f = ln.split()
        rows.append({'arm': f[0], 'rep': int(f[1]), 'stamp': f[6],
                     'status': f[7], 'peak': float(f[8])})

    # ---- ① 前置硬条件 ----
    bad = [r for r in rows if r['status'] != 'ok']
    print(f"轮次 {len(rows)}  非 ok {len(bad)}")
    for r in bad:
        print(f"  ✗ {r['arm']} rep{r['rep']} {r['stamp']} {r['status']} peak={r['peak']}")
    if bad:
        print("\n① 前置硬条件**未通过** —— 一票否决。有轮次发散/异常时,"
              "主判据的数字不具解释力,先查发散原因。")

    data = {}
    for r in rows:
        if r['status'] != 'ok':
            continue
        d = per_run(r['stamp'])
        if d:
            data[(r['arm'], r['rep'])] = d

    arms = sorted({a for a, _ in data})
    if len(arms) != 2:
        print(f"臂数不是 2:{arms}"); return
    base, test = ('off', 'on') if 'off' in arms else tuple(arms)

    print(f"\n{'rep':<5}{base+' lift_peak':>16}{test+' lift_peak':>16}{'配对差':>10}")
    pairs = []
    for rep in sorted({p for _, p in data}):
        kb, kt = (base, rep), (test, rep)
        if kb in data and kt in data:
            b, t = data[kb]['lift_peak'], data[kt]['lift_peak']
            pairs.append(t - b)
            print(f"{rep:<5}{b:16.3f}{t:16.3f}{t-b:+10.3f}")
    if len(pairs) < 3:
        print("配对数不足,无法给 CI"); return
    d = np.array(pairs)
    n = len(d)
    se = d.std(ddof=1) / np.sqrt(n)
    # 小样本用 t 分布;没有 scipy 就退回 1.96(n>=8 时差别很小,会在输出里注明)
    try:
        from scipy import stats
        tcrit = float(stats.t.ppf(0.975, n - 1)); crit_src = f't({n-1})'
    except Exception:
        tcrit = 1.96; crit_src = 'z(无 scipy,n 小时略窄)'
    lo, hi = d.mean() - tcrit * se, d.mean() + tcrit * se
    mdd = 2.9 * SD_PRIOR * np.sqrt(2) / np.sqrt(n)

    print(f"\n② 主判据 LIFT 段 pos_err 峰值,配对差({test} − {base}),n={n}")
    print(f"   均值 {d.mean():+.4f} m   95% CI [{lo:+.4f}, {hi:+.4f}]  ({crit_src})")
    print(f"   预设可检出最小差异(80% 功效) {mdd:+.4f} m")
    if lo <= 0 <= hi:
        print(f"   → CI 含 0。**只能写**\"在 {100*mdd/0.783:.1f}% 分辨率下未检出差异\","
              "\n     不能写\"两者等价\"或\"改默认无害\"(措辞纪律见 cem-benefit-refuted)。")
    elif hi < 0:
        print(f"   → {test} 显著更好(峰值更低)。支持保留新默认。")
    else:
        print(f"   → {base} 显著更好 → **回退默认**"
              "(acados_nmpc_node.py 的 dj_track_mest 声明改回 False)。")

    print("\n③ 次要指标(只描述不判定)")
    hdr = ('fig_med', 'fig_p90', 'w_err_med', 'solve_failed', 'm_bias', 'dj_final')
    print(f"{'arm':<6}" + ''.join(f'{h:>14}' for h in hdr))
    for a in (base, test):
        vals = [data[k] for k in data if k[0] == a]
        cells = []
        for h in hdr:
            v = np.array([x.get(h, np.nan) for x in vals], dtype=float)
            v = v[~np.isnan(v)]
            cells.append(f'{v.mean():.4f}' if len(v) else 'n/a')
        print(f"{a:<6}" + ''.join(f'{c:>14}' for c in cells))
    on_dj = [data[k]['dj_final'] for k in data
             if k[0] == test and not np.isnan(data[k]['dj_final'])]
    if on_dj:
        v = np.array(on_dj)
        print(f"\n   {test} 臂 dJ 终值: 均值 {v.mean():.4f} SD {v.std(ddof=1):.4f} "
              f"跨度 {v.max()-v.min():.4f}  (自身口径真值 0.0579)")


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else sys.exit(__doc__))
