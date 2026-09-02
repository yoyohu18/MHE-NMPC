#!/usr/bin/env python3
"""dJ 先验去除 A/B 的聚合(2026-08-25),配 run_prior_ab_4ms.sh。

⚠️ **这是非劣性检验,不是优效性检验。** 目标是"去掉先验且不付代价",prior=0
不需要更好、只需要不显著更差。所以 p>0.05 **不等于**"没差别 = 成功"——p 大也
可能只是功效不足。判定必须靠预设的等效边界 δ,不能靠 p 值。

判定规则(与驱动脚本文件头一致,跑前钉死):
  ① 前置硬条件(一票否决):全部 ok、无发散(pos_err<2m)、无触界。
     "能飞"是前提,只要有一轮发散,"去掉先验"就不能写进论文,bias 再好也没用。
  ② 主判据:d = |bias|(prior0) − |bias|(prior0.3) 的 **95% CI 上界 < δ=+1.0pp**
     → 非劣。δ 的依据:08-24 λ=1.0 臂 n=8 的 bias 范围 −2.21~−3.40 跨度 1.19pp,
     批内噪声本来就这么大,小于这个尺度的差异没有物理意义。
  ③ 次要(只描述不判定):pos_err、iqr、solve failed。

用法: python3 aggregate_prior_ab.py <manifest.txt>
"""
import re
import sys
from pathlib import Path

import numpy as np

RUNDIR = Path('/home/clear/ros2_ws_HJH/nmpc_test_results')
M_TRUE = 2.0643 + 0.3
M_MIN = 1.961
AW = re.compile(r'\[attach-window\] t=([\d.]+)s pos_err=([\d.]+)m T=([\d.]+)N '
                r'z=(-?[\d.]+) m_est=([\d.]+)')
DYN = re.compile(r't=([\d.]+)s \| DYNAMIC: switch to figure8')
DROP = re.compile(r't=([\d.]+)s \| DROP: released')


def one_cell(stamp, ramp, settle):
    """提取带载稳态段；固定在本检验中，避免依赖已退役的窗口扫描聚合器。"""
    log = RUNDIR / f'grip_nmpc_{stamp}.log'
    if not log.exists():
        return None
    txt = log.read_text(errors='ignore')
    dyn = DYN.search(txt)
    rows = np.array([[float(g) for g in m.groups()] for m in AW.finditer(txt)])
    if dyn is None or not len(rows):
        return None
    t, pe, _, _, me = rows.T
    selected = t >= (float(dyn.group(1)) + ramp + settle)
    dropped = DROP.search(txt)
    if dropped is not None:
        selected &= t < float(dropped.group(1))
    if selected.sum() < 20:
        return None
    mass = me[selected]
    q1, q3 = np.percentile(mass, [25, 75])
    return dict(dropped=dropped is not None,
                n=int(selected.sum()), med=float(np.median(mass)),
                iqr=float(q3 - q1), mmin=float(mass.min()),
                bias_pct=float((np.median(mass) / M_TRUE - 1) * 100),
                pos_err=float(np.median(pe[selected])),
                clipped=bool(mass.min() <= M_MIN + 0.0005),
                failed=txt.count('solve failed'))

DELTA = 1.0          # 等效边界 [百分点]
# 默认臂名 = run_prior_ab_4ms.sh 的两臂;--base/--test 可覆盖,供同结构的
# run_mhe_prior_ab_4ms.sh(mhe0.3 vs mhe0.5)复用同一套判定与措辞纪律。
BASE, TEST = 'prior0.3', 'prior0.0'


def main(man, base=None, test=None):
    global BASE, TEST
    if base:
        BASE = base
    if test:
        TEST = test
    rows, hard_fail = [], []
    for ln in Path(man).read_text().splitlines():
        if ln.startswith('#') or not ln.strip():
            continue
        f = ln.split()
        if len(f) < 9:
            continue
        arm, rep, stamp, status, peak = f[0], int(f[1]), f[6], f[7], float(f[8])
        if status != 'ok' or peak > 2.0:
            hard_fail.append(f'{arm} rep{rep}: status={status} peak={peak}')
            continue
        c = one_cell(stamp, float(f[3]), float(f[4]))
        if c is None:
            hard_fail.append(f'{arm} rep{rep} {stamp}: 稳态段样本<20')
            continue
        c.update(arm=arm, rep=rep, peak=peak, abs_bias=abs(c['bias_pct']))
        if c['clipped']:
            hard_fail.append(f'{arm} rep{rep}: m_est 触下界')
        rows.append(c)

    if not rows:
        print('没有可用数据。'); return
    print(f'\n带载真值 m_true = {M_TRUE:.4f} kg   有效轮次 {len(rows)}\n')
    hdr = (f'{"arm":>9} {"rep":>4} {"n":>5} {"m_med":>8} {"bias%":>8} '
           f'{"|bias|%":>8} {"iqr":>7} {"pos_err":>8} {"peak":>7} {"clip":>5} {"fail":>5}')
    print(hdr); print('-' * len(hdr))
    for r in sorted(rows, key=lambda x: (x['arm'], x['rep'])):
        print(f'{r["arm"]:>9} {r["rep"]:>4} {r["n"]:>5} {r["med"]:>8.3f} '
              f'{r["bias_pct"]:>+7.2f}% {r["abs_bias"]:>7.2f}% {r["iqr"]:>7.3f} '
              f'{r["pos_err"]:>8.3f} {r["peak"]:>7.3f} '
              f'{"YES" if r["clipped"] else "-":>5} {r["failed"]:>5}')

    print('\n--- 每臂汇总(次要指标,只描述) ---')
    for a in (BASE, TEST):
        g = [r for r in rows if r['arm'] == a]
        if not g:
            continue
        ab = [r['abs_bias'] for r in g]
        print(f'{a:>9}  n={len(g)}  |bias| 中位 {np.median(ab):.2f}% '
              f'(min {min(ab):.2f} max {max(ab):.2f})  '
              f'iqr 中位 {np.median([r["iqr"] for r in g]):.3f}  '
              f'pos_err 中位 {np.median([r["pos_err"] for r in g]):.3f}  '
              f'peak 中位 {np.median([r["peak"] for r in g]):.3f}  '
              f'solve_failed 合计 {sum(r["failed"] for r in g)}')

    # ---- ① 前置硬条件 ----
    print('\n=== ① 前置硬条件:全部能飞、无触界 ===')
    if hard_fail:
        print('  **不通过** —— 以下轮次有问题:')
        for h in hard_fail:
            print(f'    - {h}')
        print('  ⇒ "去掉 dJ 先验"**不能**写进论文,bias 再好也没用。')
    else:
        print(f'  通过:{len(rows)} 轮全部 ok、无发散、无触界。')

    # ---- ② 主判据:非劣性 ----
    reps = sorted({r['rep'] for r in rows})
    pairs, d = [], []
    for rp in reps:
        b = [r for r in rows if r['arm'] == BASE and r['rep'] == rp]
        t = [r for r in rows if r['arm'] == TEST and r['rep'] == rp]
        if b and t:
            pairs.append((rp, b[0], t[0]))
            d.append(t[0]['abs_bias'] - b[0]['abs_bias'])
    print(f'\n=== ② 主判据:d = |bias|({TEST}) − |bias|({BASE}),等效边界 δ=+{DELTA}pp ===')
    print(f'{"rep":>5} {BASE:>11} {TEST:>11} {"d":>9}')
    for (rp, b, t), dd in zip(pairs, d):
        print(f'{rp:>5} {b["abs_bias"]:>10.2f}% {t["abs_bias"]:>10.2f}% {dd:>+8.2f}')
    if len(d) < 2:
        print('配对不足,无法判定。'); return
    d = np.array(d)
    n = len(d)
    mean, sd = d.mean(), d.std(ddof=1)
    se = sd / np.sqrt(n)
    try:
        from scipy import stats
        tcrit = stats.t.ppf(0.975, n - 1)
        _, p_two = stats.ttest_rel(
            [t['abs_bias'] for _, _, t in pairs],
            [b['abs_bias'] for _, b, _ in pairs])
    except Exception:
        tcrit, p_two = 2.365, float('nan')
    lo, hi = mean - tcrit * se, mean + tcrit * se
    print(f'\n配对数 n={n}   d 均值 {mean:+.3f}pp   sd {sd:.3f}   '
          f'95% CI [{lo:+.3f}, {hi:+.3f}]')
    print(f'{TEST} 更好(d<0)的 rep: {int((d < 0).sum())}/{n}   '
          f'双边配对 t 检验 p={p_two:.4f}')
    print(f'\n  CI 上界 {hi:+.3f}pp  vs  等效边界 +{DELTA}pp')
    noninf = hi < DELTA
    if noninf and not hard_fail:
        print(f'  ⇒ **非劣成立且全部能飞:{TEST} 可以取代 {BASE}。**')
        if hi < 0:
            print(f'     (CI 完全在 0 以下 —— {TEST} 甚至更好,意外但可报)')
    elif noninf:
        print('  ⇒ 非劣在统计上成立,**但前置硬条件没过**,结论不成立。')
    else:
        print('  ⇒ 证据不足:CI 上界越过等效边界,不能断言先验可去。'
              f'\n     (需要 n ≈ {int(np.ceil((tcrit * sd / DELTA) ** 2)) + 1} 才能把 CI 压进 δ)')
    print('\n措辞纪律:非劣成立说的是"未测出代价",不是"两者等价";'
          '\n          不成立说的是"证据不足",不是"先验必需"。')


if __name__ == '__main__':
    args = sys.argv[1:]
    base = test = None
    pos = []
    while args:
        a = args.pop(0)
        if a == '--base':
            base = args.pop(0)
        elif a == '--test':
            test = args.pop(0)
        else:
            pos.append(a)
    if not pos:
        print(__doc__); sys.exit(1)
    main(pos[0], base, test)
