#!/usr/bin/env python3
"""遗忘因子 λ 4m/s 配对 A/B 的聚合(2026-08-24),配 run_lambda_ab_4ms.sh。

**主判据(跑前钉死,不许事后换)**:带载 figure8 稳态段 m_est 相对偏差的**绝对值**
`|bias_pct|`。用绝对值而不是带符号值,因为立论的那组对照是 −8.30% → +1.33%,
说的是**偏差幅值变小**而非往某个方向移动;直接比带符号值会把"从 −8.3 抬到 +1.3"
误判成"变大了 9.6 个百分点"。带符号值同时打印,但只作描述。

配对方式:每个 rep 内两臂各一次,按 rep 配对求差 d = |bias|(λ0.8) − |bias|(λ1.0)。
检验:优先 Wilcoxon signed-rank(scipy),无 scipy 时退回精确符号检验(binomial)。
两者都是双边;单边期望是 d<0(λ0.8 更好)。

次要指标(只描述,不判定):iqr(稳态段离散度)、pos_err、solve failed 数、触界率。
⚠️ 触界(m_est <= m_min+0.5g)的轮次 bias 是**截断值**不是估计值,单独标注;
   若任一臂出现触界,主判据的解读要打折扣(08-18 踩过:1.961 恰好 = 0.95*m_b
   又 ≈ cos(18.1°)*m_b,数值巧合极易误判成"倾角效应")。
⚠️ `attach 入带`指标本批**不计算**:±2% 带在 8 字下量的是波动不是收敛(已知坏)。

用法: python3 aggregate_lambda_ab.py <manifest.txt>
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from aggregate_mhe_window import M_TRUE, one_cell  # noqa: E402  复用同一套提取口径


def sign_test(d):
    """精确符号检验(双边)。零差值按惯例剔除。"""
    d = [x for x in d if x != 0]
    n = len(d)
    if n == 0:
        return float('nan'), 0
    k = sum(1 for x in d if x < 0)          # λ0.8 更好的轮数
    tail = min(k, n - k)
    p = 2.0 * sum(math.comb(n, i) for i in range(tail + 1)) / (2 ** n)
    return min(1.0, p), n


def main(man):
    rows = []
    for ln in Path(man).read_text().splitlines():
        if ln.startswith('#') or not ln.strip():
            continue
        f = ln.split()
        if len(f) < 8 or f[6] == '-':
            print(f'  [skip] {ln}')
            continue
        arm, rep, ramp, settle, stamp, status = f[0], int(f[1]), float(f[3]), \
            float(f[4]), f[6], f[7]
        c = one_cell(stamp, ramp, settle)
        if c is None:
            print(f'  [skip] {arm} rep{rep} {stamp}: 稳态段样本 < 20')
            continue
        c.update(arm=arm, rep=rep, status=status, stamp=stamp)
        c['abs_bias'] = abs(c['bias_pct'])
        rows.append(c)

    if not rows:
        print('没有可用数据。'); return
    arms = sorted({r['arm'] for r in rows})
    print(f'\n带载真值 m_true = {M_TRUE:.4f} kg   有效轮次 {len(rows)}\n')
    hdr = (f'{"arm":>8} {"rep":>4} {"n":>5} {"m_med":>8} {"bias%":>8} '
           f'{"|bias|%":>8} {"iqr":>7} {"pos_err":>8} {"clip":>5} {"fail":>5}')
    print(hdr); print('-' * len(hdr))
    for r in sorted(rows, key=lambda x: (x['arm'], x['rep'])):
        print(f'{r["arm"]:>8} {r["rep"]:>4} {r["n"]:>5} {r["med"]:>8.3f} '
              f'{r["bias_pct"]:>+7.2f}% {r["abs_bias"]:>7.2f}% {r["iqr"]:>7.3f} '
              f'{r["pos_err"]:>8.3f} {"YES" if r["clipped"] else "-":>5} '
              f'{r["failed"]:>5}')

    print('\n--- 每臂汇总(次要指标,只描述) ---')
    for a in arms:
        g = [r for r in rows if r['arm'] == a]
        ab = [r['abs_bias'] for r in g]
        print(f'{a:>8}  n={len(g)}  |bias| 中位 {np.median(ab):.2f}% '
              f'(min {min(ab):.2f} max {max(ab):.2f})  '
              f'bias 中位 {np.median([r["bias_pct"] for r in g]):+.2f}%  '
              f'iqr 中位 {np.median([r["iqr"] for r in g]):.3f}  '
              f'触界 {sum(r["clipped"] for r in g)}/{len(g)}  '
              f'solve_failed 中位 {np.median([r["failed"] for r in g]):.0f}')

    if len(arms) != 2:
        print('\n(臂数 != 2,跳过配对检验)'); return
    base, test = 'lam1.0', 'lam0.8'
    if base not in arms or test not in arms:
        base, test = arms[0], arms[1]

    reps = sorted({r['rep'] for r in rows})
    pairs, d = [], []
    for rp in reps:
        b = [r for r in rows if r['arm'] == base and r['rep'] == rp]
        t = [r for r in rows if r['arm'] == test and r['rep'] == rp]
        if not b or not t:
            print(f'  [pair-skip] rep{rp} 缺臂'); continue
        pairs.append((rp, b[0], t[0]))
        d.append(t[0]['abs_bias'] - b[0]['abs_bias'])

    print(f'\n=== 主判据: |bias_pct| 配对差 d = |bias|({test}) - |bias|({base}) ===')
    print(f'{"rep":>5} {base:>12} {test:>12} {"d":>9}')
    for (rp, b, t), dd in zip(pairs, d):
        print(f'{rp:>5} {b["abs_bias"]:>11.2f}% {t["abs_bias"]:>11.2f}% {dd:>+8.2f}')
    if not d:
        print('无完整配对。'); return
    d = np.array(d)
    n_better = int((d < 0).sum())
    print(f'\n配对数 n = {len(d)}   d 中位 {np.median(d):+.2f} 个百分点   '
          f'均值 {d.mean():+.2f}')
    print(f'{test} 更好(d<0)的 rep: {n_better}/{len(d)}')
    try:
        from scipy.stats import wilcoxon
        st, p = wilcoxon(d)
        print(f'Wilcoxon signed-rank (双边): W={st:.1f}, p={p:.4f}')
    except Exception as e:
        print(f'(scipy 不可用: {e})')
    p_s, n_s = sign_test(list(d))
    print(f'精确符号检验 (双边): n={n_s}, p={p_s:.4f}')
    print('\n判定口径:p<0.05 且 d 中位为负 → λ=0.8 收益站得住,进第二批做机制归因'
          '(补 λ=1.0 + MHE_N=10 格,分开"遗忘因子" vs "有效窗口变短")。'
          '\n          否则 → 措辞用"未复现出可测收益",不要写成"效果差"。')
    if any(r['clipped'] for r in rows):
        print('\n⚠️ 有轮次触下界,该轮 bias 是截断值,主判据解读要打折扣。')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    main(sys.argv[1])
