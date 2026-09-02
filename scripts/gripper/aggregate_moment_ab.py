#!/usr/bin/env python3
"""一阶质量矩增广 A/B 的聚合(2026-09-02,配 run_moment_ab_4ms.sh)。

主判据:drop 后窗口内 m̂ 的**触界占空比** —— m̂ ≤ m_min+1e-4 的采样比例。
凡触界样本都是 lbx 硬约束的截断值(只说明"无约束解更低"),所以:
  · 它们计入触界占空比与最长连续触界时长(那正是要量的东西);
  · **不计入** drop 后偏差的中位数 —— 混进去等于拿约束值冒充估计值。
偏差中位数因此附带报"有效样本占比",低于 50% 的轮次标 †,单独读。

段落切分一律用 NMPC 日志的墙钟(ATTACH / DROP 行),不用 m_true 标签
(drop 后不刷新,踩过 —— 见 aggregate_cxy_mass_repeat.py)。
"""
import os
import re
import sys
import statistics as st

RES = '/home/clear/ros2_ws_HJH/nmpc_test_results'
M_B = 2.0643
DROP_WIN = 30.0          # drop 后主判据窗口 [s]
LOAD_SKIP = 6.0          # attach 之后跳过的收敛期 [s]
# 带载真值 = M_B + M_P。⚠️ 2026-09-02 修:原先硬编码 0.15,跑 0.3kg 批时带载偏差
# 会被整体算高 6.8pp。现在 main() 从 manifest 头的 "mass=" 解析后覆盖本值。
M_P = 0.15


def wall(line):
    m = re.search(r'\[(\d+\.\d+)\]', line)
    return float(m.group(1)) if m else None


def one(stamp, m_p=None):
    m_p = M_P if m_p is None else m_p
    n = os.path.join(RES, f'grip_nmpc_{stamp}.log')
    m = os.path.join(RES, f'grip_mhe_{stamp}.log')
    if not (os.path.exists(n) and os.path.exists(m)):
        return None
    attach_w = drop_w = fig8_w = None
    sf = 0
    pos_err = []
    lift_err = []      # LIFT 段(attach → attach+20s):s 尚未被 figure8 激励充分
                       # 辨识出来的窗口,c_xy=s/m_T 若偏小 NMPC 会配平不足
    for line in open(n, errors='ignore'):
        if attach_w is None and re.search(r't=[\d.]+s \| ATTACH', line):
            attach_w = wall(line)
        if drop_w is None and 'DROP: released' in line:
            drop_w = wall(line)
        if fig8_w is None and 'DYNAMIC: switch to figure8' in line:
            fig8_w = wall(line)
        if 'solve failed' in line:
            sf += 1
        g = re.search(r'pos_err=([\d.]+)', line)
        if g:
            w = wall(line)
            if fig8_w is not None and drop_w is None:
                pos_err.append(float(g.group(1)))   # figure8 段、drop 之前
            if (attach_w is not None and w is not None
                    and attach_w <= w <= attach_w + 20.0):
                lift_err.append(float(g.group(1)))
    if attach_w is None or drop_w is None:
        return None

    # m_min 从 MHE 启动行读,不写死(MHE_M_MIN 可被环境变量改)
    m_min = 1.961
    load, after = [], []          # (t, m_hat)
    s_after, s_load = [], []      # |s| 与 (|s_hat|,|s_true|) 带载段
    for line in open(m, errors='ignore'):
        g = re.search(r'm_min=([\d.]+)', line)
        if g:
            m_min = float(g.group(1))
        g = re.search(r'\[truth\] m_hat=([\d.]+)', line)
        if not g:
            continue
        w, mh = wall(line), float(g.group(1))
        if w is None:
            continue
        if attach_w + LOAD_SKIP <= w <= drop_w - 1.0:
            load.append(mh)
            gs = re.search(r's_hat=\[([+-][\d.]+),([+-][\d.]+)\] '
                           r's_true=\[([+-][\d.]+),([+-][\d.]+)\]', line)
            if gs:
                v = [float(gs.group(i)) for i in (1, 2, 3, 4)]
                s_load.append(((v[0]**2 + v[1]**2) ** 0.5,
                               (v[2]**2 + v[3]**2) ** 0.5))
        elif drop_w < w <= drop_w + DROP_WIN:
            after.append((w - drop_w, mh))
            gs = re.search(r's_hat=\[([+-][\d.]+),([+-][\d.]+)\]', line)
            if gs:
                s_after.append((w - drop_w,
                                abs(float(gs.group(1))) + abs(float(gs.group(2)))))
    if not after:
        return None

    pinned = [mh <= m_min + 1e-4 for _, mh in after]
    # 最长连续触界时长:按采样时刻算,不假设固定周期
    longest = cur = 0.0
    t_prev = None
    for (t, _), p in zip(after, pinned):
        if p and t_prev is not None:
            cur += t - t_prev
            longest = max(longest, cur)
        elif not p:
            cur = 0.0
        t_prev = t
    free = [mh for (_, mh), p in zip(after, pinned) if not p]
    return dict(
        stamp=stamp,
        pin_frac=100.0 * sum(pinned) / len(pinned),
        pin_longest=longest,
        n_after=len(after),
        free_frac=100.0 * len(free) / len(after),
        bias_after=(100.0 * (st.median(free) - M_B) / M_B) if free else float('nan'),
        bias_load=(100.0 * (st.median(load) - (M_B + m_p)) / (M_B + m_p))
        if load else float('nan'),
        p50=st.median(pos_err) if pos_err else float('nan'),
        lift_peak=max(lift_err) if lift_err else float('nan'),
        p90=(sorted(pos_err)[int(0.9 * len(pos_err))] if pos_err else float('nan')),
        sf=sf,
        s_end=(st.median([v for t, v in s_after if t > DROP_WIN - 10])
               if any(t > DROP_WIN - 10 for t, v in s_after) else float('nan')),
        s_load_hat=st.median([a for a, _ in s_load]) if s_load else float('nan'),
        s_load_true=st.median([b for _, b in s_load]) if s_load else float('nan'),
    )


def paired(a, b, label, unit=''):
    """配对差 + 符号检验(精确二项)。Wilcoxon 在大量并列 0 时退化,所以两个都报。"""
    d = [x - y for x, y in zip(a, b)]
    nz = [x for x in d if abs(x) > 1e-9]
    pos = sum(1 for x in nz if x > 0)
    n = len(nz)
    from math import comb
    p = (sum(comb(n, k) for k in range(0, min(pos, n - pos) + 1)) * 2 / 2 ** n
         if n else float('nan'))
    p = min(p, 1.0)
    try:
        from scipy.stats import wilcoxon
        pw = wilcoxon(d).pvalue if n else float('nan')
    except Exception:
        pw = float('nan')
    print(f'  {label:22s} cur={st.median(a):8.3f}{unit}  mom={st.median(b):8.3f}{unit}'
          f'  Δ中位={st.median(d):+8.3f}  同向 {max(pos, n-pos)}/{n}'
          f'  符号p={p:.4f}  wilcoxon p={pw:.4f}')


def main(man):
    rows = {'cur': [], 'mom': []}
    bad = []
    m_p = M_P
    win = DROP_WIN
    for line in open(man):
        if line.startswith('#'):
            g = re.search(r'mass=([\d.]+)', line)
            if g:
                m_p = float(g.group(1))
            g = re.search(r'recover=([\d.]+)', line)
            if g:
                globals()['DROP_WIN'] = win = float(g.group(1))
            continue
        if line.startswith('#') or not line.strip():
            continue
        f = line.split()
        arm, rep, stamp, status = f[0], f[1], f[2], f[3]
        if status != 'ok':
            bad.append((arm, rep, status))
            continue
        r = one(stamp, m_p)
        if r is None:
            bad.append((arm, rep, 'PARSE_FAIL'))
            continue
        r['rep'] = int(rep)
        rows[arm].append(r)

    print(f'=== {os.path.basename(man)} ===')
    print(f'    载荷真值 m_P={m_p}kg (带载真值 {M_B + m_p:.4f}kg), drop 后窗口 {win:.0f}s')
    if bad:
        print(f'⚠️ 前置硬条件不满足({len(bad)} 轮): {bad}')
        print('   判定规则 ① 是一票否决 —— 下面的数字只能当诊断读,不能下结论。')
    for arm in ('cur', 'mom'):
        print(f'\n--- {arm} (n={len(rows[arm])}) ---')
        print(f'{"rep":>4} {"stamp":>16} {"触界%":>7} {"最长s":>6} {"有效%":>6}'
              f' {"drop后偏差%":>11} {"带载偏差%":>10} {"fig8p50":>8} {"failed":>7}')
        for r in sorted(rows[arm], key=lambda x: x['rep']):
            mark = '†' if r['free_frac'] < 50 else ' '
            print(f'{r["rep"]:>4} {r["stamp"]:>16} {r["pin_frac"]:>7.1f}'
                  f' {r["pin_longest"]:>6.1f} {r["free_frac"]:>6.1f}'
                  f' {r["bias_after"]:>10.2f}{mark} {r["bias_load"]:>10.2f}'
                  f' {r["p50"]:>8.3f} {r["sf"]:>7d}')

    ca = {r['rep']: r for r in rows['cur']}
    ma = {r['rep']: r for r in rows['mom']}
    reps = sorted(set(ca) & set(ma))
    print(f'\n=== 配对比较 (n={len(reps)} 对) ===')
    if not reps:
        return
    for key, lab, unit in (('pin_frac', '触界占空比', '%'),
                           ('pin_longest', '最长连续触界', 's'),
                           ('bias_after', 'drop后偏差(有效样本)', '%'),
                           ('bias_load', '带载稳态偏差', '%'),
                           ('p50', 'figure8 pos_err p50', 'm'),
                           ('lift_peak', 'LIFT段 pos_err 峰值', 'm'),
                           ('sf', 'solve failed', '')):
        a = [ca[r][key] for r in reps]
        b = [ma[r][key] for r in reps]
        if any(x != x for x in a + b):      # NaN 就跳过,别静默出错数
            print(f'  {lab:22s} 含 NaN,跳过')
            continue
        paired(a, b, lab, unit)
    s_end = [ma[r]['s_end'] for r in reps if ma[r]['s_end'] == ma[r]['s_end']]
    if s_end:
        lh = [ma[r]['s_load_hat'] for r in reps if ma[r]['s_load_hat'] == ma[r]['s_load_hat']]
        lt = [ma[r]['s_load_true'] for r in reps if ma[r]['s_load_true'] == ma[r]['s_load_true']]
        print(f'  mom 臂 |s|: 带载段 est={st.median(lh):.5f} true={st.median(lt):.5f} kg·m'
              f'  ->  drop 后 20~30s = {st.median(s_end):.5f} kg·m'
              f' (衰减 {100*(1-st.median(s_end)/st.median(lt)):.0f}%)'
              if lh and lt else
              f'  mom 臂 |s| drop 后 20~30s 中位 = {st.median(s_end):.5f} kg·m')
    print('\n措辞纪律:CI/符号检验不显著只能写"未检出差异",不能写"等价"。'
          '† 行有效样本 <50%,其偏差中位数不可单独引用。')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else
         max((os.path.join(RES, f) for f in os.listdir(RES)
              if f.startswith('moment_ab_4ms_')), key=os.path.getmtime))
