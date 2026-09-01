#!/usr/bin/env python3
"""质量域卸载判据重复批次的聚合(2026-09-01,配 run_cxy_mass_repeat.sh)。

两个指标:
  误触发   释放时刻早于 drop 即为误触发(修复前实测在 drop-71.5s)。
           纯计数功效有限,附 rule of three 的 95% 置信上界。
  武装时刻 相对 attach 的秒数 —— 机制指标。修复前(单点冲高即武装)集中在
           attach+1.5~2.7s(早于 m_est 收敛),修复后应整齐落到收敛之后。
           两组零重叠时用 Mann-Whitney U 报秩和检验。

⚠️ 段落切分一律用 NMPC 的 `| DROP` 墙钟,不用日志里的 m_true 标签(drop 后不刷新)。
"""
import os
import re
import sys
import statistics as st

RES = '/home/clear/ros2_ws_HJH/nmpc_test_results'
M_B, M_CARRY_TRUE = 2.0643, 2.2143

# 武装门控修复**之前**的 8 轮(run_dj_ratchet_ab 第一批),用于对照
BEFORE_FIX = ['20260901_112525', '20260901_112854', '20260901_113218',
              '20260901_113547', '20260901_113915', '20260901_114239',
              '20260901_114603', '20260901_114932']
# 修复**之后**已有的 6 轮(run_dj_ratchet_ab 第二批)
AFTER_FIX_PRIOR = ['20260901_120638', '20260901_121002', '20260901_121326',
                   '20260901_121650', '20260901_122019', '20260901_122343']


def wall(line):
    m = re.search(r'\[(\d+\.\d+)\]', line)
    return float(m.group(1)) if m else None


def one(stamp):
    n = os.path.join(RES, f'grip_nmpc_{stamp}.log')
    m = os.path.join(RES, f'grip_mhe_{stamp}.log')
    if not (os.path.exists(n) and os.path.exists(m)):
        return None
    attach_w = drop_w = None
    sf = 0
    for line in open(n):
        if attach_w is None and re.search(r't=[\d.]+s \| ATTACH', line):
            attach_w = wall(line)
        if drop_w is None and 'DROP: released' in line:
            drop_w = wall(line)
        if 'solve failed' in line:
            sf += 1
    if attach_w is None or drop_w is None:
        return None
    arm = rel = None
    mh = []
    for line in open(m):
        if arm is None and '已武装' in line:
            arm = wall(line) - attach_w
        if rel is None and '载荷释放' in line:
            rel = wall(line) - drop_w
        g = re.search(r'\[truth\] m_hat=([\d.]+)', line)
        if g:
            w = wall(line)
            if attach_w + 6 <= w <= drop_w - 1:
                mh.append(float(g.group(1)))
    return dict(stamp=stamp, arm=arm, rel=rel, sf=sf,
                bias=(100 * (st.median(mh) - M_CARRY_TRUE) / M_CARRY_TRUE) if mh else float('nan'),
                false_trip=(rel is not None and rel < -5.0),
                no_release=(rel is None))


def mannwhitney(a, b):
    """小样本 Mann-Whitney U + 正态近似 p(双侧)。零重叠时给出精确的最小 p。"""
    import math
    n1, n2 = len(a), len(b)
    if n1 == 0 or n2 == 0:
        return None, None
    allv = sorted([(v, 0) for v in a] + [(v, 1) for v in b])
    ranks = {}
    i = 0
    while i < len(allv):
        j = i
        while j + 1 < len(allv) and allv[j + 1][0] == allv[i][0]:
            j += 1
        r = (i + j) / 2 + 1
        for k in range(i, j + 1):
            ranks.setdefault(k, r)
        i = j + 1
    r1 = sum(ranks[k] for k, (_, g) in enumerate(allv) if g == 0)
    u1 = r1 - n1 * (n1 + 1) / 2
    u = min(u1, n1 * n2 - u1)
    mu = n1 * n2 / 2
    sd = math.sqrt(n1 * n2 * (n1 + n2 + 1) / 12)
    z = (u - mu) / sd if sd else 0.0
    p = 2 * (1 - 0.5 * (1 + math.erf(abs(z) / math.sqrt(2))))
    return u, p


def block(name, stamps):
    rows = [r for r in (one(s) for s in stamps) if r]
    if not rows:
        return rows
    print(f'\n=== {name}（n={len(rows)}）===')
    print(f'{"stamp":<17}{"武装 attach+":>13}{"释放 drop":>12}{"m_bias":>9}{"sf":>5}  判定')
    for r in rows:
        a = f'{r["arm"]:.1f}s' if r['arm'] is not None else '未武装'
        rl = f'{r["rel"]:+.2f}s' if r['rel'] is not None else '未释放'
        verdict = '⚠️ 误触发' if r['false_trip'] else ('⚠️ 漏释放' if r['no_release'] else 'ok')
        print(f'{r["stamp"]:<17}{a:>13}{rl:>12}{r["bias"]:>+8.2f}%{r["sf"]:>5}  {verdict}')
    return rows


def main():
    man = os.path.join(RES, 'cxy_mass_repeat_manifest.csv')
    new = []
    if os.path.exists(man):
        for line in open(man):
            if line.startswith('idx') or ',ok' not in line:
                continue
            new.append(line.split(',')[1])

    before = block('武装门控修复前', BEFORE_FIX)
    after = block('修复后（已有 6 轮 + 本批）', AFTER_FIX_PRIOR + new)

    print('\n' + '=' * 66)
    for nm, rows in (('修复前', before), ('修复后', after)):
        if not rows:
            continue
        ft = sum(r['false_trip'] for r in rows)
        nr = sum(r['no_release'] for r in rows)
        n = len(rows)
        line = f'{nm}: 误触发 {ft}/{n}'
        if ft == 0:
            line += f'  → rule of three 95% 上界 {300/n:.1f}%'
        else:
            line += f' = {100*ft/n:.1f}%'
        if nr:
            line += f'   漏释放 {nr}/{n}'
        print(line)
        ok = [r['rel'] for r in rows if r['rel'] is not None and not r['false_trip']]
        if ok:
            print(f'      正常释放延迟 中位 {st.median(ok):+.2f}s  范围 [{min(ok):+.2f}, {max(ok):+.2f}]')

    a = [r['arm'] for r in before if r['arm'] is not None]
    b = [r['arm'] for r in after if r['arm'] is not None]
    if a and b:
        print(f'\n武装时刻(相对 attach):')
        print(f'  修复前 n={len(a)} 中位 {st.median(a):.1f}s  范围 [{min(a):.1f}, {max(a):.1f}]')
        print(f'  修复后 n={len(b)} 中位 {st.median(b):.1f}s  范围 [{min(b):.1f}, {max(b):.1f}]')
        overlap = max(a) >= min(b)
        print(f'  两组区间{"有重叠" if overlap else "零重叠"}')
        u, p = mannwhitney(a, b)
        print(f'  Mann-Whitney U={u:.1f}  p={p:.4g}（正态近似，小样本仅供参考）')


if __name__ == '__main__':
    sys.exit(main())
