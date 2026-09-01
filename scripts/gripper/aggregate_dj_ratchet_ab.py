#!/usr/bin/env python3
"""dJ 棘轮 on/off 配对批次的聚合(2026-09-01,配 run_dj_ratchet_ab.sh)。

每轮四个指标:
  dJ_med   带载段 dJ 稳态中位(跳过 attach 后前 2 个样本 = 收敛期),真值 mu*d^2
  m_bias   带载段 m_hat 相对真值的中位偏差 [%]
  rel_dt   c_xy 质量域释放相对 NMPC drop 的延迟 [s](该批残差路径全关,
           所以释放只可能由质量域判据完成)
  floor_n  m_p_model 落在地板上的样本数(棘轮要治的"LIFT 段冻结"的判据)

⚠️ 段落切分一律用 NMPC 的 `| DROP` 墙钟,**不用日志里的 m_true 标签** ——
   那个在 drop 后不刷新(见记忆 cxy-truth-label-defect)。
"""
import re
import sys
import os
import statistics as st

RES = '/home/clear/ros2_ws_HJH/nmpc_test_results'
M_B, M_P_TRUE, ARM_D = 2.0643, 0.15, 0.47
MU = M_B * M_P_TRUE / (M_B + M_P_TRUE)
DJ_TRUE = MU * ARM_D ** 2
M_CARRY_TRUE = M_B + M_P_TRUE


def wall(line):
    m = re.search(r'\[(\d+\.\d+)\]', line)
    return float(m.group(1)) if m else None


def one(stamp):
    n = os.path.join(RES, f'grip_nmpc_{stamp}.log')
    m = os.path.join(RES, f'grip_mhe_{stamp}.log')
    if not (os.path.exists(n) and os.path.exists(m)):
        return None
    attach_w = drop_w = None
    dj, floor_n = [], 0
    solve_failed = 0
    for line in open(n):
        if attach_w is None and re.search(r't=[\d.]+s \| ATTACH', line):
            attach_w = wall(line)
        if drop_w is None and 'DROP: released' in line:
            drop_w = wall(line)
        if 'solve failed' in line:
            solve_failed += 1
        g = re.search(r'\[dJ_online\].*m_p_model=([\d.]+)\((\w+)\) dJ=([\d.]+)', line)
        if g:
            dj.append((wall(line), float(g.group(3))))
            if g.group(2) == 'floor':
                floor_n += 1
    if drop_w is None or attach_w is None:
        return None
    # 带载段 dJ:attach 后跳过前 2 个样本(5s/行 -> 约 10s 收敛期)
    dj_car = [v for w, v in dj if attach_w < w < drop_w][2:]
    mh = []
    for line in open(m):
        g = re.search(r'\[truth\] m_hat=([\d.]+)', line)
        if g:
            w = wall(line)
            if attach_w + 6 <= w <= drop_w - 1:
                mh.append(float(g.group(1)))
    rel = [wall(l) - drop_w for l in open(m) if '载荷释放' in l]
    rel_src = [l.split('载荷释放')[1].split('→')[0].strip()
               for l in open(m) if '载荷释放' in l]
    return dict(
        stamp=stamp,
        dj_med=st.median(dj_car) if dj_car else float('nan'),
        m_bias=(100 * (st.median(mh) - M_CARRY_TRUE) / M_CARRY_TRUE) if mh else float('nan'),
        m_min=min(mh) if mh else float('nan'),
        rel_dt=rel[0] if rel else float('nan'),
        rel_src=rel_src[0] if rel_src else '-',
        floor_n=floor_n, sf=solve_failed, n_dj=len(dj_car))


def main():
    man = os.path.join(RES, 'dj_ratchet_ab_manifest.csv')
    rows = {'true': [], 'false': []}
    print(f'dJ 自身口径真值 = {DJ_TRUE:.4f}   带载真值 m = {M_CARRY_TRUE}\n')
    hdr = (f'{"#":>2} {"棘轮":>5} {"stamp":<16} {"dJ_med":>7} {"dJ误差":>8} '
           f'{"m_bias":>7} {"m_min":>7} {"释放Δt":>7} {"floor":>5} {"sf":>3}')
    print(hdr); print('-' * len(hdr))
    for line in open(man):
        if line.startswith('idx') or ',ok' not in line:
            continue
        idx, arm, stamp = line.split(',')[:3]
        r = one(stamp)
        if not r:
            print(f'{idx:>2} {arm:>5} {stamp:<16}  (日志不全)')
            continue
        rows[arm].append(r)
        print(f'{idx:>2} {arm:>5} {stamp:<16} {r["dj_med"]:7.4f} '
              f'{100*(r["dj_med"]-DJ_TRUE)/DJ_TRUE:+7.1f}% {r["m_bias"]:+6.2f}% '
              f'{r["m_min"]:7.4f} {r["rel_dt"]:+6.2f}s {r["floor_n"]:5d} {r["sf"]:3d}')
    print()
    for arm in ('true', 'false'):
        v = rows[arm]
        if not v:
            continue
        dj = [x['dj_med'] for x in v]
        print(f'棘轮={arm:<5} n={len(v)}  dJ 中位={st.median(dj):.4f} '
              f'({100*(st.median(dj)-DJ_TRUE)/DJ_TRUE:+.1f}%)  '
              f'm_bias 中位={st.median([x["m_bias"] for x in v]):+.2f}%  '
              f'释放Δt 中位={st.median([x["rel_dt"] for x in v]):+.2f}s  '
              f'floor 总={sum(x["floor_n"] for x in v)}  sf 总={sum(x["sf"] for x in v)}')
    a, b = rows['true'], rows['false']
    if a and b:
        k = min(len(a), len(b))
        print(f'\n配对(按出现顺序取前 {k} 对) dJ 误差 on−off:')
        for i in range(k):
            ea = 100 * (a[i]['dj_med'] - DJ_TRUE) / DJ_TRUE
            eb = 100 * (b[i]['dj_med'] - DJ_TRUE) / DJ_TRUE
            print(f'   #{i+1}: on {ea:+7.1f}%  off {eb:+7.1f}%  差 {ea-eb:+7.1f}pp')


if __name__ == '__main__':
    sys.exit(main())
