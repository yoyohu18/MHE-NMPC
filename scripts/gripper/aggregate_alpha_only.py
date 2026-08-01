#!/usr/bin/env python3
"""α-only / rhythm-only 消融聚合(2026-07-30):把 θ* 的收益拆到"阈值 α"和"节奏维"。

四臂构成正交 2×2 析因(见 run_alpha_only_ablation.sh):
                 │ 阈值 1.5N     │ 阈值 α=0.9875
    节奏 = M0    │ M0            │ alphaonly
    节奏 = θ*    │ rhythmonly    │ thetastar

同 rep 内四臂背靠背、臂序按拉丁方轮转,故走**同 rep 配对差**,不报跨批绝对值
(07-10 已记录 M0 绝对基线跨时段漂移 ±0.5s;07-15/07-23 两批 M0 阈值口径本就不同)。

统计口径(预先固定,不事后挑):
  · 主指标 = settle(驻留收敛)。enter 仅辅助——smoke 实证 m_est 会在暂态期偶然
    穿带一次即被记为"入带",该指标不稳健(07-23 批 enter/settle 结论分叉的机制)。
  · 配对 Wilcoxon 符号秩(精确)+ 每 rep 差值全列 + 中位数差 bootstrap 95% CI。
  · 主效应各有两条独立估计,并列报告;不合并,以便看出交互。

用法:  python3 aggregate_alpha_only.py [manifest]  (不给取最新 alpha_only_*.txt)"""
import glob
import itertools
import os
import re
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, '/home/clear/ros2_ws_HJH/src/scripts/gripper')
sys.path.insert(0, '/home/clear/ros2_ws_HJH/src/scripts/masschanger')
import parse_m0_logs as est          # noqa: E402
import parse_dropwindow_logs as ctl  # noqa: E402

D = '/home/clear/ros2_ws_HJH/nmpc_test_results/'
M_EMPTY = 2.064
ARMS = ['M0', 'alphaonly', 'rhythmonly', 'thetastar']
LABEL = {'M0': 'M0 基线', 'alphaonly': 'α-only', 'rhythmonly': 'rhythm-only',
         'thetastar': 'θ* 完整'}
# (基准臂, 对比臂, 这条差估计的是什么)
CONTRASTS = [
    ('M0', 'alphaonly', 'α 主效应 @节奏=M0'),
    ('rhythmonly', 'thetastar', 'α 主效应 @节奏=θ*'),
    ('M0', 'rhythmonly', '节奏 主效应 @阈值=1.5N'),
    ('alphaonly', 'thetastar', '节奏 主效应 @阈值=α'),
    ('M0', 'thetastar', '总收益(校验 B.4)'),
]
# (key, 标签, 小数位, 是否越小越好)
METRICS = [
    ('settle',    'settle驻留[s]',  2, True),
    ('enter',     'enter入带[s]',   2, True),
    ('overshoot', '过冲[kg]',       3, True),
    ('n_exit',    '再出带[次]',     1, True),
    ('pe_ss',     '稳态pos_err[m]', 4, True),
    ('pe_peak',   'pos_err峰[m]',   3, True),
    ('t_rec',     '恢复[s]',        2, True),
    ('t_confirm', '确认延迟[s]',    2, False),   # 机制量,非优劣
    ('n_trig',    '触发[次]',       1, False),
]
PRIMARY = 'settle'


def parse_mhe_events(path):
    """从 mhe 日志提事件层信号:武装/确认时刻、确认延迟、触发次数。

    确认延迟 = confirmed 行时间戳 − armed 行时间戳,直接量化 α 的机制作用
    (阈值抬高 → 需要更大 T_phys 残差 → 确认更晚)。未确认(超时兜底)时返回 None。
    """
    t_armed = t_conf = None
    n_armed = n_conf = 0
    ts = re.compile(r'^\[INFO\] \[(\d+\.\d+)\]')
    for line in open(path, errors='ignore'):
        m = ts.match(line)
        if not m:
            continue
        t = float(m.group(1))
        if 'armed, waiting for T_phys confirmation' in line:
            n_armed += 1
            if t_armed is None:
                t_armed = t
        elif 'confirmed by T_phys' in line:
            n_conf += 1
            if t_conf is None:
                t_conf = t
    dt = (t_conf - t_armed) if (t_armed is not None and t_conf is not None) else None
    return dict(t_confirm=dt, n_trig=n_armed, n_conf=n_conf)


def parse_run(nstamp, mass):
    """per-mass M_TRUE/BAND/THRESH 口径(见 aggregate_b4_matrix),外加事件层指标。"""
    est.M_TRUE = M_EMPTY + mass
    est.BAND = max(0.15 * mass, 0.03)
    est.THRESH = min(1.5, max(0.7 * 9.81 * mass, 0.2))
    ctl.THRESH = est.THRESH
    try:
        r = est.parse(D + f'grip_mhe_{nstamp}.log')
        c = ctl.parse(D + f'grip_nmpc_{nstamp}.log')
    except Exception:
        return None
    inband = (r['t'] >= 0) & (np.abs(r['m'] - est.M_TRUE) <= est.BAND)
    if not np.any(inband):
        return None
    k_in = int(np.argmax(inband))
    overshoot = float(np.max(np.abs(r['m'][k_in:] - est.M_TRUE)))
    # 再出带次数:首次入带之后,带内→带外的转换次数(衡量"入带后是否稳住")
    seq = inband[k_in:]
    n_exit = int(np.sum(seq[:-1] & ~seq[1:])) if len(seq) > 1 else 0
    # 稳态 pos_err:恢复完成后的尾段 RMS(恢复时刻起 +0.5s 留余量)
    t_ss = float(c['t_rec']) + 0.5
    tail = c['t'] >= t_ss
    pe_ss = float(np.sqrt(np.mean(c['pe'][tail] ** 2))) if np.any(tail) else float('nan')
    out = dict(enter=float(r['t'][inband][0]), settle=float(r['t_settle']),
               overshoot=overshoot, n_exit=n_exit, pe_ss=pe_ss,
               pe_peak=float(c['pe_peak']), t_rec=float(c['t_rec']))
    ev = parse_mhe_events(D + f'grip_mhe_{nstamp}.log')
    out['t_confirm'] = ev['t_confirm'] if ev['t_confirm'] is not None else float('nan')
    out['n_trig'] = float(ev['n_trig'])
    return out


def wilcoxon_exact(d):
    """精确 Wilcoxon 符号秩双尾 p。d=配对差(已剔 0)。n>20 退正态近似。

    已对 scipy.stats.wilcoxon(mode='exact') 校验:无 ties 时逐位一致。|d| 有 ties
    时本实现用平均秩枚举,比 scipy 略保守(0.219 vs 0.156)——偏保守=不会把不显著
    判成显著,故保留。连续量 ties 罕见;计数类指标(n_exit/n_trig)ties 多,其 p 按
    "上界"理解。
    """
    d = np.asarray([x for x in d if x != 0 and np.isfinite(x)], dtype=float)
    n = len(d)
    if n == 0:
        return None, None
    order = np.argsort(np.abs(d))
    a = np.abs(d)[order]
    ranks = np.empty(n)
    i = 0
    while i < n:
        j = i
        while j + 1 < n and a[j + 1] == a[i]:
            j += 1
        ranks[i:j + 1] = (i + j) / 2 + 1
        i = j + 1
    signs = np.sign(d)[order]
    w_plus = float(np.sum(ranks[signs > 0]))
    tot_rank = n * (n + 1) / 2
    if n > 20:
        from math import erf, sqrt
        mu, sd = tot_rank / 2, np.sqrt(n * (n + 1) * (2 * n + 1) / 24)
        z = (w_plus - mu) / sd
        return w_plus, 2 * (1 - 0.5 * (1 + erf(abs(z) / sqrt(2))))
    stat = min(w_plus, tot_rank - w_plus)
    hit = sum(1 for combo in itertools.product([0, 1], repeat=n)
              if min(s := sum(ranks[k] for k in range(n) if combo[k]),
                     tot_rank - s) <= stat)
    return w_plus, hit / 2 ** n


def boot_ci(d, n_boot=10000, seed=0):
    """配对差**中位数**的 bootstrap 95% CI(用户要求:不要只报百分比均值)。"""
    d = np.asarray([x for x in d if np.isfinite(x)], dtype=float)
    if len(d) < 2:
        return float('nan'), float('nan')
    rng = np.random.default_rng(seed)
    meds = np.median(rng.choice(d, size=(n_boot, len(d)), replace=True), axis=1)
    return float(np.percentile(meds, 2.5)), float(np.percentile(meds, 97.5))


def main():
    manifest = sys.argv[1] if len(sys.argv) > 1 else None
    if not manifest:
        cands = sorted(glob.glob(D + 'alpha_only_*.txt'), key=os.path.getmtime)
        if not cands:
            print('没找到 alpha_only_*.txt,先跑 run_alpha_only_ablation.sh')
            return
        manifest = cands[-1]
    print(f'manifest: {manifest}')
    print(f'主指标 = {PRIMARY}(预先固定);enter 仅辅助(暂态穿带不稳健)\n')

    runs = {}
    div, ntot, bad = defaultdict(int), defaultdict(int), defaultdict(list)
    for line in open(manifest):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        p = line.split()
        if len(p) < 6 or p[0] not in ARMS:
            continue
        method, mass, ecc, rep, nstamp, status = p[:6]
        ntot[(mass, ecc, method)] += 1
        if status.startswith('DIVERGED'):
            div[(mass, ecc, method)] += 1
        if status != 'ok':
            bad[(mass, ecc, method)].append(f'{nstamp}:{status}')
            continue
        m = parse_run(nstamp, float(mass))
        if m is None:
            bad[(mass, ecc, method)].append(f'{nstamp}:parse_fail')
            continue
        runs[(mass, ecc, rep, method)] = m

    for mass, ecc in sorted({(m, e) for (m, e, _, _) in runs}):
        print('=' * 78)
        print(f'===== 质量 {mass}kg × 偏心 {ecc}m =====')
        reps = sorted({r for (m, e, r, _) in runs if (m, e) == (mass, ecc)},
                      key=lambda x: int(x))
        print(f'\n  --- 各臂绝对值(仅参考,结论走配对) ---')
        for arm in ARMS:
            got = [runs[(mass, ecc, r, arm)] for r in reps
                   if (mass, ecc, r, arm) in runs]
            b = bad.get((mass, ecc, arm), [])
            head = (f'  {LABEL[arm]:<12} n={len(got)}/{ntot[(mass,ecc,arm)]}'
                    f' 发散{div[(mass,ecc,arm)]}')
            print(head + (f' 剔除{b}' if b else ''))
            if got:
                s = '  '.join(
                    f'{lab}={np.nanmean([g[k] for g in got]):.{p}f}'
                    f'±{np.nanstd([g[k] for g in got]):.{p}f}'
                    for k, lab, p, _ in METRICS)
                print(f'      {s}')

        for a, b_arm, why in CONTRASTS:
            common = [r for r in reps
                      if (mass, ecc, r, a) in runs and (mass, ecc, r, b_arm) in runs]
            print(f'\n  ▶ {LABEL[a]} → {LABEL[b_arm]}   [{why}]   配对 n={len(common)}')
            if len(common) < 3:
                print('      配对数 <3,不做检验')
                continue
            for key, lab, prec, lower_better in METRICS:
                va = np.array([runs[(mass, ecc, r, a)][key] for r in common])
                vb = np.array([runs[(mass, ecc, r, b_arm)][key] for r in common])
                d = vb - va
                ok = np.isfinite(d)
                if not np.any(ok):
                    continue
                w, pv = wilcoxon_exact(d[ok])
                lo, hi = boot_ci(d[ok])
                med = float(np.median(d[ok]))
                win = int(np.sum(d[ok] < 0))
                star = ' **' if (pv is not None and pv < 0.05) else ''
                mark = '(越小越好)' if lower_better else '(机制量)'
                # pv=None ⇔ 配对差全为 0(两臂该指标逐 rep 完全相同),无秩可检
                pstr = f'p={pv:.4f}{star}' if pv is not None else 'p=n/a(差值全0)'
                print(f'      {lab:<15} 中位差={med:+.{prec}f} '
                      f'95%CI[{lo:+.{prec}f},{hi:+.{prec}f}] '
                      f'胜/负={win}/{int(np.sum(ok))-win} {pstr}')
                if key == PRIMARY:
                    per = '  '.join(f'{x:+.{prec}f}' for x in d[ok])
                    print(f'      {"":>15} 逐rep差: {per}   {mark}')
        print()


if __name__ == '__main__':
    main()
