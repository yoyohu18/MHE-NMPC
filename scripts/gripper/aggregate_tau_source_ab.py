#!/usr/bin/env python3
"""MHE 力矩来源 A/B 的聚合(2026-09-05),配 run_tau_source_ab.sh。

**双侧差异检验,不是等价性检验。** CI 含 0 只能写"在本批分辨率下未检出差异",
不能写"两者等价 / 改默认无害"(措辞纪律见记忆 cem-benefit-refuted)。

判定(与驱动脚本文件头一致,跑前钉死):
  ① 前置硬条件(一票否决):两臂全部 status=ok。physfull 臂若发散 → 直接判负。
  ② 主判据:带载稳态 m̂ 的**有符号**相对偏差(%),配对差的 95% CI + 符号检验。
  ③ 次要(只描述不判定):|err| p50、peak_pos_err、figure8 段 pos_err 中位、
     solve failed、solve 耗时中位(阴性对照)、drop 后触界占空比、attach 偏心平衡。

★ 真值口径:**不读 MHE 的 [truth] 标签里的 m_true**。那个字段受 _payload_attached
  门控,主线档下常年 False → 拿空机当真值,误差整体虚高约 6pp(记忆
  cxy-truth-label-defect / continuous-mainline-attach-gate)。这里一律用 NMPC 日志
  的墙钟段落(ATTACH command issued → DROP command issued)判定"物理上带没带载",
  真值 = M_B + M_P,与 aggregate_moment_ab.py 的做法一致。

用法: python3 aggregate_tau_source_ab.py <manifest.txt>
"""
import math
import re
import sys
from pathlib import Path

import numpy as np

RUNDIR = Path('/home/clear/ros2_ws_HJH/nmpc_test_results')
M_B = 2.0643            # 空机质量 [kg]
M_P = 0.15              # 载荷真值 [kg](本批固定,与脚本底座一致)
M_MIN = 0.95 * M_B      # MHE 的 lbx 硬下界,用来数触界
STEADY_WIN = 40.0       # 带载稳态窗口 [s],锚在 DROP 指令之前。
                        # 取 40 而不是 extract_ab_metrics 的 20:本工作点 figure8
                        # 从 t≈21s 起、ramp 9.4s、drop 在 t≈84s,DROP−40s 仍落在
                        # 稳定机动段内,而 [truth] 行只有 0.5Hz —— 20s 窗口每轮才
                        # ~10 个样本,加倍窗口把配对噪声压下来且不引入暂态。
DYN_SKIP = 12.0         # figure8 起点后跳过的 ramp 秒数

RE_TRUTH = re.compile(r'\[truth\] m_hat=([\d.]+)')
RE_WALL = re.compile(r'\[(\d+\.\d+)\]')
RE_PE = re.compile(r'pos_err=([\d.]+)')
RE_SOLVE = re.compile(r'solve=([\d.]+)ms')


def _wall(line):
    m = RE_WALL.search(line)
    return float(m.group(1)) if m else None


def per_run(stamp):
    """一轮的全部指标;拿不到关键锚点就返回 None。"""
    n = RUNDIR / f'grip_nmpc_{stamp}.log'
    m = RUNDIR / f'grip_mhe_{stamp}.log'
    if not (n.exists() and m.exists()):
        return None

    t_att = t_dyn = t_drop = None
    # solve failed 必须和 peak 一样按 DROP 切开(2026-09-05 第一次聚合踩到):
    # 用全程计数当前置条件,会把 drop 后爆炸、drop 前完全正常的轮次(command
    # rep3 nfail=105 / rep7 nfail=1379,两轮 peak_pre 只有 0.72~0.75m)误判成
    # "drop 前发散"而剔除 —— 那正是本判据明确要放行的那一类。
    fails = []
    pe_all, pe_dyn = [], []
    for ln in n.open(errors='ignore'):
        w = _wall(ln)
        if t_att is None and 'ATTACH command issued' in ln:
            t_att = w
        if t_dyn is None and 'DYNAMIC: switch to figure8' in ln:
            t_dyn = w
        # 释放锚点取**指令**时刻(见 extract_ab_metrics 里同一处注释):
        # "DROP complete" 晚 3s,那段载荷已脱手。
        if t_drop is None and ('DROP command issued' in ln
                               or 'DROP: released gripper' in ln):
            t_drop = w
        if 'solve failed' in ln and w is not None:
            fails.append(w)
        for v in RE_PE.findall(ln):
            f = float(v)
            if w is not None:
                pe_all.append((w, f))
            if t_dyn is not None and w is not None and w >= t_dyn + DYN_SKIP \
                    and (t_drop is None or w <= t_drop):
                pe_dyn.append(f)
    if t_drop is None or t_att is None:
        return None

    # ★ peak 必须按 DROP 切开(2026-09-05 跑前发现,见驱动脚本判据 ①)。
    # 同底座的 command 档 12 轮里有 3 轮在 **drop 后约 13s** 失稳穿地
    # (z 5.6→2.4→−0.76,status=2 连续几十次),而这 3 轮 drop **之前**的
    # peak 只有 0.71~0.74m —— 与另外 9 轮无法区分。若沿用"全程 peak>5 判
    # CRASH 一票否决",这个 ~25% 的基线故障率会随机废掉两臂的轮次,而它
    # 发生在主判据窗口(drop 前 40s)**之后**,不能污染带载稳态的解释力。
    # 所以:前置硬条件只看 peak_pre;drop 后失稳单列为次要指标按臂比发生率。
    peak_pre = max([v for w, v in pe_all if w <= t_drop], default=float('nan'))
    peak_post = max([v for w, v in pe_all if w > t_drop], default=float('nan'))
    nfail_pre = sum(1 for w in fails if w <= t_drop)
    nfail_post = sum(1 for w in fails if w > t_drop)

    lo, hi = t_drop - STEADY_WIN, t_drop
    bias, solves, post = [], [], []
    for ln in m.open(errors='ignore'):
        w = _wall(ln)
        if w is None:
            continue
        # ⚠️ solve 耗时也只取 **DROP 之前**。全程统计会被 drop 后失稳的轮次染色,
        # 而"坠机把 solve 变快/变慢"正是记忆 mhe-coupled-realtime-fix 里那个
        # 已经骗过我一次的伪影 —— 本批 command 臂有 2 轮 drop 后爆炸,不切分
        # 就会把它们的求解耗时算进"档位差异"。
        s = RE_SOLVE.search(ln)
        if s and w <= t_drop:
            solves.append(float(s.group(1)))
        t = RE_TRUTH.search(ln)
        if not t:
            continue
        m_hat = float(t.group(1))
        if lo <= w <= hi:
            # 真值由墙钟段落给,不用日志里的 m_true 标签(见模块 docstring)
            bias.append((m_hat - (M_B + M_P)) / (M_B + M_P) * 100.0)
        elif t_drop + 5.0 <= w <= t_drop + 35.0:
            post.append(m_hat)

    def med(v):
        return float(np.median(v)) if len(v) else float('nan')

    return {
        'bias': med(bias),                                  # 有符号 %(主判据)
        'abs_err': med([abs(b) for b in bias]),
        'n_steady': len(bias),
        'peak_pre': peak_pre,
        'peak_post': peak_post,
        'crash_post': float(peak_post > 5.0) if peak_post == peak_post else 0.0,
        'nfail_pre': nfail_pre,
        'nfail_post': nfail_post,
        'pe_dyn': med(pe_dyn),
        'solve_med': med(solves),
        # drop 后触界占空比:m̂ ≤ lbx+1e-4 的采样比例(触界样本是约束截断值,
        # 只说明无约束解更低,所以单独统计、不混进偏差)
        'floor_duty': (float(np.mean([x <= M_MIN + 1e-4 for x in post]))
                       if post else float('nan')),
    }


def main(man):
    rows = []
    for ln in Path(man).read_text().splitlines():
        if ln.startswith('#') or not ln.strip():
            continue
        f = ln.split()
        rows.append({'arm': f[0], 'rep': int(f[1]), 'stamp': f[2],
                     'status': f[3],
                     'peak': float(f[4]) if f[4] not in ('-', 'nan') else float('nan'),
                     'ecc': float(f[6]) if f[6] not in ('-', 'nan') else float('nan')})

    print(f"轮次 {len(rows)}")
    data = {}
    for r in rows:
        if r['status'] in ('LAUNCH_FAIL', 'PARAM_MISMATCH', 'EXTRACT_FAIL'):
            print(f"  ✗ {r['arm']} rep{r['rep']} {r['stamp']} {r['status']} —— 该轮作废")
            continue
        d = per_run(r['stamp'])
        if d is None:
            print(f"  ✗ {r['arm']} rep{r['rep']} {r['stamp']} 拿不到 ATTACH/DROP 锚点,作废")
            continue
        d['ecc'] = r['ecc']
        d['status'] = r['status']
        data[(r['arm'], r['rep'])] = d

    # ① 前置硬条件:只看 DROP 之前(理由见 per_run 里 peak_pre 的注释)
    bad = [(k, v) for k, v in data.items()
           if not (v['peak_pre'] < 2.0) or v['nfail_pre'] > 100]
    for (a, rep), v in bad:
        print(f"  ✗ {a} rep{rep} drop 前就发散 peak_pre={v['peak_pre']:.2f} "
              f"nfail_pre={v['nfail_pre']}")
    if bad:
        print("① 前置硬条件**未通过** —— 一票否决。先查是哪一臂在 drop 前发散;"
              "主判据在有发散轮次时不具解释力。")
    else:
        print(f"① 前置硬条件通过:{len(data)} 轮 drop 前均无发散(peak_pre<2m)")
    for k, _ in bad:
        data.pop(k, None)
    arms = sorted({a for a, _ in data})
    if len(arms) != 2:
        print(f"臂数不是 2:{arms}"); return
    base = 'command' if 'command' in arms else arms[0]
    test = [a for a in arms if a != base][0]

    print(f"\n② 主判据:带载稳态(DROP 前 {STEADY_WIN:.0f}s)m̂ 有符号偏差 [%]")
    print(f"{'rep':<5}{base:>12}{test:>12}{'配对差':>12}{'n_'+base:>8}{'n_'+test:>8}")
    pairs, ecc_d = [], []
    for rep in sorted({p for _, p in data}):
        kb, kt = (base, rep), (test, rep)
        if kb in data and kt in data:
            b, t = data[kb]['bias'], data[kt]['bias']
            if math.isnan(b) or math.isnan(t):
                print(f"{rep:<5}{'nan':>12}{'nan':>12}   稳态窗口无样本,跳过")
                continue
            pairs.append(t - b)
            ecc_d.append(data[kt]['ecc'] - data[kb]['ecc'])
            print(f"{rep:<5}{b:12.2f}{t:12.2f}{t-b:+12.2f}"
                  f"{data[kb]['n_steady']:8d}{data[kt]['n_steady']:8d}")
    if len(pairs) < 3:
        print("配对数不足,无法给 CI"); return

    d = np.array(pairs)
    n = len(d)
    se = d.std(ddof=1) / np.sqrt(n)
    try:
        from scipy import stats
        tcrit = float(stats.t.ppf(0.975, n - 1)); crit = f't({n-1})'
        pv = float(stats.binomtest(int((d > 0).sum()), n).pvalue)
    except Exception:
        tcrit, crit = 1.96, 'z(无 scipy)'
        k = int((d > 0).sum())
        pv = min(1.0, 2 * sum(math.comb(n, i) for i in range(min(k, n - k) + 1)) / 2 ** n)
    lo, hi = d.mean() - tcrit * se, d.mean() + tcrit * se
    print(f"\n   配对差({test} − {base})n={n}  均值 {d.mean():+.3f}pp  "
          f"95% CI [{lo:+.3f}, {hi:+.3f}] ({crit})")
    print(f"   同号 {int((d > 0).sum())}/{n} 为正,符号检验 p={pv:.4f}")
    if lo <= 0 <= hi:
        print("   → CI 含 0:**在本批分辨率下未检出差异**(不等于两者等价)。"
              "\n     两个入口的默认可以按别的理由(可复现性)统一,但不能宣称收益。")
    elif abs(np.mean([data[k]['bias'] for k in data if k[0] == test])) < \
            abs(np.mean([data[k]['bias'] for k in data if k[0] == base])):
        print(f"   → {test} 的偏差显著更接近 0 → 支持把 headless 默认切到 phys_full+avg。")
    else:
        print(f"   → {base} 显著更好 → 保留 headless 现默认,并把 viz 入口对齐到它。")

    print("\n③ 次要指标(均值,只描述不判定)")
    hdr = ('bias', 'abs_err', 'peak_pre', 'pe_dyn', 'nfail_pre', 'nfail_post',
           'solve_med', 'floor_duty', 'peak_post', 'crash_post', 'ecc')
    print(f"{'arm':<10}" + ''.join(f'{h:>12}' for h in hdr))
    for a in (base, test):
        vals = [data[k] for k in data if k[0] == a]
        cells = []
        for h in hdr:
            v = np.array([x.get(h, np.nan) for x in vals], dtype=float)
            v = v[~np.isnan(v)]
            cells.append(f'{v.mean():.4f}' if len(v) else 'n/a')
        print(f"{a:<10}" + ''.join(f'{c:>12}' for c in cells))
    for a in (base, test):
        cp = [data[k]['crash_post'] for k in data if k[0] == a]
        print(f"   {a}: drop 后失稳(peak_post>5m) {int(sum(cp))}/{len(cp)} 轮 "
              f"—— 同底座 command 档历史基线 3/12,n=8 对这个率的分辨力弱,只记录。")

    if ecc_d:
        e = np.array(ecc_d)
        print(f"\n   attach 偏心配对差 均值 {e.mean():+.4f}m "
              f"(|max| {np.abs(e).max():.4f}m) —— 这是已知会影响估计难度和坠机率的"
              "混杂变量,两臂应大致平衡。")


if __name__ == '__main__':
    main(sys.argv[1])
