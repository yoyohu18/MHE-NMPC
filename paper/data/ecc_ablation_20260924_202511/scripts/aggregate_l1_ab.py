#!/usr/bin/env python3
"""L1-NMPC 对照批次(run_l1_ab.sh)的逐架次指标 + 配对统计。

比的是**带载飞行期间的控制性能**,所以所有段都截在 drop 之前(drop 之后是空机,
两臂没有分歧)。段的切法跟着日志里的阶段行走:
  LIFT 段  [LIFT: raising hover, DYNAMIC: switch to figure8)   ← 承重暂态
  FIG8 段  [DYNAMIC: switch to figure8, DROP)                  ← 机动跟踪
W1(hover)没有 FIG8 段,LIFT 段一直到 DROP。

⚠️ B 臂(l1)必须打过 "arm is NOT handicapped",否则该轮内环补偿没生效,
   算进去就是拿绑手的 baseline 做对照(见 run_l1_ab.sh 文件头 D2(b))。
   这类轮次会被标 handicapped 并**排除出配对统计**,但仍留在明细里。

用法: python3 aggregate_l1_ab.py [manifest.csv]
"""
import csv
import math
import os
import re
import sys
from collections import defaultdict

WS = '/home/clear/ros2_ws_HJH'
RES = os.path.join(WS, 'nmpc_test_results')

TS = re.compile(r'^\[[A-Z]+\] \[(\d+\.\d+)\]')
# ⚠️ 数值一律收 [-+0-9.eE]:pos_err 是正的,但 t/z/d_hat 都可能带符号,
#    漏一个 '+' 就会按符号静默丢轮(2026-09-14 Fig.3 图注全错的原因)。
NUM = r'[-+0-9.eE]+'
AW = re.compile(rf'\[attach-window\] t=({NUM})s pos_err=({NUM})m '
                rf'T=({NUM})N z=({NUM}) m_est=({NUM})')
FD = re.compile(
    rf'\[flight-diag\] t=({NUM})s \| om_scale=({NUM})\(lo ({NUM}) hi ({NUM})\) \| '
    rf'tilt_max=({NUM})deg \| w_max=\[r({NUM}) p({NUM}) y({NUM})\]rad/s \| '
    rf'u_frac=\[T({NUM}) r({NUM}) p({NUM}) y({NUM})\] \| '
    rf'u_sat=\[T({NUM}) r({NUM}) p({NUM}) y({NUM})\]%')
L1 = re.compile(rf'\[l1\] d_hat_f=\[({NUM}),({NUM}),({NUM})\]m/s\^2 '
                rf'\|equiv_dm=({NUM})kg')
PHASE_LIFT = re.compile(rf't=({NUM})s \| LIFT: raising hover')
PHASE_FIG8 = re.compile(rf't=({NUM})s \| DYNAMIC: switch to figure8')
PHASE_DROP = re.compile(rf't=({NUM})s \| DROP')

COLS = ['idx', 'wid', 'arm', 'mode', 'stamp', 'status', 'fail_mode', 'handicapped',
        'om_scale_peak', 'lift_peak', 'lift_rms', 'fig8_rms', 'fig8_peak',
        'tilt_max', 'w_max_rp', 'u_sat_rp', 'u_frac_rp', 'mot_sat',
        'solve_fail', 'l1_equiv_dm', 'z_min', 'n_aw']


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return math.nan


def _rms(v):
    v = [x for x in v if not math.isnan(x)]
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else math.nan


def parse_run(log):
    r = {k: math.nan for k in COLS if k not in
         ('idx', 'wid', 'arm', 'mode', 'stamp', 'status', 'handicapped')}
    r['status'] = 'ok'
    r['handicapped'] = ''
    aw, fd, l1 = [], [], []
    t_lift = t_fig8 = t_drop = None
    w_lift = w_drop = None        # 同两个阶段的**墙钟**([l1] 行只有墙钟)
    solve_fail = 0
    mode = 'mhe'
    not_handicapped = False

    with open(log, errors='ignore') as f:
        for line in f:
            if 'solve failed' in line:
                solve_fail += 1
                continue
            if 'CONTROL MODE: l1' in line:
                mode = 'l1'
            if 'arm is NOT handicapped' in line:
                not_handicapped = True
            m = AW.search(line)
            if m:
                aw.append(tuple(_f(g) for g in m.groups()))
                continue
            m = FD.search(line)
            if m:
                fd.append(tuple(_f(g) for g in m.groups()))
                continue
            m = L1.search(line)
            if m:
                w = TS.match(line)
                l1.append((_f(w.group(1)) if w else math.nan,
                           *(_f(g) for g in m.groups())))
                continue
            if t_lift is None:
                m = PHASE_LIFT.search(line)
                if m:
                    t_lift = _f(m.group(1))
                    w = TS.match(line)
                    w_lift = _f(w.group(1)) if w else None
            if t_fig8 is None:
                m = PHASE_FIG8.search(line)
                if m:
                    t_fig8 = _f(m.group(1))
            if t_drop is None:
                m = PHASE_DROP.search(line)
                if m:
                    t_drop = _f(m.group(1))
                    w = TS.match(line)
                    w_drop = _f(w.group(1)) if w else None

    r['mode'] = mode
    r['solve_fail'] = solve_fail
    r['n_aw'] = len(aw)
    if mode == 'l1' and not not_handicapped:
        r['handicapped'] = 'no-inner-loop-comp'
    if l1 and w_lift is not None:
        # 带载段 = [LIFT, DROP);取后半段的中位,避开 d̂ 的爬升暂态。
        hi = w_drop if w_drop is not None else math.inf
        seg = [x for x in l1 if w_lift <= x[0] < hi]
        if seg:
            tail = seg[len(seg) // 2:]
            r['l1_equiv_dm'] = med([x[4] for x in tail])

    end = t_drop if t_drop is not None else math.inf
    if t_lift is not None and aw:
        lift_end = t_fig8 if t_fig8 is not None else end
        seg = [a for a in aw if t_lift <= a[0] < lift_end]
        if seg:
            r['lift_peak'] = max(a[1] for a in seg)
            r['lift_rms'] = _rms([a[1] for a in seg])
        if t_fig8 is not None:
            seg = [a for a in aw if t_fig8 <= a[0] < end]
            if seg:
                r['fig8_peak'] = max(a[1] for a in seg)
                r['fig8_rms'] = _rms([a[1] for a in seg])
        loaded = [a for a in aw if t_lift <= a[0] < end]
        if loaded:
            r['z_min'] = min(a[3] for a in loaded)
    # flight-diag 是 5s 摘要,只取带载窗口内的极值
    seg = [d for d in fd if (t_lift or 0) <= d[0] < end] or fd
    if seg:
        r['om_scale_peak'] = max(d[1] for d in seg)
        r['tilt_max'] = max(d[4] for d in seg)
        r['w_max_rp'] = max(max(d[5], d[6]) for d in seg)
        r['u_frac_rp'] = max(max(d[10], d[11]) for d in seg)
        r['u_sat_rp'] = max(max(d[14], d[15]) for d in seg)
    if not math.isnan(r['z_min']) and r['z_min'] < 0.2:
        r['status'] = 'FAIL'          # 穿地
    return r


def fail_mode(r):
    """把失败轮按**发生在哪一段**分类。

    2026-09-23 W3(0.25kg/ecc0.10)实测:两臂的失败模式零重叠 ——
    主线 3/8 全是 LIFT 段发散、L1 7/8 全是 figure-8 段失控。判据用 LIFT 段
    的 rms:坠机在 LIFT 段的轮次 lift_rms 是 3.2~16.0,坠在 figure-8 段的轮次
    lift_rms 0.32~0.65(与存活轮的 0.44 同量级,即坠机前 LIFT 段完全正常)。
    两簇差一个数量级,阈值放在 1.0 不敏感。

    ⚠️ 这是**事后**分类。物理依据明确(段切分本来就在,判据是连续量的双峰)
    且与 W2 的连续指标独立印证(LIFT 段 L1 更好 0/7、FIG8 段 L1 更差 7/7),
    但要当成探索性发现,真要写进论文该预注册重跑一次。
    """
    if r['status'] != 'FAIL':
        return 'ok'
    lr = r.get('lift_rms')
    if lr is None or (isinstance(lr, float) and math.isnan(lr)):
        return 'FAIL-unknown'
    return 'LIFT-diverge' if lr > 1.0 else 'FIG8-loss'


def _wilcoxon(d):
    try:
        from scipy.stats import wilcoxon
        d = [x for x in d if not math.isnan(x) and x != 0.0]
        if len(d) < 5:
            return math.nan
        return float(wilcoxon(d).pvalue)
    except Exception:
        return math.nan


def _sign_p(d):
    """双侧符号检验(精确二项)。n 小的时候这个比 Wilcoxon 更该信。"""
    d = [x for x in d if not math.isnan(x) and x != 0.0]
    n = len(d)
    if n == 0:
        return math.nan, 0, 0
    k = sum(1 for x in d if x > 0)
    tail = sum(math.comb(n, i) for i in range(0, min(k, n - k) + 1))
    return min(1.0, 2.0 * tail / 2 ** n), k, n


def med(v):
    v = sorted(x for x in v if isinstance(x, (int, float)) and not math.isnan(x))
    if not v:
        return math.nan
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


def main():
    man = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        RES, 'l1_ab_manifest.csv')
    rows = []
    with open(man) as f:
        for m in csv.DictReader(f):
            if m['stamp'] in ('NA', ''):
                continue
            log = os.path.join(RES, f"grip_nmpc_{m['stamp']}.log")
            if not os.path.exists(log):
                print(f"⚠️ 清单有 {m['stamp']} 但日志不在,跳过")
                continue
            r = parse_run(log)
            r.update(idx=int(m['idx']), wid=m['wid'], arm=m['arm'],
                     stamp=m['stamp'])
            if m.get('status') not in ('ok', None, ''):
                r['status'] = 'FAIL'
            r['fail_mode'] = fail_mode(r)
            rows.append(r)

    out = os.path.join(RES, 'l1_ab_metrics.csv')
    with open(out, 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=COLS, extrasaction='ignore')
        wr.writeheader()
        for r in sorted(rows, key=lambda x: x['idx']):
            wr.writerow(r)
    print(f'{len(rows)} 架次 -> {out}\n')

    # ---- 配置自证 ----
    bad = [r for r in rows if r['handicapped']]
    if bad:
        print(f"⚠️ {len(bad)} 轮 L1 臂内环补偿未确认,已排除出配对统计: "
              f"{[r['stamp'] for r in bad]}\n")
    wrong_mode = [r for r in rows if (r['arm'] == 'B') != (r['mode'] == 'l1')]
    if wrong_mode:
        print(f"⚠️ {len(wrong_mode)} 轮的 control_mode 与臂标不符: "
              f"{[(r['stamp'], r['arm'], r['mode']) for r in wrong_mode]}\n")

    by = defaultdict(lambda: defaultdict(list))
    for r in rows:
        by[r['wid']][r['arm']].append(r)

    print(f"{'工况':<5}{'臂':<6}{'n':>3}{'失败':>6}{'ωscale':>8}"
          f"{'LIFT峰':>9}{'LIFT rms':>10}{'FIG8 rms':>10}"
          f"{'倾角max':>9}{'w_rp':>7}{'u_sat_rp':>10}{'solveF':>8}")
    for wid in sorted(by):
        for arm, name in (('A', 'A/mhe'), ('B', 'B/l1')):
            g = by[wid].get(arm, [])
            if not g:
                continue
            nf = sum(1 for r in g if r['status'] == 'FAIL')
            print(f"{wid:<5}{name:<6}{len(g):>3}{nf:>6}"
                  f"{med([r['om_scale_peak'] for r in g]):>8.2f}"
                  f"{med([r['lift_peak'] for r in g]):>9.3f}"
                  f"{med([r['lift_rms'] for r in g]):>10.4f}"
                  f"{med([r['fig8_rms'] for r in g]):>10.4f}"
                  f"{med([r['tilt_max'] for r in g]):>9.2f}"
                  f"{med([r['w_max_rp'] for r in g]):>7.3f}"
                  f"{med([r['u_sat_rp'] for r in g]):>10.2f}"
                  f"{med([r['solve_fail'] for r in g]):>8.0f}")

    # ---- 配对检验 ----
    # 排除两类轮次,理由不同:
    #   handicapped → 该轮 L1 臂内环补偿没生效,对照本身不成立;
    #   status=FAIL → 坠机轮的 rms/tilt 是发散后的数值(实测见过 fig8_rms=9.44
    #     对同组正常值 0.12),留在配对里**一个轮次就能把均值符号拉反**,
    #     看上去像"L1 更好"。失败是另一个终点,单独按失败率报,不混进连续指标。
    print('\n配对检验(B−A;正=L1 更差;已排除失败轮与 handicapped 轮):')
    for wid in sorted(by):
        def _usable(rs):
            return [r for r in sorted(rs, key=lambda x: x['idx'])
                    if not r['handicapped'] and r['status'] != 'FAIL']
        A = _usable(by[wid]['A'])
        B = _usable(by[wid]['B'])
        dropped = (len(by[wid]['A']) - len(A)) + (len(by[wid]['B']) - len(B))
        if dropped:
            print(f"  [{wid}] 排除 {dropped} 轮(失败/handicapped),"
                  f"剩 A={len(A)} B={len(B)}")
        n = min(len(A), len(B))
        if n == 0:
            continue
        for key in ('lift_peak', 'lift_rms', 'fig8_rms', 'tilt_max',
                    'w_max_rp', 'u_sat_rp'):
            d = [B[i][key] - A[i][key] for i in range(n)]
            d = [x for x in d if not math.isnan(x)]
            if not d:
                continue
            p, k, nn = _sign_p(d)
            mean = sum(d) / len(d)
            pw = _wilcoxon(d)
            print(f"  {wid} {key:<11} n={len(d)} 均值Δ={mean:+.4f} "
                  f"同向={k}/{nn} sign_p={p:.4f}"
                  + ('' if math.isnan(pw) else f" wilcoxon_p={pw:.4f}"))

    # ---- 失败机理分解(两臂的失败可能根本不是同一件事)----
    print('\n失败机理分解:')
    modes = ('ok', 'LIFT-diverge', 'FIG8-loss', 'FAIL-unknown')
    print(f"{'工况':<5}{'臂':<7}" + ''.join(f'{m:>14}' for m in modes))
    for wid in sorted(by):
        for arm, name in (('A', 'A/mhe'), ('B', 'B/l1')):
            g = by[wid].get(arm, [])
            if not g:
                continue
            print(f"{wid:<5}{name:<7}" + ''.join(
                f"{sum(1 for r in g if r['fail_mode'] == m):>14}" for m in modes))
    try:
        from scipy.stats import fisher_exact
        for wid in sorted(by):
            gA, gB = by[wid].get('A', []), by[wid].get('B', [])
            if not gA or not gB:
                continue
            for m in ('LIFT-diverge', 'FIG8-loss'):
                tab = [[sum(1 for r in g if r['fail_mode'] == m),
                        sum(1 for r in g if r['fail_mode'] != m)] for g in (gB, gA)]
                if tab[0][0] == 0 and tab[1][0] == 0:
                    continue
                print(f"  {wid} {m:<13} B={tab[0][0]}/{sum(tab[0])} "
                      f"A={tab[1][0]}/{sum(tab[1])} "
                      f"Fisher p={fisher_exact(tab)[1]:.5f}")
    except ImportError:
        pass

    nfail = defaultdict(int)
    ntot = defaultdict(int)
    for r in rows:
        ntot[r['arm']] += 1
        if r['status'] == 'FAIL':
            nfail[r['arm']] += 1
    print(f"\n存活: A/mhe {ntot['A']-nfail['A']}/{ntot['A']}  "
          f"B/l1 {ntot['B']-nfail['B']}/{ntot['B']}")
    l1v = [r['l1_equiv_dm'] for r in rows
           if r['arm'] == 'B' and not math.isnan(r['l1_equiv_dm'])]
    if l1v:
        print(f"L1 带载段后半 equiv_dm 中位 {med(l1v):+.3f}kg (n={len(l1v)}) "
              f"— 与该工况真实载荷比对,看 d̂ 是否补到位")


if __name__ == '__main__':
    main()
