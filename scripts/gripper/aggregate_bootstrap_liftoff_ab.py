#!/usr/bin/env python3
"""J bootstrap 武装时机 A/B 聚合。终点定义见 run_bootstrap_liftoff_ab.sh 预注册块。

危险窗口 = attach -> 真离地(离线 z 判据,两臂同一套,与 liftdur 批一致)。
"""
import re, sys, glob, math, statistics as st

RUN = '/home/clear/ros2_ws_HJH/nmpc_test_results'
AW = re.compile(r'\[attach-window\] t=([0-9.]+)s pos_err=[0-9.]+m T=([0-9.]+)N z=([0-9.-]+)')
WC = re.compile(r'\[wcmd\] raw=\[[-+0-9.]+,[-+0-9.]+,[-+0-9.]+\] w=\[([-+0-9.]+),([-+0-9.]+),'
                r'[-+0-9.]+\].*?s=([0-9.]+)')
ATT = re.compile(r't=([0-9.]+)s \| ATTACH command issued')
LIF = re.compile(r't=([0-9.]+)s \| LIFT: raising')
STAMP = re.compile(r'\[(\d+\.\d+)\]')


def metrics(stamp):
    f = f'{RUN}/grip_nmpc_{stamp}.log'
    try:
        lines = open(f, errors='ignore').readlines()
    except FileNotFoundError:
        return None, 'no-log'
    aw = []; wc = []; ta = tl = t0 = None
    sf = 0; deferred = False; detected = None; fallback = False
    for x in lines:
        if 'solve failed' in x: sf += 1
        g = AW.search(x)
        if g: aw.append((float(g.group(1)), float(g.group(2)), float(g.group(3))))
        g = WC.search(x)
        if g:
            ts = STAMP.search(x)
            if ts: wc.append((float(ts.group(1)), float(g.group(1)),
                              float(g.group(2)), float(g.group(3))))
        g = ATT.search(x)
        if g and ta is None:
            ta = float(g.group(1))
            ts = STAMP.search(x); t0 = float(ts.group(1)) if ts else None
        g = LIF.search(x)
        if g and tl is None: tl = float(g.group(1))
        if 'bootstrap DEFERRED' in x: deferred = True
        m = re.search(r'\[liftoff\] DETECTED .*attach\+([0-9.]+)s', x)
        if m: detected = float(m.group(1))
        if 'arming J bootstrap on timeout fallback' in x: fallback = True
    if ta is None or tl is None or t0 is None or not wc:
        return None, 'no-phase'
    base = [r[2] for r in aw if ta <= r[0] <= ta + 0.8]
    if not base: return None, 'no-base'
    z0 = sum(base) / len(base)
    post = [r for r in aw if r[0] > tl]
    t_off = None
    for i, r in enumerate(post):
        if r[2] > z0 + 0.03 and all(x[2] > z0 + 0.02 for x in post[i:i + 20]):
            t_off = r[0]; break
    if t_off is None: return None, 'no-liftoff'
    # 首次力矩饱和时刻 + 阶段分类。两种机理必须分开数,否则"总失稳率没变"
    # 会把"旧故障消掉、新故障冒出来"这件事整个盖住(教训见 mhe-geom-mest-decoupling
    # 的"混算两种故障压低统计功效")。
    # ⚠️ 单帧饱和不算失稳:上一批 25 个正常轮里有 4 个在 LIFT 爬升段闪过一两帧
    # 饱和,结局完全正常(status=ok, 不穿地, solve_fail=0)。只有**持续**饱和才是
    # 失稳。阈值 5 帧(0.25s @20Hz):失稳轮实测饱和占窗口 20~47%,良性暂态 1~2 帧。
    MU = re.compile(r'\[model-used\] t=([0-9.]+).*?tr=([0-9.-]+) tp=([0-9.-]+)')
    sat_frames = []
    for x in lines:
        g = MU.search(x)
        if g and float(g.group(1)) >= ta:
            if abs(float(g.group(2))) >= 0.499 or abs(float(g.group(3))) >= 0.499:
                sat_frames.append(float(g.group(1)))
    n_sat = len(sat_frames)
    sat = sat_frames[0] if n_sat >= 5 else None
    if sat is None:
        phase = 'none'
    elif sat < tl:
        phase = 'pre-lift'      # 旧机理:attach 后 / LIFT 前的危险窗口
    elif sat <= t_off + 1.0:
        phase = 'at-liftoff'    # 新机理:离地瞬间,保护迟到
    else:
        phase = 'post-liftoff'  # 抬升爬升段
    seg = [(t - t0 + ta, wx, wy, s) for t, wx, wy, s in wc]
    win = [r for r in seg if ta <= r[0] <= t_off]
    if len(win) < 10: return None, 'short-window'
    wmax = max((r[1]**2 + r[2]**2)**0.5 for r in win)
    wrms = (sum(r[1]**2 + r[2]**2 for r in win) / len(win))**0.5
    return dict(wmax=wmax, wrms=wrms, s_peak=max(r[3] for r in win),
                t_off=t_off - ta, sf=sf, zmin=min(r[2] for r in aw if r[0] > tl),
                deferred=deferred, detected=detected, fallback=fallback,
                sat=(sat - ta) if sat else None, n_sat=n_sat,
                sat_rel_lift=(sat - tl) if sat else None, phase=phase), None


def mannwhitney(a, b):
    allv = sorted(a + b)
    def rk(v):
        idx = [i for i, x in enumerate(allv) if x == v]
        return sum(idx) / len(idx) + 1
    R = sum(rk(v) for v in b); n1, n2 = len(b), len(a)
    U = R - n1 * (n1 + 1) / 2
    sd = (n1 * n2 * (n1 + n2 + 1) / 12) ** 0.5
    if sd == 0: return 1.0
    z = (abs(U - n1 * n2 / 2) - 0.5) / sd
    return 2 * (1 - 0.5 * (1 + math.erf(abs(z) / math.sqrt(2))))


def main(man):
    arms = {}
    for ln in open(man):
        if ln.startswith('#') or not ln.strip(): continue
        p = ln.split()
        if len(p) < 9: continue
        arms.setdefault(p[0], []).append(dict(rep=p[1], stamp=p[6], status=p[7]))
    print(f'manifest: {man}\n')
    data = {}
    for arm, runs in arms.items():
        print(f'=== 臂 {arm} (on_liftoff={"true" if arm=="on" else "false"}) n={len(runs)} ===')
        vals = []
        for r in runs:
            if r['stamp'] == '-':
                print(f'  rep{r["rep"]:>2} {r["status"]:<10} 无日志'); continue
            m, why = metrics(r['stamp'])
            if m is None:
                print(f'  rep{r["rep"]:>2} {r["status"]:<10} 指标不可得: {why}'); continue
            vals.append(m)
            tag = ''
            if m['fallback']: tag = ' [兜底arm]'
            elif m['detected'] is not None: tag = f' [离地@+{m["detected"]:.2f}s]'
            ph = (f'  [良性饱和{m["n_sat"]}帧]' if m['phase'] == 'none' and m['n_sat']
                  else ('' if m['phase'] == 'none'
                        else f'  ★{m["phase"]}(attach+{m["sat"]:.2f}s, {m["n_sat"]}帧)'))
            print(f'  rep{r["rep"]:>2} {r["status"]:<10} s峰={m["s_peak"]:.2f}  '
                  f'|w|峰={m["wmax"]:.4f}  |w|RMS={m["wrms"]:.4f}  '
                  f't_off={m["t_off"]:.2f}s  sf={m["sf"]}  zmin={m["zmin"]:.2f}{tag}{ph}')
        data[arm] = vals
        div = sum(1 for r in runs if r['status'] == 'DIVERGED')
        crash = sum(1 for v in vals if v['zmin'] < 0)
        fb = sum(1 for v in vals if v['fallback'])
        print(f'  -- 发散 {div}/{len(runs)}  穿地 {crash}  指标可得 {len(vals)}/{len(runs)}'
              + (f'  兜底arm {fb}/{len(vals)}' if arm == 'on' else '') + '\n')

    if 'off' not in data or 'on' not in data:
        print('缺臂,不对比'); return
    A, B = data['off'], data['on']
    print('=== 终点① 机制生效检查 ===')
    sa = [v['s_peak'] for v in A]; sb = [v['s_peak'] for v in B]
    print(f'  危险窗口 omega_scale 峰值: off {min(sa):.2f}~{max(sa):.2f} (应≈5.00)   '
          f'on {min(sb):.2f}~{max(sb):.2f} (应≈1.00)')
    ok = all(s > 4.0 for s in sa) and all(s < 2.0 for s in sb)
    print(f'  -> {"✅ 机制生效" if ok else "❌ 有轮次不符,整批作废(见预注册①)"}')
    print('\n=== 终点② 主终点 (Mann-Whitney 双侧,不预设方向) ===')
    for key, lab, res in (('wmax', '|w| 峰值', 0.0777), ('wrms', '|w| RMS', 0.0162)):
        a = [v[key] for v in A]; b = [v[key] for v in B]
        if len(a) < 2 or len(b) < 2: print(f'  {lab}: 样本不足'); continue
        p = mannwhitney(a, b)
        print(f'  {lab}: off 中位 {st.median(a):.4f} (均值 {st.mean(a):.4f}) → '
              f'on 中位 {st.median(b):.4f} (均值 {st.mean(b):.4f}) rad/s   p={p:.4f}')
        if p >= 0.05:
            print(f'      措辞纪律: p≥.05 → 只能写"在 {res} rad/s 分辨率下未检出差异"')
    print('\n=== 终点③ 只描述(功效不足,不作判定)===')
    for lab, arr in (('off', A), ('on', B)):
        crash = sum(1 for v in arr if v['zmin'] < 0)
        print(f'  {lab}: 穿地 {crash}/{len(arr)}  solve_fail 中位 {st.median([v["sf"] for v in arr]):.0f}  '
              f'离地时刻中位 {st.median([v["t_off"] for v in arr]):.2f}s')
    print('  ⚠️ 失稳率 n=8 功效不足(基础率 22%,0/8 vs 2/8 的 p≈0.47),仅作描述')
    print('\n=== ★ 失败轮的阶段分类(本改动的关键:是消掉了还是搬了家)===')
    print('  ⚠️ 只统计**真失败**的轮次(穿地 zmin<0)。饱和本身不等于失稳:'
          '离地瞬间是应力\n     集中点,正常轮也会饱和十几帧却撑住 '
          '(实测 off 正常轮 16 帧 vs 失败轮 91 帧)。')
    print(f'  {"":6s}{"pre-lift":>10s}{"at-liftoff":>12s}{"post-liftoff":>14s}'
          f'{"失败总数":>10s}{"存活轮饱和帧中位":>18s}')
    for lab, arr in (('off', A), ('on', B)):
        fail = [v for v in arr if v['zmin'] < 0]
        alive = [v for v in arr if v['zmin'] >= 0]
        c = {k: sum(1 for v in fail if v['phase'] == k)
             for k in ('pre-lift', 'at-liftoff', 'post-liftoff')}
        med = (st.median([v['n_sat'] for v in alive]) if alive else float('nan'))
        print(f'  {lab:6s}{c["pre-lift"]:10d}{c["at-liftoff"]:12d}'
              f'{c["post-liftoff"]:14d}{len(fail):10d}{med:18.0f}')
    print('  pre-lift  = 旧机理(attach 后危险窗口,增益提前拉满)')
    print('  at-liftoff= 新机理(离地瞬间保护迟到)')
    print('  -> off 集中在 pre-lift 而 on 集中在 at-liftoff = 故障搬家,不是消掉;')
    print('     两臂失败总数都降 = 真的有效;只有 on 的 at-liftoff 增加 = 改动有害。')


if __name__ == '__main__':
    if len(sys.argv) < 2: print(__doc__); sys.exit(1)
    main(sys.argv[1])
