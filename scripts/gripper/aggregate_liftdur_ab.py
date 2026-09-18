#!/usr/bin/env python3
"""LIFT 抬升速率 A/B 聚合 (2026-09-16)。

终点定义见 run_liftdur_ab.sh 头部的预注册块。这里只算,不改判定规则。

  主判据 ΔT_overshoot = T_peak(离地±1s) − T_steady(离地后 5~8s 中位数)
  次要   dT/dt 峰值(0.25s 差分)
  操纵检查 t_off = 离地时刻 − LIFT 开始
  只描述 pos_err 峰值 / 发散 / solve failed

离地判据(与跑前估噪声底用的是同一套,不许换):
  z 首次超过 z_base+0.03 且其后 20 帧(1s)持续高于 z_base+0.02,
  z_base = LIFT 前 2.5s 的 z 中位数。

用法: python3 aggregate_liftdur_ab.py <manifest.txt>
"""
import re, sys, glob, math, statistics as st

PAT = re.compile(r'\[attach-window\] t=([0-9.]+)s pos_err=([0-9.]+)m '
                 r'T=([0-9.]+)N z=([0-9.-]+) m_est=([0-9.]+)')
LIFT = re.compile(r't=([0-9.]+)s \| LIFT: raising hover [0-9.]+->([0-9.]+)m')
RUNDIR = '/home/clear/ros2_ws_HJH/nmpc_test_results'


def metrics(stamp, ldur):
    """从一轮的 nmpc 日志算出全部终点;拿不到就返回 None + 原因。

    ldur = 该轮的 GRIP_LIFT_DUR,用来定位"抬升完成"时刻。T_steady 必须取在
    抬升完成之后,否则两臂测的是不同物理状态的推力:旧定义(离地后 5~8s)下
    A 臂(8.4s)那会儿已在 6m 悬停,B 臂(16.8s)还在 s≈0.5 的最大速度段,差值
    会被这个系统性偏差污染。
    """
    hits = glob.glob(f'{RUNDIR}/grip_nmpc_{stamp}.log')
    if not hits:
        return None, 'no-log'
    rows, t_lift, sf = [], None, 0
    for ln in open(hits[0], errors='ignore'):
        if 'solve failed' in ln:
            sf += 1
        m = PAT.search(ln)
        if m:
            rows.append(tuple(float(x) for x in m.groups()))
        m2 = LIFT.search(ln)
        if m2 and t_lift is None:
            t_lift = float(m2.group(1))
    if t_lift is None or len(rows) < 80:
        return None, 'no-lift'
    pre = [r for r in rows if t_lift - 2.5 <= r[0] <= t_lift]
    post = [r for r in rows if t_lift < r[0] <= t_lift + ldur + 6.0]
    if len(pre) < 20 or len(post) < 150:
        return None, 'short'
    # ── 轮次质量门(跑前钉死,两臂对称施加,见预注册 ②)──────────────────
    # 都是 LIFT **起点条件**不合格或判据失效,与臂无关;剔除率必须按臂报告,
    # 差异大则主判据存在选择偏倚。
    if max(r[3] for r in pre) > 1.0:
        return None, 'not-low'        # 根本不在低空:串轮/异常
    if st.pstdev(r[3] for r in pre) > 0.02:
        return None, 'unsettled'      # attach 后悬停没稳住,起点条件不合格
    z_base = st.median(r[3] for r in pre)
    t_off = None
    for i, r in enumerate(post):
        if r[3] > z_base + 0.03 and all(x[3] > z_base + 0.02 for x in post[i:i + 20]):
            t_off = r[0]
            break
    if t_off is None:
        return None, 'no-liftoff'
    if t_off - t_lift < 0.3:
        # smoothstep 起步太慢,物理上不可能 0.3s 内离地 -> 判据被 z 抖动触发
        return None, 'false-liftoff'
    pk = [r[2] for r in post if abs(r[0] - t_off) <= 1.0]
    # T_steady:抬升完成后 2~5s 的悬停推力。两臂在这个窗口里都已到 z_high
    # 悬停,物理状态相同才可比(见函数 docstring)。
    # 窗口必须落在 settle(3s) 之内:历史轮次抬升完成 3s 后就切 figure-8 并
    # ramp 加速,t_done+2~5 会被它污染(实测 SD 0.823 -> 2.165,差 2.6x)。
    t_done = t_lift + ldur
    ss = [r[2] for r in post if t_done + 0.5 <= r[0] <= t_done + 2.5]
    if len(pk) < 20 or len(ss) < 40:
        return None, 'no-window'
    # ⚠️ 终点定义的一部分,不是事后筛选(见 run_liftdur_ab.sh 预注册 ②):
    # 推力一旦打到饱和(真实上限 31.4N,见 nmpc-thrust-envelope-mismatch),
    # T_peak 就不再是"突破地面平衡的超出量"而是执行器上限,ΔT_overshoot 失去
    # 物理含义(冒烟实测这类轮给出 12~13N,把 SD 从 0.51 撑到 5.3)。这类轮记
    # saturated,不进主判据,但计入发散率分母。噪声底 2.228±0.511 就是在
    # T_pk<=26 的样本上估的,门必须一致,否则功效声明失效。
    if max(pk) > 26.0:
        return None, 'saturated'
    seq = [r for r in post if t_lift <= r[0] <= t_off + 0.5]
    slope = max(((seq[i + 5][2] - seq[i][2]) / (seq[i + 5][0] - seq[i][0])
                 for i in range(len(seq) - 5)), default=float('nan'))
    lift_seg = [r for r in post if r[0] <= t_off + 10]
    return dict(t_off=t_off - t_lift, dT_over=max(pk) - st.median(ss),
                T_pk=max(pk), T_ss=st.median(ss), dTdt=slope, sf=sf,
                pos_pk=max(r[1] for r in lift_seg)), None


def welch(a, b):
    """Welch t 检验 + 差值的双侧 95% CI。"""
    na, nb = len(a), len(b)
    ma, mb = st.mean(a), st.mean(b)
    va, vb = st.variance(a), st.variance(b)
    se = math.sqrt(va / na + vb / nb)
    if se == 0:
        return mb - ma, 0, 0, float('nan')
    df = (va / na + vb / nb) ** 2 / ((va / na) ** 2 / (na - 1) + (vb / nb) ** 2 / (nb - 1))
    t = (mb - ma) / se
    # df>=10 时 t_{.975} 落在 2.0~2.23,取 2.145 (df=14) 够用;小样本不追求精确
    tc = 2.145
    return mb - ma, (mb - ma) - tc * se, (mb - ma) + tc * se, t


def main(man):
    arms = {}
    for ln in open(man):
        if ln.startswith('#') or not ln.strip():
            continue
        p = ln.split()
        if len(p) < 9:
            continue
        arm, rep, ldur, stamp, status = p[0], p[1], p[2], p[6], p[7]
        arms.setdefault(arm, []).append(dict(rep=rep, ldur=ldur, stamp=stamp,
                                             status=status, peak=float(p[8])))
    print(f'manifest: {man}\n')
    data = {}
    for arm, runs in arms.items():
        print(f'=== 臂 {arm} (lift_dur={runs[0]["ldur"]}) n={len(runs)} ===')
        vals = []
        for r in runs:
            if r['stamp'] == '-':
                print(f'  rep{r["rep"]:>2} {r["status"]:<10} (无日志)')
                continue
            m, why = metrics(r['stamp'], float(r['ldur']))
            if m is None:
                print(f'  rep{r["rep"]:>2} {r["status"]:<10} 指标不可得: {why}')
                continue
            vals.append(m)
            print(f'  rep{r["rep"]:>2} {r["status"]:<10} t_off={m["t_off"]:.2f}s  '
                  f'ΔT_over={m["dT_over"]:.2f}N  dT/dt={m["dTdt"]:.1f}N/s  '
                  f'pos_pk={m["pos_pk"]:.3f}m  sf={m["sf"]}')
        data[arm] = vals
        div = sum(1 for r in runs if r['status'] == 'DIVERGED')
        print(f'  -- 发散 {div}/{len(runs)}  指标可得 {len(vals)}/{len(runs)}\n')

    if len(data) != 2:
        print('臂数 != 2,不做对比'); return
    A, B = 'a84', 'b168'
    if A not in data or B not in data:
        A, B = sorted(data)
    print('=== 两臂对比 (B − A) ===')
    for key, unit, label, res in (
            ('dT_over', 'N', '① 主判据 ΔT_overshoot', 0.82),
            ('dTdt', 'N/s', '② 次要 dT/dt 峰', 3.32),
            ('t_off', 's', '③ 操纵检查 t_off', None),
            ('pos_pk', 'm', '   (描述) LIFT pos_err 峰', None)):
        a = [v[key] for v in data[A]]
        b = [v[key] for v in data[B]]
        if len(a) < 2 or len(b) < 2:
            print(f'{label}: 样本不足'); continue
        d, lo, hi, t = welch(a, b)
        sig = '显著' if (lo > 0 or hi < 0) else 'CI 含 0'
        print(f'{label}: A {st.mean(a):.3f}±{st.stdev(a):.3f} → '
              f'B {st.mean(b):.3f}±{st.stdev(b):.3f} {unit}   '
              f'差 {d:+.3f} [{lo:+.3f}, {hi:+.3f}]  {sig}')
        if res and not (lo > 0 or hi < 0):
            print(f'    ⚠️ 措辞纪律:CI 含 0 → 只能写"在 {res}{unit} '
                  f'分辨率下未检出差异",不能写"拉长无效"')
    ta = [v['t_off'] for v in data[A]]
    tb = [v['t_off'] for v in data[B]]
    if len(ta) > 1 and len(tb) > 1:
        _, lo, hi, _ = welch(ta, tb)
        if not lo > 0:
            print('\n❌ 操纵检查未通过:B 臂 t_off 没有显著更长 → '
                  'LIFT_DUR 可能没生效,整批作废(见预注册 ④)')
        else:
            print('\n✅ 操纵检查通过:B 臂离地过程确实更长')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    main(sys.argv[1])
