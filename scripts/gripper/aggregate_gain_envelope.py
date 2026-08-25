#!/usr/bin/env python3
"""增益整定 包线上界 vs 点估计先验 A/B 的聚合(2026-08-25),配 run_gain_envelope_ab.sh。

⚠️ **非劣性检验,不是优效性检验。** 目标是"用机架规格的包线上界代替任务信息的
点估计先验、且不付代价"。env 臂不需要更好,只需要不显著更差。所以 p>0.05
**不等于**"没差别 = 成功"——p 大也可能只是功效不足。判定靠预设的等效边界 δ。

判定规则(与驱动脚本文件头一致,跑前钉死):
  ① 前置硬条件(一票否决):16/16 全 ok、无发散(peak<2m)、无触界。
     本批的风险侧是**过增益**(env 臂增益高 30.3%),若有害最先在这里现形。
  ② 主判据:d = pos_err中位(env0.5) − pos_err中位(point0.2),
     95% CI 上界 < **δ=+0.015m** → 非劣。
     δ 依据:prior 批(20260825_144337)pos_err 批内跨度 0.007m,取 2×。
     ⚠️ 那批是 0.3kg;本批 0.2kg。脚本会**先**报出本批 point 臂的实际跨度,
        若显著大于 0.007m 则 δ 需重定 —— 但必须在看 d 之前声明,不许事后调。
  ③ 过增益的直接证据(描述性,不做硬判定):figure8 稳态段 body 角速度的
     rms(dω/dt) [rad/s²],报告 env/point 比值。≈1=没引入额外抖动。
     时间窗用"采集末尾 TAIL_SEC 秒"——resid 是 monotonic 时基、nmpc 日志是接管
     相对时基,跨时基对齐容易出错;飞行末段必在 figure8 稳态,两臂同样处理,
     配对比较公平。
  ④ 次要(只描述):solve failed、peak、m_est bias/iqr。

用法: python3 aggregate_gain_envelope.py <manifest.txt>
"""
import sys
from pathlib import Path

import numpy as np
from scipy import stats

sys.path.insert(0, str(Path(__file__).parent))
import aggregate_mhe_window as amw  # noqa: E402

DELTA = 0.015              # 等效边界 [m],pos_err 中位数的配对差
BASE, TEST = 'point0.2', 'env0.5'
TAIL_SEC = 60.0            # ③ 的时间窗:采集末尾这么多秒
REF_SPREAD = 0.007         # prior 批的 pos_err 批内跨度,用来校验 δ 是否仍适用


def omega_hf(resid_dir):
    """figure8 稳态段(采集末尾 TAIL_SEC 秒)body 角速度的高频能量。

    返回 dict(rms_rate, rms_w, n, src, hz) 或 None。
    rms_rate = rms(dω/dt) [rad/s²]:过增益引起的内环高频振荡的直接代理。
    优先用 IMU(频率远高于 odom 的 ~31Hz);没有就退回 odom 的 wb_*。
    """
    d = Path(resid_dir)
    for src, cols in (('imu', ('wx', 'wy')), ('odom', ('wb_x', 'wb_y'))):
        for csv in sorted(d.glob(f'resid_{src}_*.csv')):
            try:
                a = np.genfromtxt(csv, delimiter=',', names=True)
            except Exception:
                continue
            if a is None or a.size < 100 or a.ndim == 0:
                continue
            # 时基:t_header_ns 常因 ros_gz_bridge 不填 header 而为 -1,退回 t_recv_ns
            t = a['t_header_ns'] * 1e-9
            if not np.all(np.isfinite(t)) or np.nanmin(t) <= 0:
                t = a['t_recv_ns'] * 1e-9
            t = t - t[0]
            o = np.argsort(t)
            t = t[o]
            w = np.stack([a[c][o] for c in cols])
            sel = t >= (t[-1] - TAIL_SEC)
            t, w = t[sel], w[:, sel]
            dt = np.diff(t)
            ok = dt > 1e-6                       # 去掉重复/乱序时间戳
            if ok.sum() < 50:
                continue
            rate = np.diff(w, axis=1)[:, ok] / dt[ok]
            return dict(rms_rate=float(np.sqrt(np.mean(rate ** 2))),
                        rms_w=float(np.sqrt(np.mean(w ** 2))),
                        n=int(ok.sum()), src=src,
                        hz=float(1.0 / np.median(dt[ok])))
    return None


def main(man):
    man = Path(man)
    # 本批载荷是 0.2kg,不是 aggregate_mhe_window 里硬编码的 0.3 —— 必须改,
    # 否则 bias_pct 会用错的真值算。
    mass = 0.2
    for ln in man.read_text().splitlines():
        if ln.startswith('# 载荷: mass='):
            mass = float(ln.split('mass=')[1].split()[0])
    amw.M_PAYLOAD = mass
    amw.M_TRUE = amw.M_EMPTY + mass
    print(f'带载真值 m_true = {amw.M_TRUE:.4f} kg  (载荷 {mass} kg)')
    resid_base = None
    for ln in man.read_text().splitlines():
        if ln.startswith('# resid 采集:'):
            resid_base = ln.split(':', 1)[1].strip().rstrip('/<arm>_<rep>/')

    rows, hard_fail = [], []
    for ln in man.read_text().splitlines():
        if ln.startswith('#') or not ln.strip():
            continue
        f = ln.split()
        if len(f) < 9:
            continue
        arm, rep, stamp, status, peak = f[0], int(f[1]), f[6], f[7], float(f[8])
        if status != 'ok' or peak > 2.0:
            hard_fail.append(f'{arm} rep{rep}: status={status} peak={peak}')
            continue
        c = amw.one_cell(stamp, float(f[3]), float(f[4]))
        if c is None:
            hard_fail.append(f'{arm} rep{rep} {stamp}: 稳态段样本<20')
            continue
        hf = None
        if resid_base:
            rd = Path(f'{resid_base}/{arm}_{rep}')
            if rd.is_dir():
                hf = omega_hf(rd)
        c.update(arm=arm, rep=rep, peak=peak, stamp=stamp,
                 abs_bias=abs(c['bias_pct']), hf=hf)
        rows.append(c)

    if not rows:
        print('没有有效轮次'); return
    print(f'有效轮次 {len(rows)}\n')
    hdr = (f'{"arm":>9} {"rep":>4} {"n":>5} {"pos_err":>8} {"bias%":>8} '
           f'{"iqr":>7} {"peak":>7} {"clip":>5} {"fail":>5} '
           f'{"w_rms":>8} {"dw/dt":>9} {"src@Hz":>10}')
    print(hdr); print('-' * len(hdr))
    for r in sorted(rows, key=lambda x: (x['arm'], x['rep'])):
        h = r['hf']
        print(f'{r["arm"]:>9} {r["rep"]:>4} {r["n"]:>5} {r["pos_err"]:>8.3f} '
              f'{r["bias_pct"]:>+7.2f}% {r["iqr"]:>7.3f} {r["peak"]:>7.3f} '
              f'{"Y" if r["clipped"] else "-":>5} {r["failed"]:>5} '
              + (f'{h["rms_w"]:>8.4f} {h["rms_rate"]:>9.3f} '
                 f'{h["src"]+"@"+format(h["hz"],".0f"):>10}' if h else
                 f'{"-":>8} {"-":>9} {"-":>10}'))

    print('\n--- 每臂汇总 ---')
    for a in (BASE, TEST):
        g = [r for r in rows if r['arm'] == a]
        if not g:
            continue
        pe = [r['pos_err'] for r in g]
        hs = [r['hf']['rms_rate'] for r in g if r['hf']]
        print(f'{a:>9}  n={len(g)}  pos_err 中位 {np.median(pe):.4f} '
              f'(min {min(pe):.4f} max {max(pe):.4f} 跨度 {max(pe)-min(pe):.4f})  '
              f'|bias| 中位 {np.median([r["abs_bias"] for r in g]):.2f}%  '
              f'solve_failed 合计 {sum(r["failed"] for r in g)}'
              + (f'  dω/dt 中位 {np.median(hs):.3f}' if hs else ''))

    print('\n=== ① 前置硬条件:全部能飞、无发散、无触界 ===')
    clipped = [f'{r["arm"]} rep{r["rep"]}: m_est 触下界' for r in rows if r['clipped']]
    if hard_fail or clipped:
        print('  **不通过** —— 以下轮次有问题:')
        for x in hard_fail + clipped:
            print(f'    - {x}')
        print('  ⇒ "包线上界取代点估计先验"**不能**写进论文,②③再好也没用。')
    else:
        print(f'  通过({len(rows)} 轮全 ok、无发散、无触界)')

    # δ 适用性自检:必须在看 d 之前
    gb = [r['pos_err'] for r in rows if r['arm'] == BASE]
    if gb:
        sp = max(gb) - min(gb)
        print(f'\n=== δ 适用性自检(先于主判据) ===')
        print(f'  本批 {BASE} 臂 pos_err 跨度 {sp:.4f}m  vs  prior 批参考 {REF_SPREAD:.4f}m')
        print(f'  δ={DELTA}m {"仍适用(>=2×本批跨度)" if DELTA >= 2*sp else "**偏紧**:本批噪声更大,d 若落在边界附近须谨慎解读"}')

    pairs = []
    for rp in sorted({r['rep'] for r in rows}):
        b = [r for r in rows if r['arm'] == BASE and r['rep'] == rp]
        t = [r for r in rows if r['arm'] == TEST and r['rep'] == rp]
        if b and t:
            pairs.append((rp, b[0], t[0]))
    print(f'\n=== ② 主判据:d = pos_err({TEST}) − pos_err({BASE}),等效边界 δ=+{DELTA}m ===')
    if len(pairs) < 3:
        print(f'  配对数 {len(pairs)} < 3,不做统计')
    else:
        d = np.array([t['pos_err'] - b['pos_err'] for _, b, t in pairs])
        print(f'{"rep":>5} {BASE:>10} {TEST:>10} {"d":>9}')
        for (rp, b, t), dd in zip(pairs, d):
            print(f'{rp:>5} {b["pos_err"]:>10.4f} {t["pos_err"]:>10.4f} {dd:>+9.4f}')
        se = d.std(ddof=1) / np.sqrt(len(d))
        ci = d.mean() + np.array([-1, 1]) * stats.t.ppf(0.975, len(d) - 1) * se
        tt, p = stats.ttest_rel([t['pos_err'] for _, _, t in pairs],
                                [b['pos_err'] for _, b, _ in pairs])
        print(f'\n配对数 n={len(d)}   d 均值 {d.mean():+.4f}m   sd {d.std(ddof=1):.4f}   '
              f'95% CI [{ci[0]:+.4f}, {ci[1]:+.4f}]')
        print(f'{TEST} 更好(d<0) 的 rep: {int((d<0).sum())}/{len(d)}   双边配对 t 检验 p={p:.4f}')
        print(f'\n  CI 上界 {ci[1]:+.4f}m  vs  等效边界 +{DELTA}m')
        ok2 = ci[1] < DELTA
        print(f'  ⇒ {"非劣在统计上成立" if ok2 else "证据不足(CI 上界越过 δ)"}'
              + ('' if not (hard_fail or clipped) else ',**但前置硬条件没过**,结论不成立'))

    print(f'\n=== ③ 过增益直接证据:rms(dω/dt),末尾 {TAIL_SEC:.0f}s ===')
    hb = [r['hf']['rms_rate'] for r in rows if r['arm'] == BASE and r['hf']]
    ht = [r['hf']['rms_rate'] for r in rows if r['arm'] == TEST and r['hf']]
    if hb and ht:
        rb, rt = float(np.median(hb)), float(np.median(ht))
        print(f'  {BASE:>9}(ratio 3.836): {rb:.3f} rad/s²   n={len(hb)}')
        print(f'  {TEST:>9}(ratio 5.000): {rt:.3f} rad/s²   n={len(ht)}')
        print(f'  比值 env/point = {rt/rb:.3f}  '
              f'{"≈1,未见额外抖动" if rt/rb < 1.15 else "**明显偏高**,即使①②通过也须在论文如实写"}')
    else:
        print('  无 resid 数据(未开采集或文件缺失),③ 无法评估')

    print('\n措辞纪律:非劣成立说的是"未测出代价",不是"两者等价";')
    print('          不成立说的是"证据不足",不是"点估计先验必需"。')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else
         max(Path('/home/clear/ros2_ws_HJH/nmpc_test_results')
             .glob('gain_envelope_ab_*.txt'), key=lambda p: p.stat().st_mtime))
