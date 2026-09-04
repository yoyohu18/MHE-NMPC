#!/usr/bin/env python3
"""连续主线 A/B(run_mainline_ab.sh)的逐架次判定 + 配对统计。

每架次自动判定用户 09-03 定的那张清单:
  health/freshness、drop confidence 延迟、omega-scale 恢复时间与斜率、
  最大倾角/角速度、allocator 与电机饱和、位置误差、求解失败。

数据源两条:
  [flight-diag]   5s 一条,窗口极值(倾角/角速度/饱和/omega_scale/health)
  [attach-window] 逐帧(20Hz),pos_err/T/z/m_est —— 暂态峰只能从这条来,
                  5s 摘要的相位伪影曾经害得三方对比误报过收益。

用法: python3 aggregate_mainline_ab.py [manifest.csv]
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
FD = re.compile(
    r'\[flight-diag\] t=([\d.]+)s \| om_scale=([\d.]+)\(lo ([\d.]+) hi ([\d.]+)\) \| '
    r'tilt_max=([\d.]+)deg \| w_max=\[r([\d.]+) p([\d.]+) y([\d.]+)\]rad/s \| '
    r'u_frac=\[T([\d.]+) r([\d.]+) p([\d.]+) y([\d.]+)\] \| '
    r'u_sat=\[T([\d.]+) r([\d.]+) p([\d.]+) y([\d.]+)\]% \| '
    r'mot_peak=([\d.]+) mot_sat=([\d.nan]+)% \| '
    r'(health=(\w+) age=([\d.e+-]+|na)s? conf=([\d.]+|na))')
AW = re.compile(r'\[attach-window\] t=([\d.-]+)s pos_err=([\d.]+)m '
                r'T=([\d.-]+)N z=([\d.-]+) m_est=([\d.-]+)')


def _stamp(line):
    m = TS.match(line)
    return float(m.group(1)) if m else None


def parse_run(nmpc_log):
    """把一份 NMPC 日志压成一行指标。"""
    r = {'log': os.path.basename(nmpc_log)}
    fd = []          # (t_nmpc, wall, om, tilt, w[3], u_frac[4], u_sat[4], mot_peak, mot_sat, health, age, conf)
    aw = []          # (t, pos_err, T, z, m_est)
    t_cmd = t_done = t_inject = t_lost = None
    solve_fail_stamps = []
    lift_wall = None

    with open(nmpc_log, errors='ignore') as f:
        for line in f:
            w = _stamp(line)
            if 'solve failed' in line:
                solve_fail_stamps.append(w)
                continue
            if 'LIFT: raising hover' in line and lift_wall is None:
                lift_wall = w
            if 'DROP command issued' in line or 'DROP: released' in line:
                if t_cmd is None:
                    t_cmd = w
            if 'DROP complete' in line and t_done is None:
                t_done = w
            if line.startswith('UNPLANNED_RELEASE_INJECTED'):
                # 批次脚本注入释放指令时追加的锚点。第二个字段是 unix 秒,
                # 与 ROS 日志行首的时间戳同一时基,W6 的恢复曲线就以它为原点。
                try:
                    t_inject = float(line.split()[1])
                except (IndexError, ValueError):
                    t_inject = None
            if 'PAYLOAD LOST' in line or 'payload-lost' in line:
                if t_lost is None:
                    t_lost = w
            m = FD.search(line)
            if m:
                g = m.groups()
                fd.append(dict(
                    t=float(g[0]), wall=w, om=float(g[1]),
                    om_lo=float(g[2]), om_hi=float(g[3]), tilt=float(g[4]),
                    w=[float(g[5]), float(g[6]), float(g[7])],
                    u_frac=[float(x) for x in g[8:12]],
                    u_sat=[float(x) for x in g[12:16]],
                    mot_peak=float(g[16]),
                    mot_sat=(float(g[17]) if g[17] not in ('nan',) else math.nan),
                    health=g[19], age=g[20], conf=g[21]))
                continue
            m = AW.search(line)
            if m:
                aw.append(tuple(float(x) for x in m.groups()))

    r['n_diag'] = len(fd)
    r['n_frame'] = len(aw)
    if not fd:
        r['status'] = 'no-diag'
        return r

    # ---- 安全极值(全轮)----
    r['tilt_max'] = max(d['tilt'] for d in fd)
    r['w_max'] = max(max(d['w']) for d in fd)
    r['w_max_roll'] = max(d['w'][0] for d in fd)
    r['w_max_pitch'] = max(d['w'][1] for d in fd)
    r['w_max_yaw'] = max(d['w'][2] for d in fd)
    for i, ch in enumerate(('T', 'r', 'p', 'y')):
        r[f'u_frac_{ch}'] = max(d['u_frac'][i] for d in fd)
        r[f'u_sat_{ch}'] = max(d['u_sat'][i] for d in fd)
    r['mot_peak'] = max(d['mot_peak'] for d in fd)
    r['mot_sat'] = max((d['mot_sat'] for d in fd
                        if not math.isnan(d['mot_sat'])), default=math.nan)

    # ---- health / freshness(只有连续主线档有)----
    hs = [d for d in fd if d['health'] != 'na']
    if hs:
        r['health_bad_windows'] = sum(1 for d in hs if d['health'] == '0')
        ages = [float(d['age']) for d in hs if d['age'] != 'na']
        r['age_max'] = max(ages) if ages else math.nan
        confs = [float(d['conf']) for d in hs if d['conf'] != 'na']
        r['conf_final'] = confs[-1] if confs else math.nan
    else:
        r['health_bad_windows'] = ''
        r['age_max'] = ''
        r['conf_final'] = ''

    # ---- drop confidence 延迟 ----
    # B 臂 = 指令 → confidence 持续达标;A 臂两者同帧(事件即完成),记 0。
    if t_cmd is not None and t_done is not None:
        r['drop_latency_s'] = round(t_done - t_cmd, 3)
    elif t_cmd is not None:
        r['drop_latency_s'] = 0.0
    else:
        r['drop_latency_s'] = ''
    # W6:外部注入没有 ROS 时间戳,用 payload_lost / 第一个 conf 达标窗口兜底
    if t_cmd is None and t_lost is not None and lift_wall is not None:
        r['lost_detect_after_lift_s'] = round(t_lost - lift_wall, 2)

    # ---- omega-scale 恢复(峰 → ≤1.05)----
    t_ref_wall = t_cmd if t_cmd is not None else t_inject
    r['om_peak'] = max(d['om_hi'] for d in fd)
    if t_ref_wall is not None:
        post = [d for d in fd if d['wall'] is not None and d['wall'] >= t_ref_wall]
        if post:
            pk = max(d['om_hi'] for d in post)
            rec = next((d for d in post if d['om'] <= 1.05), None)
            if rec is not None and pk > 1.05:
                dt = rec['wall'] - t_ref_wall
                r['om_recover_s'] = round(dt, 2)
                r['om_recover_slope'] = round((pk - 1.0) / dt, 4) if dt > 0 else ''
            else:
                r['om_recover_s'] = '' if pk > 1.05 else 0.0
                r['om_recover_slope'] = ''

    # ---- 位置误差(逐帧)----
    if aw:
        errs = [a[1] for a in aw]
        r['pos_err_peak'] = round(max(errs), 4)
        tail = errs[-int(min(len(errs), 400)):]      # 末 20s @20Hz
        r['pos_err_rms_tail'] = round(
            math.sqrt(sum(e * e for e in tail) / len(tail)), 4)
        r['z_min'] = round(min(a[3] for a in aw), 3)
        r['m_est_final'] = round(aw[-1][4], 3)
    # drop 前后切分的暂态峰:以 drop 指令的 nmpc 时刻为界
    if t_cmd is not None and aw:
        # 用 wall 找不到 nmpc_time,退而用 [flight-diag] 里最接近的 t
        near = min(fd, key=lambda d: abs((d['wall'] or 0) - t_cmd))
        t_split = near['t']
        post = [a for a in aw if a[0] >= t_split]
        if post:
            r['pos_err_peak_post_drop'] = round(max(a[1] for a in post), 4)

    # ---- 求解失败(按 drop 切分)----
    r['solve_fail'] = len(solve_fail_stamps)
    if t_cmd is not None:
        r['solve_fail_post'] = sum(1 for s in solve_fail_stamps
                                   if s is not None and s >= t_cmd)
        r['solve_fail_pre'] = r['solve_fail'] - r['solve_fail_post']

    # ---- 安全失败判定 ----
    fails = []
    if r['tilt_max'] > 60.0:
        fails.append('tilt>60deg')
    if r['solve_fail'] > 100:
        fails.append('solve_fail>100')       # 闭环失稳的既有判据
    if r.get('pos_err_peak', 0) > 3.0:
        fails.append('pos_err>3m')
    if r.get('z_min', 9e9) < 0.15:
        fails.append('z<0.15m')
    r['safety_fail'] = ';'.join(fails)
    r['status'] = 'FAIL' if fails else 'ok'
    return r


COLS = ['idx', 'wid', 'arm', 'stamp', 'status', 'safety_fail',
        'drop_latency_s', 'om_peak', 'om_recover_s', 'om_recover_slope',
        'tilt_max', 'w_max', 'w_max_roll', 'w_max_pitch', 'w_max_yaw',
        'u_frac_T', 'u_frac_r', 'u_frac_p', 'u_frac_y',
        'u_sat_T', 'u_sat_r', 'u_sat_p', 'u_sat_y', 'mot_peak', 'mot_sat',
        'pos_err_peak', 'pos_err_peak_post_drop', 'pos_err_rms_tail', 'z_min',
        'm_est_final', 'health_bad_windows', 'age_max', 'conf_final',
        'lost_detect_after_lift_s',
        'solve_fail', 'solve_fail_pre', 'solve_fail_post',
        'n_diag', 'n_frame', 'log']


def main():
    man = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        RES, 'mainline_ab_manifest.csv')
    rows = []
    with open(man) as f:
        for m in csv.DictReader(f):
            if m['stamp'] in ('NA', ''):
                continue
            log = os.path.join(RES, f"grip_nmpc_{m['stamp']}.log")
            if not os.path.exists(log):
                continue
            r = parse_run(log)
            r.update(idx=m['idx'], wid=m['wid'], arm=m['arm'], stamp=m['stamp'])
            if m.get('status') not in ('ok', None, ''):
                r['safety_fail'] = ((r.get('safety_fail') or '')
                                    + ';batch:' + m['status']).strip(';')
                r['status'] = 'FAIL'
            rows.append(r)

    out = os.path.join(RES, 'mainline_ab_metrics.csv')
    with open(out, 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=COLS, extrasaction='ignore')
        wr.writeheader()
        for r in rows:
            wr.writerow(r)
    print(f'{len(rows)} 架次 -> {out}\n')

    # ---- 逐工况配对汇总 ----
    by = defaultdict(lambda: defaultdict(list))
    for r in rows:
        by[r['wid']][r['arm']].append(r)

    def med(v):
        v = sorted(x for x in v if isinstance(x, (int, float))
                   and not (isinstance(x, float) and math.isnan(x)))
        if not v:
            return math.nan
        n = len(v)
        return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])

    print(f"{'工况':<5}{'臂':<4}{'n':>3}{'安全失败':>9}"
          f"{'倾角max':>9}{'ω max':>8}{'posErr峰':>10}{'稳态rms':>9}"
          f"{'drop延迟':>9}{'ω恢复s':>8}{'solveFail':>10}")
    for wid in sorted(by):
        for arm in ('A', 'B'):
            g = by[wid].get(arm, [])
            if not g:
                continue
            nf = sum(1 for r in g if r['status'] == 'FAIL')
            print(f"{wid:<5}{arm:<4}{len(g):>3}{nf:>9}"
                  f"{med([r.get('tilt_max') for r in g]):>9.2f}"
                  f"{med([r.get('w_max') for r in g]):>8.3f}"
                  f"{med([r.get('pos_err_peak') for r in g]):>10.3f}"
                  f"{med([r.get('pos_err_rms_tail') for r in g]):>9.4f}"
                  f"{med([r.get('drop_latency_s') for r in g]):>9.2f}"
                  f"{med([r.get('om_recover_s') for r in g]):>8.2f}"
                  f"{med([r.get('solve_fail') for r in g]):>10.1f}")

    nB = [r for r in rows if r['arm'] == 'B']
    print(f"\n主线判据: B 臂 {sum(1 for r in nB if r['status']=='FAIL')}/{len(nB)} "
          f"架次安全失败(要求 0)")


if __name__ == '__main__':
    main()
