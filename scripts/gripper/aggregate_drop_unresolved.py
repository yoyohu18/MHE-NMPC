#!/usr/bin/env python3
"""UNRESOLVED 构造回归的聚合(2026-09-07),配 run_drop_unresolved_regress.sh。

判据(与驱动脚本头一致,跑前钉死):
  ① 机制判据(确定性):UNRESOLVED 行必须声明平滑制动到停止点，且之后**不应**
     再出现 "DYNAMIC: switch to figure8"。n=1 即可判,不受随机性影响。
  ② 结局判据(计数):drop 后 peak_post>5m 的轮次数。
  ③ 前置:drop 之前必须正常(peak_pre<2m),否则该轮作废(drop 前的发散是另一个问题)。

用法: python3 aggregate_drop_unresolved.py <before.txt> [after.txt]
"""
import re
import sys
from pathlib import Path

RUNDIR = Path('/home/clear/ros2_ws_HJH/nmpc_test_results')
W = re.compile(r'\[(\d+\.\d+)\]')


def per_run(stamp):
    n = RUNDIR / f'grip_nmpc_{stamp}.log'
    if not n.exists():
        return None
    t_drop = t_unres = t_restart = None
    restart_after_unres = False
    fingerprint = []
    pe = []
    for ln in n.open(errors='ignore'):
        w = W.search(ln)
        w = float(w.group(1)) if w else None
        if w is None:
            continue
        if t_drop is None and ('DROP command issued' in ln
                               or 'DROP: released gripper' in ln):
            t_drop = w
        if t_unres is None and 'PAYLOAD STATE UNRESOLVED' in ln:
            t_unres = w
            # ★ 版本指纹(2026-09-07 血泪):run_gripper_headless.sh:258 每轮都
            # colcon build,批次跑到一半改 src 会让后续轮次换成新代码,而
            # manifest 与日志名**完全看不出来**(实测一批 6 轮里 rep1 旧、
            # rep2~6 新)。这一行的文案随补丁改过,拿它当逐轮的版本标记。
            code_ver = ('brake-hover' if '平滑制动后在停止点悬停' in ln
                        else ('latch-only' if '继续按当前参考飞行' in ln
                              else ('buggy' if '退出 figure-8 转保守悬停' in ln
                                    else '?')))
            fingerprint.append(code_ver)
        if 'DYNAMIC: switch to figure8' in ln:
            # ★ 机制判据:只数 UNRESOLVED **之后**那一次(第一次是正常的进入机动)
            if t_unres is not None and w >= t_unres:
                restart_after_unres = True
                if t_restart is None:
                    t_restart = w
        m = re.search(r'pos_err=([\d.]+)', ln)
        if m:
            pe.append((w, float(m.group(1))))
    if t_drop is None:
        return None
    return {
        'unres': (t_unres - t_drop) if t_unres else None,
        'restart': (t_restart - t_unres) if (t_restart and t_unres) else None,
        'restarted': restart_after_unres,
        'peak_pre': max([v for w, v in pe if w <= t_drop], default=float('nan')),
        'peak_post': max([v for w, v in pe if w > t_drop], default=float('nan')),
        'code': fingerprint[0] if fingerprint else '-',
    }


def group(man):
    rows = [l.split() for l in Path(man).read_text().splitlines()
            if not l.startswith('#') and l.strip()]
    tag = rows[0][0] if rows else '?'
    out = []
    for f in rows:
        if f[2] == '-':
            print(f"  ✗ {f[0]} rep{f[1]} {f[3]} 作废"); continue
        d = per_run(f[2])
        if d is None:
            print(f"  ✗ {f[0]} rep{f[1]} {f[2]} 拿不到 DROP 锚点,作废"); continue
        d['rep'] = f[1]; d['stamp'] = f[2]
        out.append(d)
    return tag, out


def report(tag, rows):
    print(f"\n===== {tag}  n={len(rows)} =====")
    print(f"{'rep':<5}{'stamp':<18}{'code':>7}{'UNRES@drop+':>12}{'重启延迟':>10}"
          f"{'peak_pre':>10}{'peak_post':>11}  结局")
    ok = crash = voided = 0
    restarts = 0
    for r in rows:
        if not (r['peak_pre'] < 2.0):
            verdict = '前置作废(drop 前就发散)'; voided += 1
        elif r['peak_post'] > 5.0:
            verdict = '★穿地'; crash += 1
        else:
            verdict = '正常'; ok += 1
        if r['restarted']:
            restarts += 1
        u = f"{r['unres']:+.2f}" if r['unres'] is not None else "未触发"
        rs = f"{r['restart']:+.3f}" if r['restart'] is not None else "  无"
        print(f"{r['rep']:<5}{r['stamp']:<18}{r['code']:>7}{u:>12}{rs:>10}"
              f"{r['peak_pre']:10.2f}{r['peak_post']:11.2f}  {verdict}")
    vers = sorted({r['code'] for r in rows if r['code'] != '-'})
    if len(vers) > 1:
        print(f"\n  ⚠️ **这一组混了 {len(vers)} 个代码版本** {vers} —— 批次中途改过 src"
              "(headless 每轮 colcon build)。必须按 code 列分开读,不能整组统计。")
    n_valid = ok + crash
    n_unres = sum(1 for r in rows if r['unres'] is not None)
    print(f"\n  ① 机制:UNRESOLVED 触发 {n_unres}/{len(rows)} 轮;"
          f"其后又重启 figure-8 的 **{restarts}/{len(rows)}** 轮")
    print(f"  ② 结局:有效 {n_valid} 轮中 drop 后穿地 **{crash}**,正常 {ok}"
          + (f"(另有 {voided} 轮 drop 前就发散,已作废)" if voided else ""))
    return {'n': len(rows), 'restarts': restarts, 'crash': crash,
            'valid': n_valid, 'unres': n_unres}


def main(mans):
    res = []
    for man in mans:
        tag, rows = group(man)
        res.append((tag, report(tag, rows)))
    if len(res) == 2:
        (t0, a), (t1, b) = res
        print("\n===== 对比 =====")
        print(f"  figure-8 重启:  {t0} {a['restarts']}/{a['n']}  →  {t1} {b['restarts']}/{b['n']}")
        print(f"  drop 后穿地:    {t0} {a['crash']}/{a['valid']}  →  {t1} {b['crash']}/{b['valid']}")
        if b['unres'] == 0:
            print("  ⚠️ after 组一次都没触发 UNRESOLVED —— 构造没生效,这组不能用来判修复。")
        elif b['restarts'] == 0 and a['restarts'] > 0:
            print("  → ① 机制判据通过:缺陷在场时每轮重启,修复后不再重启。")
            if b['crash'] == 0 and a['crash'] > 0:
                print("  → ② 结局判据通过:构造工况下的 drop 后穿地已消失。")
            elif b['crash'] > 0:
                print("  → ⚠️ 机制修好了但仍有穿地 —— 说明还有第二条路径,不能收工。")
        else:
            print("  → ① 机制判据**未通过**:修复后仍在重启,先查 latch 是否真的生效。")


if __name__ == '__main__':
    main(sys.argv[1:])
