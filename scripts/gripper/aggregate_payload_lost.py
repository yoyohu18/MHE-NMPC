#!/usr/bin/env python3
"""载荷意外脱落看门狗 on/off 批次聚合 (2026-08-30)。

时间锚点用 proximity 日志的 DETACH epoch —— 那是载荷真正离开机体的**真值**时刻。
不能用 driver 的墙钟或 "[pl] >>>" 那行:前者精度只到秒,后者不带时间戳。
off 臂没有 payload-lost 事件,只有这个锚点能让两臂对齐同一 t0。

⚠️ run_payload_lost_test.sh 自己打的 "peak pos_err" 是**全轮**最大值,含起飞/ramp
段,不可用作臂间比较 —— 本脚本只统计 t0 之后的窗口。
"""
import re, sys, glob, os, statistics as st

RUNDIR = "/home/clear/ros2_ws_HJH/nmpc_test_results"
TS = re.compile(r'\[(?:INFO|WARN|ERROR)\] \[(\d+\.\d+)\]')
PE = re.compile(r'pos_err=([0-9.]+)')
SETTLE_BAND = 0.30   # m,回到带内
SETTLE_HOLD = 3.0    # s,持续时长

def ts(line):
    m = TS.search(line)
    return float(m.group(1)) if m else None

def one_round(stamp):
    npath = f"{RUNDIR}/grip_nmpc_{stamp}.log"
    mpath = f"{RUNDIR}/grip_mhe_{stamp}.log"
    ppath = f"{RUNDIR}/grip_proximity_{stamp}.log"
    if not os.path.exists(npath):
        return None
    t0 = t_att = None
    for ln in open(ppath, errors='ignore') if os.path.exists(ppath) else []:
        if '-> ATTACH' in ln and t_att is None:
            t_att = ts(ln)
        if '-> DETACH' in ln:
            t0 = ts(ln)          # 最后一次 DETACH = 本轮意外脱落
    if t0 is None:
        return {'stamp': stamp, 'err': 'no DETACH'}

    # MHE 检出 + NMPC 复位
    # ⚠️ 2026-08-31 rep3_on 实测:看门狗可能在 ATTACH 后 ~4s 的 lift 瞬态里**误触发**
    # (ΔT=-2.40N 刚过 2.0N 门槛),此时载荷仍在机上,增益被错误复位 → 炸机。
    # 这类轮次的失稳始于**触发时刻**而非 DETACH,若仍按 t0 切窗口会把语义算错
    # (post-t0 段量到的是坠毁后的残值)。故分析起点 t_ref 取首次触发与 t0 的**较早者**。
    t_det = t_rst = None
    n_false = 0
    trigs = []
    for ln in open(mpath, errors='ignore') if os.path.exists(mpath) else []:
        if '检出载荷意外脱落' in ln:
            t = ts(ln)
            if t is None: continue
            trigs.append(t)
            if t < t0: n_false += 1          # 脱落**前**触发 = 误报
            elif t_det is None: t_det = t
    series, sf_pre, sf_post, t_rst = [], 0, 0, None
    for ln in open(npath, errors='ignore'):
        t = ts(ln)
        if 'MC_ROLLRATE_K' in ln and 'reset' in ln and t and t >= t0 and t_rst is None:
            t_rst = t
        if 'solve failed' in ln and t:
            if t >= t0: sf_post += 1
            else: sf_pre += 1
        _ = None
        m = PE.search(ln)
        if m and t is not None:
            series.append((t, float(m.group(1))))
    t_fp = min((t for t in trigs if t < t0), default=None)
    t_ref = t_fp if t_fp is not None else t0
    post = [(t - t_ref, v) for t, v in series if t >= t_ref]
    if not post:
        return {'stamp': stamp, 'err': 'no post-t0 samples'}
    peak = max(v for _, v in post)
    # 恢复时间:首次进入 ±SETTLE_BAND 并保持 SETTLE_HOLD 秒不再出带
    settle = None
    for i, (dt, v) in enumerate(post):
        if v > SETTLE_BAND: continue
        if all(vv <= SETTLE_BAND for tt, vv in post[i:] if tt <= dt + SETTLE_HOLD):
            if post[-1][0] - dt >= SETTLE_HOLD:      # 窗口够长才算数(防右删失)
                settle = dt; break
    tail = [v for dt, v in post if dt >= post[-1][0] - 10.0]
    return {'stamp': stamp, 'obs': round(post[-1][0], 1), 'peak': round(peak, 3),
            'fp_at': None if (t_fp is None or t_att is None) else round(t_fp - t_att, 1),
            'settle': None if settle is None else round(settle, 2),
            'tail_med': round(st.median(tail), 3) if tail else None,
            'det_ms': None if t_det is None else round((t_det - t0) * 1e3),
            'rst_ms': None if t_rst is None else round((t_rst - t0) * 1e3),
            'false_trig': n_false, 'sf_pre': sf_pre, 'sf_post': sf_post}

if __name__ == '__main__':
    rows = []
    for tag, stamp in (l.split() for l in sys.stdin if l.strip()):
        r = one_round(stamp)
        if r: r['tag'] = tag; rows.append(r)
    hdr = ['tag','stamp','obs','peak','settle','tail_med','det_ms','fp_at','false_trig','sf_post']
    print(' | '.join(f'{h:>10}' for h in hdr))
    for r in rows:
        print(' | '.join(f'{str(r.get(h, r.get("err","-"))):>10}' for h in hdr))
    for arm in ('on', 'off'):
        sub = [r for r in rows if arm == r['tag'].split('_')[-1] and 'peak' in r]
        pk = [r['peak'] for r in sub if 'peak' in r]
        se = [r['settle'] for r in sub if r.get('settle') is not None]
        fp = [r for r in sub if r.get('false_trig')]
        crash = [r for r in sub if r['peak'] > 5.0]
        if pk:
            print(f"\n[{arm}] n={len(sub)} 误触发 {len(fp)}/{len(sub)} 轮 | "
                  f"峰值>5m {len(crash)}/{len(sub)} 轮")
            print(f"[{arm}] peak 中位={st.median(pk):.3f}m "
                  f"({min(pk):.3f}~{max(pk):.3f}) | 恢复 {len(se)}/{len(sub)} 轮"
                  + (f",中位 {st.median(se):.2f}s" if se else ""))
