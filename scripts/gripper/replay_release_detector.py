#!/usr/bin/env python3
"""离线回放:拿历史 SITL 日志验证 drop 侧释放判据(2026-09-05)。

为什么要离线回放:09-04 的死锁是"跑一轮才发现"的,而单轮 n=1 又严重高估了
判据的分离度(首轮看着 5.5×,五轮一算只剩 1.10×)。判据的阈值必须先在**已有的
上百轮日志**上验证过,再花几十分钟去跑 SITL。

数据源(按优先级):
  [s-collapse] 行 —— 2026-09-05 起才有,10Hz 抽样成 1Hz,字段最全;
  [truth]      行 —— 历史轮次都有,~0.5Hz,含 s_hat 与 m_hat,够重建 (|s|, m_p)。

用法:
  python3 replay_release_detector.py                 # 全量回放 + 各项检查
  python3 replay_release_detector.py --limit 40      # 只看最近 40 轮
  python3 replay_release_detector.py --loocv         # 留一轮交叉验证阈值
"""

import argparse
import glob
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..',
                                'offboard_test_acados'))

from offboard_test_acados.payload_estimate import (  # noqa: E402
    RELEASE_MASS_MP,
    RELEASE_RATIO_THR,
    RELEASE_S_ABS_MAX,
    release_decision,
    release_evidence,
)

RES = '/home/clear/ros2_ws_HJH/nmpc_test_results'
M_B = 2.0643

RE_T = re.compile(r'\[(\d+\.\d+)\]')
RE_COLLAPSE = re.compile(r'\|s\|=([\d.]+) peak=([\d.]+) ratio=([\d.]+).*?m_p=([-+\d.]+)')
RE_TRUTH_S = re.compile(r's_hat=\[\s*([-+0-9.]+),\s*([-+0-9.]+)\]')
RE_TRUTH_M = re.compile(r'm_hat=([-+0-9.]+)')


class Round:
    """一轮的可回放序列。"""

    def __init__(self, stamp, frames, t_drop_cmd, src, released_online):
        self.stamp = stamp
        self.frames = frames            # [(t, s_norm, m_p)]
        self.t_drop_cmd = t_drop_cmd    # None = 这一轮没发过 drop 指令
        self.src = src
        self.released_online = released_online

    @property
    def loaded_frames(self):
        """带载帧:drop 指令之前(没 drop 的轮次 = 全程带载)。"""
        if self.t_drop_cmd is None:
            return self.frames
        return [f for f in self.frames if f[0] < self.t_drop_cmd]

    @property
    def unloaded_frames(self):
        if self.t_drop_cmd is None:
            return []
        return [f for f in self.frames if f[0] >= self.t_drop_cmd]


def load_round(mhe_path):
    stamp = re.sub(r'^.*?mhe_', '', os.path.basename(mhe_path))[:-4]
    nmpc_path = mhe_path.replace('mhe_', 'nmpc_')
    if not os.path.exists(nmpc_path):
        return None
    try:
        ntxt = open(nmpc_path, errors='ignore').read()
        mtxt = open(mhe_path, errors='ignore').read()
    except OSError:
        return None
    md = re.search(r'\[(\d+\.\d+)\].*DROP command', ntxt)
    if md is None:
        md = re.search(r'\[(\d+\.\d+)\].*DROP: released', ntxt)
    t_drop = float(md.group(1)) if md else None

    frames, src = [], None
    if '[s-collapse]' in mtxt:
        src = 's-collapse'
        for ln in mtxt.splitlines():
            if '[s-collapse]' not in ln:
                continue
            mt, mc = RE_T.search(ln), RE_COLLAPSE.search(ln)
            if mt and mc:
                frames.append((float(mt.group(1)), float(mc.group(1)),
                               float(mc.group(4))))
    if len(frames) < 15:
        frames, src = [], 'truth'
        for ln in mtxt.splitlines():
            if '[truth]' not in ln:
                continue
            mt, ms, mm = RE_T.search(ln), RE_TRUTH_S.search(ln), RE_TRUTH_M.search(ln)
            if mt and ms and mm:
                sx, sy = float(ms.group(1)), float(ms.group(2))
                frames.append((float(mt.group(1)), (sx * sx + sy * sy) ** 0.5,
                               float(mm.group(1)) - M_B))
    if len(frames) < 15:
        return None
    return Round(stamp, frames, t_drop, src, '载荷释放' in mtxt)


def _sample_dt(frames):
    """帧间隔中位数。日志的抽样率决定 persist 该折算成几帧。"""
    if len(frames) < 3:
        return 1.0
    d = sorted(frames[i + 1][0] - frames[i][0] for i in range(len(frames) - 1))
    return max(d[len(d) // 2], 1e-3)


def replay(rnd, ratio_thr, mass_mp, s_abs, slow_persist, fast_persist,
           residual_at=None, arm_mp=0.09, arm_persist=3):
    """回放一轮,返回 (释放时刻 or None, 是否在带载段误释放, 峰值)。

    武装:m_p 连续 arm_persist 帧高于 arm_mp 才算"可靠进入过 LOADED"。
    峰值:只在武装后累计(与在线实现一致)。
    residual_at:注入残差 DROP 证据的时刻(None = 该通道全程为假,
    对应默认档 4.4N 把残差路堵死的现实)。

    ⚠️ persist 传进来的是**在线 10Hz 下的帧数**,这里按本轮日志的实际抽样率
    折算成秒再折回帧。不折算的话,0.5Hz 的 [truth] 数据要求"连续 5 帧"就等于
    要求连续 10 秒同时满足 —— 实测会把 33/72 轮判成漏检,而那些轮的 ratio 与
    m_p 其实早就双双达标,只是没连着 5 个采样点。那是回放框架的伪影,不是判据。
    """
    dt = _sample_dt(rnd.frames)
    slow_persist = max(1, round(slow_persist * 0.1 / dt))
    fast_persist = max(1, round(fast_persist * 0.1 / dt))
    armed = False
    hi = 0
    peak = 0.0
    slow = fast = 0
    for t, s_norm, m_p in rnd.frames:
        if not armed:
            hi = hi + 1 if m_p > arm_mp else 0
            if hi >= arm_persist:
                armed = True
            continue
        peak = max(peak, s_norm)
        resid = bool(residual_at is not None and t >= residual_at)
        ev = release_evidence(m_p, s_norm, peak, resid,
                              ratio_thr=ratio_thr, s_abs_max=s_abs,
                              mass_release_mp=mass_mp)
        fire, _why, slow, fast = release_decision(
            ev, armed, slow, fast,
            slow_persist=slow_persist, fast_persist=fast_persist)
        if fire:
            false_pos = (rnd.t_drop_cmd is None or t < rnd.t_drop_cmd)
            return t, false_pos, peak
    return None, False, peak


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--limit', type=int, default=0)
    ap.add_argument('--loocv', action='store_true')
    ap.add_argument('--ratio', type=float, default=RELEASE_RATIO_THR)
    args = ap.parse_args()

    # 按 mtime 排,不是字典序:字典序会把 2026-07 的 mhe_node_*.log 排到最后,
    # --limit 于是专挑最老的那批(全是 wrench 场景,没有 s_hat 也没有配对 nmpc)。
    paths = sorted(glob.glob(os.path.join(RES, '*mhe_*.log')),
                   key=os.path.getmtime)
    if args.limit:
        paths = paths[-args.limit:]
    rounds = [r for r in (load_round(p) for p in paths) if r is not None]
    with_drop = [r for r in rounds if r.t_drop_cmd is not None]
    no_drop = [r for r in rounds if r.t_drop_cmd is None]
    print(f'可回放轮次 {len(rounds)} (有 drop 指令 {len(with_drop)}, '
          f'无 drop {len(no_drop)}); 数据源 s-collapse '
          f'{sum(1 for r in rounds if r.src == "s-collapse")} / truth '
          f'{sum(1 for r in rounds if r.src == "truth")}')

    # ---- 1. 默认阈值下的全量回放 ----
    print(f'\n=== 1. 全量回放 (ratio<{args.ratio}, mass<{RELEASE_MASS_MP}, '
          f'|s|<{RELEASE_S_ABS_MAX}; 残差通道关=默认档) ===')
    miss, fp, ok = [], [], []
    for r in with_drop:
        t, false_pos, _ = replay(r, args.ratio, RELEASE_MASS_MP,
                                 RELEASE_S_ABS_MAX, 5, 2)
        if t is None:
            miss.append(r)
        elif false_pos:
            fp.append((r, t))
        else:
            ok.append((r, t - r.t_drop_cmd))
    print(f'  释放成功 {len(ok)}/{len(with_drop)}; 漏检 {len(miss)}; '
          f'带载段误释放 {len(fp)}')
    if ok:
        lat = sorted(d for _, d in ok)
        print(f'  释放延迟 中位 {lat[len(lat)//2]:.1f}s  '
              f'p90 {lat[int(len(lat)*0.9)]:.1f}s  max {lat[-1]:.1f}s')
    for r, t in fp[:8]:
        print(f'  !! 误释放 {r.stamp} @t={t:.1f} (drop 指令 @{r.t_drop_cmd:.1f})')
    for r in miss[:8]:
        print(f'  -- 漏检 {r.stamp} (src={r.src}, {len(r.frames)} 帧)')

    # ---- 2. 全部历史带载帧的零误释放 ----
    print('\n=== 2. 无 drop 轮次(全程带载)的误释放 ===')
    fp2 = 0
    for r in no_drop:
        t, _, _ = replay(r, args.ratio, RELEASE_MASS_MP, RELEASE_S_ABS_MAX, 5, 2)
        if t is not None:
            fp2 += 1
            if fp2 <= 5:
                print(f'  !! {r.stamp} @t={t:.1f}')
    n_loaded = sum(len(r.loaded_frames) for r in rounds)
    print(f'  {len(no_drop)} 轮全程带载: 误释放 {fp2} 轮')
    print(f'  全库带载帧合计 {n_loaded} 帧')

    # ---- 3. 对抗:最小 peak x 最大残噪 ----
    print('\n=== 3. 对抗测试(人工组合最坏情况) ===')
    peaks = [p for p in (replay(r, args.ratio, RELEASE_MASS_MP,
                                RELEASE_S_ABS_MAX, 5, 2)[2]
                         for r in with_drop) if p > 0]
    resid = []
    for r in with_drop:
        u = r.unloaded_frames
        if u:
            resid.append(min(s for _, s, _ in u[:20]) if len(u) > 3
                         else u[0][1])
    if peaks and resid:
        p_min, s_max = min(peaks), max(resid)
        print(f'  实测 peak 最小 {p_min:.4f}, 卸载后残噪最大 {s_max:.4f}')
        print(f'  最坏组合 ratio = {s_max/p_min:.3f}  '
              f'=> {"漏检(退回死锁)" if s_max/p_min >= args.ratio else "仍可释放"}')
        for name, rd in (('残差通道关(默认档)', None), ('残差通道开', 0.0)):
            ev = release_evidence(0.0, s_max, p_min, rd is not None,
                                  ratio_thr=args.ratio)
            fire, why, _, _ = release_decision(ev, True, 99, 99)
            print(f'    {name}: {"可释放" if fire else "不释放"} '
                  f'{why.split("(")[0] if why else ""}')

    # ---- 4. 留一轮交叉验证 ----
    if args.loocv:
        print('\n=== 4. LOOCV 阈值(在 N-1 轮上选,在留出轮上验) ===')
        grid = [0.15, 0.18, 0.20, 0.22, 0.25, 0.30, 0.35]
        held_ok = 0
        picks = []
        for i, held in enumerate(with_drop):
            best, best_score = None, (-1, 1e9)
            for g in grid:
                good = bad = 0
                for j, r in enumerate(with_drop):
                    if j == i:
                        continue
                    t, f_pos, _ = replay(r, g, RELEASE_MASS_MP,
                                         RELEASE_S_ABS_MAX, 5, 2)
                    if t is not None and not f_pos:
                        good += 1
                    if f_pos:
                        bad += 1
                score = (good - 5 * bad, g)
                if score > best_score:
                    best_score, best = score, g
            picks.append(best)
            t, f_pos, _ = replay(held, best, RELEASE_MASS_MP,
                                 RELEASE_S_ABS_MAX, 5, 2)
            if t is not None and not f_pos:
                held_ok += 1
        from collections import Counter
        print(f'  留出轮成功 {held_ok}/{len(with_drop)}')
        print(f'  选中的阈值分布: {dict(Counter(picks))}')


if __name__ == '__main__':
    main()
