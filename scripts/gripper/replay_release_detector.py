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
  python3 replay_release_detector.py --manifest paper/manifests/release_replay_20260905.csv --loocv
"""

import argparse
import csv
import glob
import hashlib
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

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_BUNDLED_RES = os.path.join(_REPO, 'paper', 'data')
_LEGACY_RES = os.path.join(os.path.dirname(_REPO), 'nmpc_test_results')
RES = os.environ.get('NMPC_RESULTS_DIR')
if not RES:
    RES = (_BUNDLED_RES if os.path.isfile(
           os.path.join(_BUNDLED_RES, 'mainline_ab2_manifest.csv'))
           else _LEGACY_RES)
M_B = 2.0643

RE_T = re.compile(r'\[(\d+\.\d+)\]')
RE_COLLAPSE = re.compile(r'\|s\|=([\d.]+) peak=([\d.]+) ratio=([\d.]+).*?m_p=([-+\d.]+)')
RE_TRUTH_S = re.compile(r's_hat=\[\s*([-+0-9.]+),\s*([-+0-9.]+)\]')
RE_TRUTH_M = re.compile(r'm_hat=([-+0-9.]+)')


def _sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def load_manifest(path, verify_hashes=True):
    """Load an ordered, frozen replay cohort and verify its file identities."""
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    required = {'order', 'stamp', 'mhe_log', 'nmpc_log', 'validity',
                'mhe_sha256', 'nmpc_sha256'}
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f'{path}: missing manifest columns {sorted(required)}')

    seen_stamps = set()
    entries = []
    for expected_order, row in enumerate(rows, 1):
        if int(row['order']) != expected_order:
            raise ValueError(f'{path}: order is not contiguous at row {expected_order}')
        if row['stamp'] in seen_stamps:
            raise ValueError(f'{path}: duplicate stamp {row["stamp"]}')
        seen_stamps.add(row['stamp'])
        if (os.path.basename(row['mhe_log']) != row['mhe_log']
                or os.path.basename(row['nmpc_log']) != row['nmpc_log']):
            raise ValueError(f'{path}: log paths must be basenames')
        mhe_path = os.path.join(RES, row['mhe_log'])
        nmpc_path = os.path.join(RES, row['nmpc_log'])
        for label, log_path, digest in (
                ('MHE', mhe_path, row['mhe_sha256']),
                ('NMPC', nmpc_path, row['nmpc_sha256'])):
            if not os.path.isfile(log_path):
                raise FileNotFoundError(f'{path}: missing {label} log {log_path}')
            if verify_hashes and _sha256(log_path) != digest:
                raise ValueError(f'{path}: SHA-256 mismatch for {log_path}')
        entries.append((row, mhe_path))
    return entries


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
    slow = fast = strong = 0
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
        fire, _why, slow, fast, strong = release_decision(
            ev, armed, slow, fast,
            slow_persist=slow_persist, fast_persist=fast_persist,
            strong_frames=strong)
        if fire:
            false_pos = (rnd.t_drop_cmd is None or t < rnd.t_drop_cmd)
            return t, false_pos, peak
    return None, False, peak


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--limit', type=int, default=0)
    ap.add_argument('--manifest', help='ordered frozen-cohort CSV')
    ap.add_argument('--skip-hash-check', action='store_true',
                    help='do not verify manifest SHA-256 values')
    ap.add_argument('--loocv', action='store_true')
    ap.add_argument('--ratio', type=float, default=RELEASE_RATIO_THR)
    args = ap.parse_args()

    if args.manifest and args.limit:
        ap.error('--manifest and --limit are mutually exclusive')
    if args.manifest:
        entries = load_manifest(args.manifest, not args.skip_hash_check)
        rounds = []
        for row, path in entries:
            derived_nmpc = os.path.basename(path.replace('mhe_', 'nmpc_'))
            if derived_nmpc != row['nmpc_log']:
                raise ValueError(
                    f'{args.manifest}: MHE/NMPC pair mismatch for {row["stamp"]}')
            r = load_round(path)
            if r is None:
                raise ValueError(
                    f'{args.manifest}: frozen round is no longer parseable: {path}')
            if r.stamp != row['stamp']:
                raise ValueError(
                    f'{args.manifest}: stamp mismatch {row["stamp"]} != {r.stamp}')
            r.validity = row['validity']
            rounds.append(r)
        print(f'冻结清单 {args.manifest}: {len(rounds)} 轮；文件 SHA-256 '
              f'{"未检查" if args.skip_hash_check else "全部通过"}')
    else:
        # 探索模式保留历史行为。论文数字不得使用这个会随目录增长的入口。
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

    valid_with_drop = None
    if args.manifest:
        valid_rounds = [r for r in rounds if r.validity == 'valid']
        valid_with_drop = [r for r in valid_rounds if r.t_drop_cmd is not None]
        valid_ok = valid_miss = valid_fp = 0
        for r in valid_with_drop:
            t, false_pos, _ = replay(r, args.ratio, RELEASE_MASS_MP,
                                     RELEASE_S_ABS_MAX, 5, 2)
            if t is None:
                valid_miss += 1
            elif false_pos:
                valid_fp += 1
            else:
                valid_ok += 1
        print(f'  manifest-valid 子集 {valid_ok}/{len(valid_with_drop)} 检出; '
              f'漏检 {valid_miss}; 带载段误释放 {valid_fp}')

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
        for name, strong in (('普通残差票', False), ('强残差票', True)):
            ev = release_evidence(0.0, s_max, p_min, True,
                                  ratio_thr=args.ratio, residual_strong=strong)
            fire, why, _, _, _ = release_decision(ev, True, 99, 99,
                                                  strong_frames=99)
            print(f'    {name}: {"可释放" if fire else "不释放"} '
                  f'{why.split("(")[0] if why else ""}')
        print('    (2026-09-05 起任何释放都必须带 moment 证据,'
              'ratio>阈值时两种票都不该释放)')

    # ---- 4. 留一轮交叉验证 ----
    if args.loocv:
        print('\n=== 4. LOOCV 阈值(在 N-1 轮上选,在留出轮上验) ===')
        grid = [0.15, 0.18, 0.20, 0.22, 0.25, 0.30, 0.35]
        cv_rounds = valid_with_drop if valid_with_drop is not None else with_drop
        held_ok = 0
        picks = []
        for i, held in enumerate(cv_rounds):
            best, best_score = None, (float('-inf'), float('-inf'))
            for g in grid:
                good = bad = 0
                for j, r in enumerate(cv_rounds):
                    if j == i:
                        continue
                    t, f_pos, _ = replay(r, g, RELEASE_MASS_MP,
                                         RELEASE_S_ABS_MAX, 5, 2)
                    if t is not None and not f_pos:
                        good += 1
                    if f_pos:
                        bad += 1
                # 同分时选更小阈值（更保守），而不是意外偏向网格最大值。
                score = (good - 5 * bad, -g)
                if score > best_score:
                    best_score, best = score, g
            picks.append(best)
            t, f_pos, _ = replay(held, best, RELEASE_MASS_MP,
                                 RELEASE_S_ABS_MAX, 5, 2)
            if t is not None and not f_pos:
                held_ok += 1
        from collections import Counter
        print(f'  留出轮成功 {held_ok}/{len(cv_rounds)}')
        print(f'  选中的阈值分布: {dict(Counter(picks))}')


def check_release_residual(fixture=None, half=3, floor=0.9, strong_persist=4,
                          persist=2):
    """用高频 T_phys fixture 校验 release-residual 票(默认档的第三个信息源)。

    0.5Hz 的 [truth] 日志做不了这一步:短窗阶跃要的是 10Hz 原始推力。
    """
    fixture = fixture or os.path.join(
        os.path.dirname(__file__), '..', '..', 'offboard_test_acados',
        'test', 'fixtures', 'tphys_replay_20260826_165455.csv')
    if not os.path.exists(fixture):
        print('  (缺 fixture,跳过)')
        return
    ts, T = [], []
    for ln in open(fixture):
        if ln.startswith('#'):
            continue
        parts = ln.strip().split(',')
        try:
            ts.append(float(parts[0]))
            T.append(float(parts[1]))
        except (ValueError, IndexError):
            pass
    ATT, FIG, DROP = 6.50, 19.5, 71.50
    d = [None] * len(T)
    for i in range(2 * half, len(T)):
        d[i] = (sum(T[i - half:i]) / half
                - sum(T[i - 2 * half:i - half]) / half)

    def longest(a, b, thr, need):
        best = cur = 0
        first = None
        for i, v in enumerate(d):
            if v is None or not (a <= ts[i] < b):
                continue
            if v < -thr:
                cur += 1
                best = max(best, cur)
                if cur >= need and first is None:
                    first = ts[i]
            else:
                cur = 0
        return best, first

    fig_n, _ = longest(FIG + 2.5, DROP - 1.5, floor, persist)
    drop_n, drop_t = longest(DROP, DROP + 2.0, floor, persist)
    sfig_n, _ = longest(FIG + 2.5, DROP - 1.5, floor, strong_persist)
    sdrop_n, sdrop_t = longest(DROP, DROP + 2.0, floor, strong_persist)
    print(f'  普通票(floor={floor}N, persist={persist}): '
          f'figure-8 最长连续 {fig_n} 帧, drop {drop_n} 帧'
          + (f', 延迟 {drop_t - DROP:.2f}s' if drop_t else ''))
    print(f'  强票  (floor={floor}N, persist={strong_persist}): '
          f'figure-8 最长连续 {sfig_n} 帧, drop {sdrop_n} 帧'
          + (f', 延迟 {sdrop_t - DROP:.2f}s' if sdrop_t else ''))
    print(f'  => 普通票在机动中{"会" if fig_n >= persist else "不会"}出现'
          f'(它只允许与质量通道配对,故{"无害" if fig_n >= persist else "更安全"});'
          f' 强票{"会" if sfig_n >= strong_persist else "不会"}误报')
    fig_min = min(v for i, v in enumerate(d)
                  if v is not None and FIG + 2.5 <= ts[i] < DROP - 1.5)
    print(f'  ⚠️ 幅度不可分:figure-8 段最负 ΔT {fig_min:.2f}N '
          'vs 0.15kg 卸载真信号 -1.47N')


if __name__ == '__main__':
    main()
    print('\n=== 5. release-residual 票(高频 fixture 标定) ===')
    check_release_residual()
