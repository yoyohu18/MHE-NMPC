#!/usr/bin/env python3
"""Aggregate run_partial_loss.sh (four-box staircase, 2026-09-15).

Pre-registration: ~/ros2_ws_HJH/部分丢失四箱实验_预注册_20260915.md
  aggregate_partial_loss.py MANIFEST.csv [...]
  aggregate_partial_loss.py --validity RESULTS_DIR STAMP   (used by the batch)
Truth: payload mass = 0.05 kg x boxes still attached; first moment
s = sum(m_i * r_xy_i) from each box's measured attach offset (box - drone).
"""
import csv
import re
import statistics as st
import sys
from pathlib import Path

TS = re.compile(r'^\[[A-Z]+\] \[([\d.]+)\]')
AW = re.compile(r'\[attach-window\] t=([\d.-]+)s pos_err=([\d.]+)m T=[\d.-]+N '
                r'z=[\d.-]+ m_est=([\d.-]+)')
ATT = re.compile(r'-> ATTACH "\S+ \S+ (\S+) \S+"')
OFF = re.compile(r'attach offset \(box - drone\) = \[([+-][\d.]+), ([+-][\d.]+), ([+-][\d.]+)\]')
LOSS = re.compile(r'PARTIAL LOSS (?:\(dry\) |injected )?(?:for )?"?(box\d?)"?')
SCOL = re.compile(r'\[s-collapse\] \|s\|=([\d.]+)')
FD = re.compile(r'u_frac=\[T[\d.]+ r([\d.]+) p([\d.]+) y[\d.]+\] \| '
                r'u_sat=\[T[\d.]+ r([\d.]+) p([\d.]+) y[\d.]+\]%')
M_B = 2.064
M_BOX = 0.05
BAND = 0.02
CRASH = 2.0


def ep(l):
    m = TS.search(l)
    return float(m.group(1)) if m else None


def load(root, stamp):
    rd = lambda p: p.read_text(errors='ignore').splitlines() if p.exists() else []
    return (rd(root / f'grip_nmpc_{stamp}.log'), rd(root / f'grip_mhe_{stamp}.log'),
            rd(root / f'grip_proximity_{stamp}.log'))


def parse(root, stamp):
    n, h, p = load(root, stamp)
    boxes, last_att, t_att0 = {}, None, None
    losses = []
    for l in p:
        if (m := ATT.search(l)):
            last_att = m.group(1)
            t_att0 = t_att0 if t_att0 is not None else ep(l)
        elif (m := OFF.search(l)) and last_att and last_att not in boxes:
            boxes[last_att] = (float(m.group(1)), float(m.group(2)))
        elif 'PARTIAL LOSS' in l and (m := LOSS.search(l)):
            losses.append((ep(l), m.group(1), '(dry)' in l))
    aw = [(ep(l), float(m.group(2)), float(m.group(3))) for l in n
          if (m := AW.search(l)) and ep(l) is not None]
    return dict(n=n, h=h, boxes=boxes, t_att0=t_att0, losses=losses, aw=aw)


def validity(d):
    if len(d['boxes']) < 4:
        return f'invalid:attach-fail ({len(d["boxes"])}/4 boxes)'
    if not d['losses']:
        return 'invalid:no-schedule-marks'
    t1 = d['losses'][0][0]
    if any(pe > CRASH for t, pe, _ in d['aw'] if t < t1):
        return 'invalid:pre-loss-crash'
    return 'valid'


def truth_after(d, k):
    gone = {b for _, b, dry in d['losses'][:k] if not dry}
    left = {b: r for b, r in d['boxes'].items() if b not in gone}
    mp = M_BOX * len(left)
    sx = sum(M_BOX * r[0] for r in left.values())
    sy = sum(M_BOX * r[1] for r in left.values())
    return mp, (sx * sx + sy * sy) ** 0.5


def windows(d, post):
    ts = [t for t, _, _ in d['losses']]
    out = []
    for k, t in enumerate(ts):
        t_end = ts[k + 1] if k + 1 < len(ts) else t + post
        out.append((k + 1, t, t_end))
    return out


def level_metrics(d, k, t0, t1):
    mp, s_true = truth_after(d, k)
    half = t0 + 0.5 * (t1 - t0)
    ms = [(t, m) for t, _, m in d['aw'] if t0 <= t < t1]
    m_true = M_B + mp
    conv = None
    for i, (t, _) in enumerate(ms):
        if all(abs(m - m_true) <= BAND for _, m in ms[i:]):
            conv = t - t0
            break
    m2 = [m for t, m in ms if t >= half]
    pe2 = [pe for t, pe, _ in d['aw'] if half <= t < t1]
    pe_all = [pe for t, pe, _ in d['aw'] if t0 <= t < t1]
    s2 = [float(m.group(1)) for l in d['h'] if (m := SCOL.search(l))
          and (e := ep(l)) is not None and half <= e < t1]
    fd = [[float(x) for x in m.groups()] for l in d['n'] if '[flight-diag]' in l
          and (m := FD.search(l)) and (e := ep(l)) is not None and half <= e < t1]
    rel = sum(1 for l in d['h'] if ('[payload-state] LOADED->EMPTY (drop' in l
              or '载荷释放(' in l) and (e := ep(l)) is not None and t0 <= e < t1)
    return dict(level=k, mp_true=mp, s_true=s_true,
                m_err=(st.median(m2) - m_true) if m2 else None,
                conv=conv, in_band=conv is not None,
                s_est=st.median(s2) if s2 else None,
                rms=(sum(x * x for x in pe2) / len(pe2)) ** 0.5 if pe2 else None,
                crash=any(x > CRASH for x in pe_all),
                ufrac_r=st.median([f[0] for f in fd]) if fd else None,
                ufrac_p=st.median([f[1] for f in fd]) if fd else None,
                usat=(sum(f[2] + f[3] for f in fd) / len(fd)) if fd else None,
                release=rel)


def fmt(x, f='{:+.4f}'):
    return 'nan' if x is None else f.format(x)


def main():
    if sys.argv[1] == '--validity':
        print(validity(parse(Path(sys.argv[2]), sys.argv[3])))
        return
    post = 30.0
    for arg in sys.argv[1:]:
        man = Path(arg).resolve()
        rows = list(csv.DictReader(man.open()))
        print(f'\n=== {man.name}: {len(rows)} attempts ===')
        per = {}
        for r in rows:
            if r['stamp'] == 'NA':
                continue
            d = parse(man.parent, r['stamp'])
            v = validity(d)
            print(f'  pair {r["pair"]} arm {r["arm"]} {r["stamp"]}: {v} boxes='
                  + ','.join(f'{b}({x:+.3f},{y:+.3f})' for b, (x, y) in sorted(d['boxes'].items())))
            if v != 'valid':
                continue
            for k, t0, t1 in windows(d, post):
                lm = level_metrics(d, k, t0, t1)
                per.setdefault((r['pair'], r['arm']), []).append(lm)
                print(f'     L{k}: m_P true {lm["mp_true"]:.2f} |s| true {lm["s_true"]:.4f} '
                      f'est {fmt(lm["s_est"])} | m err {fmt(lm["m_err"])}kg '
                      f'conv {fmt(lm["conv"], "{:.1f}")}s | RMS {fmt(lm["rms"], "{:.3f}")}m '
                      f'u_frac r/p {fmt(lm["ufrac_r"], "{:.2f}")}/{fmt(lm["ufrac_p"], "{:.2f}")} '
                      f'usat {fmt(lm["usat"], "{:.1f}")}% release {lm["release"]} crash {lm["crash"]}')
        pairs = sorted({p for p, a in per if (p, 'L') in per and (p, 'N') in per}, key=int)
        print(f'  complete pairs: {len(pairs)}')
        for k in (1, 2, 3):
            L = [x for p in pairs for x in per[(p, 'L')] if x['level'] == k]
            N = [x for p in pairs for x in per[(p, 'N')] if x['level'] == k]
            if not L:
                continue
            diffs = [a['rms'] - b['rms'] for a, b in zip(L, N)
                     if a['rms'] is not None and b['rms'] is not None]
            print(f'  level {k}: L in-band {sum(x["in_band"] for x in L)}/{len(L)} '
                  f'conv med {fmt(st.median([x["conv"] for x in L if x["conv"] is not None]) if any(x["conv"] is not None for x in L) else None, "{:.1f}")}s '
                  f'| m err L {fmt(st.median([x["m_err"] for x in L if x["m_err"] is not None]))} '
                  f'N {fmt(st.median([x["m_err"] for x in N if x["m_err"] is not None]) if N else None)}kg '
                  f'| RMS diff L-N med {fmt(st.median(diffs) if diffs else None, "{:+.3f}")}m '
                  f'| false empty L {sum(x["release"] > 0 for x in L)}/{len(L)} '
                  f'N {sum(x["release"] > 0 for x in N)}/{len(N)} '
                  f'| crash L {sum(x["crash"] for x in L)} N {sum(x["crash"] for x in N)}')


if __name__ == '__main__':
    main()
