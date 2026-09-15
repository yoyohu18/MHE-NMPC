#!/usr/bin/env python3
"""Pre-command false-release audit of the eventless release detector (2026-09-15).

For every arm-run in the given manifests: loaded exposure = first MHE
EMPTY->LOADED to the release command (or log end if none), and the count of
MHE release-path firings inside that exposure ("LOADED->EMPTY (drop" or the
c_xy "载荷释放(" line).  Sustained-mass exits of the presence flag are not
release decisions and are reported separately.  One-sided 95% upper bound on
the per-flight rate uses the exact binomial (0 events -> 1-0.05**(1/n)).
Usage: audit_precommand_false_release.py MANIFEST.csv [...]
"""
import csv
import re
import sys
from pathlib import Path

TS = re.compile(r'^\[[A-Z]+\] \[([\d.]+)\]')
CMD_KEYS = ('DROP command issued', 'DROP: released gripper',
            'UNCOMMANDED LOSS injected')


def ep(line):
    m = TS.search(line)
    return float(m.group(1)) if m else None


def audit(root, stamp):
    mhe = root / f'grip_mhe_{stamp}.log'
    nmpc = root / f'grip_nmpc_{stamp}.log'
    if not mhe.exists() or not nmpc.exists():
        return None
    t_cmd = None
    t_last = None
    for l in nmpc.open(errors='ignore'):
        e = ep(l)
        if e is not None:
            t_last = e
        if t_cmd is None and any(k in l for k in CMD_KEYS):
            t_cmd = e
    t_load, fires, exits = None, [], []
    for l in mhe.open(errors='ignore'):
        e = ep(l)
        if e is None:
            continue
        if t_load is None and '[payload-state] EMPTY->LOADED' in l:
            t_load = e
        end = t_cmd if t_cmd is not None else t_last
        if t_load is None or end is None or e >= end:
            continue
        if '[payload-state] LOADED->EMPTY (drop' in l or '载荷释放(' in l:
            if not fires or e - fires[-1] > 1.0:   # 同一次释放的两行合并
                fires.append(e)
        elif '[payload-state] LOADED->EMPTY (sustained' in l:
            exits.append(e)
    if t_load is None:
        return dict(loaded=False)
    end = t_cmd if t_cmd is not None else t_last
    return dict(loaded=True, exposure=max(0.0, end - t_load), fires=len(fires),
                exits=len(exits), has_cmd=t_cmd is not None,
                first_fire_before_cmd=(fires[0] - end) if fires else None)


def ub95(k, n):
    if n == 0:
        return float('nan')
    if k == 0:
        return 1 - 0.05 ** (1 / n)
    lo, hi = k / n, 1.0
    from math import comb
    for _ in range(60):
        mid = (lo + hi) / 2
        cdf = sum(comb(n, i) * mid ** i * (1 - mid) ** (n - i) for i in range(k + 1))
        lo, hi = (mid, hi) if cdf > 0.05 else (lo, mid)
    return hi


def main():
    tot = {}
    for arg in sys.argv[1:]:
        man = Path(arg).resolve()
        rows = list(csv.DictReader(man.open()))
        per = {}
        for r in rows:
            if r.get('stamp') in (None, '', 'NA'):
                continue
            a = audit(man.parent, r['stamp'])
            if not a or not a['loaded']:
                continue
            key = (r['arm'], r.get('wid', r.get('workpoint', '')))
            d = per.setdefault(key, dict(n=0, nv=0, exp=0.0, fires=0, fflights=0,
                                         exits=0, ex=[]))
            d['n'] += 1
            d['nv'] += r.get('validity') == 'valid'
            d['exp'] += a['exposure']
            d['fires'] += a['fires']
            d['fflights'] += a['fires'] > 0
            d['exits'] += a['exits']
            if a['fires']:
                d['ex'].append(f"{r['stamp']}({a['fires']},{r.get('validity')[:20]})")
            t = tot.setdefault(r['arm'] in ('B', 'S0', 'S1', 'C') and 'eventless-consumed'
                               or 'A-arm (MHE identical, not consumed)',
                               dict(n=0, exp=0.0, ff=0))
            t['n'] += 1; t['exp'] += a['exposure']; t['ff'] += a['fires'] > 0
        print(f'\n{man.name}')
        for (arm, wid), d in sorted(per.items()):
            print(f'  arm {arm:>2} {wid:>3}: loaded flights {d["n"]:3d} (valid {d["nv"]:3d}) '
                  f'exposure {d["exp"]/60:6.1f} min | false-release flights '
                  f'{d["fflights"]}/{d["n"]} (events {d["fires"]}) | sustained exits {d["exits"]} '
                  f'{" ".join(d["ex"][:6])}')
    print('\n=== pooled ===')
    for k, t in tot.items():
        print(f'{k}: flights {t["n"]}, loaded exposure {t["exp"]/3600:.2f} h, '
              f'false-release flights {t["ff"]} -> per-flight rate {t["ff"]}/{t["n"]}, '
              f'95% upper {100*ub95(t["ff"], t["n"]):.1f}%')


if __name__ == '__main__':
    main()
