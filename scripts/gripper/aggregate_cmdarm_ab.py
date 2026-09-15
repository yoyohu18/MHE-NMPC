#!/usr/bin/env python3
"""Aggregate run_matched_cmd_armed_ab.sh batches (2026-09-14).

FAULT=delayed      : premature model clear (NMPC DROP complete before physical
                     DETACH - 0.25 s), clear-minus-detach, UNRESOLVED.
FAULT=uncommanded  : whether the estimator recognised the loss (MHE LOADED->EMPTY
                     after physical DETACH within the log), its latency, and the
                     post-loss position-error peak.
FAULT=thrustmap    : pre-command mass bias, pre-command false release,
                     completion, confirmation delay.
Every arm-run in every manifest is reported; pairs require all arms valid.
"""
import csv
import re
import statistics as st
import sys
from pathlib import Path

TS = re.compile(r'^\[[A-Z]+\] \[([\d.]+)\]')
AW = re.compile(r'\[attach-window\] t=([\d.-]+)s pos_err=([\d.]+)m T=[\d.-]+N '
                r'z=[\d.-]+ m_est=([\d.-]+)')
M_B = 2.064


def lines(p):
    return p.read_text(errors='ignore').splitlines() if p.exists() else []


def epochs(ls, needle):
    return [float(m.group(1)) for l in ls if needle in l and (m := TS.search(l))]


def first(xs):
    return xs[0] if xs else None


def analyze(root, row, fault):
    m_true = M_B + (0.30 if fault.endswith('-W4') else 0.15)
    st_ = row['stamp']
    n = lines(root / f'grip_nmpc_{st_}.log')
    h = lines(root / f'grip_mhe_{st_}.log')
    pr = lines(root / f'grip_proximity_{st_}.log')
    t_cmd = first(epochs(n, 'DROP command issued') or epochs(n, 'UNCOMMANDED LOSS injected'))
    t_det = first(epochs(pr, '-> DETACH'))
    t_done = first(epochs(n, 'DROP complete'))
    empties = epochs(h, '[payload-state] LOADED->EMPTY')
    aw = [(float(m.group(1)), float(m.group(2)), float(m.group(3)), float(TS.search(l).group(1)))
          for l in n if (m := AW.search(l)) and TS.search(l)]
    r = dict(arm=row['arm'], pair=int(row['pair']), stamp=st_, validity=row['validity'],
             unresolved=any('PAYLOAD STATE UNRESOLVED' in l for l in n),
             cmd_armed_release=bool(epochs(h, '[cmd-armed] RELEASE confirmed')),
             suppressed=len(epochs(h, '[cmd-armed] suppressed')))
    r['pre_cmd_false_release'] = (t_cmd is not None
                                  and any(e < t_cmd for e in empties))
    r['clear_minus_detach'] = (t_done - t_det) if None not in (t_done, t_det) else None
    r['premature'] = (t_done < t_det - 0.25) if None not in (t_done, t_det) else None
    r['complete'] = t_done is not None
    post_e = [e for e in empties if t_det is not None and e >= t_det - 0.25]
    r['recognised'] = bool(post_e)
    r['recog_latency'] = (post_e[0] - t_det) if post_e else None
    r['post_peak'] = (max((a[1] for a in aw if t_det is not None and a[3] >= t_det), default=None))
    pre = [a[2] for a in aw if t_cmd is not None and t_cmd - 20.0 <= a[3] < t_cmd]
    r['m_bias_pct'] = (100.0 * (st.median(pre) - m_true) / m_true) if pre else None
    # 幽灵模型/后果指标(预注册附录 A):t_det+5 s 起的 flight-diag,t_det 起的 pos_err
    FD = re.compile(r'u_frac=\[T([\d.]+) r([\d.]+) p([\d.]+) y[\d.]+\] \| '
                    r'u_sat=\[T[\d.]+ r([\d.]+) p([\d.]+) y[\d.]+\]%.*conf=([\d.]+)')
    fd = []
    for l in n:
        if '[flight-diag]' in l and (m := FD.search(l)) and (e := TS.search(l)):
            if t_det is not None and float(e.group(1)) >= t_det + 5.0:
                fd.append([float(x) for x in m.groups()])
    if fd:
        a_ = list(zip(*fd))
        r['post_conf'] = st.median(a_[5])
        r['post_ufrac_r'] = st.median(a_[1])
        r['post_ufrac_p'] = st.median(a_[2])
        r['post_usat_rp'] = (sum(a_[3]) + sum(a_[4])) / len(fd)
    else:
        r['post_conf'] = r['post_ufrac_r'] = r['post_ufrac_p'] = r['post_usat_rp'] = None
    pe = [a[1] for a in aw if t_det is not None and a[3] >= t_det]
    r['post_rms'] = (sum(x * x for x in pe) / len(pe)) ** 0.5 if pe else None
    r['crash_post'] = any(x > 2.0 for x in pe)
    last = [a[2] for a in aw[-40:]]
    r['m_end'] = st.median(last) if last else None
    return r


def med(xs):
    xs = [x for x in xs if x is not None]
    return f'{st.median(xs):+.2f}' if xs else 'nan'


def main():
    runs = []
    for b, arg in enumerate(sys.argv[1:], 1):
        man = Path(arg).resolve()
        fault = 'uncommanded' if 'uncommanded' in man.name else (
            'thrustmap' if 'thrustmap' in man.name else 'delayed')
        wp = re.search(r'_(W\d)_', man.name)
        fault = f'{fault}-{wp.group(1) if wp else "W5"}'
        for row in csv.DictReader(man.open()):
            if row['stamp'] == 'NA':
                runs.append(dict(arm=row['arm'], pair=int(row['pair']), stamp='NA',
                                 validity=row['validity'], batch=b, fault=fault))
                continue
            r = analyze(man.parent, row, fault)  # fault 已含 -Wx 后缀
            r['batch'], r['fault'] = b, fault
            runs.append(r)
    for fault in sorted({r['fault'] for r in runs}):
        sub = [r for r in runs if r['fault'] == fault]
        arms = sorted({r['arm'] for r in sub})
        valid = [r for r in sub if r['validity'] == 'valid']
        keys = {}
        for r in valid:
            keys.setdefault((r['batch'], r['pair']), set()).add(r['arm'])
        paired = {k for k, v in keys.items() if v == set(arms)}
        print(f'\n=== {fault}: {len(sub)} attempts, {len(valid)} valid, {len(paired)} complete pairs ===')
        for a in arms:
            s = [r for r in valid if r['arm'] == a and (r['batch'], r['pair']) in paired]
            k = len(s)
            print(f'arm {a}: n={k} complete {sum(r["complete"] for r in s)}/{k} '
                  f'premature {sum(r["premature"] is True for r in s)}/{k} '
                  f'unresolved {sum(r["unresolved"] for r in s)}/{k} '
                  f'pre-cmd false release {sum(r["pre_cmd_false_release"] for r in s)}/{k} '
                  f'recognised {sum(r["recognised"] for r in s)}/{k} '
                  f'| med clear-detach {med([r["clear_minus_detach"] for r in s])}s '
                  f'recog latency {med([r["recog_latency"] for r in s])}s '
                  f'post peak {med([r["post_peak"] for r in s])}m '
                  f'm bias {med([r["m_bias_pct"] for r in s])}% '
                  f'm_end {med([r["m_end"] for r in s])}kg')
            print(f'         post-loss: conf {med([r.get("post_conf") for r in s])} '
                  f'u_frac roll {med([r.get("post_ufrac_r") for r in s])} '
                  f'pitch {med([r.get("post_ufrac_p") for r in s])} '
                  f'u_sat(r+p)% {med([r.get("post_usat_rp") for r in s])} '
                  f'pos_err RMS {med([r.get("post_rms") for r in s])}m '
                  f'crash {sum(bool(r.get("crash_post")) for r in s)}/{k}')
        print('run-level:')
        for r in sub:
            print('  ' + ' '.join(f'{k}={v:.3f}' if isinstance(v, float) else f'{k}={v}'
                                  for k, v in r.items() if k not in ('fault',)))


if __name__ == '__main__':
    main()
