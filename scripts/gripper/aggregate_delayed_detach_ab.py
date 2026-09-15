#!/usr/bin/env python3
"""Aggregate the delayed physical-detachment A/B fault injection."""

import csv
import re
import statistics as st
import sys
from pathlib import Path


ROS_TS = re.compile(r'^\[[A-Z]+\] \[([\d.]+)\]')
NMPC_T = re.compile(r't=([\d.]+)s')
POS = re.compile(r'\[attach-window\] t=([\d.]+)s pos_err=([\d.]+)m')


def first_epoch(lines, needle):
    for line in lines:
        if needle in line and (m := ROS_TS.search(line)):
            return float(m.group(1))
    return None


def first_nmpc_time(lines, needles):
    for line in lines:
        if any(x in line for x in needles) and (m := NMPC_T.search(line)):
            return float(m.group(1))
    return None


def read_lines(path):
    return path.read_text(errors='ignore').splitlines() if path.exists() else []


def analyze(root, row):
    stamp = row['stamp']
    n_lines = read_lines(root / f'grip_nmpc_{stamp}.log')
    p_lines = read_lines(root / f'grip_proximity_{stamp}.log')
    s_lines = read_lines(root / f'grip_state_{stamp}.log')
    arm = row['arm']
    cmd_key = 'DROP: released gripper' if arm == 'A' else 'DROP command issued'
    clear_key = 'DROP: released gripper' if arm == 'A' else 'DROP complete'
    t_cmd_epoch = first_epoch(n_lines, cmd_key)
    t_clear_epoch = first_epoch(n_lines, clear_key)
    t_det_epoch = first_epoch(p_lines, '-> DETACH')
    t_cmd = first_nmpc_time(n_lines, [cmd_key])
    hold_peak = None
    if None not in (t_cmd_epoch, t_det_epoch, t_cmd):
        t_det_nmpc = t_cmd + t_det_epoch - t_cmd_epoch
        vals = [float(m.group(2)) for line in n_lines if (m := POS.search(line))
                and t_cmd <= float(m.group(1)) <= t_det_nmpc]
        hold_peak = max(vals) if vals else None
    return {
        'pair': int(row['pair']), 'arm': arm, 'stamp': stamp,
        'validity': row['validity'],
        'state_detached': any('DETACHED ' in line for line in s_lines),
        'physical_delay': None if None in (t_cmd_epoch, t_det_epoch)
                          else t_det_epoch - t_cmd_epoch,
        'clear_minus_detach': None if None in (t_clear_epoch, t_det_epoch)
                              else t_clear_epoch - t_det_epoch,
        'hold_peak': hold_peak,
        'unresolved': any('PAYLOAD STATE UNRESOLVED' in line for line in n_lines),
        'premature_clear': (None if None in (t_clear_epoch, t_det_epoch)
                            else t_clear_epoch < t_det_epoch - 0.25),
    }


def med(values):
    vals = [x for x in values if x is not None]
    return st.median(vals) if vals else float('nan')


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: aggregate_delayed_detach_ab.py MANIFEST.csv [...]')
    runs = []
    attempts = []
    for batch, arg in enumerate(sys.argv[1:], 1):
        manifest = Path(arg).resolve()
        root = manifest.parent
        with manifest.open(newline='') as f:
            rows = list(csv.DictReader(f))
        attempts.extend((batch, row) for row in rows)
        for row in rows:
            if row.get('stamp') == 'NA':
                continue
            run = analyze(root, row)
            run['batch'] = batch
            run['pair_key'] = (batch, run['pair'])
            runs.append(run)

    eligible = [r for r in runs if r['validity'] == 'valid'
                and r['physical_delay'] is not None]
    by_pair = {}
    for run in eligible:
        by_pair.setdefault(run['pair_key'], {})[run['arm']] = run
    paired_keys = {key for key, arms in by_pair.items()
                   if set(arms) == {'A', 'B'}}
    paired = [r for r in eligible if r['pair_key'] in paired_keys]
    print(f'delayed-detach fault injection: {len(attempts)} attempts, '
          f'{len(eligible)} eligible arm-runs, {len(paired_keys)} valid pairs')
    for arm in ('A', 'B'):
        sub = [r for r in paired if r['arm'] == arm]
        early = sum(r['premature_clear'] is True for r in sub)
        truth = sum(r['state_detached'] for r in sub)
        unr = sum(r['unresolved'] for r in sub)
        print(f'arm {arm}: n={len(sub)}, physical truth {truth}/{len(sub)}, '
              f'premature model clear {early}/{len(sub)}, '
              f'UNRESOLVED {unr}/{len(sub)}, '
              f'median injected delay {med([r["physical_delay"] for r in sub]):.2f}s, '
              f'median clear-detach {med([r["clear_minus_detach"] for r in sub]):+.2f}s, '
              f'median hold peak error {med([r["hold_peak"] for r in sub]):.3f}m')
    invalid = [(batch, row) for batch, row in attempts
               if row.get('validity') != 'valid']
    unpaired = [r for r in eligible if r['pair_key'] not in paired_keys]
    print(f'attempt accounting: invalid={len(invalid)}, '
          f'eligible-but-unpaired={len(unpaired)}')
    print('\nrun-level audit')
    for r in runs:
        print(f'batch {r["batch"]} pair {r["pair"]} arm {r["arm"]} {r["stamp"]}: '
              f'delay={r["physical_delay"]!s}, clear-detach={r["clear_minus_detach"]!s}, '
              f'hold_peak={r["hold_peak"]!s}, premature={r["premature_clear"]}, '
              f'unresolved={r["unresolved"]}, truth={r["state_detached"]}, '
              f'paired={r["pair_key"] in paired_keys}, validity={r["validity"]}')


if __name__ == '__main__':
    main()
