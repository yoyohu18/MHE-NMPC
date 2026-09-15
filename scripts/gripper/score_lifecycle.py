#!/usr/bin/env python3
"""Per-flight lifecycle scorer for the experiment plan (docs/实验计划.md §2, §3, §13).

Usage:
  score_lifecycle.py MANIFEST.csv [MANIFEST.csv ...] [--out runs.csv] [options]
  score_lifecycle.py --stamps 20260915_042351 [...] --results-dir DIR

Every manifest row with a stamp becomes one output row (intention-to-test,
plan §12.3); nothing is excluded here.  Infrastructure flags are reported so the
analysis step can apply its pre-registered exclusion rules explicitly.

Time sources, in priority order:
  1. ``[EVT] name=... wall=...`` lines (lifecycle_trace.py, added 2026-09-15);
  2. legacy log sentences for older batches (``event_source=legacy`` in the output).
     Legacy logs cannot give t_empty (no [model-used]) and miss eventless
     accepts that happen without a pending command, so those fields are NA.

Definitions (plan §2.1/§2.2/§3):
  t_sep        first physical DETACH after the first physical ATTACH; +inf if none
  t_cmd        first release command after ATTACH
  t_declare    first estimator EMPTY declaration after it declared LOADED
  t_confirm    controller persistence met (after ATTACH)
  t_accept     first accept WITH control consequence (after ATTACH): pending=1
               (a release command was outstanding: DROP complete / UNRESOLVED
               cancelled), or a legacy command-clears accept (no pending field)
  t_empty      model parameters actually used by NMPC stay inside the empty band
               for --empty-hold seconds, after they had been outside it post-ATTACH
  t_clear      min(t_accept, t_empty): the controller actually flies an empty model
               or acts on "payload gone" (plan §4.1 primary endpoint, 2026-09-16)
  premature    t_clear < t_sep (t_sep=+inf -> any clear)
  timely       t_sep <= t_clear <= t_sep + tau_mission  ([EVT] walls are ms-rounded,
               so an Oracle accept can share t_sep's millisecond)
Secondary (not in outcome):
  confirm_false_accepts  pending=0 accepts in (ATTACH, t_sep): the confirmation chain
               believed "empty" while loaded, with no control consequence in the
               continuous mainline (also catches the take-off latch residue)
  premature_accept_any   old definition: first accept of any kind < t_sep
All reported latencies are seconds relative to t_sep unless named otherwise.
"""
import argparse
import csv
import math
import re
import sys
from pathlib import Path

HDR = re.compile(r'^\[[A-Z]+\] \[([\d.]+)\]')
EVT = re.compile(r'\[EVT\] (.*)$')
MU = re.compile(r'\[model-used\] (.*)$')
AW = re.compile(r'\[attach-window\] t=[\d.-]+s pos_err=([\d.]+)m')
OFFSET = re.compile(r'attach offset \(box - drone\) = \[([+\-\d.]+), ([+\-\d.]+), ([+\-\d.]+)\]')
SREF = re.compile(r'\[moment-ref\] 基准已建立并冻结: \|s_ref\|=([\d.]+)')

INF = math.inf


def kv(s):
    out = {}
    for tok in s.split():
        if '=' in tok:
            k, v = tok.split('=', 1)
            out[k] = v
    return out


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def read(p):
    return p.read_text(errors='ignore').splitlines() if p.exists() else []


def parse_events(lines, node):
    """Return list of (t, name, fields) from [EVT] lines, else from legacy sentences."""
    ev = []
    for l in lines:
        m = EVT.search(l)
        if m:
            f = kv(m.group(1))
            ev.append((float(f['wall']), f['name'], f))
    if ev:
        return ev, 'evt'
    for l in lines:
        h = HDR.search(l)
        if not h:
            continue
        t = float(h.group(1))
        if node == 'prox':
            if '-> ATTACH' in l:
                ev.append((t, 'attach', {}))
            elif '-> DETACH' in l:
                ev.append((t, 'sep', {}))
            elif (m := OFFSET.search(l)):
                ev.append((t, 'attach_offset', dict(rx=m.group(1), ry=m.group(2), rz=m.group(3))))
        elif node == 'nmpc':
            if 'DROP command issued' in l:
                ev.append((t, 'cmd', {}))
            elif 'UNCOMMANDED LOSS injected' in l:
                ev.append((t, 'inject_loss', {}))
            elif 'DROP: released gripper' in l:
                ev.append((t, 'cmd', {}))
                ev.append((t, 'accept', dict(src='command')))
            elif 'DROP complete' in l:
                ev.append((t, 'confirm', {}))
                ev.append((t, 'accept', dict(src='confidence', pending='1')))
            elif 'PAYLOAD STATE UNRESOLVED' in l:
                ev.append((t, 'unresolved', {}))
            elif 'PAYLOAD STATE RESOLVED' in l:
                ev.append((t, 'resolved', {}))
        elif node == 'mhe':
            if '[payload-state] LOADED->EMPTY' in l:
                src = l.split('(', 1)[-1].split(',', 1)[0]
                ev.append((t, 'declare', dict(state='EMPTY', src=src)))
            elif '[payload-state] EMPTY->LOADED' in l:
                ev.append((t, 'declare', dict(state='LOADED')))
            elif '[cmd-armed] RELEASE confirmed' in l:
                ev.append((t, 'cmd_armed_confirm', {}))
            elif (m := SREF.search(l)):
                ev.append((t, 'moment_ref', dict(s_ref=m.group(1))))
    return ev, 'legacy'


def first(ev, name, after=-INF, pred=None):
    for t, n, f in ev:
        if n == name and t > after and (pred is None or pred(f)):
            return t, f
    return None, None


def controlling(f):
    """accept 是否有控制后果:有待确认指令(pending=1),或 legacy 指令即清(无 pending 字段)。"""
    p = f.get('pending')
    return p is None or p in ('1', 'True', 'true')


def parse_model_used(lines):
    rows = []
    for l in lines:
        m = MU.search(l)
        h = HDR.search(l)
        if m and h:
            f = kv(m.group(1))
            f['_t'] = float(h.group(1))
            rows.append(f)
    return rows


def t_empty_from_trace(mu, t_attach, a):
    """First entry into the empty band that lasts >= empty_hold, after having been loaded."""
    if not mu or t_attach is None:
        return None, None
    if any(r.get('mode') == 'coupled' for r in mu):
        return None, 'coupled-mode'

    def empty(r):
        m, g0, g1, g2 = (num(r.get(k)) for k in ('m', 'g0', 'g1', 'g2'))
        if None in (m, g0, g1, g2):
            return False
        return (abs(m - a.m_b) <= a.empty_tol_m and abs(g0) <= a.empty_tol_dj
                and math.hypot(g1, g2) <= a.empty_tol_c)

    was_loaded = False
    start = None
    for r in mu:
        if r['_t'] <= t_attach:
            continue
        e = empty(r)
        if not was_loaded:
            was_loaded = not e
            continue
        if e:
            start = r['_t'] if start is None else start
            if r['_t'] - start >= a.empty_hold:
                return start, None
        else:
            start = None
    return None, ('never-loaded' if not was_loaded else None)


def score_run(root, stamp, a, meta):
    n = read(root / f'grip_nmpc_{stamp}.log')
    h = read(root / f'grip_mhe_{stamp}.log')
    pr = read(root / f'grip_proximity_{stamp}.log')
    ev_n, s1 = parse_events(n, 'nmpc')
    ev_h, s2 = parse_events(h, 'mhe')
    ev_p, s3 = parse_events(pr, 'prox')
    ev = sorted(ev_n + ev_h + ev_p, key=lambda x: x[0])
    src = 'evt' if 'evt' in (s1, s2, s3) and len({s1, s2, s3}) == 1 else (
        'legacy' if {s1, s2, s3} == {'legacy'} else 'mixed')
    mu = parse_model_used(n)

    r = dict(meta)
    r['stamp'] = stamp
    r['event_source'] = src
    r['log_present'] = bool(n)
    t_attach, _ = first(ev, 'attach')
    r['phys_attached'] = t_attach is not None
    _, off = first(ev, 'attach_offset')
    for k in ('rx', 'ry', 'rz'):
        r[f'offset_{k}'] = num(off.get(k)) if off else None
    base = t_attach if t_attach is not None else -INF

    t_sep, _ = first(ev, 'sep', after=base)
    r['n_sep'] = sum(1 for t, nm, _ in ev if nm == 'sep' and t > base)
    t_cmd, _ = first(ev, 'cmd', after=base)
    t_inj, _ = first(ev, 'inject_loss', after=base)
    t_loaded, _ = first(ev, 'declare', after=base, pred=lambda f: f.get('state') == 'LOADED')
    t_decl, fdecl = first(ev, 'declare', after=t_loaded if t_loaded else INF,
                          pred=lambda f: f.get('state') == 'EMPTY')
    t_conf, _ = first(ev, 'confirm', after=base)
    t_acc, facc = first(ev, 'accept', after=base, pred=controlling)
    t_acc_any, _ = first(ev, 'accept', after=base)
    t_unres, _ = first(ev, 'unresolved', after=base)
    t_brake, fbr = first(ev, 'brake', after=base)
    t_mref, fmr = first(ev, 'moment_ref', after=base)
    hdr_ts = [float(m.group(1)) for l in n if (m := HDR.search(l))]
    t_end = max(hdr_ts) if hdr_ts else None     # NMPC 日志最后一行 = 观测截止

    t_sep_eff = t_sep if t_sep is not None else INF
    r.update(t_attach=t_attach, t_cmd=t_cmd, t_inject=t_inj, t_sep=t_sep, t_loaded=t_loaded,
             t_declare=t_decl, declare_src=(fdecl or {}).get('src'),
             t_confirm=t_conf, t_accept=t_acc, accept_src=(facc or {}).get('src'),
             t_unresolved=t_unres, t_brake=t_brake,
             brake_armed=(fbr or {}).get('armed'), t_end=t_end,
             moment_ref_formed=t_mref is not None,
             s_ref=num((fmr or {}).get('s_ref')))

    t_emp, why = t_empty_from_trace(mu, t_attach, a)
    r['t_empty'] = t_emp
    r['t_empty_na_reason'] = why if t_emp is None else None
    if src == 'legacy' and not mu:
        r['t_empty_na_reason'] = 'legacy-log'

    rel = lambda t: (t - t_sep) if (t is not None and t_sep is not None) else None
    cands = [(t, k) for t, k in ((t_acc, 'accept'), (t_emp, 'model_empty')) if t is not None]
    t_clear, clear_src = min(cands) if cands else (None, None)
    r['t_clear'] = t_clear
    r['clear_src'] = clear_src
    r['declare_minus_sep'] = rel(t_decl)
    r['accept_minus_sep'] = rel(t_acc)
    r['empty_minus_sep'] = rel(t_emp)
    r['clear_minus_sep'] = rel(t_clear)
    r['accept_minus_cmd'] = (t_acc - t_cmd) if None not in (t_acc, t_cmd) else None
    r['sep_minus_cmd'] = (t_sep - t_cmd) if None not in (t_sep, t_cmd) else None

    r['premature_clear'] = t_clear is not None and t_clear < t_sep_eff
    r['timely'] = (t_sep is not None and t_clear is not None
                   and t_sep <= t_clear <= t_sep + a.tau_mission)
    false_acc = [t for t, nm, f in ev if nm == 'accept' and base < t < t_sep_eff
                 and not controlling(f)]
    r['confirm_false_accepts'] = len(false_acc)
    r['t_first_false_accept'] = false_acc[0] if false_acc else None
    r['premature_accept_any'] = t_acc_any is not None and t_acc_any < t_sep_eff

    # 模型—物理不一致时长(右删失标记:日志先结束)
    wrong_empty = ghost_loaded = None
    ghost_censored = False
    if t_emp is not None and t_emp < t_sep_eff:
        wrong_empty = (min(t_sep_eff, t_end) - t_emp) if t_end is not None else None
    if t_sep is not None:
        if t_emp is not None and t_emp >= t_sep:
            ghost_loaded = t_emp - t_sep
        elif t_emp is None and t_end is not None and mu:
            ghost_loaded, ghost_censored = t_end - t_sep, True
    r['wrong_empty_s'] = wrong_empty
    r['ghost_loaded_s'] = ghost_loaded
    r['ghost_loaded_censored'] = ghost_censored

    pe = [(x['_t'], num(x.get('pos_err'))) for x in mu if num(x.get('pos_err')) is not None]
    if not pe:
        pe = [(float(HDR.search(l).group(1)), float(m.group(1)))
              for l in n if (m := AW.search(l)) and HDR.search(l)]
    after_attach = [e for t, e in pe if t_attach is None or t > t_attach]
    r['peak_pos_err'] = max(after_attach) if after_attach else None
    post = [e for t, e in pe if t_sep is not None and t >= t_sep]
    win = [e for t, e in pe if t_sep is not None and t_sep <= t <= t_sep + a.window]
    r['post_sep_peak_pos_err'] = max(post) if post else None
    r['post_sep_window_auc'] = (sum(win) * 0.1) if win and mu else None   # 10 Hz trace
    # 越界 = 曾超过阈值(计划 §3.2 的"越界",算安全失败);坠机 = 越界且末 5s 未恢复。
    # 两者分开:09-15 thrustmap G0.95 有一轮 UNRESOLVED 后推力打满偏离 2.24m 又回到 0.10m。
    r['excursion'] = any(e > a.crash_pos_err for e in after_attach)
    tail = [e for t, e in pe if t_end is not None and t >= t_end - 5.0]
    r['crash'] = r['excursion'] and (not tail or sorted(tail)[len(tail) // 2] > 1.0)
    tf = [float(HDR.search(l).group(1)) for l in n if 'acados solve failed' in l and HDR.search(l)]
    r['nmpc_solve_failed'] = len(tf)
    r['nmpc_solve_failed_post_sep'] = sum(1 for t in tf if t_sep is not None and t >= t_sep)
    r['mhe_solve_failed'] = sum(1 for l in h if 'MHE solve failed' in l)

    # §3.2 结果分类
    ref = t_sep if t_sep is not None else t_cmd
    if not n:
        outcome = 'no-log'
    elif src != 'evt' and t_inj is not None and t_clear is None and not r['excursion']:
        # 旧日志只在"有待确认指令"时打 DROP complete,无指令时的 accept 不可见
        outcome = 'unscorable-legacy'
    elif r['premature_clear'] or r['excursion']:
        outcome = 'safe_failure'
    elif ref is None:
        outcome = 'negative_ok'                       # 无事件无指令:无误声明且未坠
    elif t_sep is not None and r['timely'] and (t_unres is None or t_unres > t_clear):
        outcome = 'task_success'
    else:
        handled = [t for t in (t_clear, t_unres) if t is not None and t >= ref]
        if handled and min(handled) <= ref + a.tau_safe:
            outcome = 'safe_success'
        elif t_end is not None and t_end < ref + a.tau_safe:
            outcome = 'censored'
        elif t_sep is None and t_clear is None and t_cmd is None:
            outcome = 'negative_ok'
        else:
            outcome = 'safe_failure'
    r['outcome'] = outcome
    r['infra_flag'] = ('no-log' if not n else
                       'no-physical-attach' if not r['phys_attached'] else '')
    return r


def load_meta(root, stamp):
    meta = dict(exp_run_id=None, exp_block_id=None, exp_method=None,
                code_head=None, code_dirty=None)
    pv = root / f'provenance_{stamp}'
    for l in read(pv / 'experiment.env'):
        k, _, v = l.partition('=')
        if k in ('EXP_RUN_ID', 'EXP_BLOCK_ID', 'EXP_METHOD'):
            meta[k.lower()] = v
    for l in read(pv / 'workspace.git.txt'):
        k, _, v = l.partition('=')
        if k == 'head':
            meta['code_head'] = v
        elif k == 'dirty':
            meta['code_dirty'] = v
    return meta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('manifests', nargs='*')
    ap.add_argument('--stamps', nargs='*', default=[])
    ap.add_argument('--results-dir', default='/home/clear/ros2_ws_HJH/nmpc_test_results')
    ap.add_argument('--out')
    ap.add_argument('--m-b', type=float, default=2.0643)
    ap.add_argument('--tau-mission', type=float, default=12.0)
    ap.add_argument('--tau-safe', type=float, default=20.0)
    ap.add_argument('--window', type=float, default=5.0, help='post-sep AUC window [s]')
    ap.add_argument('--crash-pos-err', type=float, default=2.0)
    ap.add_argument('--empty-tol-m', type=float, default=0.03, help='|m - m_B| [kg]')
    ap.add_argument('--empty-tol-dj', type=float, default=0.005, help='|dJ| [kg m^2]')
    ap.add_argument('--empty-tol-c', type=float, default=0.002, help='|c_xy| [m]')
    ap.add_argument('--empty-hold', type=float, default=0.5, help='[s]')
    a = ap.parse_args()

    jobs = []
    for mpath in a.manifests:
        man = Path(mpath).resolve()
        for i, row in enumerate(csv.DictReader(man.open())):
            extra = {f'manifest_{k}': v for k, v in row.items() if k != 'stamp'}
            extra['manifest'] = man.name
            stamp = row.get('stamp', 'NA')
            root = man.parent
            if not (root / f'grip_nmpc_{stamp}.log').exists():
                root = Path(a.results_dir)      # manifest 不与日志同目录时
            jobs.append((root, stamp, extra))
    for s in a.stamps:
        jobs.append((Path(a.results_dir), s, {}))
    if not jobs:
        ap.error('no manifests or stamps given')

    rows = []
    for root, stamp, extra in jobs:
        if stamp in (None, '', 'NA'):
            rows.append(dict(extra, stamp=stamp, outcome='no-stamp', infra_flag='no-stamp'))
            continue
        r = score_run(root, stamp, a, load_meta(root, stamp))
        rows.append({**extra, **r})

    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    out = open(a.out, 'w', newline='') if a.out else sys.stdout
    w = csv.DictWriter(out, fieldnames=keys)
    w.writeheader()
    for r in rows:
        w.writerow({k: (f'{v:.3f}' if isinstance(v, float) and math.isfinite(v) else v)
                    for k, v in r.items()})
    if a.out:
        out.close()
        print(f'wrote {len(rows)} rows -> {a.out}', file=sys.stderr)


if __name__ == '__main__':
    main()
