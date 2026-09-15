#!/usr/bin/env python3
"""E0 保真度门(实验计划 §5.2):批量回放 → 与在线 m_est 对表 → 汇总报告。

每轮用**该轮当时的代码**回放(provenance 的 HEAD + tracked.patch +
untracked.tar.gz 重建,按三元组缓存,同一代码状态只重建一次)。没有
provenance 的轮次无法确定代码版本,列为 ``no-provenance`` 不进门,而不是拿
近似代码凑。

批次记录(.txt)里每行只要同时含 NMPC stamp(YYYYmmdd_HHMMSS)和 resid 目录
绝对路径即被收录;注释行跳过。

用法(需先 source ROS/工作区并导出 ACADOS 环境):
  python3 e0_fidelity_gate.py --record stepab_20260912_234849.txt \\
      --record resid015_20260912_223952.txt --cache /tmp/e0_src --out /tmp/e0_gate
"""
import argparse
import json
import re
import subprocess
import sys
import tarfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
RESULTS_ROOT = HERE.parents[2] / 'nmpc_test_results'
REPO = HERE.parents[1]
_STAMP = re.compile(r'\b(\d{8}_\d{6})\b')


def parse_record(path):
    runs = []
    for line in Path(path).read_text().splitlines():
        if line.lstrip().startswith('#'):
            continue
        toks = line.split()
        dirs = [t for t in toks if t.startswith('/') and Path(t).is_dir()]
        stamps = [t for t in toks if _STAMP.fullmatch(t)]
        if dirs and stamps:
            label = '_'.join(t for t in toks[:2] if not t.startswith('/'))
            j = toks.index(stamps[0])
            runs.append({'record': Path(path).name, 'label': label,
                         'stamp': stamps[0], 'resid_dir': dirs[-1],
                         'status': toks[j + 1] if j + 1 < len(toks) else ''})
    return runs


def _kv(path):
    out = {}
    for line in Path(path).read_text().splitlines():
        if '=' in line:
            k, v = line.split('=', 1)
            out[k.strip()] = v.strip()
    return out


def reconstruct(stamp, results_root, cache):
    """返回 (src_root, key) 或抛出带原因的异常。"""
    prov = Path(results_root) / f'provenance_{stamp}'
    git = prov / 'workspace.git.txt'
    if not git.exists():
        raise LookupError('no-provenance')
    g = _kv(git)
    head = g['head']
    key = f"{head[:10]}_{g.get('tracked_patch_sha256', 'clean')[:10]}_" \
          f"{g.get('untracked_archive_sha256', 'none')[:10]}"
    root = Path(cache) / key
    done = root / '.reconstructed'
    if done.exists():
        return root, key
    root.mkdir(parents=True, exist_ok=True)
    subprocess.run(f'git -C {REPO} archive {head} | tar -x -C {root}',
                   shell=True, check=True)
    patch = prov / 'workspace.tracked.patch'
    if g.get('dirty') == 'true' and patch.exists() and patch.stat().st_size:
        sha = subprocess.run(['sha256sum', str(patch)], capture_output=True,
                             text=True, check=True).stdout.split()[0]
        if sha != g.get('tracked_patch_sha256'):
            raise RuntimeError(f'tracked.patch sha 不符: {sha}')
        subprocess.run(['git', 'apply', '--whitespace=nowarn', str(patch)],
                       cwd=root, check=True)
    tgz = prov / 'workspace.untracked.tar.gz'
    if tgz.exists():
        with tarfile.open(tgz) as tf:
            tf.extractall(root, filter='data')
    done.write_text(json.dumps(g, indent=1))
    return root, key


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--record', action='append', required=True)
    ap.add_argument('--results-root', default=str(RESULTS_ROOT))
    ap.add_argument('--cache', required=True, help='重建代码树缓存目录')
    ap.add_argument('--out', required=True)
    ap.add_argument('--gate-kg', type=float, default=0.01)
    ap.add_argument('--solve-clock', default='measured')
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    runs = []
    for r in args.record:
        p = Path(r)
        if not p.is_absolute():
            p = Path(args.results_root) / p
        runs += parse_record(p)

    results = []
    for i, run in enumerate(runs, 1):
        tag = f"{Path(run['record']).stem}__{run['label']}__{run['stamp']}"
        rdir = out / tag
        row = dict(run, tag=tag)
        try:
            src, key = reconstruct(run['stamp'], args.results_root, args.cache)
            row['code_key'] = key
        except LookupError as e:
            row.update(verdict=str(e))
            results.append(row)
            print(f'[{i}/{len(runs)}] {tag}: {e}', flush=True)
            continue
        cmd = [sys.executable, str(HERE / 'e0_replay.py'), '--stamp', run['stamp'],
               '--resid-dir', run['resid_dir'], '--src-root', str(src),
               '--results-root', args.results_root, '--out', str(rdir),
               '--gate-kg', str(args.gate_kg), '--solve-clock', args.solve_clock]
        rdir.mkdir(parents=True, exist_ok=True)
        with open(rdir / 'replay.log', 'w') as lf:
            rc = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT).returncode
        sj = rdir / 'replay_summary.json'
        if not sj.exists():
            row.update(verdict=f'replay-error(rc={rc})')
        else:
            s = json.loads(sj.read_text())
            mlog = Path(args.results_root) / f"grip_mhe_{run['stamp']}.log"
            # 在线日志比原始流多出最后一次 flush 之后的几秒,计数可能略多。
            row['online_solve_fail'] = (sum('MHE solve failed' in ln for ln in
                                            mlog.read_text(errors='replace').splitlines())
                                        if mlog.exists() else None)
            row.update(verdict='PASS' if s['gate_pass'] else 'FAIL',
                       fidelity=s['fidelity'], gate_segments=s['gate_segments'],
                       phantom_fresh=s['phantom_fresh'], missed_fresh=s['missed_fresh'],
                       solves=s['solves'], solve_fail=s['solve_fail'],
                       first_divergence=s['first_divergence'],
                       clock_residual_ms=s['clock_map']['residual_ms_p05_p50_p95'],
                       has_detach=s['truth']['detach_mono_ns'] is not None)
        results.append(row)
        print(f"[{i}/{len(runs)}] {tag}: {row['verdict']}", flush=True)

    (out / 'gate_results.json').write_text(json.dumps(results, indent=2, ensure_ascii=False))

    def f(x, seg, q):
        v = (x.get('fidelity') or {}).get(seg, {})
        return f"{v[q]:.2e}" if v.get('n') else '—'

    lines = ['# E0 保真度门结果', '',
             f"门限: 带载稳态段与 drop 后段 |m_replay − m_online| P95 ≤ {args.gate_kg} kg;"
             f" solve_clock={args.solve_clock}", '',
             '| 轮次 | 代码 | 判定 | 带载稳态 P95 | drop后 P95 | 全程 max | phantom/missed | solve 失败 在线/回放 |',
             '|---|---|---|---|---|---|---|---|']
    for x in results:
        lines.append(f"| {x['tag']} | {x.get('code_key', '—')} | {x['verdict']} | "
                     f"{f(x, 'loaded_steady', 'p95')} | {f(x, 'post_detach', 'p95')} | "
                     f"{f(x, 'all', 'max')} | {x.get('phantom_fresh', '—')}/"
                     f"{x.get('missed_fresh', '—')} | {x.get('online_solve_fail', '—')}/"
                     f"{x.get('solve_fail', '—')} |")
    n = {v: sum(1 for x in results if x['verdict'] == v)
         for v in sorted({x['verdict'] for x in results})}
    lines += ['', f'汇总: {n}']
    (out / 'gate_report.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines[-1:]))
    return 0 if all(x['verdict'] in ('PASS', 'no-provenance') for x in results) else 2


if __name__ == '__main__':
    sys.exit(main())
