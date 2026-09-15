#!/usr/bin/env python3
"""First mass moment: estimate vs ground truth, eventless configuration.

Ground truth is reconstructed offline and never reaches the estimator:
  s_true = m_payload(nominal, per condition) x r_attach(measured by Gazebo)
where r_attach comes from the proximity-gripper node log line
``attach offset (box - drone) = [...]``. The MHE runs with
``truth_attached=False`` and ``geom=[0,0,-0.47]`` (vertical rack prior only),
so it has no horizontal attachment geometry at any point.

(a) W5 runs (0.15 kg, 2 m/s), whose event times coincide, normalised by each
    run's own truth, through grasp -> lift -> figure-eight -> release.
(b) All usable arm-B runs: carry-phase mean estimate against truth.
"""
import os, re, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from paper_style import apply_style, BLUE, ORANGE, AQUA, INK, INK2, GRID

HERE = os.path.dirname(os.path.abspath(__file__))
NR = os.environ.get('NMPC_RESULTS_DIR',
                    os.path.abspath(os.path.join(HERE, '..', '..', '..', 'nmpc_test_results')))
MP = {'W3': 0.15, 'W4': 0.30, 'W5': 0.15}
MANIFESTS = ('mainline_ab3b_manifest.csv', 'mainline_ab3c_manifest.csv',
             'mainline_ab3_manifest.csv')

_S = re.compile(r'\[(\d+)\.(\d+)\].*s_hat=\[([-+\d.]+),([-+\d.]+)\]')
# Offsets are logged with an explicit sign ("+0.020"); the character class must accept
# '+', otherwise every flight with a positive component is silently dropped.
_OFF = re.compile(r'attach offset \(box - drone\) = \[([-+\d.]+), ([-+\d.]+), ([-+\d.]+)\]')
_DYN = re.compile(r'\[(\d+)\.(\d+)\].*t=([\d.]+)s \| DYNAMIC: switch to figure8')
_DROP = re.compile(r'\[(\d+)\.(\d+)\].*t=([\d.]+)s \| DROP command issued')


def collect():
    runs, seen = [], set()
    for man in MANIFESTS:
        p = os.path.join(NR, man)
        if not os.path.exists(p):
            continue
        for ln in open(p).read().splitlines()[1:]:
            f = ln.split(',')
            if len(f) < 9 or f[3] != 'B' or 'invalid' in ln or f[1] not in MP:
                continue
            wid, stamp = f[1], f[4]
            if stamp in seen:
                continue
            seen.add(stamp)
            po = os.path.join(NR, f'grip_proximity_{stamp}.log')
            mo = os.path.join(NR, f'grip_mhe_{stamp}.log')
            no = os.path.join(NR, f'grip_nmpc_{stamp}.log')
            if not all(map(os.path.exists, (po, mo, no))):
                continue
            m = _OFF.search(open(po).read())
            if not m:
                raise RuntimeError(f'{stamp}: attach offset line not parsed from {po}')
            r = tuple(map(float, m.groups()))
            s_true = MP[wid] * r[1]
            if not s_true:
                continue
            t, sy = [], []
            for line in open(mo):
                g = _S.search(line)
                if g:
                    t.append(float(g.group(1)) + float(g.group(2)) * 1e-9)
                    sy.append(float(g.group(4)))
            txt = open(no).read()
            a, b = _DYN.search(txt), _DROP.search(txt)
            if not (t and a and b):
                continue
            t0 = float(a.group(1)) + float(a.group(2)) * 1e-9 - float(a.group(3))
            runs.append(dict(wid=wid, stamp=stamp, s_true=s_true,
                             t=np.array(t) - t0, sy=np.array(sy),
                             t_dyn=float(a.group(3)), t_drop=float(b.group(3))))
    return runs


def main():
    runs = collect()
    w5 = [r for r in runs if r['wid'] == 'W5']
    apply_style()
    plt.rcParams['savefig.bbox'] = 'tight'
    fig, ax = plt.subplots(figsize=(3.45, 1.52))

    # ---- (a) normalised timeline, W5 ----
    grid = np.arange(0, 75.01, 0.5)
    M = np.full((len(w5), grid.size), np.nan)
    for i, r in enumerate(w5):
        ok = (grid >= r['t'][0]) & (grid <= r['t'][-1])
        M[i, ok] = np.interp(grid[ok], r['t'], r['sy']) / r['s_true']
        ax.plot(grid, M[i], color=BLUE, lw=.45, alpha=.32, zorder=2)
    ax.plot(grid, np.nanmedian(M, 0), color=BLUE, lw=1.5, zorder=4, label='median')
    ax.axhline(1.0, color=AQUA, lw=.9, ls=':', zorder=3)
    ax.text(73.5, 1.045, 'truth', color='#0f7a55', fontsize=6.5, ha='right')
    ax.axhline(0.0, color=GRID, lw=.7, zorder=1)

    td, tp = np.mean([r['t_dyn'] for r in w5]), np.mean([r['t_drop'] for r in w5])
    for x, lab in ((6.3, 'grasp'), (td, 'figure-8'), (tp, 'release')):
        ax.axvline(x, color=INK2, lw=.6, ls='--', alpha=.75, zorder=1)
        ax.text(x + .9, -0.36, lab, fontsize=6.2, color=INK2)
    ax.set_xlim(0, 75); ax.set_ylim(-0.45, 1.32)
    ax.set_ylabel('$\\hat{s}_y\\,/\\,s_y^{\\mathrm{true}}$')
    ax.set_xlabel('time since NMPC handoff [s]', labelpad=1)
    

    fx, fy = [], []
    for r in runs:
        ok = (r['t'] >= r['t_dyn'] + 12) & (r['t'] <= r['t_drop'] - 1)
        if ok.sum() >= 3:
            fx.append(abs(r['s_true'])); fy.append(abs(r['sy'][ok].mean()))
    fx, fy = np.array(fx), np.array(fy)
    slope = float(np.sum(fx * fy) / np.sum(fx ** 2))

    for a_ in (ax,):
        a_.spines[['top', 'right']].set_visible(False)
        a_.grid(alpha=.55, lw=.5); a_.set_axisbelow(True)
    fig.tight_layout(pad=.25)
    out = os.path.join(HERE, 'fig_moment_truth')
    fig.savefig(out + '.pdf'); fig.savefig(out + '.png', dpi=300)

    car = np.array([100 * (r['sy'][(r['t'] >= r['t_dyn'] + 12) &
                                   (r['t'] <= r['t_drop'] - 1)].mean() - r['s_true'])
                    / abs(r['s_true']) for r in runs])
    print(f'wrote {out}.pdf  ({len(runs)} runs, W5 n={len(w5)}, slope {slope:.2f})')
    print(f'carry-phase median error {np.median(car):+.1f}%  |err| {np.median(abs(car)):.1f}%  '
          f'same sign {np.sum(car > 0)}/{len(car)}')


if __name__ == '__main__':
    main()
