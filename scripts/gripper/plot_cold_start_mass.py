import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

SURFACE  = '#fcfcfb'
TEXT_PRI = '#0b0b0b'
TEXT_SEC = '#52514e'
TEXT_MUT = '#8a8985'
SERIES_1 = '#2a78d6'
GRID     = '#e6e5e1'
BAND     = '#efeeea'

d = np.load('/tmp/claude-1000/-home-clear-ros2-ws-HJH/3f7b5199-2f24-4edb-8d3a-022d6b1e689a/scratchpad/coldstart.npz')
t, T = d['t'], d['trajs']
m_true, starts = float(d['m_true']), d['starts']
lbx_default = 0.95 * m_true
traj = T[0]
max_div = float(np.ptp(T, axis=0).max())

fig = plt.figure(figsize=(11, 4.9), dpi=170)
fig.patch.set_facecolor(SURFACE)
gs = GridSpec(1, 2, width_ratios=[1, 2.35], wspace=0.28,
              left=0.075, right=0.90, top=0.775, bottom=0.13)

# ---------------- panel A: 起点塌缩 ----------------
axA = fig.add_subplot(gs[0]); axA.set_facecolor(SURFACE)
axA.set_xlim(-0.5, 1.32); axA.set_ylim(0.05, 4.5)

axA.axhspan(0.05, lbx_default, color=BAND, lw=0, zorder=0)
axA.axhline(lbx_default, color=TEXT_MUT, lw=1, ls=(0, (4, 3)), zorder=1)
axA.text(1.28, 0.30, '$0.95\\cdot m_B$ = 1.961 kg\n(excluded by default)',
         ha='right', va='bottom', fontsize=8, color=TEXT_MUT,
         style='italic', linespacing=1.4)

for m0 in starts:
    axA.plot([0, 1], [m0, traj[0]], color=TEXT_MUT, lw=1,
             ls=(0, (2, 2.5)), zorder=3, alpha=0.9)
    axA.plot([0], [m0], 'o', ms=8, mfc=SURFACE, mec=TEXT_SEC, mew=1.8, zorder=5)
    axA.text(-0.10, m0, f'{m0:.2f}', ha='right', va='center',
             fontsize=9.5, color=TEXT_SEC)
axA.plot([1], [traj[0]], 'o', ms=9, color=SERIES_1, zorder=6)
axA.axhline(m_true, color=TEXT_SEC, lw=1.2, ls=(0, (5, 3)), zorder=2)

axA.set_xticks([0, 1])
axA.set_xticklabels(['initial\nguess', 'first\nsolve'], fontsize=9, color=TEXT_SEC)
axA.set_ylabel('mass (kg)', fontsize=10, color=TEXT_SEC)
axA.set_title('four starts collapse', fontsize=10.5, color=TEXT_PRI,
              loc='left', pad=8)
axA.grid(axis='y', color=GRID, lw=0.8); axA.set_axisbelow(True)
for sp in ('top', 'right'): axA.spines[sp].set_visible(False)
for sp in ('left', 'bottom'): axA.spines[sp].set_color(GRID)
axA.tick_params(colors=TEXT_SEC, labelsize=9)

# ---------------- panel B: 轨迹细节 ----------------
axB = fig.add_subplot(gs[1]); axB.set_facecolor(SURFACE)
lo, hi = 2.030, 2.108
axB.set_xlim(-0.45, t[-1] + 0.35); axB.set_ylim(lo, hi)

axB.axhline(m_true, color=TEXT_SEC, lw=1.2, ls=(0, (5, 3)), zorder=2)
axB.text(t[-1] + 0.22, lo + 0.0045,
         f'dashed line = true mass {m_true:.4f} kg',
         ha='right', va='bottom', fontsize=8.8, color=TEXT_SEC)

axB.plot(t, traj, color=SERIES_1, lw=2, solid_capstyle='round', zorder=6)
axB.plot([t[0]], [traj[0]], 'o', ms=9, color=SERIES_1, zorder=7)

axB.annotate(f'{traj[-1]:.4f} kg   {100*(traj[-1]-m_true)/m_true:+.2f}%',
             xy=(t[-1], traj[-1]), xytext=(t[-1] - 0.3, hi - 0.006),
             ha='right', va='top', fontsize=10, color=TEXT_PRI,
             arrowprops=dict(arrowstyle='-', color=TEXT_MUT, lw=0.9,
                             shrinkA=2, shrinkB=4))
axB.annotate('first solve (window full, N·dt = 2.0 s):\nalready within 0.1% of truth',
             xy=(t[0], traj[0]), xytext=(t[0] + 0.55, hi - 0.004),
             ha='left', va='top', fontsize=8.8, color=TEXT_MUT, linespacing=1.4,
             arrowprops=dict(arrowstyle='-', color=TEXT_MUT, lw=0.9,
                             shrinkA=3, shrinkB=6,
                             connectionstyle='angle,angleA=0,angleB=90,rad=0'))

axB.set_xlabel('time after window filled  (s)', fontsize=10, color=TEXT_SEC)
axB.set_ylabel('MHE mass estimate  (kg)', fontsize=10, color=TEXT_SEC)
axB.set_title('estimate trajectory  (all four starts identical to 1e-10 kg)',
              fontsize=10.5, color=TEXT_PRI, loc='left', pad=8)
axB.grid(axis='y', color=GRID, lw=0.8); axB.set_axisbelow(True)
for sp in ('top', 'right'): axB.spines[sp].set_visible(False)
for sp in ('left', 'bottom'): axB.spines[sp].set_color(GRID)
axB.tick_params(colors=TEXT_SEC, labelsize=9)

fig.text(0.075, 0.945, 'MHE cold start: the initial guess does not enter the solution',
         fontsize=13.5, color=TEXT_PRI, ha='left', va='bottom')
fig.text(0.075, 0.893,
         f'MHE_NO_MASS_PRIOR=1  ·  Q0[m] = {float(d["Q0_m"]):.0e} (σ ≈ 1000 kg)  ·  '
         f'lbx = [{float(d["m_min"]):.1f}, {float(d["m_max"]):.1f}] kg  ·  '
         f'max divergence across the four starts = {max_div:.1e} kg',
         fontsize=8.6, color=TEXT_MUT, ha='left', va='bottom')
fig.text(0.075, 0.845,
         'synthetic hover data, constant true mass  ·  N=20, dt=0.1 s  ·  '
         'trajectories recorded from the first solve (window full)',
         fontsize=8.6, color=TEXT_MUT, ha='left', va='bottom')

out = 'nmpc_test_results/mhe_cold_start_mass_estimate.png'
fig.savefig(out, facecolor=SURFACE, bbox_inches='tight')
print('saved', out)
