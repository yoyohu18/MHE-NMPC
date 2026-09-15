#!/usr/bin/env python3
"""Fig. 2: measured three-dimensional mission trajectory (PX4 SITL).

Data provenance
---------------
Source is the PX4 ulog topic ``vehicle_local_position_groundtruth`` (simulator
ground truth, 50 Hz), converted NED -> ENU, the frame the NMPC reference is
expressed in. Each ulog is matched to its NMPC run log through
``boot_time_utc_us`` (both runs give t0 - boot = +49.8 s) and the match is
cross-checked against the altitude channel logged independently by the NMPC
node (corr 0.9998, rms 0.03 m).

  main  = 20260911_033026  (W5, arm B)  confirms in maneuver, +4.46 s
  brake = 20260911_030319  (W5, arm B)  UNRESOLVED at +12 s, 3 s smooth brake,
                                        confirms +16.40 s

Extracted traces are cached under ``../data`` so the figure rebuilds without
the PX4 log tree. The reference curve uses the nominal batch parameters of
run_mainline_ab.sh (W5), with no fitting to the measurement.
"""
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, proj3d  # noqa: F401

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from paper_style import apply_style, BLUE, ORANGE, AQUA, INK, INK2, GRID

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, '..', 'data')
PX4LOG = "/home/clear/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-10"

# run key -> (ulog, boot_time_utc_us [s], NMPC handoff t=0 [s, same epoch])
RUNS = {
    'main':  ('20260911_033026', f"{PX4LOG}/19_30_39.ulg", 1789068634.783007, 1789068684.600),
    'brake': ('20260911_030319', f"{PX4LOG}/19_03_32.ulg", 1789067007.994,    1789067057.734),
}

# W5 trajectory parameters, scripts/gripper/run_mainline_ab.sh
R, W, RAMP, DZ, ZH, HOVER = 10.0, 0.1415, 9.36, 0.8, 6.0, 2.0
CX, CY = 0.99, 0.10                                   # figure-eight centre, from the run log
EV = dict(attach=6.30, lift=9.30, dyn=20.705, drop=56.046, confirm=60.502)   # main
EV_B = dict(dyn=20.80, drop=56.1, unres=68.1, confirm=72.50)                 # brake


def load(key):
    """Measured ENU trajectory, from the cached CSV if present."""
    stamp, ulg, boot, t0 = RUNS[key]
    csv = os.path.join(DATA, f'track3d_{stamp}.csv')
    if os.path.exists(csv):
        a = np.loadtxt(csv, delimiter=',', skiprows=1)
        return a[:, 0], a[:, 1], a[:, 2], a[:, 3]

    from pyulog import ULog
    u = ULog(ulg, ['vehicle_local_position_groundtruth'])
    d = [x for x in u.data_list if x.name == 'vehicle_local_position_groundtruth'][0]
    t = boot + d.data['timestamp'] / 1e6 - t0
    x, y, z = d.data['y'].astype(float), d.data['x'].astype(float), -d.data['z'].astype(float)
    m = (t >= -2.0) & (t <= 105.0)
    t, x, y, z = t[m], x[m], y[m], z[m]
    np.savetxt(csv, np.column_stack([t, x, y, z]), delimiter=',', fmt='%.5f',
               header='t_since_nmpc_handoff_s,x_enu_m,y_enu_m,z_enu_m', comments='')
    print('cached', csv)
    return t, x, y, z


def reference(t):
    """Nominal NMPC reference at mission time t (no fitting)."""
    tau = t - EV['dyn']
    if tau < HOVER:
        return CX, CY, ZH
    tc = tau - HOVER
    s = min(tc / RAMP, 1.0)
    al = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    a = W * tc
    return (al * R * math.sin(a) + CX,
            al * .5 * R * math.sin(2 * a) + CY,
            ZH + al * DZ * math.sin(a))


def seg(t, *arrs, lo, hi):
    m = (t >= lo) & (t <= hi)
    return [a[m] for a in arrs]


def at(tt, t, *arrs):
    i = int(np.argmin(np.abs(t - tt)))
    return [a[i] for a in arrs]


def main():
    apply_style()
    plt.rcParams['savefig.bbox'] = None   # keep the declared figsize aspect
    t, x, y, z = load('main')
    tb, xb, yb, zb = load('brake')

    fig = plt.figure(figsize=(3.5, 1.95))
    ax = fig.add_subplot(111, projection='3d')

    # nominal reference, drawn above the measurement so the (small) gap is visible
    tr = np.linspace(EV['dyn'], EV['dyn'] + HOVER + 2 * math.pi / W, 1600)
    ref = np.array([reference(v) for v in tr])

    xc, yc, zc = seg(t, x, y, z, lo=EV['dyn'], hi=EV['drop'])
    ax.plot(xc, yc, np.zeros_like(zc), color=GRID, lw=.9, zorder=0)          # ground shadow

    xa, ya, za = seg(t, x, y, z, lo=0.0, hi=EV['dyn'])
    ax.plot(xa, ya, za, color=INK2, lw=.75, alpha=.75, zorder=4)             # descend + lift

    ax.plot(xc, yc, zc, color=BLUE, lw=1.6, zorder=6, solid_capstyle='round',
            label='carrying')
    xr, yr, zr = seg(t, x, y, z, lo=EV['drop'], hi=EV['drop'] + 13)
    ax.plot(xr, yr, zr, color=ORANGE, lw=1.5, zorder=7, solid_capstyle='round',
            label='after release')
    xk, yk, zk = seg(tb, xb, yb, zb, lo=EV_B['unres'] - 3.5, hi=EV_B['confirm'] + 14)
    ax.plot(xk, yk, zk, color=AQUA, lw=1.3, ls=(0, (2.6, 1.9)), zorder=8,
            label='brake to hover (unresolved)')
    ax.plot(*ref.T, color='#C3CEDA', lw=.68, ls=(0, (3.0, 2.6)), zorder=9,
            label='reference')

    # drop-lines at the two tips make the one-lobe-high / one-lobe-low geometry readable
    for a_t in (math.pi / 2, 3 * math.pi / 2):
        tt_ = EV['dyn'] + HOVER + a_t / W
        if tt_ > EV['drop']:
            tt_ = EV['dyn'] + HOVER + (a_t + 2 * math.pi) / W
        xt, yt, zt = at(tt_, t, x, y, z)
        ax.plot([xt, xt], [yt, yt], [0, zt], color=INK2, lw=.5, ls=':', alpha=.6, zorder=3)
        ax.scatter([xt], [yt], [zt], s=7, color=INK2, zorder=9, depthshade=False)

    px, py, pz = at(EV['attach'], t, x, y, z)
    ax.scatter([px], [py], [pz], s=20, color=INK, zorder=12, depthshade=False)
    dx, dy, dz_ = at(EV['drop'], t, x, y, z)
    ax.scatter([dx], [dy], [dz_], s=26, color=ORANGE, zorder=12, depthshade=False,
               edgecolors='white', linewidths=.5)
    hx, hy, hz = at(EV_B['confirm'] + 12, tb, xb, yb, zb)
    ax.scatter([hx], [hy], [hz], s=46, facecolors='none', edgecolors=AQUA,
               linewidths=1.0, zorder=12, depthshade=False)
    ax.scatter([hx], [hy], [hz], s=7, color=AQUA, zorder=13, depthshade=False)

    ax.set_xlabel('$x$ [m]', labelpad=-7)
    ax.set_ylabel('$y$ [m]', labelpad=-7)
    ax.set_zlabel('$z$ [m]', labelpad=-6)
    ax.set_xlim(-11.5, 11.5); ax.set_ylim(-6.5, 6.5); ax.set_zlim(0, 7.6)
    ax.set_xticks([-10, 0, 10]); ax.set_yticks([-5, 0, 5]); ax.set_zticks([0, 3, 6])
    ax.set_box_aspect((2.30, 1.0, 1.00), zoom=1.12)
    ax.view_init(elev=21, azim=-60)
    ax.tick_params(pad=-3.2)
    for a_ in (ax.xaxis, ax.yaxis, ax.zaxis):
        a_.pane.set_facecolor('white'); a_.pane.set_alpha(1.0)
        a_._axinfo['grid'].update(color=GRID, linewidth=.4)
    ax.set_position([-.03, -.035, 1.04, 1.0])

    ax.legend(loc='upper left', bbox_to_anchor=(.03, .985), bbox_transform=fig.transFigure,
              fontsize=5.9, ncol=2,
              handlelength=1.4, handletextpad=.45, labelspacing=.22,
              columnspacing=1.0, borderpad=.0)

    def tag(text, p, dxy, color, size=6.8, ha='center'):
        x2, y2, _ = proj3d.proj_transform(*p, ax.get_proj())
        ax.annotate(text, xy=(x2, y2), xycoords='data', xytext=dxy,
                    textcoords='offset points', color=color, fontsize=size,
                    ha=ha, va='center', annotation_clip=False)

    tag('1 grasp', (px, py, pz), (11, -7), INK, ha='left')
    tag('3 release', (dx, dy, dz_), (-3, -9), ORANGE, ha='right')
    tag('hover', (hx, hy, hz), (9, 6), AQUA, size=6.3, ha='left')

    out = os.path.join(HERE, 'fig_track_3d_measured')
    fig.savefig(out + '.pdf', bbox_inches=None)
    fig.savefig(out + '.png', dpi=320, bbox_inches=None)
    print('wrote', out + '.pdf/.png')
    print(f"carry z range {zc.min():.2f}..{zc.max():.2f} m  (dz={DZ} m stereo figure-eight)")


if __name__ == '__main__':
    main()
