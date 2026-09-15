#!/usr/bin/env python3
"""One frozen W5/B flight. Panel (a) is the measured XY path (PX4 ulog ground
truth, cached in ../data/track3d_20260911_033026.csv) against the nominal reference."""
from pathlib import Path
import re
import hashlib
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from paper_style import apply_style, BLUE, ORANGE, AQUA

HERE = Path(__file__).resolve().parent
DATA = HERE.parent / 'data/mission_20260911_033026'
STAMP = re.compile(r'\[(\d{10}\.\d+)\]')
nt = (DATA / 'grip_nmpc_20260911_033026.log').read_text()
mt = (DATA / 'grip_mhe_20260911_033026.log').read_text()
rows = []
events = {}
for line in nt.splitlines():
    wall = STAMP.search(line)
    if not wall:
        continue
    wall = float(wall[1])
    m = re.search(r'\[attach-window\] t=([\d.]+)s pos_err=([\d.]+)m T=([\d.]+)N z=([-\d.]+) m_est=([\d.]+)', line)
    if m:
        rows.append([wall] + list(map(float, m.groups())))
    for key, token in [('attach', 'ATTACH command issued'), ('lift', '| LIFT:'),
                       ('carry', 'DYNAMIC: switch'), ('drop', 'DROP command issued'),
                       ('confirm', 'DROP complete:')]:
        if token in line:
            events[key] = wall
arr = np.asarray(rows)
assert len(arr) > 1000 and len(events) == 5
origin = np.median(arr[:, 0] - arr[:, 1])
events = {k: v-origin for k,v in events.items()}
decay = []
for line in mt.splitlines():
    m = re.search(r'\[s-decay\].*?\|s_out\|=([\d.]+).*?conf=([\d.]+)', line)
    ts = STAMP.search(line)
    if m and ts:
        decay.append([float(ts[1])-origin, float(m[1]), float(m[2])])
decay = np.asarray(decay)
assert len(decay) >= 50
apply_style()
blue, orange, green = BLUE, ORANGE, AQUA
fig = plt.figure(figsize=(7, 1.88), layout='constrained')
gs = fig.add_gridspec(3, 3, width_ratios=[1.25,1,1])
xy = fig.add_subplot(gs[:,0])
trk = np.loadtxt(HERE.parent/'data/track3d_20260911_033026.csv',
                 delimiter=',', skiprows=1)
CX, CY = 0.99, 0.10                      # figure-eight centre, from the run log
tk, xk, yk = trk[:,0], trk[:,1]-CX, trk[:,2]-CY
# nominal reference incl. the 9.36 s amplitude ramp (run_mainline_ab.sh, W5)
R_, W_, RAMP_, HOV_ = 10.0, 0.1415, 9.36, 2.0
tc = np.clip(tk - events['carry'] - HOV_, 0, None)
sr = np.clip(tc/RAMP_, 0, 1)
al = 10*sr**3 - 15*sr**4 + 6*sr**5
ang = W_*tc
rx, ry = al*R_*np.sin(ang), al*.5*R_*np.sin(2*ang)
rm = (tk >= events['carry']) & (tk <= events['drop'])
xy.plot(rx[rm], ry[rm], '--', color='#8E9BA8', lw=.9, zorder=2, label='reference')
car = (tk >= events['carry']) & (tk <= events['drop'])
rel = (tk >= events['drop']) & (tk <= events['drop']+13)
xy.plot(xk[car], yk[car], color=blue, lw=1.0, zorder=3, label='carrying')
xy.plot(xk[rel], yk[rel], color=orange, lw=1.0, zorder=4, label='after release')
xy.scatter([0],[0],color=green,s=20,zorder=5)
xy.annotate('carry starts', (0,0), xytext=(1.4,1.7),fontsize=6,
            arrowprops={'arrowstyle':'-', 'lw':.6, 'color':green})
xy.set(xlabel='relative x [m]', ylabel='relative y [m]', aspect='equal',
       title='(a) Measured horizontal path')
axes = [fig.add_subplot(gs[i,1:]) for i in range(3)]
axes[0].plot(arr[:,1],arr[:,4],color=blue,lw=.85,label='height')
axes[0].set(ylabel='z [m]',title='(b) Recorded grasp–carry–release flight')
axes[1].plot(arr[:,1],arr[:,5],color=blue,lw=.85)
axes[1].axhline(2.0643,color=green,ls=':',lw=.8)
axes[1].set(ylabel='mass [kg]')
axes[2].plot(arr[:,1],arr[:,2],color=blue,lw=.85)
axes[2].set(ylabel='error [m]',xlabel='time since NMPC handoff [s]')
for ax in [xy]+axes:
    ax.spines[['top','right']].set_visible(False)
    ax.grid(alpha=.16,lw=.5)
for ax in axes:
    ax.set_xlim(arr[0,1],arr[-1,1])
    for key in ('lift','carry','drop','confirm'):
        ax.axvline(events[key],color=orange if key in ('drop','confirm') else green,
                   ls='--',lw=.65,alpha=.7)
for key,label in [('lift','lift'),('carry','carry'),('drop','release')]:
    axes[0].text(events[key]+.5,.94,label,transform=axes[0].get_xaxis_transform(),
                 fontsize=5.8,va='top',color=orange if key=='drop' else green)
axes[2].annotate(f'confirmed +{events["confirm"]-events["drop"]:.2f} s',
                 (events['confirm'],.08),xytext=(65,.42),fontsize=6,color=orange,
                 arrowprops={'arrowstyle':'->','lw':.7,'color':orange})
fig.savefig(HERE/'fig_mission.pdf',bbox_inches='tight',pad_inches=.03)
fig.savefig(HERE/'fig_mission.png',dpi=220,bbox_inches='tight',pad_inches=.03)
plt.close(fig)
# Separate detail artifact retains the first-moment / confidence release transient.
fig, axes = plt.subplots(2,1,figsize=(3.45,2.4),sharex=True,layout='constrained')
axes[0].plot(decay[:,0]-events['drop'],decay[:,1],color=blue)
axes[0].set(ylabel=r'$|s_{out}|$ [kg m]',title='Published release transient (same flight)')
axes[1].plot(decay[:,0]-events['drop'],decay[:,2],color=orange)
axes[1].axhline(.9,color=green,ls='--',lw=.8)
axes[1].set(ylabel='confidence',xlabel='time since release command [s]')
for ax in axes:
    ax.spines[['top','right']].set_visible(False)
    ax.grid(alpha=.15)
fig.savefig(HERE/'fig_mission_release_detail.pdf',bbox_inches='tight')
plt.close(fig)
report = {'run':'20260911_033026','selection':'W5 final complete pair, B arm; illustrative, not best-error selection',
          'events_s':events,'dense_samples':len(arr),'s_decay_samples':len(decay),
          'xy':'measured, from PX4 ulog vehicle_local_position_groundtruth (19_30_39.ulg)',
          'sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(DATA.iterdir()) if p.is_file()}}
(HERE/'fig_mission_provenance.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
