#!/usr/bin/env python3
"""Static scientific plot of paired eccentricity compensation ablation."""
import csv, pathlib, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
folder=pathlib.Path(sys.argv[1]); rows=list(csv.DictReader((folder/'metrics.csv').open()))
fig,axes=plt.subplots(1,2,figsize=(9,3.6),layout='constrained')
for ax,metric,title in zip(axes,['lift_rms','fig8_rms'],['Lift tracking','Loaded figure-eight tracking']):
    for idx in sorted(set(r['idx'] for r in rows)):
        d={r['arm']:r for r in rows if r['idx']==idx}
        if set(d)!={'ON','OFF'} or any(r['run_status']!='ok' or r['status']!='ok' for r in d.values()): continue
        ax.plot([0,1],[float(d[k][metric]) for k in ['ON','OFF']],'-o',alpha=.8,label=f'Pair {idx}')
    ax.set_xticks([0,1],['Compensation ON','Compensation OFF'])
    ax.set_ylabel('Position RMS error (m)'); ax.set_title(title)
    ax.grid(axis='y',alpha=.25); ax.set_xlim(-.2,1.2)
    ax.set_ylim(0, max(float(r[metric]) for r in rows if r.get(metric) not in (None, '', 'nan')) * 1.15)
axes[1].legend(frameon=False,fontsize=8)
fig.suptitle('Control-side CoM ablation | 0.15 kg, nominal 0.10 m offset')
fig.savefig(folder/'paired_rms.png',dpi=180)
fig.savefig(folder/'paired_rms.svg')
