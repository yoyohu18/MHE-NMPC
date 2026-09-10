"""Frozen parser used only to reproduce the legacy event-timeline paper figure."""
import re

import numpy as np

M_TRUE = 1.564
BAND = 0.08
THRESH = 1.5

LINE = re.compile(
    r'\[(\d+)\.(\d+)\].*\[(transition|post-event)\] '
    r'm_est=([\d.]+) kg \(T_phys=([\d.]+)N\)')
EVENT = re.compile(r'\[(\d+)\.(\d+)\].*mass event \[')


def parse(path):
    t, mass, thrust = [], [], []
    t_event = None
    with open(path) as stream:
        for line in stream:
            event = EVENT.search(line)
            if event and t_event is None:
                t_event = float(event.group(1)) + float(event.group(2)) * 1e-9
            sample = LINE.search(line)
            if sample:
                t.append(float(sample.group(1)) + float(sample.group(2)) * 1e-9)
                mass.append(float(sample.group(4)))
                thrust.append(float(sample.group(5)))
    t, mass, thrust = map(np.array, (t, mass, thrust))
    if t_event is None or len(t) == 0:
        raise RuntimeError(f'{path}: no event-window samples found')
    baseline = np.mean(thrust[:3])
    changed = np.abs(thrust - baseline) > THRESH
    if not np.any(changed):
        raise RuntimeError(f'{path}: physical thrust never left its baseline')
    t_phys = t[changed][0]
    after = t >= t_phys
    outside = after & (np.abs(mass - M_TRUE) > BAND)
    t_settle = (t[outside][-1] - t_phys + 0.1) if np.any(outside) else 0.0
    tail = t >= (t_phys + t_settle + 0.3)
    return dict(t_event=t_event, t_phys=t_phys, lag=t_phys - t_event,
                t_settle=t_settle,
                m_ss=float(np.mean(mass[tail])) if np.any(tail) else float('nan'),
                m_min=float(np.min(mass)), n=len(t),
                t=t - t_phys, m=mass, T=thrust)
