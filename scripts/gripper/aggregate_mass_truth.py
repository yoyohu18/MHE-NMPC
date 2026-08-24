#!/usr/bin/env python3
"""estimate vs ground truth 的定量指标 + 图(第 8/10 项)。

数据源:mhe_node 的 residual_logger 'internal' 流(resid_internal_<stamp>.csv),
需要 residual_log_dir 打开、且 eval_true_payload_mass 传了真值(见
run_gripper_headless.sh 的 EVAL_TRUE_PAYLOAD_MASS)。真值列全部只从那个**评估专用**
参数算,不进任何模型——这是"不许把未知参数真值喂给估计器"的实现方式。

输出(每个 stamp 一行 + 汇总):
  attach 前空机段 bias/RMSE
  attach 暂态: 入带时间(±2%,稳定入带)、过冲峰值
  带载稳态 bias/RMSE
  drop 暂态: 入带时间、下冲峰值(是否触 m_min 下界)
  drop 后稳态 bias/RMSE
  c / J 的估计-真值偏差(它们由 m_hat 与同一 r_p 代数生成,所以纯反映质量误差)

用法:
  python3 aggregate_mass_truth.py <resid_dir> [stamp ...]
  python3 aggregate_mass_truth.py <resid_dir> --plot out.png
"""
import glob
import os
import sys

import numpy as np

M_MIN_DEFAULT = 1.961        # 0.95*m_B,solver lbx(见 mhe_params.m_min)
TOL = 0.02                   # ±2% 入带判据


def load(path):
    a = np.genfromtxt(path, delimiter=',', names=True)
    if a.size < 2 or 'm_true' not in (a.dtype.names or ()):
        return None
    t = (a['t_recv_ns'] - a['t_recv_ns'][0]) / 1e9
    ok = np.isfinite(a['m_true'])
    return t[ok], a[ok]


def stable_entry(t, m_hat, m_ref, t0, t1):
    """[t0,t1) 段内稳定进入 ±TOL 带的时刻(相对 t0)。稳定 = 之后不再出带。"""
    sel = (t >= t0) & (t < t1)
    if not sel.any():
        return float('nan')
    tt, mm = t[sel], m_hat[sel]
    inb = np.abs(mm - m_ref) / m_ref <= TOL
    if not inb.any():
        return float('nan')
    out = np.where(~inb)[0]
    first = 0 if out.size == 0 else out[-1] + 1
    return float(tt[first] - t0) if first < len(tt) else float('nan')


def analyse(path, settle=2.5):
    got = load(path)
    if got is None:
        return None
    t, a = got
    m_hat, m_true, att = a['m_est'], a['m_true'], a['payload_attached']
    # 事件时刻:payload_attached 的 0->1 与 1->0
    d = np.diff(att.astype(int))
    t_att = t[np.argmax(d > 0) + 1] if (d > 0).any() else None
    t_drop = t[np.argmax(d < 0) + 1] if (d < 0).any() else None
    m_load = float(np.max(m_true))
    m_bare = float(np.min(m_true))
    r = {'file': os.path.basename(path), 'n': len(t),
         't_attach': t_att, 't_drop': t_drop,
         'm_bare': m_bare, 'm_load': m_load}

    def seg(t0, t1):
        s = (t >= t0) & (t < t1) if t1 is not None else (t >= t0)
        return m_hat[s] - m_true[s]

    if t_att is not None:
        e = seg(0.0, t_att)
        r['pre_bias'], r['pre_rmse'] = _bm(e)
        end = t_drop if t_drop is not None else t[-1]
        e = seg(min(t_att + settle, end), end)
        r['load_bias'], r['load_rmse'] = _bm(e)
        r['attach_entry'] = stable_entry(t, m_hat, m_load, t_att, end)
        sel = (t >= t_att) & (t < end)
        r['attach_overshoot'] = float(np.max(m_hat[sel]) - m_load) if sel.any() else float('nan')
    if t_drop is not None:
        e = seg(t_drop + settle, None)
        r['post_bias'], r['post_rmse'] = _bm(e)
        r['drop_entry'] = stable_entry(t, m_hat, m_bare, t_drop, t[-1] + 1e-6)
        sel = t >= t_drop
        r['drop_undershoot'] = float(np.min(m_hat[sel]) - m_bare)
        r['hit_m_min'] = int(np.any(m_hat[sel] <= M_MIN_DEFAULT + 1e-4))
    # c / J 偏差(纯由质量误差驱动,几何用同一个 r_p)
    names = a.dtype.names or ()
    for k in ('cy', 'Jxx', 'Jyz'):
        if k + '_hat' not in names or k + '_true' not in names:
            r[k + '_mae'] = float('nan')
            continue
        h, tr = a[k + '_hat'], a[k + '_true']
        m = np.isfinite(h) & np.isfinite(tr) & (att > 0)
        r[k + '_mae'] = float(np.mean(np.abs(h[m] - tr[m]))) if m.any() else float('nan')
    return r


def _bm(e):
    if not len(e):
        return float('nan'), float('nan')
    return float(np.mean(e)), float(np.sqrt(np.mean(e ** 2)))


def fmt(r):
    def g(k, f='{:+.4f}'):
        v = r.get(k, float('nan'))
        return f.format(v) if v is not None and np.isfinite(v) else '   n/a'
    return (f"{r['file'][:42]:<42} "
            f"pre {g('pre_bias')}/{g('pre_rmse','{:.4f}')}  "
            f"load {g('load_bias')}/{g('load_rmse','{:.4f}')}  "
            f"post {g('post_bias')}/{g('post_rmse','{:.4f}')}  "
            f"entry a={g('attach_entry','{:.2f}')}s d={g('drop_entry','{:.2f}')}s  "
            f"over {g('attach_overshoot')} under {g('drop_undershoot')} "
            f"m_min{'撞' if r.get('hit_m_min') else '未'}")


def main(argv):
    if not argv:
        print(__doc__)
        return 1
    d = argv[0]
    files = sorted(glob.glob(os.path.join(d, 'resid_internal_*.csv')))
    rest, skip = argv[1:], False
    for s in rest:
        if skip:                      # --plot 后面那个是输出路径,不是 stamp 过滤
            skip = False
            continue
        if s == '--plot':
            skip = True
            continue
        if not s.startswith('--'):
            files = [f for f in files if s in f]
    rows = [x for x in (analyse(f) for f in files) if x]
    if not rows:
        print(f'no usable resid_internal_*.csv with m_true column in {d}\n'
              '(需要 residual_log_dir + eval_true_payload_mass 都传了)')
        return 1
    print('bias/RMSE 单位 kg;entry = 稳定进入 ±2% 带的时间')
    for r in rows:
        print(fmt(r))
    if '--plot' in argv:
        out = argv[argv.index('--plot') + 1]
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 4.5))
        for f in files[:6]:
            got = load(f)
            if got is None:
                continue
            t, a = got
            ax.plot(t, a['m_true'], 'k--', lw=1.2,
                    label='m_true' if f == files[0] else None)
            ax.plot(t, a['m_est'], lw=1.0, label=os.path.basename(f)[:28])
        ax.axhline(M_MIN_DEFAULT, color='r', ls=':', lw=0.8, label='m_min (lbx)')
        ax.set_xlabel('t [s]'); ax.set_ylabel('mass [kg]')
        ax.set_title('MHE mass estimate vs ground truth')
        ax.legend(fontsize=7); ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(out, dpi=130)
        print('plot ->', out)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
