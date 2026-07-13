#!/usr/bin/env python3
"""无信号消融聚合(2026-07-13,长期计划 A.2)。读 run_nosignal_ablation.sh 写的
manifest,按 (delta, cfg) 分组解析每轮 mhe 日志的事件段(复用 parse_m0_logs 的
同一判据:T_phys 偏离基线 >1.5N 定物理生效时刻),输出三配置在 ±0.3/±0.8 两档下
入带/驻留/过冲/稳态误差的均值±标准差。自动剔除 drop 失效轮(T_phys 未偏离)。

用法:  python3 aggregate_nosignal.py [manifest]
不给参数则取 nmpc_test_results/ 下最新的 nosignal_ablation_*.txt(每批次一份)。"""
import glob
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, '/home/clear/ros2_ws_HJH/src/scripts/masschanger')
import parse_m0_logs as est          # noqa: E402 估计层(mhe 日志)
import parse_dropwindow_logs as ctl  # noqa: E402 控制层(nmpc [drop-window] 逐帧)

D = '/home/clear/ros2_ws_HJH/nmpc_test_results/'
M_EMPTY = 2.064  # x500 空机(真实平动质量,见 mhe_node THRUST_CAL_GAIN 注释)
CFG_ORDER = ['fixed', 'signal', 'nosignal']
CFG_LABEL = {'fixed': '固定权重', 'signal': '有信号版(M0)',
             'nosignal': '无信号版(残差自触发)'}


def fmt(vals, prec=2):
    v = np.array(vals, dtype=float)
    v = v[np.isfinite(v)]
    if len(v) == 0:
        return 'n/a'
    return f'{np.mean(v):.{prec}f}±{np.std(v):.{prec}f} (n={len(v)})'


def load_manifest(path):
    # (delta, cfg) -> [(mhe_stamp, nmpc_stamp), ...],只收 status=ok 的轮
    groups = defaultdict(list)
    for line in open(path):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) < 6:
            continue
        cfg, delta, rep, mhe_ts, nmpc_ts, status = parts[:6]
        if status != 'ok':
            continue
        groups[(float(delta), cfg)].append((mhe_ts, nmpc_ts))
    return groups


def main():
    if len(sys.argv) > 1:
        manifest = sys.argv[1]
    else:
        cands = sorted(glob.glob(D + 'nosignal_ablation_*.txt'),
                       key=os.path.getmtime)
        if not cands:
            print('没找到 nosignal_ablation_*.txt manifest,先跑 '
                  'run_nosignal_ablation.sh'); return
        manifest = cands[-1]
    print(f'manifest: {manifest}')
    groups = load_manifest(manifest)
    deltas = sorted({d for (d, _) in groups}, reverse=True)

    for delta in deltas:
        m_true = M_EMPTY + delta
        band = max(0.08, 0.15 * abs(delta))
        est.M_TRUE, est.BAND = m_true, band  # parse_m0_logs 用模块级全局
        print(f'\n===== delta={delta:+.2f} kg  '
              f'(真值 {m_true:.3f}kg, 带宽 ±{band:.3f}kg) =====')
        for cfg in CFG_ORDER:
            stamps = groups.get((delta, cfg), [])
            ent, stl, osd, mss = [], [], [], []
            pk, rec = [], []          # 控制层:pos_err 峰值 / 恢复时间
            bad = []
            for mhe_ts, nmpc_ts in stamps:
                try:
                    r = est.parse(D + f'mhe_node_{mhe_ts}.log')
                except (RuntimeError, FileNotFoundError):
                    bad.append(mhe_ts)
                    continue
                inband = (r['t'] >= 0) & (np.abs(r['m'] - m_true) <= band)
                ent.append(r['t'][inband][0] if np.any(inband) else np.nan)
                stl.append(r['t_settle'])
                osd.append(abs(r['m_min'] - m_true))
                mss.append(r['m_ss'])
                # 控制层(A.3):nmpc [drop-window] 逐帧 pos_err 峰值 + 恢复时间。
                # 与估计层同源判据(T 偏离基线 >1.5N 定物理生效)。缺日志或
                # 无 drop-window 段则跳过该轮控制层(估计层仍计)。
                try:
                    c = ctl.parse(D + f'acados_nmpc_node_{nmpc_ts}.log')
                    pk.append(c['pe_peak'])
                    rec.append(c['t_rec'])
                except (RuntimeError, FileNotFoundError):
                    pass
            print(f'  --- {CFG_LABEL[cfg]} ---')
            if bad:
                print(f'      剔除失效轮: {bad}')
            print(f'    [估计层] 首次入带 [s]  : {fmt(ent)}')
            print(f'    [估计层] 驻留收敛 [s]  : {fmt(stl)}')
            print(f'    [估计层] 过冲深度 [kg] : {fmt(osd, 3)}')
            print(f'    [估计层] 稳态估计 [kg] : {fmt(mss, 3)}')
            print(f'    [控制层] pos_err峰 [m] : {fmt(pk, 3)}')
            print(f'    [控制层] 恢复<5cm [s]  : {fmt(rec)}')


if __name__ == '__main__':
    main()
