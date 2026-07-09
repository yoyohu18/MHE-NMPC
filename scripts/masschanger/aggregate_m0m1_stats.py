#!/usr/bin/env python3
"""M0/M1 SITL 批量统计聚合:对三配置(fixed / M0 规则 / θ*)的全部有效轮,
估计层指标(入带/驻留/过冲)取自 mhe 日志,控制层指标(pos_err 峰/恢复)取自
nmpc 日志的 [drop-window] 逐帧段。自动剔除 drop 失效轮(T_phys 未偏离基线)。
输出均值±标准差表。运行于 nmpc_test_results 目录。"""
import sys

import numpy as np

sys.path.insert(0, '/home/clear/ros2_ws_HJH/src/scripts/masschanger')
import parse_m0_logs as est                      # noqa: E402(M_TRUE/BAND
import parse_dropwindow_logs as ctl              # noqa: E402 从环境变量取)

# 有效轮清单(2026-07-03 全天;mhe 日志, nmpc 日志或 None=该轮无 drop-window 段)
RUNS = {
    'fixed': [
        ('162003', None), ('171510', '171510'), ('172905', '172905'),
        ('173506', '173506'), ('175143', '175143'), ('175726', '175726'),
    ],
    'M0 rule': [
        ('031851', None), ('171711', '171710'), ('173105', '173105'),
        ('173706', '173706'), ('174602', '174602'), ('175533', '175533'),
    ],
    'theta*': [
        ('170600', None), ('172353', '172352'), ('173306', '173306'),
        ('173906', '173906'), ('175338', '175338'), ('175920', '175920'),
    ],
}

D = '/home/clear/ros2_ws_HJH/nmpc_test_results/'


def fmt(vals):
    v = np.array(vals)
    return f'{np.mean(v):.2f}±{np.std(v):.2f} (n={len(v)})'


def fmt3(vals):
    v = np.array(vals)
    return f'{np.mean(v):.3f}±{np.std(v):.3f} (n={len(v)})'


for cfg, runs in RUNS.items():
    ent, stl, osd, pk, rec = [], [], [], [], []
    bad = []
    for mhe_ts, nmpc_ts in runs:
        try:
            r = est.parse(D + f'mhe_node_20260703_{mhe_ts}.log')
        except RuntimeError:
            bad.append(mhe_ts)
            continue
        inband = (r['t'] >= 0) & (np.abs(r['m'] - est.M_TRUE) <= est.BAND)
        ent.append(r['t'][inband][0] if np.any(inband) else np.nan)
        stl.append(r['t_settle'])
        osd.append(abs(r['m_min'] - est.M_TRUE))   # 过冲深度(kg)
        if nmpc_ts:
            try:
                c = ctl.parse(D + f'acados_nmpc_node_20260703_{nmpc_ts}.log')
                pk.append(c['pe_peak'])
                rec.append(c['t_rec'])
            except RuntimeError:
                pass
    print(f'===== {cfg} =====')
    if bad:
        print(f'  剔除失效轮: {bad}')
    print(f'  首次入带 [s]  : {fmt(ent)}')
    print(f'  驻留收敛 [s]  : {fmt(stl)}')
    print(f'  过冲深度 [kg] : {fmt3(osd)}')
    if pk:
        print(f'  pos_err 峰 [m]: {fmt3(pk)}')
        print(f'  恢复<5cm [s]  : {fmt(rec)}')
