#!/usr/bin/env python3
# yaw 反扭矩符号标定(2026-08-25)。从残差诊断 CSV(resid_motor_* / resid_command_*)
# 重算电机反扭矩,与 NMPC 意图 tau_z 对表定符号。
#
# 2026-08-25 结论(nmpc_test_results/residual/ 的 20 批 07-31 数据):
#   候选 A(-dir,gz MulticopterMotorModel 约定) 20/20 批正相关,中位 r=+0.911 => 采用
#   候选 B(+dir)                                全负相关                    => 排除
#   ⚠️ 斜率中位只有 0.320:NMPC 意图 yaw 力矩比执行器实际输出**大 3 倍**。
#      对照 roll 斜率 1.083 / pitch 1.101 —— yaw 是唯一严重脱节的通道,
#      这正是把 yaw 也从 NMPC 解耦(mhe_tau_source=phys_full)的动机。
# 跑法:在工作区根目录 python3 src/scripts/gripper/calib_yaw_torque_sign.py

import glob, numpy as np

KF, KM = 8.54858e-06, 0.016
# motorNumber 0,1 = ccw ; 2,3 = cw  (x500 model.sdf)
DIR = np.array([+1.0, +1.0, -1.0, -1.0])       # ccw=+1, cw=-1
# gz MulticopterMotorModel: tau_z_i = -dir_i * momentConstant * thrust_i
CAND = {'A: -dir (gz 约定)': -DIR, 'B: +dir (翻转)': +DIR}

def load(p, cols):
    d = np.genfromtxt(p, delimiter=',', names=True)
    return {c: np.asarray(d[c], dtype=float) for c in cols}

rows = []
for mpath in sorted(glob.glob('nmpc_test_results/residual/resid_motor_*.csv')):
    cpath = mpath.replace('resid_motor_', 'resid_command_')
    try:
        m = load(mpath, ['t_recv_ns','w1','w2','w3','w4'])
        c = load(cpath, ['t_recv_ns','tau_z'])
    except Exception:
        continue
    if len(m['t_recv_ns']) < 50 or len(c['t_recv_ns']) < 50:
        continue
    W = np.vstack([m['w'+str(i+1)] for i in range(4)]).T   # (n,4)
    F = KF * W**2
    # 用 command 的时刻去插值 motor 流(command 更稀)
    tm, tc = m['t_recv_ns'], c['t_recv_ns']
    ok = (tc >= tm[0]) & (tc <= tm[-1])
    if ok.sum() < 50:
        continue
    tz_cmd = c['tau_z'][ok]
    if np.std(tz_cmd) < 1e-6:
        continue
    res = {}
    for name, s in CAND.items():
        tz_phys_full = KM * (F @ s)
        tz_phys = np.interp(tc[ok], tm, tz_phys_full)
        r = np.corrcoef(tz_phys, tz_cmd)[0, 1]
        k = np.polyfit(tz_cmd, tz_phys, 1)[0]
        res[name] = (r, k)
    rows.append((mpath.split('/')[-1][-18:-4], ok.sum(), res, np.std(tz_cmd)))

print(f'{"批次":<16}{"n":>6}{"std(tz_cmd)":>12} | {"A: r":>7}{"A: 斜率":>9} | {"B: r":>7}{"B: 斜率":>9}')
for stamp, n, res, sd in rows:
    a = res['A: -dir (gz 约定)']; b = res['B: +dir (翻转)']
    print(f'{stamp:<16}{n:>6}{sd:>12.5f} | {a[0]:>7.3f}{a[1]:>9.3f} | {b[0]:>7.3f}{b[1]:>9.3f}')

if rows:
    ra = np.array([r['A: -dir (gz 约定)'][0] for _,_,r,_ in rows])
    rb = np.array([r['B: +dir (翻转)'][0] for _,_,r,_ in rows])
    ka = np.array([r['A: -dir (gz 约定)'][1] for _,_,r,_ in rows])
    print(f'\n汇总 n={len(rows)} 批: A 相关中位 {np.median(ra):+.3f} (斜率中位 {np.median(ka):+.3f}), '
          f'B 相关中位 {np.median(rb):+.3f}')
    print(f'A 为正相关的批次: {(ra>0).sum()}/{len(ra)}')
