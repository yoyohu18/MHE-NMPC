#!/usr/bin/env python3
"""8 字机动段控制力矩的**周期平均可辨识性**分析(2026-08-28)。

【要回答的问题】机动段能不能用"跨周期长平均"把偏心载荷的 DC 配平力矩捞出来?
若能,c_xy 释放档的门控就不必只放在悬停段。

【为什么不能只算 mean(tau)】机体系力矩平衡是
    tau_meas = J·om_dot + om x (J·om) + tau_grav + tau_aero + ...
一个周期上:
  - <J·om_dot> ≈ 0        (周期信号的导数,均值天然为零 —— 这是"能捞"的理由)
  - <om x (J·om)> ≠ 0     (**二次项,周期均值一般不为零**) —— 这是"捞出来的东西
                           里混了什么"的关键。只报 mean(tau_meas) 会把这一项
                           当成配平力矩,正是本脚本要拆开的。
  - <tau_grav>  = 要捞的量。body 系里 tau_grav = (m_P·r_P) x (R^T·(0,0,-g)),
                  偏心矢量 r_P 机体固连 => 小倾角下近似常量,不随 yaw 转走。
故本脚本把三项分别算出来,再看 DC 残项是否 ①显著非零 ②跨周期可重复
③接近真值预测 m_P·g·(r_y, -r_x, 0)。

【时间轴】四条流的 t_recv_ns 都是 **同一个 mhe_node 进程的单调钟**,跨流对齐
用它是合法的(header.stamp 才有 Gazebo/PX4 跨时钟域问题,见 residual_logger 顶部
声明)。代价是含各自的传输延迟——对"周期均值"这种 DC 量,固定延迟不影响结论。

用法: python3 analyze_torque_dc.py <resid_dir> [--stamp YYYYmmdd_HHMMSS] [--w 0.283]
"""
import argparse
import sys
from pathlib import Path

import numpy as np

# 机体惯量(src/offboard_test/offboard_test/nmpc_node.py 的 Params)
JXX, JYY, JZZ = 0.0142, 0.0142, 0.0210
G = 9.81
J = np.array([JXX, JYY, JZZ])


def load(d, stream, stamp):
    f = d / f'resid_{stream}_{stamp}.csv'
    if not f.exists():
        return None
    a = np.genfromtxt(f, delimiter=',', names=True)
    return np.atleast_1d(a)


def quat_to_R(qw, qx, qy, qz):
    """body->world 旋转阵,逐时刻堆成 (n,3,3)。"""
    n = np.linalg.norm(np.stack([qw, qx, qy, qz]), axis=0)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    R = np.empty((len(qw), 3, 3))
    R[:, 0, 0] = 1 - 2 * (qy ** 2 + qz ** 2)
    R[:, 0, 1] = 2 * (qx * qy - qz * qw)
    R[:, 0, 2] = 2 * (qx * qz + qy * qw)
    R[:, 1, 0] = 2 * (qx * qy + qz * qw)
    R[:, 1, 1] = 1 - 2 * (qx ** 2 + qz ** 2)
    R[:, 1, 2] = 2 * (qy * qz - qx * qw)
    R[:, 2, 0] = 2 * (qx * qz - qy * qw)
    R[:, 2, 1] = 2 * (qy * qz + qx * qw)
    R[:, 2, 2] = 1 - 2 * (qx ** 2 + qy ** 2)
    return R


def detect_fig8(t, px, py, r_nom, w):
    """8 字稳态段起点:滑窗内 x 摆幅达到标称半径的 70% 且此后一直保持。
    ramp 段(振幅渐增)因此被自动排除,不需要读 NMPC 日志的墙钟时间戳。"""
    T = 2 * np.pi / w
    ok = np.zeros(len(t), dtype=bool)
    for i in range(len(t)):
        m = (t >= t[i]) & (t < t[i] + T)
        if m.sum() < 10:
            break
        ok[i] = (px[m].max() - px[m].min()) > 1.4 * r_nom * 0.7
    if not ok.any():
        return None
    # 取最后一段连续 True 的起点(避开中途的偶发命中)
    idx = np.where(ok)[0]
    brk = np.where(np.diff(idx) > 1)[0]
    start = idx[brk[-1] + 1] if len(brk) else idx[0]
    return t[start]


def cycle_stats(t, v, t0, T, ncyc):
    """逐周期均值。返回 (per_cycle_means (ncyc,k), 使用的周期数)。"""
    out = []
    for k in range(ncyc):
        m = (t >= t0 + k * T) & (t < t0 + (k + 1) * T)
        if m.sum() < 20:
            break
        out.append(v[m].mean(axis=0))
    return np.array(out)


def fmt(name, per_cyc, truth=None):
    mu = per_cyc.mean(axis=0)
    sd = per_cyc.std(axis=0, ddof=1) if len(per_cyc) > 1 else np.zeros(3)
    sem = sd / max(np.sqrt(len(per_cyc)), 1)
    line = f'  {name:22s}'
    for i, ax in enumerate('xyz'):
        line += f' {ax}={mu[i]:+.4f}±{sem[i]:.4f}'
    print(line)
    if truth is not None:
        print(f'  {"":22s} 真值预测      x={truth[0]:+.4f}      '
              f'y={truth[1]:+.4f}      z={truth[2]:+.4f}')
    return mu, sem


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('resid_dir')
    ap.add_argument('--stamp', default=None)
    ap.add_argument('--w', type=float, default=0.283)
    ap.add_argument('--r', type=float, default=10.0)
    ap.add_argument('--skip-cycles', type=float, default=0.5,
                    help='8字起点后先丢掉多少个周期(暂态)')
    args = ap.parse_args()

    d = Path(args.resid_dir)
    if args.stamp is None:
        cands = sorted(d.glob('resid_motor_*.csv'))
        if not cands:
            sys.exit(f'{d} 里没有 resid_motor_*.csv')
        args.stamp = cands[-1].stem.replace('resid_motor_', '')
    print(f'[dc] stamp={args.stamp}  w={args.w}  T={2*np.pi/args.w:.2f}s')

    motor = load(d, 'motor', args.stamp)
    odom = load(d, 'odom', args.stamp)
    imu = load(d, 'imu', args.stamp)
    intern = load(d, 'internal', args.stamp)
    for nm, a in [('motor', motor), ('odom', odom)]:
        if a is None or a.size < 50:
            sys.exit(f'[dc] {nm} 流为空或过短,无法分析')

    t_m = motor['t_recv_ns'] * 1e-9
    t_o = odom['t_recv_ns'] * 1e-9
    t00 = min(t_m[0], t_o[0])
    t_m, t_o = t_m - t00, t_o - t00

    t0 = detect_fig8(t_o, odom['px'], odom['py'], args.r, args.w)
    if t0 is None:
        sys.exit('[dc] 没检测到 8 字稳态段(振幅始终不到标称半径的 70%)')
    T = 2 * np.pi / args.w
    t0 += args.skip_cycles * T
    span = min(t_m[-1], t_o[-1]) - t0
    ncyc = int(span // T)
    print(f'[dc] 8字稳态段起点 t={t0:.1f}s(已跳 {args.skip_cycles} 周期暂态), '
          f'可用 {span:.1f}s = {span/T:.2f} 周期 -> 取 {ncyc} 个完整周期')
    if ncyc < 2:
        print('[dc] ⚠️ 完整周期 < 2,跨周期一致性无法判断,只报单周期均值')
    if ncyc < 1:
        sys.exit('[dc] 连一个完整周期都没有,采集时长不够')

    # --- 1. 实测力矩(电机转速反算,真执行器输出) ---
    tau = np.stack([motor['tau_x'], motor['tau_y'], motor['tau_z']], axis=1)
    pc_tau = cycle_stats(t_m, tau, t0, T, ncyc)
    rms = np.sqrt((tau[(t_m >= t0) & (t_m < t0 + ncyc * T)] ** 2).mean(axis=0))

    # --- 2. 陀螺项 om x (J om):周期均值一般非零,必须扣掉 ---
    if imu is not None and imu.size > 50:
        t_w = imu['t_recv_ns'] * 1e-9 - t00
        om = np.stack([imu['wx'], imu['wy'], imu['wz']], axis=1)
        src = 'imu'
    else:
        t_w = t_o
        om = np.stack([odom['wb_x'], odom['wb_y'], odom['wb_z']], axis=1)
        src = 'odom(31Hz,微分噪声大)'
    gyro = np.cross(om, om * J)
    pc_gyro = cycle_stats(t_w, gyro, t0, T, ncyc)

    # --- 3. 惯性项 J*om_dot:周期均值应≈0,作为噪声底 ---
    om_dot = np.gradient(om, t_w, axis=0)
    pc_inert = cycle_stats(t_w, om_dot * J, t0, T, ncyc)

    # --- 4. 重力配平项真值预测(只用 internal 流的评估列,不进模型) ---
    truth = None
    if intern is not None and intern.size > 10 and 'cx_true' in intern.dtype.names:
        t_i = intern['t_recv_ns'] * 1e-9 - t00
        mi = (t_i >= t0)
        if mi.sum() > 5:
            m_true = np.nanmedian(intern['m_true'][mi])
            cx = np.nanmedian(intern['cx_true'][mi])
            cy = np.nanmedian(intern['cy_true'][mi])
            if np.isfinite(m_true) and np.isfinite(cx):
                # tau_ctrl 必须抵消重力力矩 c x (0,0,-m_T g) = (-m g cy, +m g cx, 0)
                truth = np.array([m_true * G * cy, -m_true * G * cx, 0.0])
                print(f'[dc] 真值列: m_T={m_true:.4f}kg  c=({cx:+.5f},{cy:+.5f})m')

    print(f'\n[dc] 逐周期均值(n={ncyc} 周期, ±为跨周期 SEM), om 源={src}')
    mu_tau, sem_tau = fmt('实测 <tau_meas>', pc_tau)
    mu_gy, sem_gy = fmt('陀螺 <om x J om>', pc_gyro)
    mu_in, sem_in = fmt('惯性 <J om_dot>', pc_inert)
    dc = mu_tau - mu_gy - mu_in
    sem_dc = np.sqrt(sem_tau ** 2 + sem_gy ** 2 + sem_in ** 2)
    print(f'  {"DC 残项(=配平力矩)":20s}'
          + ''.join(f' {ax}={dc[i]:+.4f}±{sem_dc[i]:.4f}' for i, ax in enumerate('xyz')))
    if truth is not None:
        print(f'  {"":22s} 真值预测      x={truth[0]:+.4f}      '
              f'y={truth[1]:+.4f}      z={truth[2]:+.4f}')

    print(f'\n[dc] 瞬时力矩 RMS(机动段): '
          + ' '.join(f'{ax}={rms[i]:.4f}' for i, ax in enumerate('xyz')))
    print('[dc] DC/AC 比(捞取难度,越大越好): '
          + ' '.join(f'{ax}={abs(dc[i])/max(rms[i],1e-9):.3f}' for i, ax in enumerate('xyz')))

    # --- 判据 ---
    print('\n[dc] === 判据 ===')
    for i, ax in enumerate('xyz'):
        sig = abs(dc[i]) > 2 * sem_dc[i] if ncyc > 1 else np.nan
        s = f'  {ax}: DC={dc[i]:+.4f}Nm  2*SEM={2*sem_dc[i]:.4f}'
        if ncyc > 1:
            s += '  -> ' + ('显著非零 ✅' if sig else '与零不可分 ❌')
        if truth is not None and abs(truth[i]) > 1e-6:
            s += f'  相对真值误差 {100*(dc[i]-truth[i])/truth[i]:+.1f}%'
        print(s)
    if ncyc > 1:
        gy_frac = np.abs(mu_gy) / np.maximum(np.abs(mu_tau), 1e-9)
        print('  陀螺项占实测周期均值的比例(不扣就等于误差): '
              + ' '.join(f'{ax}={100*gy_frac[i]:.1f}%' for i, ax in enumerate('xyz')))


if __name__ == '__main__':
    main()
