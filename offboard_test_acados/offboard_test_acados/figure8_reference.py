#!/usr/bin/env python3
# 8 字轨迹(Gerono lemniscate)参考生成,跟 offboard_test.nmpc_node.build_reference
# (圆形轨迹)接口完全一致,独立放在 acados 包内,不动 offboard_test。
#
# 参数化: x=r*sin(a), y=r*sin(a)*cos(a)=r/2*sin(2a), a=w*tc 匀角速度转动。
# 在交叉点(a=0,π,即原点)处 v=wr*(cos a, cos 2a) 幅值固定为 wr*sqrt(2)≠0,
# 且 ax,ay 都正比于 sin(a)/sin(2a),在 a=0,π 处恰好为 0 ——也就是说这条曲线
# 在交叉点速度不反转、曲率不会炸到无穷大(不是尖点),但 yaw 仍会在交叉点
# 前后转得比圆形轨迹快,跟踪难度天然更高。

import math

import numpy as np

from offboard_test.nmpc_node import euler_to_quat

from .acados_params import p


def build_reference_figure8(t, r=1.0, w=0.3, z_hover=3.0, dz=0.5,
                             hover_time=2.0, ramp_time=4.0):
    # t < hover_time: 在 a=0 对应的起点 (0,0,z_hover) 悬停(a=0 时 sin(a)=0,
    # 两叶交汇处高度就是 z_hover,跟 dz 无关)
    # ramp_time 内  : 用 5 次 smooth-step 把振幅从 0 渐增到目标
    if t < hover_time:
        pos = np.array([0.0, 0.0, z_hover])
        vel = np.zeros(3)
        q   = np.array([1.0, 0.0, 0.0, 0.0])
        om  = np.zeros(3)
        return np.concatenate([pos, vel, q, om])

    tc = t - hover_time

    if tc < ramp_time:
        s = tc / ramp_time
        alpha     = 10*s**3 - 15*s**4 + 6*s**5
        alpha_dot = (30*s**2 - 60*s**3 + 30*s**4) / ramp_time
    else:
        alpha     = 1.0
        alpha_dot = 0.0

    a = w * tc
    sa, ca   = math.sin(a), math.cos(a)
    s2a, c2a = math.sin(2*a), math.cos(2*a)

    # z 用 sin(a) 而不是 cos(a):x 也是 sin(a),两者同符号同步变化,所以右侧
    # 叶子(x>0,a∈(0,π))整体抬高、左侧叶子(x<0,a∈(π,2π))整体压低,在原点
    # 交叉点(a=0,π)两叶在同一高度 z_hover 相接——这才是"一边高一边低"的
    # 立体 8 字。用 cos(a) 的话,高度只跟"穿越原点的次数"挂钩,跟左右叶子
    # 无关,达不到这个效果。
    x_f  = r * sa
    y_f  = 0.5 * r * s2a
    z_f  = z_hover + dz * sa
    vx_f = r * w * ca
    vy_f = r * w * c2a
    vz_f = dz * w * ca
    ax_f = -r * w * w * sa
    ay_f = -2.0 * r * w * w * s2a

    # 起点 (a=0): (0, 0, z_hover)
    x0, y0, z0 = 0.0, 0.0, z_hover

    x = x0 + alpha * (x_f - x0)
    y = y0 + alpha * (y_f - y0)
    z = z0 + alpha * (z_f - z0)
    vx = alpha_dot * (x_f - x0) + alpha * vx_f
    vy = alpha_dot * (y_f - y0) + alpha * vy_f
    vz = alpha_dot * (z_f - z0) + alpha * vz_f
    ax = alpha_dot * vx_f + alpha * ax_f
    ay = alpha_dot * vy_f + alpha * ay_f

    denom = vx*vx + vy*vy
    if denom > 1e-4:
        yaw      = math.atan2(vy, vx)
        yaw_rate = (vx*ay - vy*ax) / denom
    else:
        yaw      = 0.0
        yaw_rate = 0.0

    pos = np.array([x, y, z])
    vel = np.array([vx, vy, vz])
    q   = euler_to_quat(0.0, 0.0, yaw)
    om  = np.array([0.0, 0.0, yaw_rate])
    return np.concatenate([pos, vel, q, om])


def build_reference_window_figure8(t_start, N, dt, r=1.0, w=0.3, z_hover=3.0, dz=0.5):
    xref = np.zeros((p.nx, N+1))
    for i in range(N+1):
        xref[:, i] = build_reference_figure8(t_start + i*dt, r, w, z_hover, dz)
    return xref
