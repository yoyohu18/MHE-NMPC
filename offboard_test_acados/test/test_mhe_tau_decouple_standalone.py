#!/usr/bin/env python3
# MHE<->NMPC 力矩通道完全解耦验证(2026-08-25)。不需要 acados/ROS。
#
# 覆盖三件事:
#   1. yaw 反扭矩重算的**符号**与 gz MulticopterMotorModel 约定一致
#      (tau_z_i = -dir_i*k_m*F_i;实数据标定:20/20 批与 NMPC 意图正相关)
#   2. 'phys_full' 档下 u_known 四个分量**没有一个**来自 NMPC
#   3. 区间平均(抗混叠)在 F 上做而不是在 ω 上做 —— 先平均再平方会因 Jensen
#      系统性低估真实冲量
#
# 跑法:PYTHONPATH=<ws>/src/offboard_test_acados:<ws>/src/offboard_test:. python3 本文件

import numpy as np

from offboard_test_acados import mhe_node as mn


def _hover_w(m=2.0643, g=9.81):
    """悬停配平转速(四电机相同)。"""
    return np.full(4, np.sqrt(m * g / (4 * mn.MOTOR_CONSTANT)))


def test_yaw_sign_convention():
    """ccw 桨(motorNumber 0,1)转得快 => 机体收到 cw(负 z)反扭矩。"""
    w = _hover_w()
    w[0] *= 1.10          # 加速一个 ccw 桨
    f = mn.THRUST_CAL_GAIN * mn.MOTOR_CONSTANT * w**2
    tau_z = mn.YAW_TORQUE_SIGN * mn.MOMENT_CONSTANT * float(np.sum(mn.YAW_DIR * f))
    assert tau_z < 0, f'ccw 桨加速应给出负 yaw 力矩,实得 {tau_z:+.5f}'

    w = _hover_w()
    w[2] *= 1.10          # 加速一个 cw 桨
    f = mn.THRUST_CAL_GAIN * mn.MOTOR_CONSTANT * w**2
    tau_z = mn.YAW_TORQUE_SIGN * mn.MOMENT_CONSTANT * float(np.sum(mn.YAW_DIR * f))
    assert tau_z > 0, f'cw 桨加速应给出正 yaw 力矩,实得 {tau_z:+.5f}'
    print('[1] yaw 符号约定 OK (ccw->负, cw->正)')


def test_hover_yaw_is_zero():
    """配平悬停(四桨等速)时反扭矩相消 => tau_z≈0。"""
    f = mn.THRUST_CAL_GAIN * mn.MOTOR_CONSTANT * _hover_w()**2
    tau_z = mn.MOMENT_CONSTANT * float(np.sum(mn.YAW_DIR * f))
    assert abs(tau_z) < 1e-9, f'配平悬停 tau_z 应为 0,实得 {tau_z:+.3e}'
    assert abs(float(np.sum(mn.ROTOR_DIR))) < 1e-9, 'ccw/cw 必须各两个'
    print('[2] 配平悬停 yaw 相消 OK')


def test_yaw_magnitude_matches_calibration():
    """量级自检:yaw 反扭矩应比同等转速差产生的 roll 力矩小一个量级
    (四旋翼 yaw 权限本来就靠反扭矩,力臂 0.174m vs k_m 0.016)。"""
    w = _hover_w(); w[0] *= 1.05; w[1] *= 1.05      # 两个 ccw 桨同时加速
    f = mn.THRUST_CAL_GAIN * mn.MOTOR_CONSTANT * w**2
    tau_z = abs(mn.MOMENT_CONSTANT * float(np.sum(mn.YAW_DIR * f)))
    # roll 对照必须选 y **同号**的两个桨(rotor1/rotor2 都是 y=+0.174),
    # 选 rotor0+rotor2 的话 y=-0.174 与 +0.174 正好相消 => tau_roll 恒 0。
    w2 = _hover_w(); w2[1] *= 1.05; w2[2] *= 1.05
    f2 = mn.THRUST_CAL_GAIN * mn.MOTOR_CONSTANT * w2**2
    tau_roll = abs(float(np.sum(mn.ROTOR_Y * f2)))
    ratio = tau_roll / max(tau_z, 1e-12)
    assert 5 < ratio < 50, f'roll/yaw 力矩权限比 {ratio:.1f} 不在预期量级'
    print(f'[3] yaw 权限量级 OK (roll/yaw ≈ {ratio:.1f}×)')


def test_phys_full_has_no_nmpc_content():
    """'phys_full' 档:u_known 四维全部被电机反算值覆盖,NMPC 意图值一个不剩。"""
    class _Now:
        nanoseconds = 1_230_000_000

    class _Clock:
        @staticmethod
        def now():
            return _Now()

    class _S:
        tau_source = 'phys_full'
        thrust_phys = 20.5
        tau_phys = np.array([0.11, -0.22, 0.033])
        pre_offboard_estimate = False
        _u_from_nmpc = True
        resid_log = type('L', (), {'enabled': False,
                                   'log_command': staticmethod(lambda u: None)})()

        def get_logger(self):
            return type('L', (), {'info': staticmethod(lambda m: None)})()

        def get_clock(self):
            return _Clock()
    s = _S()
    nmpc_intent = np.array([99.0, 9.9, -9.9, 9.9])   # 全是"假"意图值
    mn.MHENode.u_opt_cb(s, type('M', (), {'data': nmpc_intent})())
    assert np.allclose(s.u_known, [20.5, 0.11, -0.22, 0.033]), s.u_known
    assert not np.any(np.isclose(s.u_known, nmpc_intent)), 'NMPC 意图值有残留'
    assert np.isclose(s._last_u_rx_sec, 1.23), '已知输入接收时刻未更新'
    print('[4] phys_full 四维零 NMPC 残留 OK')

    # 对照:'phys' 档 yaw 必须仍是 NMPC 意图值(既有实验可复现)
    s2 = _S(); s2.tau_source = 'phys'
    mn.MHENode.u_opt_cb(s2, type('M', (), {'data': nmpc_intent})())
    assert np.isclose(s2.u_known[3], 9.9), "'phys' 档 yaw 应保持 NMPC 意图值"
    print("[5] 'phys' 档 legacy 语义未被破坏 OK")


def test_interval_average_on_force_not_speed():
    """抗混叠必须在 F=k_f*w^2 上平均。先平均 w 再平方会低估(Jensen)。"""
    class _S:
        _motor_f_acc = np.zeros(4)
        _motor_n = 0
    s = _S()
    # 一个区间内转速在 700/900 之间摆动(模拟 250Hz 快变)
    ws = [np.full(4, 700.0), np.full(4, 900.0)]
    for w in ws:
        s._motor_f_acc = s._motor_f_acc + mn.THRUST_CAL_GAIN * mn.MOTOR_CONSTANT * w**2
        s._motor_n += 1
    T_avg, tau_avg = mn.MHENode._drain_motor_avg(s)
    T_correct = 4 * mn.MOTOR_CONSTANT * np.mean([700.0**2, 900.0**2])
    T_naive = 4 * mn.MOTOR_CONSTANT * np.mean([700.0, 900.0])**2
    assert np.isclose(T_avg, T_correct), (T_avg, T_correct)
    assert T_naive < T_correct, '构造有误'
    print(f'[6] 区间平均在 F 上做 OK '
          f'(正确 {T_correct:.3f}N vs 先平均转速 {T_naive:.3f}N, '
          f'低估 {100*(1-T_naive/T_correct):.1f}%)')
    assert s._motor_n == 0, 'flush 后累加器应清零'
    T2, _ = mn.MHENode._drain_motor_avg(s)
    assert T2 is None, '空区间应返回 None'
    print('[7] 累加器 flush 语义 OK')


if __name__ == '__main__':
    test_yaw_sign_convention()
    test_hover_yaw_is_zero()
    test_yaw_magnitude_matches_calibration()
    test_phys_full_has_no_nmpc_content()
    test_interval_average_on_force_not_speed()
    print('\nall passed')
