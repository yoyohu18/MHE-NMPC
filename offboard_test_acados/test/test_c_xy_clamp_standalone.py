#!/usr/bin/env python3
# c_xy 越界剔除,2026-09-10。不需要 acados/ROS,纯逻辑层。
#
# 背景:同一个 s_est,两条消费路径原本待遇相反 ——
#   · 释放判据 _check_s_collapse:剔除 |s|>moment_abs_max 的帧,理由写着
#     "物理不可能值、动态失真,不参与 moment 投票"。figure-8 下实测 7.6%~18.3%
#     的帧越界。
#   · c_xy 一阶矩路径 _update_c_xy_est:**零门控**,同样那批帧除以 m_t 之后原样
#     发给 NMPC 当 model.p 的 c_xy,于是约 1/6 的帧在给控制器编造物理上不可能的
#     偏心量,生成假的 τ=c×T 补偿力矩 —— 而 roll/pitch 力矩本来就有 21%/47% 的帧
#     打在约束上([flight-diag] u_sat),没有余量消化这种噪声。
#
# 本文件锁住的核心是 test_same_criterion_as_release_vote:两条路径必须用**同一个
# 判据**。剩下几条锁行为细节(保持上一拍、话题速率不变、边界含等号、legacy 可退)。
#
# 跑法:  python3 test/test_c_xy_clamp_standalone.py

import sys

import numpy as np

from offboard_test_acados.mhe_node import MHENode
from offboard_test_acados.mhe_params import p as mhe_p

RUN = MHENode._update_c_xy_est
ABS_MAX = 0.039        # = payload_mass_envelope(0.30kg) x payload_rxy_envelope(0.13m)
M_T = 2.36


class _Log:
    def __init__(self):
        self.msgs = []

    def warn(self, m):
        self.msgs.append(('W', m))

    def info(self, m):
        self.msgs.append(('I', m))


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))


class _S:
    """最小 mock self。前段的质量域释放判据置零绕开(不在本次改动范围内)。"""

    def __init__(self, clamp=True):
        self.c_xy_mass_release_mp = 0.0
        self._load_armed = False
        self.s_release_ratio = 0.0
        self.c_xy_from_moment = True
        self.c_xy_clamp_enable = clamp
        self.moment_abs_max = ABS_MAX
        self.m_est = M_T
        self.s_est = np.zeros(2)
        self.c_xy_est = np.zeros(2)
        self._c_xy_inited = False
        self._c_xy_reject = 0
        self._c_xy_total = 0
        self._log = _Log()
        self.c_xy_est_pub = _Pub()

    def get_logger(self):
        return self._log


def test_no_publish_before_first_valid():
    s = _S()
    s.s_est = np.array([0.20, 0.0])          # |s| >> 上限
    RUN(s)
    assert s.c_xy_est_pub.sent == [], '无合法初值时越界帧仍被发布'
    assert s._c_xy_reject == 1
    print('[PASS] test_no_publish_before_first_valid')


def test_valid_frame_passes_through():
    s = _S()
    s.s_est = np.array([0.015, 0.0])
    RUN(s)
    assert len(s.c_xy_est_pub.sent) == 1
    assert abs(s.c_xy_est_pub.sent[-1][0] - 0.015 / M_T) < 1e-12
    print('[PASS] test_valid_frame_passes_through')


def test_overrange_holds_last_good_and_keeps_rate():
    s = _S()
    s.s_est = np.array([0.015, 0.0])
    RUN(s)
    good = s.c_xy_est_pub.sent[-1]
    s.s_est = np.array([0.20, 0.30])
    RUN(s)
    assert len(s.c_xy_est_pub.sent) == 2, '越界帧掉了发布,话题速率被改变'
    assert s.c_xy_est_pub.sent[-1] == good, '越界帧没有保持上一拍'
    assert s._c_xy_reject == 1
    print('[PASS] test_overrange_holds_last_good_and_keeps_rate')


def test_boundary_frame_is_kept():
    """判据是严格大于:恰好等于上限的帧放行(与释放判据口径一致)。"""
    s = _S()
    s.s_est = np.array([ABS_MAX, 0.0])
    RUN(s)
    assert abs(s.c_xy_est_pub.sent[-1][0] - ABS_MAX / M_T) < 1e-12, '边界帧被误剔除'
    print('[PASS] test_boundary_frame_is_kept')


def test_legacy_switch_restores_old_behaviour():
    s = _S(clamp=False)
    s.s_est = np.array([0.20, 0.0])
    RUN(s)
    assert abs(s.c_xy_est_pub.sent[-1][0] - 0.20 / M_T) < 1e-12, (
        'c_xy_clamp_enable=false 没有逐位退回旧行为')
    print('[PASS] test_legacy_switch_restores_old_behaviour')


def test_same_criterion_as_release_vote():
    """★ 核心:被 c_xy 剔除的,正是释放判据会剔除的那一批。"""
    for s_norm in (0.0, 0.010, 0.038, ABS_MAX, ABS_MAX + 1e-7, 0.05, 0.20, 1.0):
        release_rejects = s_norm > ABS_MAX     # _check_s_collapse 的口径
        s = _S()
        s.c_xy_est = np.array([9.0, 9.0])      # 哨兵:保持上一拍 => 被剔除
        s._c_xy_inited = True
        s.s_est = np.array([s_norm, 0.0])
        RUN(s)
        c_xy_rejects = (s.c_xy_est_pub.sent[-1] == [9.0, 9.0])
        assert release_rejects == c_xy_rejects, (
            f'|s|={s_norm}: 释放判据剔除={release_rejects} 但 '
            f'c_xy 剔除={c_xy_rejects} —— 两条路径判据不一致')
    print('[PASS] test_same_criterion_as_release_vote')


if __name__ == '__main__':
    if not mhe_p.ns:
        print('[SKIP] 本档 ns=0,c_xy 一阶矩路径不启用')
        sys.exit(0)
    print(f'=== ns={mhe_p.ns} moment_abs_max={ABS_MAX} m_t={M_T} ===')
    test_no_publish_before_first_valid()
    test_valid_frame_passes_through()
    test_overrange_holds_last_good_and_keeps_rate()
    test_boundary_frame_is_kept()
    test_legacy_switch_restores_old_behaviour()
    test_same_criterion_as_release_vote()
    print('\nall passed')
    sys.exit(0)
